#!/usr/bin/env python3
"""
Convert experiment CSV time_s values to equivalent wall-clock times.

In the ROS2/Nav2 experiments, data_collector runs with use_sim_time=true, so
the CSV time_s column is simulation time. A batch that actually runs at real
time factor RTF takes approximately:

    wall_time_s = sim_time_s / RTF

Examples:
    python3 codefiles/convert_csv_time.py /home/dx/ros2_ws/A/dstar_social_2x.csv \
        -o /home/dx/ros2_ws/A/dstar_social_2x_time_equiv.csv \
        -t old_0p3x=0.3 -t actual_0p843x=0.843 \
        --source-rtf 0.843 --final-only --only-mode dstar_social

    # If the CSV is mixed, convert only the speed-changed mode:
    python3 codefiles/convert_csv_time.py mixed.csv \
        -t old_0p3x=0.3 --final-only --only-mode dstar_social
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import statistics
import sys
from collections import OrderedDict, defaultdict
from pathlib import Path
from typing import Iterable


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read a ROS2 experiment CSV and add equivalent wall-clock time "
            "columns for one or more achieved real-time factors."
        )
    )
    parser.add_argument("csv_file", help="Input CSV file.")
    parser.add_argument(
        "-o",
        "--output",
        help="Output CSV path. Default: <input_stem>_time_equiv.csv",
    )
    parser.add_argument(
        "-t",
        "--target-rtf",
        action="append",
        default=[],
        metavar="LABEL=RTF",
        help=(
            "Target achieved real-time factor. Can be repeated. "
            "Examples: old_0p3x=0.3, dstar_social_2x=0.843, 1.0"
        ),
    )
    parser.add_argument(
        "--source-rtf",
        type=float,
        help=(
            "Optional source achieved RTF. Adds source wall time and "
            "source-to-target wall-time scale columns."
        ),
    )
    parser.add_argument(
        "--time-column",
        default="time_s",
        help="CSV time column to convert. Default: time_s",
    )
    parser.add_argument(
        "--final-only",
        action="store_true",
        help=(
            "Keep only the last row for each (mode, experiment) pair before "
            "converting. Useful when CSV contains retry attempts."
        ),
    )
    parser.add_argument(
        "--only-mode",
        action="append",
        default=[],
        metavar="MODE",
        help=(
            "Only add equivalent time values for this mode. Can be repeated. "
            "Useful when only one mode was run with changed physics speed."
        ),
    )
    parser.add_argument(
        "--round",
        type=int,
        default=2,
        help="Decimal places for generated time columns. Default: 2",
    )
    parser.add_argument(
        "--no-summary",
        action="store_true",
        help="Do not print the per-mode summary table.",
    )
    return parser.parse_args()


def sanitize_label(label: str) -> str:
    label = label.strip().replace(".", "p")
    label = re.sub(r"[^A-Za-z0-9_]+", "_", label)
    label = re.sub(r"_+", "_", label).strip("_")
    return label or "target"


def parse_target(spec: str) -> tuple[str, float]:
    if "=" in spec:
        label, raw_rtf = spec.split("=", 1)
    else:
        raw_rtf = spec
        label = f"rtf_{raw_rtf}x"

    try:
        rtf = float(raw_rtf)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"Invalid RTF value: {spec}") from exc

    if rtf <= 0:
        raise argparse.ArgumentTypeError(f"RTF must be > 0: {spec}")
    return sanitize_label(label), rtf


def default_output_path(input_path: Path) -> Path:
    return input_path.with_name(f"{input_path.stem}_time_equiv{input_path.suffix}")


def read_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"{path} does not contain a CSV header")
        return list(reader.fieldnames), list(reader)


def keep_final_rows(rows: Iterable[dict[str, str]]) -> list[dict[str, str]]:
    final_rows: OrderedDict[tuple[str, str], dict[str, str]] = OrderedDict()
    passthrough: list[dict[str, str]] = []

    for row in rows:
        mode = row.get("mode")
        experiment = row.get("experiment")
        if mode is None or experiment is None:
            passthrough.append(row)
            continue
        final_rows[(mode, experiment)] = row

    if passthrough:
        return passthrough + list(final_rows.values())
    return list(final_rows.values())


def parse_float(value: str | None) -> float | None:
    if value is None:
        return None
    value = value.strip()
    if not value:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def fmt(value: float | None, digits: int) -> str:
    if value is None:
        return ""
    return f"{value:.{digits}f}"


def add_time_columns(
    fieldnames: list[str],
    rows: list[dict[str, str]],
    time_column: str,
    targets: list[tuple[str, float]],
    source_rtf: float | None,
    digits: int,
    only_modes: set[str] | None,
) -> list[str]:
    new_fields = list(fieldnames)

    if source_rtf is not None:
        if source_rtf <= 0:
            raise ValueError("--source-rtf must be > 0")
        for col in ("source_wall_s", "source_wall_min"):
            if col not in new_fields:
                new_fields.append(col)

    for label, _rtf in targets:
        for col in (f"time_wall_{label}_s", f"time_wall_{label}_min"):
            if col not in new_fields:
                new_fields.append(col)
        if source_rtf is not None:
            col = f"source_to_{label}_wall_scale"
            if col not in new_fields:
                new_fields.append(col)

    for row in rows:
        should_convert = (
            not only_modes
            or row.get("mode") in only_modes
        )
        sim_s = parse_float(row.get(time_column)) if should_convert else None

        if source_rtf is not None:
            source_wall_s = None if sim_s is None else sim_s / source_rtf
            row["source_wall_s"] = fmt(source_wall_s, digits)
            row["source_wall_min"] = fmt(
                None if source_wall_s is None else source_wall_s / 60.0,
                digits,
            )

        for label, rtf in targets:
            wall_s = None if sim_s is None else sim_s / rtf
            row[f"time_wall_{label}_s"] = fmt(wall_s, digits)
            row[f"time_wall_{label}_min"] = fmt(
                None if wall_s is None else wall_s / 60.0,
                digits,
            )
            if source_rtf is not None:
                row[f"source_to_{label}_wall_scale"] = fmt(source_rtf / rtf, 4)

    return new_fields


def write_rows(path: Path, fieldnames: list[str], rows: list[dict[str, str]]) -> None:
    parent = path.parent
    if str(parent):
        parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def mean(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def print_summary(
    rows: list[dict[str, str]],
    time_column: str,
    targets: list[tuple[str, float]],
) -> None:
    groups: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        groups[row.get("mode") or "all"].append(row)

    headers = ["mode", "rows", "success", "avg_sim_s"]
    for label, _rtf in targets:
        headers.append(f"avg_wall_{label}_s")
        headers.append(f"total_wall_{label}_min")

    table: list[list[str]] = [headers]
    for mode in sorted(groups):
        mode_rows = groups[mode]
        sim_values = [
            value
            for value in (parse_float(row.get(time_column)) for row in mode_rows)
            if value is not None
        ]
        success = sum(1 for row in mode_rows if row.get("result") == "succeeded")
        line = [
            mode,
            str(len(mode_rows)),
            str(success),
            fmt(mean(sim_values), 2),
        ]
        for label, _rtf in targets:
            wall_values = [
                value
                for value in (
                    parse_float(row.get(f"time_wall_{label}_s")) for row in mode_rows
                )
                if value is not None
            ]
            line.append(fmt(mean(wall_values), 2))
            line.append(fmt(sum(wall_values) / 60.0 if wall_values else None, 2))
        table.append(line)

    widths = [max(len(row[i]) for row in table) for i in range(len(headers))]
    for idx, row in enumerate(table):
        print("  ".join(cell.ljust(widths[i]) for i, cell in enumerate(row)))
        if idx == 0:
            print("  ".join("-" * width for width in widths))


def main() -> int:
    args = parse_args()
    input_path = Path(os.path.expanduser(args.csv_file)).resolve()
    if not input_path.exists():
        print(f"Input CSV not found: {input_path}", file=sys.stderr)
        return 2

    if not args.target_rtf:
        print(
            "At least one --target-rtf is required, for example: "
            "-t old_0p3x=0.3 -t actual_0p843x=0.843",
            file=sys.stderr,
        )
        return 2

    try:
        targets = [parse_target(spec) for spec in args.target_rtf]
        only_modes = set()
        for value in args.only_mode:
            only_modes.update(
                mode.strip() for mode in value.split(",") if mode.strip()
            )
        fieldnames, rows = read_rows(input_path)
        if args.time_column not in fieldnames:
            raise ValueError(
                f"CSV does not contain time column '{args.time_column}'. "
                f"Available columns: {', '.join(fieldnames)}"
            )
        if args.final_only:
            rows = keep_final_rows(rows)

        output_path = (
            Path(os.path.expanduser(args.output)).resolve()
            if args.output
            else default_output_path(input_path).resolve()
        )
        if output_path == input_path:
            raise ValueError("Output path must be different from input CSV")

        fieldnames = add_time_columns(
            fieldnames,
            rows,
            args.time_column,
            targets,
            args.source_rtf,
            max(args.round, 0),
            only_modes or None,
        )
        write_rows(output_path, fieldnames, rows)
    except (OSError, ValueError, argparse.ArgumentTypeError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"Wrote: {output_path}")
    print(f"Rows: {len(rows)}")
    if args.final_only:
        print("Rows were reduced with --final-only.")
    if args.only_mode:
        print(f"Converted modes only: {', '.join(sorted(only_modes))}")
    if not args.no_summary:
        print()
        print_summary(rows, args.time_column, targets)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
