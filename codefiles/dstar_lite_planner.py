#!/usr/bin/env python3
"""D* Lite grid planner for the cafe map.

This module is intentionally independent of Nav2. Nav2 does not ship a D*
global planner plugin by default, so this file provides a course-project
implementation that can be used for algorithm comparison against A* on the
same occupancy-grid map.
"""

from __future__ import annotations

import argparse
import heapq
import math
import os
import time
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np


GridNode = Tuple[int, int]
WorldPoint = Tuple[float, float]

DEFAULT_PEDESTRIANS: List[WorldPoint] = [
    (3.0, 5.0),
    (-3.0, 5.0),
    (2.0, -4.5),
    (0.0, 6.0),
    (-1.0, 2.0),
]


@dataclass(frozen=True)
class GridMap:
    occupied: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def width(self) -> int:
        return int(self.occupied.shape[1])

    @property
    def height(self) -> int:
        return int(self.occupied.shape[0])

    def in_bounds(self, node: GridNode) -> bool:
        x, y = node
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, node: GridNode) -> bool:
        x, y = node
        return self.in_bounds(node) and not bool(self.occupied[y, x])

    def world_to_grid(self, x: float, y: float) -> GridNode:
        gx = int(math.floor((x - self.origin_x) / self.resolution))
        gy = int(math.floor((y - self.origin_y) / self.resolution))
        return gx, gy

    def grid_to_world(self, node: GridNode) -> Tuple[float, float]:
        x, y = node
        return (
            self.origin_x + (x + 0.5) * self.resolution,
            self.origin_y + (y + 0.5) * self.resolution,
        )


def _parse_simple_yaml(path: str) -> Dict[str, object]:
    data: Dict[str, object] = {}
    with open(path, "r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            value = value.strip().strip("'\"")
            if value.startswith("[") and value.endswith("]"):
                items = [v.strip() for v in value[1:-1].split(",") if v.strip()]
                data[key.strip()] = [float(v) for v in items]
                continue
            try:
                data[key.strip()] = int(value)
            except ValueError:
                try:
                    data[key.strip()] = float(value)
                except ValueError:
                    data[key.strip()] = value
    return data


def _read_token(f) -> bytes:
    token = bytearray()
    while True:
        ch = f.read(1)
        if not ch:
            return bytes(token)
        if ch == b"#":
            f.readline()
            continue
        if ch.isspace():
            if token:
                return bytes(token)
            continue
        token.extend(ch)


def _read_pgm(path: str) -> np.ndarray:
    with open(path, "rb") as f:
        magic = _read_token(f)
        if magic not in (b"P2", b"P5"):
            raise ValueError(f"Unsupported PGM format {magic!r}; expected P2 or P5")
        width = int(_read_token(f))
        height = int(_read_token(f))
        maxval = int(_read_token(f))
        if maxval <= 0 or maxval > 255:
            raise ValueError("Only 8-bit PGM maps are supported")
        if magic == b"P5":
            data = np.frombuffer(f.read(width * height), dtype=np.uint8)
        else:
            values = [int(_read_token(f)) for _ in range(width * height)]
            data = np.array(values, dtype=np.uint8)
    if data.size != width * height:
        raise ValueError(f"PGM data size mismatch in {path}")
    return data.reshape((height, width))


def load_map(yaml_path: str) -> GridMap:
    meta = _parse_simple_yaml(yaml_path)
    image = str(meta.get("image", ""))
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(yaml_path), image)
    resolution = float(meta.get("resolution", 0.05))
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    if not isinstance(origin, list) or len(origin) < 2:
        raise ValueError("Map origin must be a list with at least x and y")
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.25))
    negate = int(meta.get("negate", 0))

    pgm = _read_pgm(image).astype(np.float32) / 255.0
    occ_prob = pgm if negate else (1.0 - pgm)
    occupied = occ_prob >= occupied_thresh
    unknown = (occ_prob > free_thresh) & (occ_prob < occupied_thresh)
    occupied = occupied | unknown
    return GridMap(occupied=occupied, resolution=resolution,
                   origin_x=float(origin[0]), origin_y=float(origin[1]))


class DStarLite:
    """D* Lite planner on an 8-connected occupancy grid."""

    def __init__(self, grid_map: GridMap, start: GridNode, goal: GridNode):
        if not grid_map.is_free(start):
            raise ValueError(f"Start cell {start} is not free")
        if not grid_map.is_free(goal):
            raise ValueError(f"Goal cell {goal} is not free")
        self.map = grid_map
        self.start = start
        self.last_start = start
        self.goal = goal
        self.km = 0.0
        self.g: Dict[GridNode, float] = defaultdict(lambda: math.inf)
        self.rhs: Dict[GridNode, float] = defaultdict(lambda: math.inf)
        self.rhs[self.goal] = 0.0
        self.open_heap: List[Tuple[Tuple[float, float], GridNode]] = []
        self.open_keys: Dict[GridNode, Tuple[float, float]] = {}
        self.changed_cells: Set[GridNode] = set()
        self._push(self.goal, self._calculate_key(self.goal))

    @staticmethod
    def _heuristic(a: GridNode, b: GridNode) -> float:
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return (dx + dy) + (math.sqrt(2.0) - 2.0) * min(dx, dy)

    def _calculate_key(self, node: GridNode) -> Tuple[float, float]:
        best = min(self.g[node], self.rhs[node])
        return best + self._heuristic(self.start, node) + self.km, best

    def _push(self, node: GridNode, key: Tuple[float, float]) -> None:
        self.open_keys[node] = key
        heapq.heappush(self.open_heap, (key, node))

    def _pop_valid(self) -> Tuple[Tuple[float, float], Optional[GridNode]]:
        while self.open_heap:
            key, node = heapq.heappop(self.open_heap)
            if self.open_keys.get(node) == key:
                self.open_keys.pop(node, None)
                return key, node
        return (math.inf, math.inf), None

    def _top_key(self) -> Tuple[float, float]:
        while self.open_heap:
            key, node = self.open_heap[0]
            if self.open_keys.get(node) == key:
                return key
            heapq.heappop(self.open_heap)
        return math.inf, math.inf

    def _neighbors(self, node: GridNode) -> Iterable[GridNode]:
        x, y = node
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nxt = (x + dx, y + dy)
                if self.map.in_bounds(nxt):
                    yield nxt

    def _cost(self, a: GridNode, b: GridNode) -> float:
        if not self.map.is_free(a) or not self.map.is_free(b):
            return math.inf
        diagonal = a[0] != b[0] and a[1] != b[1]
        return math.sqrt(2.0) if diagonal else 1.0

    def _successors(self, node: GridNode) -> Iterable[GridNode]:
        for nxt in self._neighbors(node):
            if self._cost(node, nxt) < math.inf:
                yield nxt

    def _predecessors(self, node: GridNode) -> Iterable[GridNode]:
        return self._successors(node)

    def _update_vertex(self, node: GridNode) -> None:
        if node != self.goal:
            self.rhs[node] = min(
                (self._cost(node, succ) + self.g[succ] for succ in self._successors(node)),
                default=math.inf,
            )
        self.open_keys.pop(node, None)
        if not math.isclose(self.g[node], self.rhs[node]):
            self._push(node, self._calculate_key(node))

    def compute_shortest_path(self, max_steps: int = 2_000_000) -> int:
        steps = 0
        while (
            self._top_key() < self._calculate_key(self.start)
            or not math.isclose(self.rhs[self.start], self.g[self.start])
        ):
            if steps >= max_steps:
                raise RuntimeError("D* Lite exceeded max_steps while planning")
            old_key, node = self._pop_valid()
            if node is None:
                break
            new_key = self._calculate_key(node)
            if old_key < new_key:
                self._push(node, new_key)
            elif self.g[node] > self.rhs[node]:
                self.g[node] = self.rhs[node]
                for pred in self._predecessors(node):
                    self._update_vertex(pred)
            else:
                self.g[node] = math.inf
                self._update_vertex(node)
                for pred in self._predecessors(node):
                    self._update_vertex(pred)
            steps += 1
        return steps

    def update_start(self, new_start: GridNode) -> None:
        if not self.map.is_free(new_start):
            raise ValueError(f"New start cell {new_start} is not free")
        self.km += self._heuristic(self.last_start, new_start)
        self.last_start = new_start
        self.start = new_start

    def set_obstacle(self, node: GridNode, occupied: bool = True) -> None:
        if not self.map.in_bounds(node):
            return
        if bool(self.map.occupied[node[1], node[0]]) == occupied:
            return
        self.map.occupied[node[1], node[0]] = occupied
        self.changed_cells.add(node)
        self._update_vertex(node)
        for pred in self._predecessors(node):
            self._update_vertex(pred)
        for nbr in self._neighbors(node):
            self._update_vertex(nbr)

    def apply_obstacle_updates(self, cells: Sequence[GridNode], occupied: bool = True) -> None:
        for cell in cells:
            self.set_obstacle(cell, occupied=occupied)

    def extract_path(self, max_len: int = 100_000) -> List[GridNode]:
        if math.isinf(self.g[self.start]) and math.isinf(self.rhs[self.start]):
            return []
        path = [self.start]
        current = self.start
        visited = {current}
        for _ in range(max_len):
            if current == self.goal:
                return path
            candidates = []
            for succ in self._successors(current):
                value = self._cost(current, succ) + self.g[succ]
                candidates.append((value, self._heuristic(succ, self.goal), succ))
            if not candidates:
                return []
            _, _, current = min(candidates)
            if current in visited:
                return []
            visited.add(current)
            path.append(current)
        return []


def path_length_cells(path: Sequence[GridNode]) -> float:
    total = 0.0
    for a, b in zip(path, path[1:]):
        total += math.sqrt(2.0) if a[0] != b[0] and a[1] != b[1] else 1.0
    return total


def astar_search(grid_map: GridMap, start: GridNode, goal: GridNode) -> Tuple[List[GridNode], int]:
    """Standard A* on the same 8-connected grid used by D* Lite."""
    if not grid_map.is_free(start):
        raise ValueError(f"Start cell {start} is not free")
    if not grid_map.is_free(goal):
        raise ValueError(f"Goal cell {goal} is not free")

    open_heap: List[Tuple[float, GridNode]] = []
    heapq.heappush(open_heap, (DStarLite._heuristic(start, goal), start))
    came_from: Dict[GridNode, GridNode] = {}
    g_score: Dict[GridNode, float] = defaultdict(lambda: math.inf)
    g_score[start] = 0.0
    closed: Set[GridNode] = set()
    expansions = 0

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path, expansions

        closed.add(current)
        expansions += 1
        for neighbor in _free_neighbors(grid_map, current):
            step = math.sqrt(2.0) if current[0] != neighbor[0] and current[1] != neighbor[1] else 1.0
            tentative = g_score[current] + step
            if tentative >= g_score[neighbor]:
                continue
            came_from[neighbor] = current
            g_score[neighbor] = tentative
            priority = tentative + DStarLite._heuristic(neighbor, goal)
            heapq.heappush(open_heap, (priority, neighbor))

    return [], expansions


def pedestrian_cells(grid_map: GridMap, pedestrians: Sequence[WorldPoint], radius: float) -> List[GridNode]:
    cells: Set[GridNode] = set()
    cell_radius = max(1, int(math.ceil(radius / grid_map.resolution)))
    for wx, wy in pedestrians:
        center = grid_map.world_to_grid(wx, wy)
        for dy in range(-cell_radius, cell_radius + 1):
            for dx in range(-cell_radius, cell_radius + 1):
                node = (center[0] + dx, center[1] + dy)
                if not grid_map.in_bounds(node):
                    continue
                cx, cy = grid_map.grid_to_world(node)
                if math.hypot(cx - wx, cy - wy) <= radius:
                    cells.add(node)
    return sorted(cells)


def path_pedestrian_metrics(
    grid_map: GridMap,
    path: Sequence[GridNode],
    pedestrians: Sequence[WorldPoint],
    encounter_threshold: float = 1.0,
    personal_zone: float = 1.2,
    intimate_zone: float = 0.45,
) -> Tuple[int, int, int, Optional[float]]:
    if not path or not pedestrians:
        return 0, 0, 0, None

    encounters = 0
    personal = 0
    intimate = 0
    prev_encounter = False
    prev_personal = False
    prev_intimate = False
    min_dist = math.inf

    for node in path:
        wx, wy = grid_map.grid_to_world(node)
        nearest = min(math.hypot(wx - px, wy - py) for px, py in pedestrians)
        min_dist = min(min_dist, nearest)

        now_encounter = nearest < encounter_threshold
        now_personal = nearest < personal_zone
        now_intimate = nearest < intimate_zone
        if now_encounter and not prev_encounter:
            encounters += 1
        if now_personal and not prev_personal:
            personal += 1
        if now_intimate and not prev_intimate:
            intimate += 1
        prev_encounter = now_encounter
        prev_personal = now_personal
        prev_intimate = now_intimate

    return encounters, personal, intimate, min_dist


def _free_neighbors(grid_map: GridMap, node: GridNode) -> Iterable[GridNode]:
    x, y = node
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nxt = (x + dx, y + dy)
            if grid_map.is_free(nxt):
                yield nxt


def _parse_world_pair(value: str) -> Tuple[float, float]:
    x_str, y_str = value.split(",", 1)
    return float(x_str), float(y_str)


def _parse_grid_cells(value: str) -> List[GridNode]:
    cells: List[GridNode] = []
    if not value:
        return cells
    for item in value.split(";"):
        x_str, y_str = item.split(",", 1)
        cells.append((int(x_str), int(y_str)))
    return cells


def _parse_pedestrians(value: str) -> List[WorldPoint]:
    value = value.strip()
    if value.lower() == "none":
        return []
    if value.lower() == "default":
        return list(DEFAULT_PEDESTRIANS)
    pedestrians: List[WorldPoint] = []
    for item in value.split(";"):
        item = item.strip()
        if not item:
            continue
        x_str, y_str = item.split(",", 1)
        pedestrians.append((float(x_str), float(y_str)))
    return pedestrians


def main() -> None:
    parser = argparse.ArgumentParser(description="Run D* Lite on a ROS map YAML.")
    parser.add_argument("--map", default="maps/cafe.yaml", help="Path to map YAML")
    parser.add_argument("--start", default="1.0,0.0", help="World start as x,y")
    parser.add_argument("--goal", default="2.0,-4.0", help="World goal as x,y")
    parser.add_argument(
        "--algorithm",
        choices=("astar", "dstar", "both"),
        default="both",
        help="Planner to run. 'both' compares A* with D* Lite.",
    )
    parser.add_argument(
        "--block",
        default="",
        help="Optional grid cells to mark occupied before replanning: x,y;x,y",
    )
    parser.add_argument(
        "--pedestrians",
        default="default",
        help="Pedestrian world positions as x,y;x,y, 'default', or 'none'",
    )
    parser.add_argument(
        "--ped-radius",
        type=float,
        default=0.35,
        help="Radius around each pedestrian treated as occupied in the grid",
    )
    parser.add_argument("--print-path", action="store_true", help="Print world path points")
    args = parser.parse_args()

    grid_map = load_map(args.map)
    start_world = _parse_world_pair(args.start)
    goal_world = _parse_world_pair(args.goal)
    start = grid_map.world_to_grid(*start_world)
    goal = grid_map.world_to_grid(*goal_world)
    blocked = _parse_grid_cells(args.block)
    pedestrians = _parse_pedestrians(args.pedestrians)
    ped_cells = pedestrian_cells(grid_map, pedestrians, args.ped_radius)

    print(f"Start grid: {start}  Goal grid: {goal}")
    print(f"Pedestrians: {len(pedestrians)}  pedestrian obstacle cells: {len(ped_cells)}")

    if args.algorithm in ("astar", "both"):
        astar_map = GridMap(grid_map.occupied.copy(), grid_map.resolution,
                            grid_map.origin_x, grid_map.origin_y)
        for cell in list(blocked) + list(ped_cells):
            if astar_map.in_bounds(cell):
                astar_map.occupied[cell[1], cell[0]] = True
        start_time = time.perf_counter()
        astar_path, astar_expansions = astar_search(astar_map, start, goal)
        astar_time = time.perf_counter() - start_time
        astar_metrics = path_pedestrian_metrics(astar_map, astar_path, pedestrians)
        print("A* result:")
        print(f"  expansions: {astar_expansions}")
        print(f"  path cells: {len(astar_path)}")
        print(f"  path length: {path_length_cells(astar_path) * astar_map.resolution:.3f} m")
        print(f"  runtime: {astar_time:.6f} s")
        print(
            "  pedestrian metrics: "
            f"encounters={astar_metrics[0]} personal={astar_metrics[1]} "
            f"intimate={astar_metrics[2]} min_dist={astar_metrics[3] if astar_metrics[3] is not None else 'nan'}"
        )

    path: List[GridNode] = []
    if args.algorithm in ("dstar", "both"):
        planner = DStarLite(grid_map, start, goal)
        start_time = time.perf_counter()
        steps = planner.compute_shortest_path()
        path = planner.extract_path()
        dstar_time = time.perf_counter() - start_time
        print("D* Lite result:")
        print(f"  initial search steps: {steps}")
        print(f"  initial path cells: {len(path)}")
        print(f"  initial path length: {path_length_cells(path) * grid_map.resolution:.3f} m")
        print(f"  runtime: {dstar_time:.6f} s")

        dynamic_cells = list(blocked) + list(ped_cells)
        if dynamic_cells:
            planner.apply_obstacle_updates(dynamic_cells, occupied=True)
            start_time = time.perf_counter()
            steps = planner.compute_shortest_path()
            path = planner.extract_path()
            replan_time = time.perf_counter() - start_time
            print(f"  changed cells: {len(dynamic_cells)}")
            print(f"  incremental replan steps: {steps}")
            print(f"  replanned path cells: {len(path)}")
            print(f"  replanned path length: {path_length_cells(path) * grid_map.resolution:.3f} m")
            print(f"  replan runtime: {replan_time:.6f} s")
        dstar_metrics = path_pedestrian_metrics(grid_map, path, pedestrians)
        print(
            "  pedestrian metrics: "
            f"encounters={dstar_metrics[0]} personal={dstar_metrics[1]} "
            f"intimate={dstar_metrics[2]} min_dist={dstar_metrics[3] if dstar_metrics[3] is not None else 'nan'}"
        )

    if args.print_path:
        for wx, wy in (grid_map.grid_to_world(p) for p in path):
            print(f"{wx:.3f},{wy:.3f}")


if __name__ == "__main__":
    main()
