import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path


def main():
    csv_path = Path(__file__).parent / "experiment_results_final.csv"
    out_path = Path(__file__).parent / "success_rate_bar.png"

    df = pd.read_csv(csv_path)

    if 'mode' not in df.columns or 'result' not in df.columns:
        raise SystemExit("CSV must contain 'mode' and 'result' columns")

    # remove baseline as requested
    df = df[df['mode'].astype(str).str.lower() != 'baseline']

    success = df['result'].astype(str).str.lower() == 'succeeded'
    rates = df.assign(success=success).groupby('mode')['success'].mean() * 100
    rates = rates.sort_values(ascending=False)

    plt.figure(figsize=(9, 5))

    # shorten/wrap long mode names by replacing '_' with newline
    labels = [str(s).replace('_', '\n') for s in rates.index]
    bars = plt.bar(labels, rates.values, color='C0')
    plt.ylabel('Success Rate (%)')
    plt.ylim(0, 100)
    plt.title('Success Rate by Mode')
    plt.grid(axis='y', linestyle='--', alpha=0.5)

    for bar, val in zip(bars, rates.values):
        plt.text(bar.get_x() + bar.get_width() / 2, val + 1, f"{val:.1f}%", ha='center', va='bottom')

    plt.tight_layout()
    out_path = Path(__file__).parent / "success_rate_bar_nobaseline.png"
    plt.savefig(out_path, dpi=200)
    print(f"Saved success rate bar chart to: {out_path}")


if __name__ == '__main__':
    main()
