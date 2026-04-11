#!/usr/bin/env python3
import argparse
import csv
import statistics
from pathlib import Path
from typing import Dict, List


def load_last_row(csv_path: Path) -> Dict[str, str]:
    if not csv_path.exists():
        return {}
    with csv_path.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return rows[-1] if rows else {}


def collect_mode_rows(input_dir: Path, mode: str) -> List[Dict[str, str]]:
    rows: List[Dict[str, str]] = []
    for p in sorted(input_dir.glob(f"{mode}_run*.csv")):
        row = load_last_row(p)
        if row:
            row["_file"] = str(p)
            rows.append(row)
    return rows


def to_float(row: Dict[str, str], key: str) -> float:
    try:
        return float(row.get(key, "0") or 0.0)
    except ValueError:
        return 0.0


def summarize(rows: List[Dict[str, str]]) -> Dict[str, float]:
    if not rows:
        return {
            "runs": 0.0,
            "coverage_mean": 0.0,
            "coverage_std": 0.0,
            "distance_mean": 0.0,
            "distance_std": 0.0,
            "event_ms_mean": 0.0,
            "event_ms_std": 0.0,
        }
    cov = [to_float(r, "coverage_ratio") for r in rows]
    dist = [to_float(r, "distance_uav1") for r in rows]
    evt = [to_float(r, "event_response_ms") for r in rows]
    std = lambda xs: statistics.pstdev(xs) if len(xs) > 1 else 0.0
    return {
        "runs": float(len(rows)),
        "coverage_mean": statistics.mean(cov),
        "coverage_std": std(cov),
        "distance_mean": statistics.mean(dist),
        "distance_std": std(dist),
        "event_ms_mean": statistics.mean(evt),
        "event_ms_std": std(evt),
    }


def write_summary_csv(out_path: Path, baseline: Dict[str, float], proposed: Dict[str, float]) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "mode",
                "runs",
                "coverage_mean",
                "coverage_std",
                "distance_mean",
                "distance_std",
                "event_ms_mean",
                "event_ms_std",
            ]
        )
        writer.writerow(["baseline", *[round(v, 6) for v in baseline.values()]])
        writer.writerow(["proposed", *[round(v, 6) for v in proposed.values()]])


def maybe_plot(out_dir: Path, baseline: Dict[str, float], proposed: Dict[str, float]) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        print("matplotlib not available, skip PNG plotting.")
        return

    metrics = [
        ("coverage_mean", "Coverage Ratio"),
        ("distance_mean", "Path Length (m)"),
        ("event_ms_mean", "Event Response (ms)"),
    ]
    for key, label in metrics:
        fig, ax = plt.subplots(figsize=(5, 3))
        values = [baseline[key], proposed[key]]
        ax.bar(["baseline", "proposed"], values)
        ax.set_title(label)
        ax.set_ylabel(label)
        fig.tight_layout()
        png = out_dir / f"{key}.png"
        fig.savefig(png, dpi=150)
        plt.close(fig)
        print(f"saved plot: {png}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate experiment summary CSV/plots.")
    parser.add_argument("--input-dir", required=True, help="Directory containing baseline_run*.csv and proposed_run*.csv")
    parser.add_argument("--output-dir", default="/tmp/orchard_report", help="Directory for summary outputs")
    args = parser.parse_args()

    input_dir = Path(args.input_dir).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    baseline_rows = collect_mode_rows(input_dir, "baseline")
    proposed_rows = collect_mode_rows(input_dir, "proposed")

    baseline_summary = summarize(baseline_rows)
    proposed_summary = summarize(proposed_rows)

    summary_csv = output_dir / "summary.csv"
    write_summary_csv(summary_csv, baseline_summary, proposed_summary)
    maybe_plot(output_dir, baseline_summary, proposed_summary)

    print(f"summary csv: {summary_csv}")
    print("baseline:", baseline_summary)
    print("proposed:", proposed_summary)


if __name__ == "__main__":
    main()
