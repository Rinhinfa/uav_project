#!/usr/bin/env python3
import argparse
import csv
import glob
import re
import statistics
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple


SUMMARY_FIELDS = [
    "runs",
    "coverage_mean",
    "coverage_std",
    "distance_mean",
    "distance_std",
    "event_ms_mean",
    "event_ms_std",
]

PLOT_METRICS = [
    ("coverage_mean", "Coverage Ratio"),
    ("distance_mean", "Path Length (m)"),
    ("event_ms_mean", "Event Response (ms)"),
]


def load_last_row(csv_path: Path) -> Dict[str, str]:
    if not csv_path.exists():
        return {}
    with csv_path.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return rows[-1] if rows else {}


def parse_modes(raw: str) -> List[str]:
    return [m.strip() for m in raw.split(",") if m.strip()]


def discover_modes(input_dir: Path) -> List[str]:
    modes: List[str] = []
    for p in sorted(input_dir.glob("*_run*.csv")):
        match = re.match(r"(.+)_run\d+\.csv$", p.name)
        if not match:
            continue
        mode = match.group(1)
        if mode not in modes:
            modes.append(mode)
    return modes


def collect_mode_rows(input_dir: Path, mode: str) -> List[Dict[str, str]]:
    rows: List[Dict[str, str]] = []
    for p in sorted(input_dir.glob(f"{mode}_run*.csv")):
        row = load_last_row(p)
        if row:
            row["_file"] = str(p)
            rows.append(row)
    return rows


def to_float(row: Dict[str, Any], key: str) -> float:
    try:
        return float(row.get(key, "0") or 0.0)
    except (TypeError, ValueError):
        return 0.0


def distance_value(row: Dict[str, Any]) -> float:
    if "distance_total" in row:
        return to_float(row, "distance_total")
    return to_float(row, "distance_uav1")


def summarize(rows: List[Dict[str, Any]]) -> Dict[str, float]:
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
    dist = [distance_value(r) for r in rows]
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


def _ordered_unique(values: Sequence[str]) -> List[str]:
    result: List[str] = []
    for value in values:
        if value and value not in result:
            result.append(value)
    return result


def build_summaries(
    grouped_rows: Dict[Tuple[str, str], List[Dict[str, Any]]],
    scenario_order: Sequence[str],
    mode_order: Sequence[str],
) -> List[Dict[str, Any]]:
    summaries: List[Dict[str, Any]] = []
    for scenario in scenario_order:
        for mode in mode_order:
            rows = grouped_rows.get((scenario, mode), [])
            if not rows:
                continue
            stats = summarize(rows)
            summaries.append({"scenario": scenario, "mode": mode, **stats})
    return summaries


def write_summary_csv(out_path: Path, summaries: List[Dict[str, Any]], include_scenario: bool) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = (["scenario"] if include_scenario else []) + ["mode", *SUMMARY_FIELDS]
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in summaries:
            out: Dict[str, Any] = {"mode": row["mode"]}
            if include_scenario:
                out["scenario"] = row["scenario"]
            out["runs"] = int(row.get("runs", 0))
            for field in SUMMARY_FIELDS:
                if field == "runs":
                    continue
                out[field] = round(float(row.get(field, 0.0)), 6)
            writer.writerow(out)


def maybe_plot(
    out_dir: Path,
    summaries: List[Dict[str, Any]],
    scenario_order: Sequence[str],
    mode_order: Sequence[str],
) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        print("matplotlib not available, skip PNG plotting.")
        return

    if not summaries:
        print("no summary rows available, skip PNG plotting.")
        return

    lookup = {(str(r["scenario"]), str(r["mode"])): r for r in summaries}
    scenarios = [s for s in scenario_order if any((s, m) in lookup for m in mode_order)]
    modes = [m for m in mode_order if any((s, m) in lookup for s in scenarios)]
    if not scenarios or not modes:
        print("no plottable scenario/mode combinations, skip PNG plotting.")
        return

    for key, label in PLOT_METRICS:
        if len(scenarios) == 1:
            scenario = scenarios[0]
            values = [float(lookup[(scenario, mode)].get(key, 0.0)) for mode in modes if (scenario, mode) in lookup]
            labels = [mode for mode in modes if (scenario, mode) in lookup]
            fig, ax = plt.subplots(figsize=(max(5.0, len(labels) * 1.2), 3.2))
            ax.bar(labels, values)
        else:
            fig, ax = plt.subplots(figsize=(max(6.0, len(scenarios) * 1.6), 3.6))
            group_width = 0.78
            bar_width = group_width / max(1, len(modes))
            x_values = list(range(len(scenarios)))
            for idx, mode in enumerate(modes):
                offsets = [
                    x - group_width / 2.0 + bar_width * (idx + 0.5)
                    for x in x_values
                ]
                values = [float(lookup.get((scenario, mode), {}).get(key, 0.0)) for scenario in scenarios]
                ax.bar(offsets, values, width=bar_width * 0.92, label=mode)
            ax.set_xticks(x_values)
            ax.set_xticklabels(scenarios, rotation=15, ha="right")
            ax.legend()
        ax.set_title(label)
        ax.set_ylabel(label)
        ax.grid(axis="y", alpha=0.25)
        if key == "coverage_mean":
            ax.set_ylim(0.0, max(1.0, ax.get_ylim()[1]))
        fig.tight_layout()
        png = out_dir / f"{key}.png"
        fig.savefig(png, dpi=150)
        plt.close(fig)
        print(f"saved plot: {png}")


def collect_input_summaries(input_dir: Path, scenario_name: str, modes: List[str]) -> Tuple[List[Dict[str, Any]], List[str], List[str]]:
    mode_order = modes or discover_modes(input_dir)
    grouped: Dict[Tuple[str, str], List[Dict[str, Any]]] = {}
    for mode in mode_order:
        rows = collect_mode_rows(input_dir, mode)
        if rows:
            grouped[(scenario_name, mode)] = rows
    summaries = build_summaries(grouped, [scenario_name], mode_order)
    return summaries, [scenario_name], mode_order


def _return_code_ok(row: Dict[str, str]) -> bool:
    raw = str(row.get("return_code", "0")).strip()
    if raw == "":
        return True
    try:
        return int(float(raw)) == 0
    except ValueError:
        return False


def _metrics_written(row: Dict[str, str]) -> bool:
    raw = str(row.get("metrics_written", "1")).strip()
    if raw == "":
        return True
    try:
        return int(float(raw)) != 0
    except ValueError:
        return True


def collect_manifest_summaries(patterns: Sequence[str], modes: List[str]) -> Tuple[List[Dict[str, Any]], List[str], List[str]]:
    manifest_paths = sorted({Path(p) for pattern in patterns for p in glob.glob(pattern, recursive=True)})
    grouped: Dict[Tuple[str, str], List[Dict[str, Any]]] = {}
    scenario_order: List[str] = []
    detected_modes: List[str] = []

    for manifest_path in manifest_paths:
        with manifest_path.open("r", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                if not _return_code_ok(row) or not _metrics_written(row):
                    continue
                scenario = str(row.get("scenario", "") or manifest_path.parent.name)
                mode = str(row.get("mode", "") or "unknown")
                if modes and mode not in modes:
                    continue
                grouped.setdefault((scenario, mode), []).append(row)
                scenario_order.append(scenario)
                detected_modes.append(mode)

    scenario_order = _ordered_unique(scenario_order)
    mode_order = modes or _ordered_unique(detected_modes)
    summaries = build_summaries(grouped, scenario_order, mode_order)
    return summaries, scenario_order, mode_order


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate experiment summary CSV/plots.")
    parser.add_argument("--input-dir", help="Directory containing <mode>_run*.csv files")
    parser.add_argument("--manifest-glob", action="append", default=[], help="Glob for run_manifest.csv files. Can be repeated.")
    parser.add_argument("--output-dir", default="/tmp/orchard_report", help="Directory for summary outputs")
    parser.add_argument("--modes", default="", help="Comma separated modes to include. Defaults to detected modes.")
    parser.add_argument("--scenario-name", default="current", help="Scenario name used for --input-dir reports.")
    args = parser.parse_args()

    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    modes = parse_modes(args.modes)

    if args.manifest_glob:
        summaries, scenario_order, mode_order = collect_manifest_summaries(args.manifest_glob, modes)
        summary_csv = output_dir / "paper_summary_all.csv"
        write_summary_csv(summary_csv, summaries, include_scenario=True)
    elif args.input_dir:
        input_dir = Path(args.input_dir).resolve()
        summaries, scenario_order, mode_order = collect_input_summaries(input_dir, args.scenario_name, modes)
        summary_csv = output_dir / "summary.csv"
        write_summary_csv(summary_csv, summaries, include_scenario=False)
    else:
        parser.error("one of --input-dir or --manifest-glob is required")

    maybe_plot(output_dir, summaries, scenario_order, mode_order)

    print(f"summary csv: {summary_csv}")
    for row in summaries:
        print(row)


if __name__ == "__main__":
    main()
