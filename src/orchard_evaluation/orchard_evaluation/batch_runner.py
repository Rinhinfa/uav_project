#!/usr/bin/env python3
import argparse
import csv
import json
import copy
import re
import statistics
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List

import yaml


def _to_int(data: Dict[str, Any], key: str, default: int) -> int:
    value = data.get(key, default)
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _to_float(data: Dict[str, Any], key: str, default: float) -> float:
    value = data.get(key, default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _sanitize_name(name: str) -> str:
    cleaned = re.sub(r"[^0-9A-Za-z_.-]+", "_", name)
    cleaned = cleaned.strip("._-")
    return cleaned or "scenario"


def _parse_modes(raw: str) -> List[str]:
    modes = [m.strip() for m in raw.split(",") if m.strip()]
    return modes or ["baseline", "proposed"]


def load_scenario(scenario_path: Path) -> Dict[str, Any]:
    with scenario_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    rows = max(1, _to_int(data, "rows", 8))
    trees_per_row = max(1, _to_int(data, "trees_per_row", 24))
    row_spacing = max(0.1, _to_float(data, "row_spacing", 5.0))
    tree_spacing = max(0.1, _to_float(data, "tree_spacing", 3.0))

    auto_width = max(tree_spacing, (trees_per_row - 1) * tree_spacing)
    auto_height = max(row_spacing, (rows - 1) * row_spacing)

    scenario_name = str(data.get("scenario_name", scenario_path.stem))
    return {
        "scenario_name": scenario_name,
        "scenario_slug": _sanitize_name(scenario_name),
        "uav_count": max(1, _to_int(data, "uav_count", 3)),
        "rows": rows,
        "trees_per_row": trees_per_row,
        "row_spacing": row_spacing,
        "tree_spacing": tree_spacing,
        "tree_radius": max(0.05, _to_float(data, "tree_radius", 0.35)),
        "orchard_width": max(1.0, _to_float(data, "orchard_width", auto_width)),
        "orchard_height": max(1.0, _to_float(data, "orchard_height", auto_height)),
        "grid_size": max(0.1, _to_float(data, "grid_size", 2.0)),
        "uav_speed_mps": max(0.1, _to_float(data, "uav_speed_mps", 1.2)),
        "max_flight_time_sec": max(10.0, _to_float(data, "max_flight_time_sec", 1800.0)),
        "usable_flight_ratio": min(1.0, max(0.05, _to_float(data, "usable_flight_ratio", 0.9))),
        "events": data.get("events", []) or [],
    }


def build_run_command(
    mode: str,
    launch_file: str,
    run_duration_sec: int,
    csv_path: Path,
    event_profile_file: Path,
    scenario: Dict[str, Any],
    random_seed: int,
) -> List[str]:
    return [
        "ros2",
        "launch",
        "orchard_bringup",
        launch_file,
        f"pipeline_mode:={mode}",
        f"run_duration_sec:={run_duration_sec}",
        f"metrics_csv:={str(csv_path)}",
        f"event_profile_file:={str(event_profile_file)}",
        f"uav_count:={scenario['uav_count']}",
        f"rows:={scenario['rows']}",
        f"trees_per_row:={scenario['trees_per_row']}",
        f"row_spacing:={scenario['row_spacing']}",
        f"tree_spacing:={scenario['tree_spacing']}",
        f"tree_radius:={scenario['tree_radius']}",
        f"orchard_width:={scenario['orchard_width']}",
        f"orchard_height:={scenario['orchard_height']}",
        f"grid_size:={scenario['grid_size']}",
        f"uav_speed_mps:={scenario['uav_speed_mps']}",
        f"max_flight_time_sec:={scenario['max_flight_time_sec']}",
        f"usable_flight_ratio:={scenario['usable_flight_ratio']}",
        f"random_seed:={random_seed}",
    ]


def run_once(
    scenario_name: str,
    mode: str,
    repeat_idx: int,
    cmd: List[str],
    timeout_sec: int,
    retries: int,
    retry_wait_sec: float,
    log_path: Path,
) -> int:
    rcode = 1
    for attempt in range(retries + 1):
        print(f"[run {repeat_idx}] scenario={scenario_name} mode={mode} attempt={attempt + 1}/{retries + 1}")
        print(" ".join(cmd))
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("a", encoding="utf-8") as logf:
            logf.write(f"\n=== run {repeat_idx} mode={mode} attempt={attempt + 1} ===\n")
            logf.write(" ".join(cmd) + "\n")
            try:
                proc = subprocess.run(
                    cmd,
                    check=False,
                    timeout=timeout_sec if timeout_sec > 0 else None,
                    stdout=logf,
                    stderr=subprocess.STDOUT,
                )
                rcode = proc.returncode
            except subprocess.TimeoutExpired:
                rcode = 124
                logf.write(f"timeout after {timeout_sec}s\n")

        if rcode == 0:
            return 0
        if attempt < retries:
            print(f"  -> non-zero return ({rcode}), retry after {retry_wait_sec}s")
            time.sleep(max(0.0, retry_wait_sec))
    return rcode


def read_last_metrics(csv_path: Path) -> dict:
    if not csv_path.exists():
        return {}
    with csv_path.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return rows[-1] if rows else {}


def adapt_events_for_duration(events: list, run_duration_sec: int) -> list:
    if not events:
        return []
    adapted = copy.deepcopy(events)
    max_t = max(float(e.get("t", 0.0)) for e in adapted)
    if max_t <= run_duration_sec * 0.9:
        return adapted
    # Compress timestamps into [10%, 80%] of run duration for short smoke runs.
    scale = (run_duration_sec * 0.7) / max(max_t, 1.0)
    for e in adapted:
        t_old = float(e.get("t", 0.0))
        e["t"] = round(run_duration_sec * 0.1 + t_old * scale, 2)
    return adapted


def to_float(row: Dict[str, Any], key: str) -> float:
    try:
        return float(row.get(key, 0.0))
    except (TypeError, ValueError):
        return 0.0


def task_completion_value(row: Dict[str, Any]) -> float:
    if "task_completion_ratio" in row:
        return to_float(row, "task_completion_ratio")
    return to_float(row, "coverage_ratio")


def summarize_final_rows(rows: List[Dict[str, Any]]) -> Dict[str, float]:
    valid = [r for r in rows if r]
    completions = [task_completion_value(r) for r in valid]
    coverages = [to_float(r, "coverage_ratio") for r in valid]
    distances = [to_float(r, "distance_total") if "distance_total" in r else to_float(r, "distance_uav1") for r in valid]
    responses = [to_float(r, "event_response_ms") for r in valid]
    return {
        "valid_runs": float(len(valid)),
        "task_completion_mean": statistics.mean(completions) if completions else 0.0,
        "task_completion_std": statistics.pstdev(completions) if len(completions) > 1 else 0.0,
        "coverage_mean": statistics.mean(coverages) if coverages else 0.0,
        "coverage_std": statistics.pstdev(coverages) if len(coverages) > 1 else 0.0,
        "distance_mean": statistics.mean(distances) if distances else 0.0,
        "distance_std": statistics.pstdev(distances) if len(distances) > 1 else 0.0,
        "event_ms_mean": statistics.mean(responses) if responses else 0.0,
        "event_ms_std": statistics.pstdev(responses) if len(responses) > 1 else 0.0,
    }


def write_run_manifest(out_csv: Path, rows: List[Dict[str, Any]]) -> None:
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "scenario",
                "mode",
                "repeat",
                "random_seed",
                "return_code",
                "skipped",
                "metrics_written",
                "csv_path",
                "log_path",
                "task_completion_ratio",
                "coverage_ratio",
                "distance_total",
                "event_response_ms",
            ],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run baseline/proposed batch experiments.")
    parser.add_argument("--scenario", action="append", required=True, help="Scenario yaml path. Can be repeated.")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--run-duration-sec", type=int, default=60)
    parser.add_argument("--modes", default="baseline,proposed", help="Comma separated modes. Default: baseline,proposed")
    parser.add_argument("--launch-file", default="sim_bringup.launch.py", help="Launch file in orchard_bringup package")
    parser.add_argument("--timeout-sec", type=int, default=0, help="Per-run timeout seconds. 0 disables timeout")
    parser.add_argument("--retry", type=int, default=1, help="Retry count when one run fails")
    parser.add_argument("--retry-wait-sec", type=float, default=2.0)
    parser.add_argument("--resume", action="store_true", help="Skip runs when target CSV already has data")
    parser.add_argument("--seed-base", type=int, default=-1, help=">=0 enables reproducible per-repeat seeds")
    parser.add_argument("--force-scenario-subdir", action="store_true", help="Always create out_dir/<scenario_name>/")
    parser.add_argument("--out-dir", default="/tmp/orchard_batch")
    args = parser.parse_args()

    modes = _parse_modes(args.modes)
    scenario_paths = [Path(s).resolve() for s in args.scenario]
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    all_summaries: Dict[str, Any] = {}
    for scenario_path in scenario_paths:
        scenario = load_scenario(scenario_path)
        scenario_name = scenario["scenario_name"]
        print(f"Loaded scenario: {scenario_name}")

        use_subdir = args.force_scenario_subdir or len(scenario_paths) > 1
        scenario_out_dir = out_dir / scenario["scenario_slug"] if use_subdir else out_dir
        scenario_out_dir.mkdir(parents=True, exist_ok=True)

        event_profile = adapt_events_for_duration(scenario.get("events", []), args.run_duration_sec)
        event_profile_file = scenario_out_dir / "event_profile.json"
        with event_profile_file.open("w", encoding="utf-8") as f:
            json.dump(event_profile, f, ensure_ascii=False, indent=2)

        summary: Dict[str, Any] = {}
        run_manifest_rows: List[Dict[str, Any]] = []

        for mode in modes:
            rcodes: List[int] = []
            finals: List[Dict[str, Any]] = []
            for i in range(args.repeats):
                repeat_idx = i + 1
                csv_path = scenario_out_dir / f"{mode}_run{repeat_idx}.csv"
                log_path = scenario_out_dir / "logs" / f"{mode}_run{repeat_idx}.log"
                repeat_seed = args.seed_base + i if args.seed_base >= 0 else -1

                existing_row = read_last_metrics(csv_path)
                skipped = bool(args.resume and existing_row)
                if skipped:
                    rcode = 0
                    final_row = existing_row
                    print(f"[run {repeat_idx}] scenario={scenario_name} mode={mode} skipped (existing CSV)")
                else:
                    cmd = build_run_command(
                        mode=mode,
                        launch_file=args.launch_file,
                        run_duration_sec=args.run_duration_sec,
                        csv_path=csv_path,
                        event_profile_file=event_profile_file,
                        scenario=scenario,
                        random_seed=repeat_seed,
                    )
                    rcode = run_once(
                        scenario_name=scenario_name,
                        mode=mode,
                        repeat_idx=repeat_idx,
                        cmd=cmd,
                        timeout_sec=args.timeout_sec,
                        retries=max(0, args.retry),
                        retry_wait_sec=max(0.0, args.retry_wait_sec),
                        log_path=log_path,
                    )
                    final_row = read_last_metrics(csv_path)

                rcodes.append(rcode)
                finals.append(final_row)
                metrics_written = bool(final_row)
                run_manifest_rows.append(
                    {
                        "scenario": scenario_name,
                        "mode": mode,
                        "repeat": repeat_idx,
                        "random_seed": repeat_seed,
                        "return_code": rcode,
                        "skipped": int(skipped),
                        "metrics_written": int(metrics_written),
                        "csv_path": str(csv_path),
                        "log_path": str(log_path),
                        "task_completion_ratio": task_completion_value(final_row),
                        "coverage_ratio": to_float(final_row, "coverage_ratio"),
                        "distance_total": to_float(final_row, "distance_total")
                        if "distance_total" in final_row
                        else to_float(final_row, "distance_uav1"),
                        "event_response_ms": to_float(final_row, "event_response_ms"),
                    }
                )
            summary[mode] = {
                "return_codes": rcodes,
                "final_rows": finals,
                "stats": summarize_final_rows(finals),
            }

        write_run_manifest(scenario_out_dir / "run_manifest.csv", run_manifest_rows)
        with (scenario_out_dir / "batch_summary.json").open("w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)

        print("=== Batch Summary ===")
        for mode, data in summary.items():
            stats = data["stats"]
            ok_cnt = sum(1 for c in data["return_codes"] if c == 0)
            print(
                f"[{mode}] ok={ok_cnt}/{len(data['return_codes'])} "
                f"mean_task_completion={stats['task_completion_mean']:.3f}±{stats['task_completion_std']:.3f} "
                f"mean_coverage={stats['coverage_mean']:.3f}±{stats['coverage_std']:.3f} "
                f"mean_distance={stats['distance_mean']:.3f}±{stats['distance_std']:.3f} "
                f"mean_event_ms={stats['event_ms_mean']:.2f}±{stats['event_ms_std']:.2f}"
            )
        all_summaries[scenario_name] = {
            "out_dir": str(scenario_out_dir),
            "summary": summary,
        }

    with (out_dir / "batch_summary_all.json").open("w", encoding="utf-8") as f:
        json.dump(all_summaries, f, ensure_ascii=False, indent=2)

    failed_runs = 0
    missing_metrics = 0
    for scenario_data in all_summaries.values():
        for mode_data in scenario_data["summary"].values():
            failed_runs += sum(1 for c in mode_data["return_codes"] if c != 0)
            missing_metrics += sum(1 for row in mode_data["final_rows"] if not row)

    if failed_runs or missing_metrics:
        print(f"batch finished with failed_runs={failed_runs}, missing_metrics={missing_metrics}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
