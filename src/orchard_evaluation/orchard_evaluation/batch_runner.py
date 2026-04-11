#!/usr/bin/env python3
import argparse
import csv
import json
import copy
import subprocess
from pathlib import Path

import yaml


def run_once(
    scenario: Path,
    mode: str,
    repeat_idx: int,
    run_duration_sec: int,
    csv_path: Path,
    event_profile_file: Path,
) -> int:
    cmd = [
        "ros2",
        "launch",
        "orchard_bringup",
        "sim_bringup.launch.py",
        f"pipeline_mode:={mode}",
        f"run_duration_sec:={run_duration_sec}",
        f"metrics_csv:={str(csv_path)}",
        f"event_profile_file:={str(event_profile_file)}",
    ]
    print(f"[run {repeat_idx}] scenario={scenario} mode={mode}")
    print(" ".join(cmd))
    proc = subprocess.run(cmd, check=False)
    return proc.returncode


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


def main() -> None:
    parser = argparse.ArgumentParser(description="Run baseline/proposed batch experiments.")
    parser.add_argument("--scenario", required=True, help="Scenario yaml path.")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--run-duration-sec", type=int, default=60)
    parser.add_argument("--out-dir", default="/tmp/orchard_batch")
    args = parser.parse_args()

    scenario_path = Path(args.scenario).resolve()
    with scenario_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    print(f"Loaded scenario: {data.get('scenario_name', 'unknown')}")
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    event_profile = adapt_events_for_duration(data.get("events", []), args.run_duration_sec)
    event_profile_file = out_dir / "event_profile.json"
    with event_profile_file.open("w", encoding="utf-8") as f:
        json.dump(event_profile, f, ensure_ascii=False)

    summary = {}
    for mode in ["baseline", "proposed"]:
        rcodes = []
        finals = []
        for i in range(args.repeats):
            csv_path = out_dir / f"{mode}_run{i+1}.csv"
            rcodes.append(run_once(scenario_path, mode, i + 1, args.run_duration_sec, csv_path, event_profile_file))
            finals.append(read_last_metrics(csv_path))
        summary[mode] = {"return_codes": rcodes, "final_rows": finals}

    print("=== Batch Summary ===")
    for mode, data in summary.items():
        coverages = [float(r.get("coverage_ratio", 0.0)) for r in data["final_rows"] if r]
        distances = [float(r.get("distance_uav1", 0.0)) for r in data["final_rows"] if r]
        responses = [float(r.get("event_response_ms", 0.0)) for r in data["final_rows"] if r]
        c_mean = sum(coverages) / len(coverages) if coverages else 0.0
        d_mean = sum(distances) / len(distances) if distances else 0.0
        r_mean = sum(responses) / len(responses) if responses else 0.0
        print(f"[{mode}] codes={data['return_codes']} mean_coverage={c_mean:.3f} mean_distance={d_mean:.3f} mean_event_ms={r_mean:.2f}")


if __name__ == "__main__":
    main()
