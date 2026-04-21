#!/usr/bin/env python3
import csv
import json
import math
from pathlib import Path
from typing import Dict, Optional, Set, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String, UInt64


class MetricsAggregator(Node):
    def __init__(self) -> None:
        super().__init__("metrics_aggregator")
        self.declare_parameter("output_csv", "/tmp/orchard_metrics.csv")
        self.declare_parameter("orchard_width", 60.0)
        self.declare_parameter("orchard_height", 40.0)
        self.declare_parameter("grid_size", 2.0)
        self.declare_parameter("pipeline_mode", "proposed")
        self.declare_parameter("uav_count", 3)
        self.declare_parameter("task_rows", 8)
        self.declare_parameter("tasks_per_row", 24)
        self.declare_parameter("task_row_spacing", 5.0)
        self.declare_parameter("task_col_spacing", 3.0)
        self.declare_parameter("task_z", 2.0)
        self.declare_parameter("task_completion_radius_m", 0.8)

        self._last_pos: Dict[str, Tuple[float, float, float]] = {}
        self._distance: Dict[str, float] = {}
        self._covered_cells: Set[Tuple[int, int]] = set()
        self._task_locations: Dict[int, Tuple[float, float, float]] = {}
        self._completed_task_ids: Set[int] = set()
        self._mission_id = 0
        self._mode = "GLOBAL_TRACK"
        self._pipeline_mode = str(self.get_parameter("pipeline_mode").value)
        self._uav_count = int(self.get_parameter("uav_count").value)
        self._event_latency_ms = 0.0
        self._start_sec = self.get_clock().now().nanoseconds / 1e9
        self._completion_radius = float(self.get_parameter("task_completion_radius_m").value)

        self._seed_static_tasks()

        self._uav_ids = [f"uav_{i+1}" for i in range(max(1, self._uav_count))]
        for uid in self._uav_ids:
            self.create_subscription(
                Odometry,
                f"/{uid}/odom",
                lambda msg, uid=uid: self._on_odom(uid, msg),
                10,
            )
        self.create_subscription(UInt64, "/allocation/mission_id", self._on_allocation, 10)
        self.create_subscription(String, "/planner/mode", self._on_mode, 10)
        self.create_subscription(String, "/allocation/result_json", self._on_allocation_result, 10)
        self.create_subscription(String, "/scheduler/event_in", self._on_event_in, 10)
        self.create_subscription(String, "/scheduler/events_log", self._on_event_log, 10)
        self.create_timer(5.0, self._dump_metrics)

    def _seed_static_tasks(self) -> None:
        rows = max(1, int(self.get_parameter("task_rows").value))
        cols = max(1, int(self.get_parameter("tasks_per_row").value))
        row_spacing = float(self.get_parameter("task_row_spacing").value)
        col_spacing = float(self.get_parameter("task_col_spacing").value)
        task_z = float(self.get_parameter("task_z").value)
        y_offset = -0.5 * (rows - 1) * row_spacing
        x_offset = -0.5 * (cols - 1) * col_spacing
        half_row = 0.5 * row_spacing
        tid = 1
        for row in range(rows):
            y_tree = y_offset + row * row_spacing
            if rows <= 1:
                y_task = y_tree
            elif row < rows - 1:
                y_task = y_tree + half_row
            else:
                y_task = y_tree - half_row
            for col in range(cols):
                self._register_task(
                    tid,
                    x_offset + col * col_spacing,
                    y_task,
                    task_z,
                )
                tid += 1

    def _register_task(self, task_id: int, x: float, y: float, z: float) -> None:
        self._task_locations[int(task_id)] = (float(x), float(y), float(z))

    def _mark_completed_tasks(self, x: float, y: float) -> None:
        radius_sq = self._completion_radius * self._completion_radius
        for task_id, (tx, ty, _tz) in self._task_locations.items():
            if task_id in self._completed_task_ids:
                continue
            dx = x - tx
            dy = y - ty
            if dx * dx + dy * dy <= radius_sq:
                self._completed_task_ids.add(task_id)

    def _on_odom(self, uid: str, msg: Odometry) -> None:
        p = msg.pose.pose.position
        if uid in self._last_pos:
            lp = self._last_pos[uid]
            self._distance[uid] = self._distance.get(uid, 0.0) + math.dist((lp[0], lp[1], lp[2]), (p.x, p.y, p.z))
        self._last_pos[uid] = (p.x, p.y, p.z)
        self._mark_completed_tasks(p.x, p.y)
        cell = self._cell_id(p.x, p.y)
        if cell is not None:
            self._covered_cells.add(cell)

    def _on_allocation(self, msg: UInt64) -> None:
        self._mission_id = int(msg.data)

    def _on_mode(self, msg: String) -> None:
        self._mode = msg.data

    def _on_allocation_result(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        for alloc in payload.get("allocations", []):
            task_ids = alloc.get("task_ids", [])
            waypoints = alloc.get("coarse_waypoints", [])
            for idx, task_id in enumerate(task_ids):
                if idx >= len(waypoints):
                    break
                pt = waypoints[idx]
                try:
                    self._register_task(
                        int(task_id),
                        float(pt.get("x", 0.0)),
                        float(pt.get("y", 0.0)),
                        float(pt.get("z", self.get_parameter("task_z").value)),
                    )
                except (TypeError, ValueError):
                    continue

    def _on_event_in(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if str(payload.get("event_type", "")).upper() != "NEW_TASK":
            return
        task = payload.get("task", {})
        location = task.get("location", {})
        try:
            self._register_task(
                int(task["id"]),
                float(location.get("x", 0.0)),
                float(location.get("y", 0.0)),
                float(location.get("z", self.get_parameter("task_z").value)),
            )
        except (KeyError, TypeError, ValueError):
            return

    def _on_event_log(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._event_latency_ms = float(payload.get("response_latency_ms", self._event_latency_ms))

    def _cell_id(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        g = float(self.get_parameter("grid_size").value)
        w = float(self.get_parameter("orchard_width").value)
        h = float(self.get_parameter("orchard_height").value)
        cols = max(1, int(w / g))
        rows = max(1, int(h / g))
        cx = math.floor((x + w / 2.0) / g)
        cy = math.floor((y + h / 2.0) / g)
        if not (0 <= cx < cols and 0 <= cy < rows):
            return None
        return (cx, cy)

    def _coverage_ratio(self) -> float:
        w = float(self.get_parameter("orchard_width").value)
        h = float(self.get_parameter("orchard_height").value)
        g = float(self.get_parameter("grid_size").value)
        total = max(1, int(w / g) * int(h / g))
        return min(1.0, len(self._covered_cells) / total)

    def _task_completion_ratio(self) -> float:
        total_tasks = len(self._task_locations)
        if total_tasks <= 0:
            return 0.0
        return min(1.0, len(self._completed_task_ids) / total_tasks)

    def _dump_metrics(self) -> None:
        elapsed = self.get_clock().now().nanoseconds / 1e9 - self._start_sec
        distance_uav1 = self._distance.get("uav_1", 0.0)
        distance_total = sum(self._distance.get(uid, 0.0) for uid in self._uav_ids)
        active_uavs = sum(1 for uid in self._uav_ids if uid in self._last_pos)
        output = Path(str(self.get_parameter("output_csv").value))
        output.parent.mkdir(parents=True, exist_ok=True)
        exists = output.exists()
        with output.open("a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            if not exists:
                writer.writerow(
                    [
                        "elapsed_sec",
                        "pipeline_mode",
                        "mission_id",
                        "distance_uav1",
                        "distance_total",
                        "active_uavs",
                        "task_completion_ratio",
                        "coverage_ratio",
                        "planner_mode",
                        "event_response_ms",
                    ]
                )
            writer.writerow(
                [
                    round(elapsed, 3),
                    self._pipeline_mode,
                    self._mission_id,
                    round(distance_uav1, 3),
                    round(distance_total, 3),
                    active_uavs,
                    round(self._task_completion_ratio(), 4),
                    round(self._coverage_ratio(), 4),
                    self._mode,
                    round(self._event_latency_ms, 2),
                ]
            )


def main() -> None:
    rclpy.init()
    node = MetricsAggregator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
