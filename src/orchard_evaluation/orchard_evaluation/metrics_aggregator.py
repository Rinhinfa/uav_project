#!/usr/bin/env python3
import csv
import json
import math
from pathlib import Path
from typing import Dict, Tuple

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

        self._last_pos: Dict[str, Tuple[float, float, float]] = {}
        self._distance: Dict[str, float] = {}
        self._covered_cells = set()
        self._mission_id = 0
        self._mode = "GLOBAL_TRACK"
        self._pipeline_mode = str(self.get_parameter("pipeline_mode").value)
        self._uav_count = int(self.get_parameter("uav_count").value)
        self._event_latency_ms = 0.0
        self._start_sec = self.get_clock().now().nanoseconds / 1e9

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
        self.create_subscription(String, "/scheduler/events_log", self._on_event_log, 10)
        self.create_timer(5.0, self._dump_metrics)

    def _on_odom(self, uid: str, msg: Odometry) -> None:
        p = msg.pose.pose.position
        if uid in self._last_pos:
            lp = self._last_pos[uid]
            self._distance[uid] = self._distance.get(uid, 0.0) + math.dist((lp[0], lp[1], lp[2]), (p.x, p.y, p.z))
        self._last_pos[uid] = (p.x, p.y, p.z)
        self._covered_cells.add(self._cell_id(p.x, p.y))

    def _on_allocation(self, msg: UInt64) -> None:
        self._mission_id = int(msg.data)

    def _on_mode(self, msg: String) -> None:
        self._mode = msg.data

    def _on_event_log(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._event_latency_ms = float(payload.get("response_latency_ms", self._event_latency_ms))

    def _cell_id(self, x: float, y: float) -> Tuple[int, int]:
        g = float(self.get_parameter("grid_size").value)
        return (int(x / g), int(y / g))

    def _coverage_ratio(self) -> float:
        w = float(self.get_parameter("orchard_width").value)
        h = float(self.get_parameter("orchard_height").value)
        g = float(self.get_parameter("grid_size").value)
        total = max(1, int(w / g) * int(h / g))
        return min(1.0, len(self._covered_cells) / total)

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
