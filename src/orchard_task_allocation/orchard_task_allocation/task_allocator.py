#!/usr/bin/env python3
import json
import math
from typing import Dict, List, Any

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String, UInt64
from std_srvs.srv import Trigger

# 持久化 QoS：晚启动的节点（如 gz_path_follower）也能收到最新一次分配结果
_TRANSIENT_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class TaskAllocator(Node):
    def __init__(self) -> None:
        super().__init__("task_allocator")
        self.declare_parameter("timer_hz", 0.5)
        self.declare_parameter("row_bias_weight", 0.35)
        self.declare_parameter("distance_weight", 0.65)
        self.declare_parameter("algorithm_mode", "proposed")
        self.declare_parameter("allocation_trigger_mode", "event_driven")
        self.declare_parameter("task_rows", 8)
        self.declare_parameter("tasks_per_row", 24)
        self.declare_parameter("task_row_spacing", 5.0)
        self.declare_parameter("task_col_spacing", 3.0)
        self.declare_parameter("task_z", 3.0)

        self._uav_states: Dict[str, Dict[str, Any]] = {}
        self._tasks: List[Dict[str, Any]] = []
        self._mission_id = 0
        self._needs_replan = True
        self._has_plan = False

        self._allocation_pub = self.create_publisher(String, "/allocation/result_json", _TRANSIENT_QOS)
        self._coarse_path_pub = self.create_publisher(Path, "/planner/coarse_waypoints", 10)
        self._mission_pub = self.create_publisher(UInt64, "/allocation/mission_id", 10)
        self.create_subscription(String, "/fleet/states_json", self._on_states, 10)
        self.create_subscription(String, "/scheduler/event_in", self._on_event, 10)
        self.create_service(Trigger, "/scheduler/reallocate_tasks", self._on_reallocate)
        self.create_timer(1.0 / float(self.get_parameter("timer_hz").value), self._periodic_allocate)
        self._seed_tasks_once()

    def _seed_tasks_once(self) -> None:
        # 与世界生成参数一致，默认覆盖每棵树对应的巡检点。
        rows = int(self.get_parameter("task_rows").value)
        cols = int(self.get_parameter("tasks_per_row").value)
        row_spacing = float(self.get_parameter("task_row_spacing").value)
        col_spacing = float(self.get_parameter("task_col_spacing").value)
        task_z = float(self.get_parameter("task_z").value)
        y_offset = -0.5 * (rows - 1) * row_spacing
        x_offset = -0.5 * (cols - 1) * col_spacing
        tid = 1
        for row in range(rows):
            y = y_offset + row * row_spacing
            for col in range(cols):
                t = {
                    "id": tid,
                    "priority": 2 if col % 6 == 0 else 1,
                    "task_type": "scan",
                    "deadline_sec": 120.0 if col % 6 == 0 else 240.0,
                    "location": {"x": x_offset + col * col_spacing, "y": y, "z": task_z},
                }
                tid += 1
                self._tasks.append(t)

    def _on_states(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._uav_states = {s["uav_id"]: s for s in payload.get("states", [])}
        if not self._has_plan and self._uav_states:
            self._needs_replan = True

    def _on_event(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        etype = str(event.get("event_type", "")).upper()
        if etype == "NEW_TASK":
            task = event.get("task", {})
            if "id" in task and "location" in task:
                self._tasks.append(
                    {
                        "id": int(task["id"]),
                        "priority": int(task.get("priority", 2)),
                        "task_type": str(task.get("task_type", "scan")),
                        "deadline_sec": float(task.get("deadline_sec", 180.0)),
                        "location": {
                            "x": float(task["location"].get("x", 0.0)),
                            "y": float(task["location"].get("y", 0.0)),
                            "z": float(task["location"].get("z", 3.0)),
                        },
                    }
                )
                self._needs_replan = True
        elif etype in {"LOW_BATTERY", "UAV_FAILURE", "COMM_LOSS"}:
            uid = str(event.get("uav_id", ""))
            if uid in self._uav_states:
                self._uav_states[uid]["healthy"] = False
                self._uav_states[uid]["battery_percent"] = min(5.0, float(self._uav_states[uid].get("battery_percent", 5.0)))
                self._needs_replan = True

    def _periodic_allocate(self) -> None:
        if not self._uav_states:
            return
        trigger_mode = str(self.get_parameter("allocation_trigger_mode").value).lower()
        if trigger_mode != "periodic" and not self._needs_replan:
            return
        result = self._compute_allocations(self._tasks, list(self._uav_states.values()), incremental_only=False)
        self._publish_allocation(result)
        self._needs_replan = False
        self._has_plan = True

    def _on_reallocate(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        result = self._compute_allocations(self._tasks, list(self._uav_states.values()), incremental_only=True)
        self._publish_allocation(result)
        self._needs_replan = False
        self._has_plan = True
        response.success = True
        response.message = f"mission_id={result['mission_id']}"
        return response

    def _publish_allocation(self, result: Dict[str, Any]) -> None:
        out = String()
        out.data = json.dumps(result, ensure_ascii=False)
        self._allocation_pub.publish(out)
        mid = UInt64()
        mid.data = int(result["mission_id"])
        self._mission_pub.publish(mid)
        self._publish_coarse_path(result)

    def _compute_allocations(self, tasks: List[Dict[str, Any]], uav_states: List[Dict[str, Any]], incremental_only: bool) -> Dict[str, Any]:
        distance_weight = float(self.get_parameter("distance_weight").value)
        row_weight = float(self.get_parameter("row_bias_weight").value)
        algorithm_mode = str(self.get_parameter("algorithm_mode").value).lower()

        pool = [t for t in tasks]
        healthy_uavs = [s for s in uav_states if s.get("healthy", False) and float(s.get("battery_percent", 0.0)) > 10.0]
        if not healthy_uavs:
            healthy_uavs = uav_states
        mapping: Dict[str, List[Dict[str, Any]]] = {s["uav_id"]: [] for s in healthy_uavs}
        row_centers = {s["uav_id"]: float(s["y"]) for s in healthy_uavs}

        sort_key = (lambda x: x["priority"]) if algorithm_mode != "baseline" else (lambda x: x["id"])
        for task in sorted(pool, key=sort_key, reverse=(algorithm_mode != "baseline")):
            best_uav = None
            best_cost = float("inf")
            for uav in healthy_uavs:
                dx = float(task["location"]["x"]) - float(uav["x"])
                dy = float(task["location"]["y"]) - float(uav["y"])
                base_dist = math.hypot(dx, dy)
                row_bias = abs(float(task["location"]["y"]) - row_centers[uav["uav_id"]])
                score = base_dist if algorithm_mode == "baseline" else distance_weight * base_dist + row_weight * row_bias
                if incremental_only:
                    score *= 1.05
                if score < best_cost:
                    best_cost = score
                    best_uav = uav["uav_id"]
            mapping[best_uav].append(task)

        self._mission_id += 1
        allocations = []
        for uav_id, assigned in mapping.items():
            assigned_sorted = sorted(assigned, key=lambda t: (round(float(t["location"]["y"]), 1), float(t["location"]["x"])))
            allocations.append(
                {
                    "uav_id": uav_id,
                    "task_ids": [int(t["id"]) for t in assigned_sorted],
                    "coarse_waypoints": [t["location"] for t in assigned_sorted],
                    "estimated_cost": self._estimate_cost(uav_id, assigned_sorted),
                }
            )
        return {"mission_id": self._mission_id, "allocations": allocations}

    def _publish_coarse_path(self, allocations: Dict[str, Any]) -> None:
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for alloc in allocations["allocations"]:
            for pt in alloc["coarse_waypoints"]:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position = Point(x=float(pt["x"]), y=float(pt["y"]), z=float(pt["z"]))
                path.poses.append(pose)
        if path.poses:
            self._coarse_path_pub.publish(path)

    def _estimate_cost(self, uav_id: str, tasks: List[Dict[str, Any]]) -> float:
        if not tasks:
            return 0.0
        state = self._uav_states.get(uav_id)
        x = float(state["x"]) if state else 0.0
        y = float(state["y"]) if state else 0.0
        distance = 0.0
        for task in tasks:
            tx = float(task["location"]["x"])
            ty = float(task["location"]["y"])
            d = math.hypot(tx - x, ty - y)
            distance += d
            x, y = tx, ty
        return float(distance)


def main() -> None:
    rclpy.init()
    node = TaskAllocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
