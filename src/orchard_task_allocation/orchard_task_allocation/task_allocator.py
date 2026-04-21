#!/usr/bin/env python3
import json
import math
from typing import Any, Dict, List, Set, Tuple

from geometry_msgs.msg import PoseArray
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
        # 巡检在树冠侧方垄沟、低空；跨行/大跳变由 gz_path_follower 爬升至 transit_z
        self.declare_parameter("task_z", 2.0)
        self.declare_parameter("landing_z", 0.25)
        self.declare_parameter("transit_z", 3.85)
        self.declare_parameter("uav_speed_mps", 1.2)
        self.declare_parameter("max_flight_time_sec", 1800.0)  # 30 min
        self.declare_parameter("usable_flight_ratio", 0.9)
        self.declare_parameter("zone_bias_weight", 0.35)

        self._uav_states: Dict[str, Dict[str, Any]] = {}
        self._uav_home: Dict[str, Tuple[float, float]] = {}
        self._tasks: List[Dict[str, Any]] = []
        self._tasks_by_id: Dict[int, Dict[str, Any]] = {}
        self._current_task_routes: Dict[str, List[int]] = {}
        self._pending_reassign_task_ids: Set[int] = set()
        self._mission_id = 0
        self._needs_replan = True
        self._has_plan = False

        self._allocation_pub = self.create_publisher(String, "/allocation/result_json", _TRANSIENT_QOS)
        self._coarse_path_pub = self.create_publisher(Path, "/planner/coarse_waypoints", 10)
        self._coarse_path_pubs: Dict[str, Any] = {}
        self._mission_pub = self.create_publisher(UInt64, "/allocation/mission_id", 10)
        self.create_subscription(String, "/fleet/states_json", self._on_states, 10)
        self.create_subscription(PoseArray, "/sim/spawn_poses", self._on_spawn_poses, _TRANSIENT_QOS)
        self.create_subscription(String, "/scheduler/event_in", self._on_event, 10)
        self.create_service(Trigger, "/scheduler/reallocate_tasks", self._on_reallocate)
        self.create_timer(1.0 / float(self.get_parameter("timer_hz").value), self._periodic_allocate)
        self._seed_tasks_once()

    @staticmethod
    def _task_id(task: Dict[str, Any]) -> int:
        return int(task["id"])

    @staticmethod
    def _xy(task: Dict[str, Any]) -> Tuple[float, float]:
        loc = task["location"]
        return float(loc["x"]), float(loc["y"])

    def _algorithm_mode(self) -> str:
        return str(self.get_parameter("algorithm_mode").value).lower()

    def _register_task(self, task: Dict[str, Any]) -> None:
        task_id = self._task_id(task)
        if task_id in self._tasks_by_id:
            self._tasks_by_id[task_id].update(task)
            return
        self._tasks.append(task)
        self._tasks_by_id[task_id] = task

    def _task_by_id(self, task_id: int) -> Dict[str, Any]:
        return self._tasks_by_id[int(task_id)]

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
        half_row = 0.5 * row_spacing
        for row in range(rows):
            y_tree = y_offset + row * row_spacing
            # 航点放在相邻树行之间的垄沟中心（非树顶）：与树干保持约半行距，避免贴树冠
            if rows <= 1:
                y_task = y_tree
            elif row < rows - 1:
                y_task = y_tree + half_row
            else:
                y_task = y_tree - half_row
            for col in range(cols):
                task = {
                    "id": tid,
                    "priority": 2 if col % 6 == 0 else 1,
                    "task_type": "scan",
                    "deadline_sec": 120.0 if col % 6 == 0 else 240.0,
                    "location": {
                        "x": x_offset + col * col_spacing,
                        "y": y_task,
                        "z": task_z,
                    },
                }
                tid += 1
                self._register_task(task)

    def _on_states(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._uav_states = {s["uav_id"]: s for s in payload.get("states", [])}
        for uid, state in self._uav_states.items():
            if uid not in self._uav_home:
                self._uav_home[uid] = (float(state.get("x", 0.0)), float(state.get("y", 0.0)))
        if not self._has_plan and self._uav_states:
            self._needs_replan = True

    def _on_spawn_poses(self, msg: PoseArray) -> None:
        for i, pose in enumerate(msg.poses):
            uid = f"uav_{i+1}"
            self._uav_home[uid] = (float(pose.position.x), float(pose.position.y))
        if self._uav_home:
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
                new_task = {
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
                self._register_task(new_task)
                if self._algorithm_mode() == "baseline" and self._has_plan:
                    self._pending_reassign_task_ids.add(self._task_id(new_task))
                self._needs_replan = True
        elif etype in {"LOW_BATTERY", "UAV_FAILURE", "COMM_LOSS"}:
            uid = str(event.get("uav_id", ""))
            if uid in self._uav_states:
                self._uav_states[uid]["healthy"] = False
                self._uav_states[uid]["battery_percent"] = min(
                    5.0,
                    float(self._uav_states[uid].get("battery_percent", 5.0)),
                )
                if self._algorithm_mode() == "baseline" and self._has_plan:
                    self._pending_reassign_task_ids.update(self._current_task_routes.get(uid, []))
                    self._current_task_routes.pop(uid, None)
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
        self._current_task_routes = {
            str(alloc.get("uav_id", "")): [int(tid) for tid in alloc.get("task_ids", [])]
            for alloc in result.get("allocations", [])
        }
        self._pending_reassign_task_ids.clear()
        out = String()
        out.data = json.dumps(result, ensure_ascii=False)
        self._allocation_pub.publish(out)
        mid = UInt64()
        mid.data = int(result["mission_id"])
        self._mission_pub.publish(mid)
        self._publish_coarse_path(result)

    def _healthy_uavs(self, uav_states: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        healthy_uavs = [
            s
            for s in uav_states
            if s.get("healthy", False) and float(s.get("battery_percent", 0.0)) > 10.0
        ]
        return healthy_uavs or list(uav_states)

    def _route_cost_with_return(self, uav_id: str, tasks: List[Dict[str, Any]]) -> float:
        state = self._uav_states.get(uav_id, {})
        x = float(state.get("x", 0.0))
        y = float(state.get("y", 0.0))
        distance = 0.0
        for task in tasks:
            tx, ty = self._xy(task)
            distance += math.hypot(tx - x, ty - y)
            x, y = tx, ty
        hx, hy = self._uav_home.get(uav_id, (x, y))
        distance += math.hypot(hx - x, hy - y)
        return float(distance)

    def _tail_xy(self, uav_id: str, tasks: List[Dict[str, Any]]) -> Tuple[float, float]:
        if tasks:
            return self._xy(tasks[-1])
        state = self._uav_states.get(uav_id, {})
        return float(state.get("x", 0.0)), float(state.get("y", 0.0))

    def _build_result(
        self,
        routes: Dict[str, List[Dict[str, Any]]],
        remaining: List[Dict[str, Any]],
        optimize_routes: bool,
        reallocation_policy: str,
    ) -> Dict[str, Any]:
        transit_z = float(self.get_parameter("transit_z").value)
        landing_z = float(self.get_parameter("landing_z").value)
        speed = float(self.get_parameter("uav_speed_mps").value)
        max_time = float(self.get_parameter("max_flight_time_sec").value)
        usable_ratio = float(self.get_parameter("usable_flight_ratio").value)
        self._mission_id += 1
        allocations = []
        for uav_id in sorted(routes.keys()):
            assigned = routes[uav_id]
            sx = float(self._uav_states.get(uav_id, {}).get("x", 0.0))
            sy = float(self._uav_states.get(uav_id, {}).get("y", 0.0))
            hx, hy = self._uav_home.get(uav_id, (sx, sy))
            route = self._optimize_route_nearest_neighbor(assigned, sx, sy) if optimize_routes else list(assigned)
            waypoints = [dict(t["location"]) for t in route]
            # 完成巡检后返航并降落
            waypoints.append({"x": hx, "y": hy, "z": transit_z})
            waypoints.append({"x": hx, "y": hy, "z": landing_z})
            allocations.append(
                {
                    "uav_id": uav_id,
                    "task_ids": [self._task_id(t) for t in route],
                    "coarse_waypoints": waypoints,
                    "estimated_cost": self._estimate_cost_from_waypoints(uav_id, waypoints),
                }
            )
        return {
            "mission_id": self._mission_id,
            "algorithm_mode": self._algorithm_mode(),
            "route_policy": "nearest_neighbor" if optimize_routes else "assigned_order",
            "reallocation_policy": reallocation_policy,
            "allocations": allocations,
            "unassigned_task_ids": [self._task_id(t) for t in remaining],
            "max_distance_budget_m": float(max(10.0, speed * max_time * usable_ratio)),
        }

    def _compute_baseline_initial_allocations(
        self,
        tasks: List[Dict[str, Any]],
        uav_states: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        speed = float(self.get_parameter("uav_speed_mps").value)
        max_time = float(self.get_parameter("max_flight_time_sec").value)
        usable_ratio = float(self.get_parameter("usable_flight_ratio").value)
        max_distance = max(10.0, speed * max_time * usable_ratio)
        healthy_uavs = self._healthy_uavs(uav_states)
        routes: Dict[str, List[Dict[str, Any]]] = {str(s["uav_id"]): [] for s in healthy_uavs}
        route_cost: Dict[str, float] = {str(s["uav_id"]): 0.0 for s in healthy_uavs}
        current_xy: Dict[str, Tuple[float, float]] = {
            str(s["uav_id"]): (float(s.get("x", 0.0)), float(s.get("y", 0.0)))
            for s in healthy_uavs
        }
        remaining = sorted(
            list(tasks),
            key=lambda t: (float(t["location"]["x"]), float(t["location"]["y"]), self._task_id(t)),
        )

        while remaining:
            assigned_any = False
            for uav in healthy_uavs:
                uid = str(uav["uav_id"])
                if not remaining:
                    break
                cx, cy = current_xy[uid]
                hx, hy = self._uav_home.get(uid, (cx, cy))
                best_idx = -1
                best_score = float("inf")
                for i, task in enumerate(remaining):
                    tx, ty = self._xy(task)
                    to_task = math.hypot(tx - cx, ty - cy)
                    projected = route_cost[uid] + to_task + math.hypot(tx - hx, ty - hy)
                    if projected > max_distance:
                        continue
                    if to_task < best_score:
                        best_score = to_task
                        best_idx = i
                if best_idx < 0:
                    continue
                task = remaining.pop(best_idx)
                tx, ty = self._xy(task)
                route_cost[uid] += math.hypot(tx - cx, ty - cy)
                current_xy[uid] = (tx, ty)
                routes[uid].append(task)
                assigned_any = True
            if not assigned_any:
                break

        return self._build_result(
            routes=routes,
            remaining=remaining,
            optimize_routes=False,
            reallocation_policy="baseline_initial",
        )

    def _compute_baseline_incremental_allocations(self, uav_states: List[Dict[str, Any]]) -> Dict[str, Any]:
        speed = float(self.get_parameter("uav_speed_mps").value)
        max_time = float(self.get_parameter("max_flight_time_sec").value)
        usable_ratio = float(self.get_parameter("usable_flight_ratio").value)
        max_distance = max(10.0, speed * max_time * usable_ratio)
        healthy_uavs = self._healthy_uavs(uav_states)
        healthy_uids = {str(s["uav_id"]) for s in healthy_uavs}
        routes: Dict[str, List[Dict[str, Any]]] = {}
        for uid in sorted(healthy_uids):
            route_ids = self._current_task_routes.get(uid, [])
            routes[uid] = [self._task_by_id(task_id) for task_id in route_ids if task_id in self._tasks_by_id]
        route_cost: Dict[str, float] = {uid: self._route_cost_with_return(uid, tasks) for uid, tasks in routes.items()}
        tail_xy: Dict[str, Tuple[float, float]] = {uid: self._tail_xy(uid, tasks) for uid, tasks in routes.items()}
        preserved_ids = {self._task_id(task) for tasks in routes.values() for task in tasks}
        remaining: List[Dict[str, Any]] = []
        pool = [
            self._task_by_id(task_id)
            for task_id in sorted(self._pending_reassign_task_ids)
            if task_id in self._tasks_by_id and task_id not in preserved_ids
        ]

        for task in pool:
            tx, ty = self._xy(task)
            best_uid = ""
            best_projected = float("inf")
            for uid in sorted(healthy_uids):
                hx, hy = self._uav_home.get(uid, tail_xy[uid])
                tail_to_home = math.hypot(tail_xy[uid][0] - hx, tail_xy[uid][1] - hy)
                projected = (
                    route_cost[uid]
                    - tail_to_home
                    + math.hypot(tx - tail_xy[uid][0], ty - tail_xy[uid][1])
                    + math.hypot(tx - hx, ty - hy)
                )
                if projected > max_distance:
                    continue
                if projected < best_projected:
                    best_projected = projected
                    best_uid = uid
            if not best_uid:
                remaining.append(task)
                continue
            routes[best_uid].append(task)
            route_cost[best_uid] = best_projected
            tail_xy[best_uid] = (tx, ty)

        return self._build_result(
            routes=routes,
            remaining=remaining,
            optimize_routes=False,
            reallocation_policy="append_only_reassign",
        )

    def _compute_allocations(
        self,
        tasks: List[Dict[str, Any]],
        uav_states: List[Dict[str, Any]],
        incremental_only: bool,
    ) -> Dict[str, Any]:
        if self._algorithm_mode() == "baseline":
            if incremental_only and self._has_plan:
                return self._compute_baseline_incremental_allocations(uav_states)
            return self._compute_baseline_initial_allocations(tasks, uav_states)

        speed = float(self.get_parameter("uav_speed_mps").value)
        max_time = float(self.get_parameter("max_flight_time_sec").value)
        usable_ratio = float(self.get_parameter("usable_flight_ratio").value)
        zone_bias = float(self.get_parameter("zone_bias_weight").value)
        max_distance = max(10.0, speed * max_time * usable_ratio)

        pool = list(tasks)
        healthy_uavs = self._healthy_uavs(uav_states)
        routes: Dict[str, List[Dict[str, Any]]] = {str(s["uav_id"]): [] for s in healthy_uavs}
        route_cost: Dict[str, float] = {str(s["uav_id"]): 0.0 for s in healthy_uavs}
        current_xy: Dict[str, Tuple[float, float]] = {
            str(s["uav_id"]): (float(s.get("x", 0.0)), float(s.get("y", 0.0)))
            for s in healthy_uavs
        }
        sorted_uids = sorted(routes.keys())
        ys = [float(t["location"]["y"]) for t in pool] if pool else [0.0]
        y_min, y_max = min(ys), max(ys)
        span = max(1e-3, y_max - y_min)
        zone_center: Dict[str, float] = {}
        for i, uid in enumerate(sorted_uids):
            ratio = (i + 0.5) / max(1, len(sorted_uids))
            zone_center[uid] = y_min + ratio * span

        # 优先高优先级任务，兼顾近邻，减少总里程
        remaining = sorted(
            pool,
            key=lambda t: (int(t.get("priority", 1)), -float(t["location"]["x"])),
            reverse=True,
        )

        while remaining:
            assigned_any = False
            for uav in healthy_uavs:
                uid = str(uav["uav_id"])
                if not remaining:
                    break
                cx, cy = current_xy[uid]
                hx, hy = self._uav_home.get(uid, (cx, cy))
                best_idx = -1
                best_score = float("inf")
                for i, task in enumerate(remaining):
                    tx, ty = self._xy(task)
                    to_task = math.hypot(tx - cx, ty - cy)
                    to_home = math.hypot(tx - hx, ty - hy)
                    projected = route_cost[uid] + to_task + to_home
                    if projected > max_distance:
                        continue
                    zone_pen = abs(ty - zone_center.get(uid, ty))
                    score = to_task + zone_bias * zone_pen - 0.35 * float(task.get("priority", 1))
                    if incremental_only:
                        score *= 1.03
                    if score < best_score:
                        best_score = score
                        best_idx = i
                if best_idx < 0:
                    continue
                task = remaining.pop(best_idx)
                tx, ty = self._xy(task)
                route_cost[uid] += math.hypot(tx - cx, ty - cy)
                current_xy[uid] = (tx, ty)
                routes[uid].append(task)
                assigned_any = True
            if not assigned_any:
                break

        return self._build_result(
            routes=routes,
            remaining=remaining,
            optimize_routes=True,
            reallocation_policy="full_reoptimize" if incremental_only else "global_optimize",
        )

    @staticmethod
    def _optimize_route_nearest_neighbor(tasks: List[Dict[str, Any]], sx: float, sy: float) -> List[Dict[str, Any]]:
        if not tasks:
            return []
        remaining = list(tasks)
        route: List[Dict[str, Any]] = []
        cx, cy = sx, sy
        while remaining:
            best_i = min(
                range(len(remaining)),
                key=lambda i: math.hypot(
                    float(remaining[i]["location"]["x"]) - cx,
                    float(remaining[i]["location"]["y"]) - cy,
                ),
            )
            nxt = remaining.pop(best_i)
            route.append(nxt)
            cx = float(nxt["location"]["x"])
            cy = float(nxt["location"]["y"])
        return route

    def _publish_coarse_path(self, allocations: Dict[str, Any]) -> None:
        stamp = self.get_clock().now().to_msg()
        merged_path = Path()
        merged_path.header.frame_id = "map"
        merged_path.header.stamp = stamp
        for alloc in allocations["allocations"]:
            uid = str(alloc.get("uav_id", "")).strip()
            if not uid:
                continue
            path = Path()
            path.header.frame_id = "map"
            path.header.stamp = stamp
            for pt in alloc["coarse_waypoints"]:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = stamp
                pose.pose.position = Point(x=float(pt["x"]), y=float(pt["y"]), z=float(pt["z"]))
                path.poses.append(pose)
                merged_path.poses.append(pose)
            if path.poses:
                self._ensure_coarse_path_pub(uid).publish(path)
        if merged_path.poses:
            self._coarse_path_pub.publish(merged_path)

    def _ensure_coarse_path_pub(self, uav_id: str):
        pub = self._coarse_path_pubs.get(uav_id)
        if pub is None:
            pub = self.create_publisher(Path, f"/{uav_id}/planner/coarse_waypoints", 10)
            self._coarse_path_pubs[uav_id] = pub
        return pub

    def _estimate_cost_from_waypoints(self, uav_id: str, waypoints: List[Dict[str, Any]]) -> float:
        state = self._uav_states.get(uav_id)
        x = float(state["x"]) if state else 0.0
        y = float(state["y"]) if state else 0.0
        distance = 0.0
        for pt in waypoints:
            tx = float(pt["x"])
            ty = float(pt["y"])
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
