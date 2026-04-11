#!/usr/bin/env python3
import json
import math
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import String

# 与 task_allocator 保持一致：确保晚启动时也能收到已发布的分配结果
_TRANSIENT_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class GzPathFollower(Node):
    def __init__(self) -> None:
        super().__init__("gz_fleet_path_follower")
        self.declare_parameter("world_name", "orchard_world")
        self.declare_parameter("uav_count", 3)
        self.declare_parameter("uav_row_spacing", 3.0)
        self.declare_parameter("uav_start_x", -2.0)
        self.declare_parameter("cruise_z", 2.0)
        self.declare_parameter("set_pose_hz", 20.0)
        self.declare_parameter("speed_mps", 2.0)
        self.declare_parameter("arrive_tolerance", 0.35)
        self.declare_parameter("allocation_topic", "/allocation/result_json")
        # 轨迹记录：每隔多远记录一个点（m）
        self.declare_parameter("trail_step_m", 0.4)
        # 每架无人机保留的最大轨迹点数
        self.declare_parameter("trail_max_pts", 500)
        # 树木参数（与 world_generator 保持一致，用于树冠碰撞回避）
        self.declare_parameter("tree_rows", 8)
        self.declare_parameter("trees_per_row", 24)
        self.declare_parameter("tree_row_spacing", 5.0)
        self.declare_parameter("tree_col_spacing", 3.0)
        self.declare_parameter("tree_radius", 0.35)
        self.declare_parameter("canopy_height_z", 2.9)
        self.declare_parameter("avoid_safety_margin", 0.5)

        world_name = str(self.get_parameter("world_name").value)
        self._uav_count = int(self.get_parameter("uav_count").value)
        self._row_spacing = float(self.get_parameter("uav_row_spacing").value)
        self._start_x = float(self.get_parameter("uav_start_x").value)
        self._cruise_z = float(self.get_parameter("cruise_z").value)
        hz = float(self.get_parameter("set_pose_hz").value)
        self._speed = float(self.get_parameter("speed_mps").value)
        self._arrive_tol = float(self.get_parameter("arrive_tolerance").value)
        allocation_topic = str(self.get_parameter("allocation_topic").value)
        self._service_name = f"/world/{world_name}/set_pose"
        self._trail_step = float(self.get_parameter("trail_step_m").value)
        self._trail_max = int(self.get_parameter("trail_max_pts").value)

        # 树冠回避：预计算所有树木 XY 位置
        _tree_rows = int(self.get_parameter("tree_rows").value)
        _trees_per_row = int(self.get_parameter("trees_per_row").value)
        _tree_row_spacing = float(self.get_parameter("tree_row_spacing").value)
        _tree_col_spacing = float(self.get_parameter("tree_col_spacing").value)
        _tree_radius = float(self.get_parameter("tree_radius").value)
        self._canopy_z = float(self.get_parameter("canopy_height_z").value)
        self._canopy_r = _tree_radius * 2.2          # 与 world_generator 一致
        self._avoid_margin = float(self.get_parameter("avoid_safety_margin").value)
        self._tree_positions = self._compute_tree_positions(
            _tree_rows, _trees_per_row, _tree_row_spacing, _tree_col_spacing
        )

        self._uav_states: Dict[str, Dict[str, float]] = {}
        self._uav_paths: Dict[str, List[Tuple[float, float, float]]] = {}
        self._path_index: Dict[str, int] = {}
        # 轨迹历史：uid → [(x, y, z), ...]
        self._trail: Dict[str, List[List[float]]] = {}
        self._last_mission_id = -1
        self._last_tick_time = self.get_clock().now()
        self._init_spawn_states()

        self.create_subscription(String, allocation_topic, self._on_allocation, _TRANSIENT_QOS)
        self._client = self.create_client(SetEntityPose, self._service_name)
        self._trail_pub = self.create_publisher(String, "/orchard_viz/trail_json", 10)
        # 发布真实位置供 fleet_state_publisher 使用，同时修正 RViz 显示
        self._positions_pub = self.create_publisher(String, "/sim/uav_positions_json", 10)
        self.create_timer(1.0 / max(hz, 1.0), self._on_timer)
        self.get_logger().info(
            f"Following {allocation_topic}, writing fleet pose to {self._service_name}"
        )

    def _init_spawn_states(self) -> None:
        y_center = 0.5 * (max(self._uav_count, 1) - 1) * self._row_spacing
        for i in range(max(self._uav_count, 1)):
            uid = f"uav_{i+1}"
            y = i * self._row_spacing - y_center
            self._uav_states[uid] = {
                "x": self._start_x,
                "y": y,
                "z": self._cruise_z,
                "yaw": 0.0,
            }
            self._uav_paths[uid] = []
            self._path_index[uid] = 0
            self._trail[uid] = [[self._start_x, y, self._cruise_z]]

    def _on_allocation(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        mission_id = int(payload.get("mission_id", -1))
        if mission_id == self._last_mission_id:
            return
        self._last_mission_id = mission_id
        allocations = payload.get("allocations", [])
        for alloc in allocations:
            uid = str(alloc.get("uav_id", "")).strip()
            if uid not in self._uav_states:
                continue
            points = []
            for pt in alloc.get("coarse_waypoints", []):
                points.append(
                    (
                        float(pt.get("x", self._uav_states[uid]["x"])),
                        float(pt.get("y", self._uav_states[uid]["y"])),
                        float(pt.get("z", self._cruise_z)),
                    )
                )
            self._uav_paths[uid] = points
            self._path_index[uid] = 0

    def _on_timer(self) -> None:
        if not self._client.wait_for_service(timeout_sec=0.0):
            return
        now = self.get_clock().now()
        dt = max(1e-3, (now - self._last_tick_time).nanoseconds / 1e9)
        self._last_tick_time = now
        max_step = self._speed * dt
        for uid in self._uav_states.keys():
            self._advance_one(uid, max_step)
            self._record_trail(uid)
            self._send_pose(uid)
        self._publish_trail()
        self._publish_positions()

    def _advance_one(self, uid: str, max_step: float) -> None:
        path = self._uav_paths.get(uid, [])
        idx = self._path_index.get(uid, 0)
        if not path:
            return
        state = self._uav_states[uid]
        while idx < len(path):
            tx, ty, tz = path[idx]
            dx = tx - state["x"]
            dy = ty - state["y"]
            dist = math.hypot(dx, dy)
            if dist <= self._arrive_tol:
                idx += 1
                self._path_index[uid] = idx
                continue
            step = min(max_step, dist)
            if dist > 1e-6:
                ux = dx / dist
                uy = dy / dist
                # 绕开路径上的非目标树树冠
                ux, uy = self._steer_around_trees(
                    state["x"], state["y"], state["z"], ux, uy, tx, ty
                )
                state["x"] += ux * step
                state["y"] += uy * step
                state["yaw"] = math.atan2(uy, ux)
            # 平滑 Z 过渡，避免突变抖动
            target_z = max(self._cruise_z, tz)
            dz = target_z - state["z"]
            if abs(dz) > 1e-3:
                z_step = math.copysign(min(abs(dz), max_step * 0.6), dz)
                state["z"] += z_step
            return

    def _record_trail(self, uid: str) -> None:
        """每隔 trail_step_m 记录一个轨迹点。"""
        state = self._uav_states[uid]
        trail = self._trail.setdefault(uid, [])
        if trail:
            lx, ly, _ = trail[-1]
            if math.hypot(state["x"] - lx, state["y"] - ly) < self._trail_step:
                return
        trail.append([state["x"], state["y"], state["z"]])
        if len(trail) > self._trail_max:
            trail.pop(0)

    def _publish_trail(self) -> None:
        msg = String()
        msg.data = json.dumps({"trails": self._trail}, ensure_ascii=False)
        self._trail_pub.publish(msg)

    def _publish_positions(self) -> None:
        """将各无人机当前真实位置发布到 /sim/uav_positions_json，供 fleet_state_publisher 同步。"""
        positions = {
            uid: {"x": s["x"], "y": s["y"], "z": s["z"]}
            for uid, s in self._uav_states.items()
        }
        msg = String()
        msg.data = json.dumps({"uav_positions": positions}, ensure_ascii=False)
        self._positions_pub.publish(msg)

    # ------------------------------------------------------------------
    # 树冠回避辅助方法
    # ------------------------------------------------------------------

    @staticmethod
    def _compute_tree_positions(
        rows: int, cols: int, row_spacing: float, col_spacing: float
    ) -> List[Tuple[float, float]]:
        """复刻 world_generator 的网格，返回所有树木 (x, y) 中心坐标。"""
        positions: List[Tuple[float, float]] = []
        y_offset = -0.5 * (rows - 1) * row_spacing
        x_offset = -0.5 * (cols - 1) * col_spacing
        for row in range(rows):
            y = y_offset + row * row_spacing
            for col in range(cols):
                positions.append((x_offset + col * col_spacing, y))
        return positions

    def _steer_around_trees(
        self,
        cx: float, cy: float, cz: float,
        ux: float, uy: float,
        tx: float, ty: float,
    ) -> Tuple[float, float]:
        """调整运动方向向量，绕开当前高度平面上与路径相交的非目标树树冠。

        参数：
            cx/cy/cz  — 无人机当前位置
            ux/uy     — 归一化目标方向向量（将被调整后返回）
            tx/ty     — 当前目标航点 XY（目标树不参与回避）
        """
        # 计算当前高度下树冠球的 XY 截面半径
        dz = cz - self._canopy_z
        r_sq = self._canopy_r ** 2 - dz ** 2
        if r_sq <= 0.0:
            # 该高度完全在树冠球以外（太低或太高），无需 XY 回避
            return ux, uy

        avoid_r = math.sqrt(r_sq) + self._avoid_margin  # 回避触发半径

        steer_x, steer_y = 0.0, 0.0
        for (tree_x, tree_y) in self._tree_positions:
            # 目标树：无人机就是要飞过去扫描该树，不回避
            if math.hypot(tx - tree_x, ty - tree_y) < avoid_r:
                continue
            dx = cx - tree_x
            dy = cy - tree_y
            dist = math.hypot(dx, dy)
            if 0.0 < dist < avoid_r:
                # 线性衰减推力：越靠近树心推力越强
                strength = (avoid_r - dist) / avoid_r
                steer_x += (dx / dist) * strength
                steer_y += (dy / dist) * strength

        if math.hypot(steer_x, steer_y) < 1e-6:
            return ux, uy  # 附近无障碍树，方向不变

        # 将目标方向与回避力融合（回避权重 2× 确保真正绕开）
        blend_x = ux + 2.0 * steer_x
        blend_y = uy + 2.0 * steer_y
        mag = math.hypot(blend_x, blend_y)
        if mag < 1e-6:
            return ux, uy
        return blend_x / mag, blend_y / mag

    def _send_pose(self, uid: str) -> None:
        state = self._uav_states[uid]
        req = SetEntityPose.Request()
        req.entity = Entity()
        req.entity.name = uid
        req.entity.type = Entity.MODEL
        req.pose.position.x = float(state["x"])
        req.pose.position.y = float(state["y"])
        req.pose.position.z = float(state["z"])
        req.pose.orientation.z = math.sin(state["yaw"] * 0.5)
        req.pose.orientation.w = math.cos(state["yaw"] * 0.5)
        self._client.call_async(req)


def main() -> None:
    rclpy.init()
    node = GzPathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
