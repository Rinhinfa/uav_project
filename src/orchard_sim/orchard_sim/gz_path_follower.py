#!/usr/bin/env python3
import json
import math
from typing import Dict, List, Tuple

from geometry_msgs.msg import PoseArray
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
        # 垄旁巡检高度（低于树冠球下缘）
        self.declare_parameter("cruise_z", 2.0)
        # 跨行/大跳变航段：从树顶上方通过
        self.declare_parameter("transit_z", 3.85)
        self.declare_parameter("set_pose_hz", 20.0)
        self.declare_parameter("speed_mps", 2.0)
        self.declare_parameter("arrive_tolerance", 0.35)
        self.declare_parameter("landing_z", 0.25)
        self.declare_parameter("max_flight_time_sec", 1800.0)
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
        self._landing_z = float(self.get_parameter("landing_z").value)
        self._max_flight_time = float(self.get_parameter("max_flight_time_sec").value)
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
        self._tree_col_spacing = _tree_col_spacing
        self._tree_row_spacing = _tree_row_spacing
        self._tree_positions = self._compute_tree_positions(
            _tree_rows, _trees_per_row, _tree_row_spacing, _tree_col_spacing
        )
        self._transit_z = float(self.get_parameter("transit_z").value)

        self._uav_states: Dict[str, Dict[str, float]] = {}
        self._uav_spawn_xy: Dict[str, Tuple[float, float]] = {}
        self._uav_paths: Dict[str, List[Tuple[float, float, float]]] = {}
        self._path_index: Dict[str, int] = {}
        self._flight_time: Dict[str, float] = {}
        self._timeout_landed: Dict[str, bool] = {}
        # 轨迹历史：uid → [(x, y, z), ...]
        self._trail: Dict[str, List[List[float]]] = {}
        self._last_mission_id = -1
        self._last_tick_time = self.get_clock().now()
        self._init_spawn_states()

        self.create_subscription(String, allocation_topic, self._on_allocation, _TRANSIENT_QOS)
        self.create_subscription(PoseArray, "/sim/spawn_poses", self._on_spawn_poses, _TRANSIENT_QOS)
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
            self._uav_spawn_xy[uid] = (self._start_x, y)
            self._uav_paths[uid] = []
            self._path_index[uid] = 0
            self._flight_time[uid] = 0.0
            self._timeout_landed[uid] = False
            self._trail[uid] = [[self._start_x, y, self._cruise_z]]

    def _on_spawn_poses(self, msg: PoseArray) -> None:
        """用 spawn_plan_publisher 的随机果园外起飞点同步初始状态。"""
        for i, pose in enumerate(msg.poses):
            uid = f"uav_{i+1}"
            if uid not in self._uav_states:
                continue
            x = float(pose.position.x)
            y = float(pose.position.y)
            z = float(pose.position.z if abs(float(pose.position.z)) > 1e-3 else self._cruise_z)
            self._uav_states[uid]["x"] = x
            self._uav_states[uid]["y"] = y
            self._uav_states[uid]["z"] = z
            self._uav_spawn_xy[uid] = (x, y)
            self._trail[uid] = [[x, y, z]]

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
            st = self._uav_states[uid]
            self._uav_spawn_xy[uid] = (st["x"], st["y"])

    def _on_timer(self) -> None:
        if not self._client.wait_for_service(timeout_sec=0.0):
            return
        now = self.get_clock().now()
        dt = max(1e-3, (now - self._last_tick_time).nanoseconds / 1e9)
        self._last_tick_time = now
        max_step = self._speed * dt
        for uid in self._uav_states.keys():
            self._flight_time[uid] = self._flight_time.get(uid, 0.0) + dt
            if self._flight_time[uid] >= self._max_flight_time and not self._timeout_landed.get(uid, False):
                self._uav_paths[uid] = self._make_emergency_landing_path(uid)
                self._path_index[uid] = 0
                self._timeout_landed[uid] = True
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
            target_z = self._leg_target_z(uid, idx, path, tz)
            dz_to_target = target_z - state["z"]
            # 同一XY上的返航降落点也必须满足Z到达，避免“悬空不落地”
            if dist <= self._arrive_tol and abs(dz_to_target) <= self._arrive_tol:
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
            # 短航段：垄旁低空；跨行或大跨度：越顶高度
            dz = target_z - state["z"]
            if abs(dz) > 1e-3:
                z_step = math.copysign(min(abs(dz), max_step * 0.6), dz)
                state["z"] += z_step
            return

    def _segment_is_long_cross(
        self, uid: str, idx: int, path: List[Tuple[float, float, float]]
    ) -> bool:
        """跨到另一树行，或沿垄大跨度跳点，视为需从树冠上方通过（仅用航段端点，避免接近终点时误判）。"""
        tx, ty, _ = path[idx]
        if idx > 0:
            px, py, _ = path[idx - 1]
        else:
            px, py = self._uav_spawn_xy.get(uid, (tx, ty))
        ddy = abs(ty - py)
        if ddy > 0.45 * self._tree_row_spacing:
            return True
        leg = math.hypot(tx - px, ty - py)
        if leg > 2.2 * self._tree_col_spacing:
            return True
        return False

    def _leg_target_z(self, uid: str, idx: int, path: List[Tuple[float, float, float]], tz: float) -> float:
        if tz < self._cruise_z:
            return tz
        if self._segment_is_long_cross(uid, idx, path):
            return max(self._transit_z, tz)
        return max(self._cruise_z, tz)

    def _make_emergency_landing_path(self, uid: str) -> List[Tuple[float, float, float]]:
        hx, hy = self._uav_spawn_xy.get(uid, (self._uav_states[uid]["x"], self._uav_states[uid]["y"]))
        return [
            (hx, hy, self._transit_z),
            (hx, hy, self._landing_z),
        ]

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
        """调整运动方向向量，绕开非目标树树冠。

        改进要点：
          1. 改用3D球体距离检测，任意飞行高度均有效
             （原2D截面法在 cz < canopy_z - canopy_r 时 r_sq≤0 直接返回，完全失效）
          2. 精确识别目标树：与目标航点XY最近且在avoid_r以内的那棵树不参与回避
          3. 路径前瞻：提前检测飞行路径段是否将穿越树冠球，及早施加侧向回避力

        参数：
            cx/cy/cz  — 无人机当前位置
            ux/uy     — 归一化目标方向向量（将被调整后返回）
            tx/ty     — 当前目标航点 XY
        """
        # 3D避障触发半径（树冠球半径 + 安全余量），与高度无关
        avoid_r = self._canopy_r + self._avoid_margin
        dz = cz - self._canopy_z  # 高度差（对所有树相同）

        # 识别目标树：与目标航点XY距离最近且在avoid_r以内的树（飞向它，不回避）
        target_tree_idx = -1
        min_target_dist = avoid_r
        for i, (tree_x, tree_y) in enumerate(self._tree_positions):
            d = math.hypot(tx - tree_x, ty - tree_y)
            if d < min_target_dist:
                min_target_dist = d
                target_tree_idx = i

        # 路径段单位方向向量（用于前瞻检测）
        path_len = math.hypot(tx - cx, ty - cy)
        if path_len > 1e-6:
            path_ux = (tx - cx) / path_len
            path_uy = (ty - cy) / path_len
        else:
            path_ux, path_uy = ux, uy

        steer_x, steer_y = 0.0, 0.0
        for i, (tree_x, tree_y) in enumerate(self._tree_positions):
            if i == target_tree_idx:
                continue

            dx = cx - tree_x
            dy = cy - tree_y
            dist_xy = math.hypot(dx, dy)

            # ── 1. 当前位置到树冠球心的3D距离近邻回避 ──
            dist_3d = math.sqrt(dist_xy * dist_xy + dz * dz)
            if 0.0 < dist_3d < avoid_r:
                # 线性衰减推力，在XY平面上远离树干方向施力
                strength = (avoid_r - dist_3d) / avoid_r
                if dist_xy > 1e-6:
                    steer_x += (dx / dist_xy) * strength
                    steer_y += (dy / dist_xy) * strength
                else:
                    steer_y += strength  # 正上/下方：向+Y偏移

            # ── 2. 路径前瞻：检测路径段最近点是否进入树冠球 ──
            if path_len > 1e-6:
                # 将 (tree - c) 投影到路径方向：proj = dot(tree-c, path_dir)
                # 注意 dx = cx-tree_x，所以 tree-c = (-dx, -dy)
                proj = -dx * path_ux - dy * path_uy
                if 0.0 < proj < path_len:
                    # 路径段上最近点的坐标
                    near_x = cx + proj * path_ux
                    near_y = cy + proj * path_uy
                    # 最近点到树冠球心的横向距离（XY平面）
                    perp_x = near_x - tree_x
                    perp_y = near_y - tree_y
                    perp_xy = math.hypot(perp_x, perp_y)
                    # 用当前飞行高度估算3D接近距离
                    near_dist_3d = math.sqrt(perp_xy * perp_xy + dz * dz)
                    if near_dist_3d < avoid_r:
                        # 路径将穿越树冠：沿垂直方向（远离树冠侧）施加预偏力
                        la_strength = 0.5 * (avoid_r - near_dist_3d) / avoid_r
                        if perp_xy > 1e-6:
                            steer_x += (perp_x / perp_xy) * la_strength
                            steer_y += (perp_y / perp_xy) * la_strength
                        else:
                            steer_y += la_strength

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
