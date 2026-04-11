#!/usr/bin/env python3
"""
viz_publisher — 订阅仿真运行时数据并发布 RViz2 可视化消息。

订阅:
  /fleet/states_json      (std_msgs/String JSON) — 无人机位置、电量、传感器类型
  /allocation/result_json (std_msgs/String JSON) — 任务分配结果及航点
  /sim/obstacles          (geometry_msgs/PoseArray) — 障碍物位置
  /planner/global_path    (nav_msgs/Path) — 全局规划路径
  /planner/local_path     (nav_msgs/Path) — 避障局部路径
  /scheduler/events_log   (std_msgs/String JSON) — 事件日志

发布:
  /orchard_viz/markers    (visualization_msgs/MarkerArray) — 10 Hz
  /orchard_viz/coverage   (nav_msgs/OccupancyGrid)         —  1 Hz
"""

import json
import math
from typing import Any, Dict, List, Set, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseArray
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


# 各无人机专属颜色 (R, G, B)
_UAV_COLOURS: List[Tuple[float, float, float]] = [
    (0.20, 0.45, 1.00),  # uav_1 — 蓝
    (0.15, 0.75, 0.35),  # uav_2 — 绿
    (1.00, 0.50, 0.10),  # uav_3 — 橙
    (0.70, 0.20, 0.90),  # uav_4 — 紫
]
_COLOUR_BAD = (1.0, 0.15, 0.15)   # 红 — 故障/低电量


def _c(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    col = ColorRGBA()
    col.r, col.g, col.b, col.a = float(r), float(g), float(b), float(a)
    return col


def _pt(x: float, y: float, z: float) -> Point:
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


class VizPublisher(Node):
    def __init__(self) -> None:
        super().__init__("orchard_viz_publisher")

        # 参数与世界生成器保持一致
        self.declare_parameter("rows", 8)
        self.declare_parameter("trees_per_row", 24)
        self.declare_parameter("row_spacing", 5.0)
        self.declare_parameter("tree_spacing", 3.0)
        self.declare_parameter("tree_radius", 0.35)
        self.declare_parameter("orchard_width", 60.0)
        self.declare_parameter("orchard_height", 40.0)
        self.declare_parameter("grid_size", 2.0)
        self.declare_parameter("uav_count", 3)

        self._fleet: Dict[str, Any] = {}
        self._allocations: Dict[str, List[Tuple[float, float, float]]] = {}
        self._obstacles: List[Tuple[float, float]] = []
        self._global_path: List[Tuple[float, float, float]] = []
        self._local_path: List[Tuple[float, float, float]] = []
        self._covered: Set[Tuple[int, int]] = set()
        self._latest_event: str = ""
        # 飞行轨迹历史：uid → [[x, y, z], ...]（来自 gz_path_follower）
        self._trails: Dict[str, List[List[float]]] = {}

        self._marker_pub = self.create_publisher(MarkerArray, "/orchard_viz/markers", 10)
        self._grid_pub = self.create_publisher(OccupancyGrid, "/orchard_viz/coverage", 10)

        self.create_subscription(String, "/fleet/states_json", self._on_fleet, 10)
        self.create_subscription(String, "/allocation/result_json", self._on_alloc, 10)
        self.create_subscription(PoseArray, "/sim/obstacles", self._on_obstacles, 10)
        self.create_subscription(Path, "/planner/global_path", self._on_global_path, 10)
        self.create_subscription(Path, "/planner/local_path", self._on_local_path, 10)
        self.create_subscription(String, "/scheduler/events_log", self._on_event_log, 10)
        self.create_subscription(String, "/orchard_viz/trail_json", self._on_trail, 10)

        self.create_timer(0.1, self._publish_markers)   # 10 Hz
        self.create_timer(1.0, self._publish_coverage)  # 1 Hz

        self.get_logger().info("orchard_viz_publisher 已启动 — 发布到 /orchard_viz/markers 和 /orchard_viz/coverage")

    # ---------------------------------------------------------------- 订阅回调

    def _on_fleet(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._fleet = {s["uav_id"]: s for s in data.get("states", [])}
        g = float(self.get_parameter("grid_size").value)
        w = float(self.get_parameter("orchard_width").value)
        h = float(self.get_parameter("orchard_height").value)
        for s in data.get("states", []):
            cx = int((float(s["x"]) + w / 2.0) / g)
            cy = int((float(s["y"]) + h / 2.0) / g)
            self._covered.add((cx, cy))

    def _on_alloc(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._allocations = {}
        for alloc in data.get("allocations", []):
            uid = str(alloc.get("uav_id", ""))
            pts = [
                (float(p["x"]), float(p["y"]), float(p.get("z", 3.0)))
                for p in alloc.get("coarse_waypoints", [])
            ]
            self._allocations[uid] = pts

    def _on_obstacles(self, msg: PoseArray) -> None:
        self._obstacles = [(p.position.x, p.position.y) for p in msg.poses]

    def _on_global_path(self, msg: Path) -> None:
        self._global_path = [
            (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z)
            for ps in msg.poses
        ]

    def _on_local_path(self, msg: Path) -> None:
        self._local_path = [
            (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z)
            for ps in msg.poses
        ]

    def _on_event_log(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            etype = str(payload.get("event_type", ""))
            uid = str(payload.get("uav_id", ""))
            lat = float(payload.get("response_latency_ms", 0.0))
            self._latest_event = f"{etype}  {uid}  {lat:.0f}ms"
        except (json.JSONDecodeError, KeyError):
            pass

    def _on_trail(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._trails = data.get("trails", {})
        except json.JSONDecodeError:
            pass

    # ------------------------------------------------------------ marker 发布

    def _now(self):
        return self.get_clock().now().to_msg()

    def _base_marker(self, ns: str, mid: int, mtype: int) -> Marker:
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self._now()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _publish_markers(self) -> None:
        ma = MarkerArray()

        rows = int(self.get_parameter("rows").value)
        tpr = int(self.get_parameter("trees_per_row").value)
        row_sp = float(self.get_parameter("row_spacing").value)
        tree_sp = float(self.get_parameter("tree_spacing").value)
        tree_r = float(self.get_parameter("tree_radius").value)
        y0 = -0.5 * (rows - 1) * row_sp
        x0 = -0.5 * (tpr - 1) * tree_sp

        # ── 1. 树木：树干 + 树冠 ──────────────────────────────────────────────
        tid = 0
        for row in range(rows):
            for col in range(tpr):
                x = x0 + col * tree_sp
                y = y0 + row * row_sp

                # 树干
                trunk = self._base_marker("trees", tid, Marker.CYLINDER)
                trunk.pose.position.x = x
                trunk.pose.position.y = y
                trunk.pose.position.z = 1.0   # 中心在 z=1.0，范围 0~2m
                trunk.scale.x = tree_r * 2
                trunk.scale.y = tree_r * 2
                trunk.scale.z = 2.0
                trunk.color = _c(0.45, 0.28, 0.10)
                trunk.lifetime.sec = 0    # 永久
                ma.markers.append(trunk)
                tid += 1

                # 树冠
                canopy = self._base_marker("canopy", tid, Marker.SPHERE)
                canopy.pose.position.x = x
                canopy.pose.position.y = y
                canopy.pose.position.z = 2.8
                canopy.scale.x = tree_r * 3.2
                canopy.scale.y = tree_r * 3.2
                canopy.scale.z = tree_r * 2.6
                canopy.color = _c(0.18, 0.52, 0.16, 0.88)
                canopy.lifetime.sec = 0
                ma.markers.append(canopy)
                tid += 1

        # ── 2. 障碍物（红色圆柱） ─────────────────────────────────────────────
        for oi, (ox, oy) in enumerate(self._obstacles):
            obs = self._base_marker("obstacles", oi, Marker.CYLINDER)
            obs.pose.position.x = ox
            obs.pose.position.y = oy
            obs.pose.position.z = 1.0
            obs.scale.x = 0.7
            obs.scale.y = 0.7
            obs.scale.z = 2.0
            obs.color = _c(0.90, 0.10, 0.10, 0.85)
            obs.lifetime.sec = 0
            ma.markers.append(obs)

        # ── 3. 无人机球体 + 标签 ─────────────────────────────────────────────
        uav_count = int(self.get_parameter("uav_count").value)
        for i in range(uav_count):
            uid = f"uav_{i + 1}"
            state = self._fleet.get(uid)
            if state is None:
                continue
            x = float(state["x"])
            y = float(state["y"])
            z = float(state["z"])
            batt = float(state.get("battery_percent", 100.0))
            healthy = bool(state.get("healthy", True))
            sensor = str(state.get("sensor_type", "rgb"))
            cr, cg, cb = _COLOUR_BAD if (not healthy or batt < 15.0) else _UAV_COLOURS[i % len(_UAV_COLOURS)]

            # 球体
            sphere = self._base_marker("uav_body", i, Marker.SPHERE)
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = z
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.55
            sphere.color = _c(cr, cg, cb)
            sphere.lifetime.sec = 1
            ma.markers.append(sphere)

            # 文字标签
            label = self._base_marker("uav_label", i, Marker.TEXT_VIEW_FACING)
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z + 0.65
            label.scale.z = 0.38
            label.color = _c(1.0, 1.0, 1.0)
            label.text = f"UAV-{i + 1}  {batt:.0f}%  {sensor}"
            label.lifetime.sec = 1
            ma.markers.append(label)

            # 电量环（横向细圆柱，颜色随电量从绿→红渐变）
            bar_r = max(0.0, min(1.0, 1.0 - batt / 100.0))
            bar_g = max(0.0, min(1.0, batt / 100.0))
            bar = self._base_marker("uav_batt", i, Marker.CYLINDER)
            bar.pose.position.x = x
            bar.pose.position.y = y
            bar.pose.position.z = z + 0.40
            # 将圆柱旋转 90° 绕 X 轴以便水平放置
            bar.pose.orientation.x = math.sin(math.pi / 4)
            bar.pose.orientation.w = math.cos(math.pi / 4)
            bar.scale.x = 0.06
            bar.scale.y = 0.06
            bar.scale.z = max(0.05, batt / 100.0 * 0.6)
            bar.color = _c(bar_r, bar_g, 0.0, 0.9)
            bar.lifetime.sec = 1
            ma.markers.append(bar)

        # ── 4. 任务分配航线（各 UAV 彩色线段 + 航点球） ─────────────────────
        for i, (uid, pts) in enumerate(self._allocations.items()):
            if len(pts) < 2:
                continue
            idx = int(uid.split("_")[-1]) - 1 if "_" in uid else i
            cr, cg, cb = _UAV_COLOURS[idx % len(_UAV_COLOURS)]

            line = self._base_marker("alloc_path", idx, Marker.LINE_STRIP)
            line.scale.x = 0.14
            line.color = _c(cr, cg, cb, 0.85)
            line.lifetime.sec = 6
            for px, py, pz in pts:
                line.points.append(_pt(px, py, pz))
            ma.markers.append(line)

            for wi, (px, py, pz) in enumerate(pts):
                dot = self._base_marker(f"wp_{uid}", wi, Marker.SPHERE)
                dot.pose.position.x = px
                dot.pose.position.y = py
                dot.pose.position.z = pz
                dot.scale.x = dot.scale.y = dot.scale.z = 0.22
                dot.color = _c(cr, cg, cb, 0.65)
                dot.lifetime.sec = 6
                ma.markers.append(dot)

        # ── 5. 全局路径（白色细线） ──────────────────────────────────────────
        if len(self._global_path) >= 2:
            gpath = self._base_marker("global_path", 0, Marker.LINE_STRIP)
            gpath.scale.x = 0.07
            gpath.color = _c(0.88, 0.88, 0.88, 0.75)
            gpath.lifetime.sec = 2
            for px, py, pz in self._global_path:
                gpath.points.append(_pt(px, py, pz))
            ma.markers.append(gpath)

        # ── 6. 局部避障路径（黄色线） ────────────────────────────────────────
        if len(self._local_path) >= 2:
            lpath = self._base_marker("local_path", 0, Marker.LINE_STRIP)
            lpath.scale.x = 0.10
            lpath.color = _c(1.0, 0.92, 0.20, 0.95)
            lpath.lifetime.sec = 2
            for px, py, pz in self._local_path:
                lpath.points.append(_pt(px, py, pz))
            ma.markers.append(lpath)

        # ── 7. 飞行轨迹拖尾（实际飞过的路线，半透明同色细线） ──────────────
        for uid, pts in self._trails.items():
            if len(pts) < 2:
                continue
            try:
                idx = int(uid.split("_")[-1]) - 1
            except (ValueError, IndexError):
                idx = 0
            cr, cg, cb = _UAV_COLOURS[idx % len(_UAV_COLOURS)]
            trail_line = self._base_marker("trail", idx, Marker.LINE_STRIP)
            trail_line.scale.x = 0.06
            trail_line.color = _c(cr * 0.7, cg * 0.7, cb * 0.7, 0.60)
            trail_line.lifetime.sec = 0  # 永久保留，直到更新
            for pt in pts:
                trail_line.points.append(_pt(float(pt[0]), float(pt[1]), float(pt[2])))
            ma.markers.append(trail_line)

        # ── 8. 最近事件文字（左上角偏移） ───────────────────────────────────
        if self._latest_event:
            ev = self._base_marker("event_log", 0, Marker.TEXT_VIEW_FACING)
            ev.pose.position.x = -25.0
            ev.pose.position.y = 20.0
            ev.pose.position.z = 4.0
            ev.scale.z = 0.55
            ev.color = _c(1.0, 0.85, 0.20)
            ev.text = f"[事件] {self._latest_event}"
            ev.lifetime.sec = 5
            ma.markers.append(ev)

        self._marker_pub.publish(ma)

    # ------------------------------------------------------------ 覆盖率 grid

    def _publish_coverage(self) -> None:
        w_m = float(self.get_parameter("orchard_width").value)
        h_m = float(self.get_parameter("orchard_height").value)
        g = float(self.get_parameter("grid_size").value)
        cols = max(1, int(w_m / g))
        rows = max(1, int(h_m / g))

        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = self._now()
        grid.info.resolution = g
        grid.info.width = cols
        grid.info.height = rows
        grid.info.origin.position.x = -w_m / 2.0
        grid.info.origin.position.y = -h_m / 2.0
        grid.info.origin.orientation.w = 1.0

        data = [0] * (cols * rows)
        for cx, cy in self._covered:
            if 0 <= cx < cols and 0 <= cy < rows:
                data[cy * cols + cx] = 65
        grid.data = data
        self._grid_pub.publish(grid)


def main() -> None:
    rclpy.init()
    node = VizPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
