#!/usr/bin/env python3
import random

from geometry_msgs.msg import Pose, PoseArray
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


class SpawnPlanPublisher(Node):
    def __init__(self) -> None:
        super().__init__("spawn_plan_publisher")
        self.declare_parameter("uav_count", 3)
        self.declare_parameter("spawn_z", 2.0)
        self.declare_parameter("random_seed", -1)
        self.declare_parameter("orchard_width", 69.0)
        self.declare_parameter("orchard_height", 35.0)
        self.declare_parameter("outside_margin", 8.0)
        self.declare_parameter("spawn_area_side", "left")
        self.declare_parameter("spawn_area_width", 8.0)
        self.declare_parameter("spawn_area_height", 10.0)
        self.declare_parameter("spawn_area_center_y", 0.0)
        self.declare_parameter("min_pair_spacing", 2.2)
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_ = self.create_publisher(PoseArray, "/sim/spawn_poses", qos)
        self.create_timer(1.0, self._publish_once)
        self._published = False
        seed = int(self.get_parameter("random_seed").value)
        self._rng = random.Random(None if seed < 0 else seed)

    def _publish_once(self) -> None:
        if self._published:
            return
        n = int(self.get_parameter("uav_count").value)
        z = float(self.get_parameter("spawn_z").value)
        orchard_w = float(self.get_parameter("orchard_width").value)
        orchard_h = float(self.get_parameter("orchard_height").value)
        margin = float(self.get_parameter("outside_margin").value)
        side = str(self.get_parameter("spawn_area_side").value).lower()
        area_w = max(1.0, float(self.get_parameter("spawn_area_width").value))
        area_h = max(1.0, float(self.get_parameter("spawn_area_height").value))
        center_y = float(self.get_parameter("spawn_area_center_y").value)
        min_spacing = float(self.get_parameter("min_pair_spacing").value)
        x_half = 0.5 * orchard_w
        y_half = 0.5 * orchard_h
        arr = PoseArray()
        arr.header.frame_id = "map"
        arr.header.stamp = self.get_clock().now().to_msg()
        placed = []
        if side == "right":
            cx = x_half + margin + 0.5 * area_w
            cy = center_y
        elif side == "top":
            cx = 0.0
            cy = y_half + margin + 0.5 * area_h
        elif side == "bottom":
            cx = 0.0
            cy = -y_half - margin - 0.5 * area_h
        else:
            cx = -x_half - margin - 0.5 * area_w
            cy = center_y
        for i in range(n):
            p = Pose()
            x = cx
            y = cy
            for _ in range(60):
                x = cx + self._rng.uniform(-0.5 * area_w, 0.5 * area_w)
                y = cy + self._rng.uniform(-0.5 * area_h, 0.5 * area_h)
                if all((x - px) ** 2 + (y - py) ** 2 >= min_spacing ** 2 for px, py in placed):
                    break
            placed.append((x, y))
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = z
            arr.poses.append(p)
        self.pub_.publish(arr)
        self._published = True
        self.get_logger().info(f"Published {n} random spawn poses in one outside-orchard area.")


def main() -> None:
    rclpy.init()
    node = SpawnPlanPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
