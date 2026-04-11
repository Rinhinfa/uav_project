#!/usr/bin/env python3
from geometry_msgs.msg import Pose, PoseArray
import rclpy
from rclpy.node import Node


class SpawnPlanPublisher(Node):
    def __init__(self) -> None:
        super().__init__("spawn_plan_publisher")
        self.declare_parameter("uav_count", 3)
        self.declare_parameter("row_spacing", 3.0)
        self.pub_ = self.create_publisher(PoseArray, "/sim/spawn_poses", 10)
        self.create_timer(1.0, self._publish_once)
        self._published = False

    def _publish_once(self) -> None:
        if self._published:
            return
        n = int(self.get_parameter("uav_count").value)
        s = float(self.get_parameter("row_spacing").value)
        arr = PoseArray()
        arr.header.frame_id = "map"
        arr.header.stamp = self.get_clock().now().to_msg()
        for i in range(n):
            p = Pose()
            p.position.x = -2.0
            p.position.y = (i - (n - 1) / 2.0) * s
            p.position.z = 1.0
            arr.poses.append(p)
        self.pub_.publish(arr)
        self._published = True
        self.get_logger().info(f"Published {n} spawn poses.")


def main() -> None:
    rclpy.init()
    node = SpawnPlanPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
