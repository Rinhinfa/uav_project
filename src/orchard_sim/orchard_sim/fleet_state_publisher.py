#!/usr/bin/env python3
import math
import json

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class FleetStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("fleet_state_publisher")
        self.declare_parameter("uav_count", 3)
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("battery_drain_per_sec", 0.08)
        self.declare_parameter("radius", 4.0)
        self.publisher_ = self.create_publisher(String, "/fleet/states_json", 10)
        self.obstacle_pub_ = self.create_publisher(PoseArray, "/sim/obstacles", 10)

        self._uav_count = int(self.get_parameter("uav_count").value)
        hz = float(self.get_parameter("publish_hz").value)
        self._drain = float(self.get_parameter("battery_drain_per_sec").value)
        self._radius = float(self.get_parameter("radius").value)
        self._batteries = [100.0 for _ in range(self._uav_count)]
        self._time_sec = 0.0
        # 来自 gz_path_follower 的真实位置；None 表示尚未收到，回退到仿真圆周运动
        self._gz_positions: dict = {}
        self._odom_pubs = {
            f"uav_{i+1}": self.create_publisher(Odometry, f"/uav_{i+1}/odom", 10)
            for i in range(self._uav_count)
        }
        # 订阅 gz_path_follower 发布的真实位置
        self.create_subscription(String, "/sim/uav_positions_json", self._on_gz_positions, 10)
        self.create_timer(1.0 / max(hz, 0.1), self._on_timer)

    def _on_gz_positions(self, msg: String) -> None:
        """接收 gz_path_follower 发布的真实 UAV 位置并缓存。"""
        try:
            data = json.loads(msg.data)
            self._gz_positions = data.get("uav_positions", {})
        except json.JSONDecodeError:
            pass

    def _on_timer(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._time_sec += 1.0 / max(float(self.get_parameter("publish_hz").value), 0.1)
        obstacles = PoseArray()
        obstacles.header.frame_id = "map"
        obstacles.header.stamp = stamp
        payload = {"states": []}
        for ox, oy in [(2.0, 2.0), (-3.0, 5.0), (6.0, -2.5)]:
            obs = Pose()
            obs.position.x = ox
            obs.position.y = oy
            obs.position.z = 0.5
            obstacles.poses.append(obs)
        for i in range(self._uav_count):
            uav_id = f"uav_{i+1}"
            sensor_type = "rgb+thermal" if i % 2 == 0 else "rgb"
            self._batteries[i] = max(0.0, self._batteries[i] - self._drain)
            battery_percent = float(self._batteries[i])
            healthy = battery_percent > 2.0
            cruise_speed = 4.5 + i * 0.3

            # 优先使用 gz_path_follower 的真实位置，否则回退到圆周仿真
            gz_pos = self._gz_positions.get(uav_id)
            if gz_pos is not None:
                px = float(gz_pos["x"])
                py = float(gz_pos["y"])
                pz = float(gz_pos["z"])
            else:
                phase = self._time_sec + i * (2.0 * math.pi / max(self._uav_count, 1))
                px = self._radius * math.cos(phase)
                py = self._radius * math.sin(phase)
                pz = 3.0

            pose = Pose()
            pose.position.x = px
            pose.position.y = py
            pose.position.z = pz
            payload["states"].append(
                {
                    "uav_id": uav_id,
                    "sensor_type": sensor_type,
                    "battery_percent": battery_percent,
                    "healthy": healthy,
                    "cruise_speed": cruise_speed,
                    "x": px,
                    "y": py,
                    "z": pz,
                }
            )
            odom = Odometry()
            odom.header.frame_id = "map"
            odom.header.stamp = stamp
            odom.child_frame_id = uav_id
            odom.pose.pose = pose
            self._odom_pubs[uav_id].publish(odom)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.publisher_.publish(msg)
        self.obstacle_pub_.publish(obstacles)


def main() -> None:
    rclpy.init()
    node = FleetStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
