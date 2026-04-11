#!/usr/bin/env python3
import argparse
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EventInjector(Node):
    def __init__(self) -> None:
        super().__init__("event_injector")
        self.pub = self.create_publisher(String, "/scheduler/event_in", 10)

    def inject(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)
        self.get_logger().info(f"Injected event: {msg.data}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Inject dynamic event JSON.")
    parser.add_argument("--type", required=True, choices=["LOW_BATTERY", "UAV_FAILURE", "NEW_TASK", "COMM_LOSS"])
    parser.add_argument("--uav", default="uav_1")
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=3.0)
    parser.add_argument("--task-id", type=int, default=9001)
    args = parser.parse_args()

    payload = {"event_type": args.type, "uav_id": args.uav}
    if args.type == "NEW_TASK":
        payload["task"] = {
            "id": args.task_id,
            "priority": 3,
            "task_type": "scan",
            "deadline_sec": 150.0,
            "location": {"x": args.x, "y": args.y, "z": args.z},
        }

    rclpy.init()
    node = EventInjector()
    node.inject(payload)
    rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
