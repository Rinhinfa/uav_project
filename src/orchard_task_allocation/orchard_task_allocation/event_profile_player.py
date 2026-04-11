#!/usr/bin/env python3
import json
import time
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EventProfilePlayer(Node):
    def __init__(self) -> None:
        super().__init__("event_profile_player")
        self.declare_parameter("events_json", "[]")
        self.declare_parameter("events_file", "")
        self._pub = self.create_publisher(String, "/scheduler/event_in", 10)
        self._start = time.time()
        self._fired_ids = set()
        self._events = self._load_events()
        self.create_timer(0.2, self._tick)

    def _load_events(self) -> List[Dict[str, Any]]:
        events_file = str(self.get_parameter("events_file").value).strip()
        if events_file:
            try:
                with open(events_file, "r", encoding="utf-8") as f:
                    events = json.load(f)
                if isinstance(events, list):
                    return events
            except Exception:
                self.get_logger().warn(f"Failed to read events_file={events_file}, fallback to events_json.")

        raw = str(self.get_parameter("events_json").value)
        try:
            events = json.loads(raw)
            if isinstance(events, list):
                return events
        except json.JSONDecodeError:
            pass
        self.get_logger().warn("Invalid events_json, fallback to empty list.")
        return []

    def _tick(self) -> None:
        now = time.time() - self._start
        for i, event in enumerate(self._events):
            if i in self._fired_ids:
                continue
            t = float(event.get("t", 0.0))
            if now >= t:
                msg = String()
                payload = {
                    "event_type": str(event.get("type", "NEW_TASK")).upper(),
                    "uav_id": str(event.get("uav_id", "uav_1")),
                }
                if "task" in event and isinstance(event["task"], dict):
                    task = event["task"]
                    payload["task"] = {
                        "id": int(task.get("id", 9001)),
                        "priority": int(task.get("priority", 2)),
                        "task_type": str(task.get("task_type", "scan")),
                        "deadline_sec": float(task.get("deadline_sec", 180.0)),
                        "location": {
                            "x": float(task.get("x", 0.0)),
                            "y": float(task.get("y", 0.0)),
                            "z": float(task.get("z", 3.0)),
                        },
                    }
                msg.data = json.dumps(payload, ensure_ascii=False)
                self._pub.publish(msg)
                self._fired_ids.add(i)
                self.get_logger().info(f"fired event[{i}] at t={t}s: {msg.data}")


def main() -> None:
    rclpy.init()
    node = EventProfilePlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
