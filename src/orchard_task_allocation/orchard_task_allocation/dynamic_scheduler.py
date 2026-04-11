#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class DynamicScheduler(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_scheduler")
        self._event_pub = self.create_publisher(String, "/scheduler/events_log", 10)
        self.create_subscription(String, "/scheduler/event_in", self._on_event_topic, 10)
        self._realloc_client = self.create_client(Trigger, "/scheduler/reallocate_tasks")
        self.create_service(Trigger, "/scheduler/inject_event", self._on_event)
        self._pending = {}

    def _on_event_topic(self, msg: String) -> None:
        self._dispatch_event(msg.data)

    def _on_event(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        sample = {
            "event_type": "LOW_BATTERY",
            "uav_id": "uav_2",
            "source": "service_default",
        }
        self._dispatch_event(json.dumps(sample, ensure_ascii=False))
        response.success = True
        response.message = "sample event injected"
        return response

    def _dispatch_event(self, payload: str) -> None:
        start = time.time()
        if not self._realloc_client.wait_for_service(timeout_sec=1.0):
            out = String()
            out.data = json.dumps(
                {"status": "failed", "reason": "reallocate service unavailable", "event": payload},
                ensure_ascii=False,
            )
            self._event_pub.publish(out)
            return

        req = Trigger.Request()
        future = self._realloc_client.call_async(req)
        key = id(future)
        self._pending[key] = {"start": start, "payload": payload}
        future.add_done_callback(lambda f, k=key: self._on_realloc_done(f, k))

    def _on_realloc_done(self, future, key: int) -> None:
        meta = self._pending.pop(key, None)
        if not meta:
            return
        latency_ms = (time.time() - meta["start"]) * 1000.0
        success = False
        try:
            result = future.result()
            success = bool(result and result.success)
        except Exception:
            success = False
        out = String()
        out.data = json.dumps(
            {
                "status": "ok" if success else "failed",
                "event": meta["payload"],
                "response_latency_ms": round(latency_ms, 2),
            },
            ensure_ascii=False,
        )
        self._event_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = DynamicScheduler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
