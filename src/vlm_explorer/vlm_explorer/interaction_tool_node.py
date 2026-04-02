#!/usr/bin/env python3
"""Placeholder interaction tool executor for semantic artifacts."""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class InteractionToolNode(Node):
    def __init__(self):
        super().__init__("interaction_tool")

        self.declare_parameter("requests_topic", "/vlm/tool_requests")
        self.declare_parameter("status_topic", "/vlm/tool_status")
        self.declare_parameter("mode", "placeholder")
        self.declare_parameter("response_delay_sec", 0.0)

        self._mode = str(self.get_parameter("mode").value).strip().lower() or "placeholder"
        self._delay = max(0.0, float(self.get_parameter("response_delay_sec").value))
        self._status_pub = self.create_publisher(String, self.get_parameter("status_topic").value, 10)
        self.create_subscription(String, self.get_parameter("requests_topic").value, self._on_request, 10)
        self.get_logger().info(f"InteractionTool: mode={self._mode} delay={self._delay:.2f}s")

    def _on_request(self, msg: String):
        try:
            request = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("InteractionTool: invalid request JSON")
            return

        if self._delay > 0.0:
            time.sleep(self._delay)

        status = {
            "tool": request.get("tool", ""),
            "robot": request.get("robot", ""),
            "artifact_id": request.get("artifact_id", ""),
            "status": "placeholder_ok" if self._mode == "placeholder" else "unhandled",
            "message": request.get("reason", ""),
            "stamp_sec": self.get_clock().now().nanoseconds / 1e9,
        }
        self.get_logger().info(
            "InteractionTool request | "
            f"tool={status['tool']} robot={status['robot']} artifact={status['artifact_id']}"
        )
        out = String()
        out.data = json.dumps(status)
        self._status_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = InteractionToolNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
