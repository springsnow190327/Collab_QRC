#!/usr/bin/env python3
"""Pass-through goal assigner.

Bridges frontier goals to an assigned-goal topic while preserving namespace fanout.
Useful for single-robot runs or for debugging centralized assignment behavior.
"""

from __future__ import annotations

import copy
from typing import Any

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class GoalAssignerPassthrough(Node):
    def __init__(self) -> None:
        super().__init__("goal_assigner_passthrough")

        self.declare_parameter("namespaces", ["go2_1", "go2_2"])
        self.declare_parameter("input_topic_suffix", "/way_point_raw")
        self.declare_parameter("output_topic_suffix", "/way_point_assigned")
        self.declare_parameter("publish_rate", 0.0)
        self.declare_parameter("hold_last", True)

        self.namespaces = [str(x) for x in self.get_parameter("namespaces").value]
        self.input_topic_suffix = str(self.get_parameter("input_topic_suffix").value)
        self.output_topic_suffix = str(self.get_parameter("output_topic_suffix").value)
        self.publish_rate = max(0.0, float(self.get_parameter("publish_rate").value))
        self.hold_last = bool(self.get_parameter("hold_last").value)

        self.goal_pubs: dict[str, Any] = {}
        self.latest_goals: dict[str, PointStamped] = {}

        for ns in self.namespaces:
            in_topic = f"/{ns}{self.input_topic_suffix}"
            out_topic = f"/{ns}{self.output_topic_suffix}"
            self.create_subscription(PointStamped, in_topic, lambda msg, n=ns: self._goal_cb(msg, n), 10)
            self.goal_pubs[ns] = self.create_publisher(PointStamped, out_topic, 10)

        if self.publish_rate > 0.0:
            self.timer = self.create_timer(1.0 / self.publish_rate, self._tick)
        else:
            self.timer = None

        self.get_logger().info(
            "Passthrough goal assigner started | "
            f"namespaces={self.namespaces} "
            f"{self.input_topic_suffix} -> {self.output_topic_suffix} "
            f"publish_rate={self.publish_rate:.2f} hold_last={self.hold_last}"
        )

    def _goal_cb(self, msg: PointStamped, ns: str) -> None:
        self.latest_goals[ns] = copy.deepcopy(msg)
        if self.publish_rate <= 0.0:
            self.goal_pubs[ns].publish(msg)

    def _tick(self) -> None:
        for ns, msg in list(self.latest_goals.items()):
            self.goal_pubs[ns].publish(msg)
            if not self.hold_last:
                self.latest_goals.pop(ns, None)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalAssignerPassthrough()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
