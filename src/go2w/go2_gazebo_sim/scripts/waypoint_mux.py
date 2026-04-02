#!/usr/bin/env python3
"""Select FAR waypoints when available, otherwise fall back to frontier goals."""

from __future__ import annotations

import copy
from dataclasses import dataclass
from typing import Dict

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time


@dataclass
class TopicState:
    latest: PointStamped | None = None
    stamp: Time | None = None


class WaypointMux(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_mux")

        self.declare_parameter("namespaces", ["go2_1", "go2_2"])
        self.declare_parameter("primary_input_suffix", "/way_point_far")
        self.declare_parameter("fallback_input_suffix", "/goal_point")
        self.declare_parameter("output_suffix", "/way_point_coord")
        self.declare_parameter("primary_timeout_sec", 1.0)
        self.declare_parameter("output_rate", 8.0)
        self.declare_parameter("hold_last_output", True)
        self.declare_parameter("stamp_now", True)

        self.namespaces = [str(x) for x in self.get_parameter("namespaces").value]
        self.primary_input_suffix = str(self.get_parameter("primary_input_suffix").value)
        self.fallback_input_suffix = str(self.get_parameter("fallback_input_suffix").value)
        self.output_suffix = str(self.get_parameter("output_suffix").value)
        self.primary_timeout = Duration(seconds=float(self.get_parameter("primary_timeout_sec").value))
        self.output_rate = max(1.0, float(self.get_parameter("output_rate").value))
        self.hold_last_output = bool(self.get_parameter("hold_last_output").value)
        self.stamp_now = bool(self.get_parameter("stamp_now").value)

        self.primary: Dict[str, TopicState] = {ns: TopicState() for ns in self.namespaces}
        self.fallback: Dict[str, TopicState] = {ns: TopicState() for ns in self.namespaces}
        self.last_output: Dict[str, PointStamped] = {}
        self.last_mode: Dict[str, str] = {ns: "none" for ns in self.namespaces}
        self._pubs = {}

        for ns in self.namespaces:
            primary_topic = f"/{ns}{self.primary_input_suffix}"
            fallback_topic = f"/{ns}{self.fallback_input_suffix}"
            output_topic = f"/{ns}{self.output_suffix}"
            self.create_subscription(
                PointStamped,
                primary_topic,
                lambda msg, n=ns: self._primary_cb(n, msg),
                10,
            )
            self.create_subscription(
                PointStamped,
                fallback_topic,
                lambda msg, n=ns: self._fallback_cb(n, msg),
                10,
            )
            self._pubs[ns] = self.create_publisher(PointStamped, output_topic, 10)

        self.timer = self.create_timer(1.0 / self.output_rate, self._tick)
        self.get_logger().info(
            "Waypoint mux started | "
            f"namespaces={self.namespaces} "
            f"primary={self.primary_input_suffix} "
            f"fallback={self.fallback_input_suffix} "
            f"output={self.output_suffix} "
            f"primary_timeout={self.primary_timeout.nanoseconds / 1e9:.2f}s "
            f"rate={self.output_rate:.1f}Hz"
        )

    def _primary_cb(self, ns: str, msg: PointStamped) -> None:
        self.primary[ns].latest = copy.deepcopy(msg)
        self.primary[ns].stamp = self.get_clock().now()

    def _fallback_cb(self, ns: str, msg: PointStamped) -> None:
        self.fallback[ns].latest = copy.deepcopy(msg)
        self.fallback[ns].stamp = self.get_clock().now()

    def _choose_msg(self, ns: str) -> tuple[PointStamped | None, str]:
        now = self.get_clock().now()
        primary_state = self.primary[ns]
        fallback_state = self.fallback[ns]

        if primary_state.latest is not None and primary_state.stamp is not None:
            if now - primary_state.stamp <= self.primary_timeout:
                return primary_state.latest, "primary"

        if fallback_state.latest is not None:
            return fallback_state.latest, "fallback"

        if self.hold_last_output and ns in self.last_output:
            return self.last_output[ns], "hold"

        return None, "none"

    def _tick(self) -> None:
        now_msg = self.get_clock().now().to_msg()
        for ns in self.namespaces:
            chosen, mode = self._choose_msg(ns)
            if chosen is None:
                continue

            msg = copy.deepcopy(chosen)
            if self.stamp_now:
                msg.header.stamp = now_msg
            self._pubs[ns].publish(msg)
            self.last_output[ns] = msg

            if mode != self.last_mode[ns]:
                self.last_mode[ns] = mode
                self.get_logger().info(f"{ns}: switched waypoint source -> {mode}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
