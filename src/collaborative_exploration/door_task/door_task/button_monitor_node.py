#!/usr/bin/env python3
"""Pressure-pad button monitor for the door task collaborative variant.

Publishes `/door_task/button_pressed` (Bool): True when ANY robot is
within `press_radius_m` of the button's (x, y) position. The button is
purely a game rule — the physics of the door is unchanged, but the
success checker refuses to grade a trial as PASS unless the button was
pressed while the door was being opened.
"""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


class ButtonMonitorNode(Node):
    def __init__(self):
        super().__init__("button_monitor")

        self.declare_parameter("robot_namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("button_x", 7.5)
        self.declare_parameter("button_y", 3.5)
        self.declare_parameter("press_radius_m", 0.5)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("button_topic", "/door_task/button_pressed")
        self.declare_parameter("latch_on_press", True)

        self._ns = [str(x) for x in self.get_parameter("robot_namespaces").value]
        self._bx = float(self.get_parameter("button_x").value)
        self._by = float(self.get_parameter("button_y").value)
        self._radius = float(self.get_parameter("press_radius_m").value)
        self._latch = bool(self.get_parameter("latch_on_press").value)
        self._ever_pressed = False
        rate = max(1.0, float(self.get_parameter("publish_rate").value))

        self._odoms: dict[str, Odometry] = {}
        for ns in self._ns:
            self.create_subscription(
                Odometry, f"/{ns}/odom/nav",
                lambda msg, _ns=ns: self._on_odom(_ns, msg),
                10,
            )

        self._pub = self.create_publisher(
            Bool, str(self.get_parameter("button_topic").value), 10,
        )
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"ButtonMonitor: pad at ({self._bx:.1f}, {self._by:.1f}), "
            f"radius={self._radius:.2f}m, robots={self._ns}"
        )

    def _on_odom(self, ns: str, msg: Odometry):
        self._odoms[ns] = msg

    def _tick(self):
        pressed = False
        for ns, od in self._odoms.items():
            dx = od.pose.pose.position.x - self._bx
            dy = od.pose.pose.position.y - self._by
            if math.hypot(dx, dy) < self._radius:
                pressed = True
                break
        if pressed and not self._ever_pressed:
            self._ever_pressed = True
            self.get_logger().info(
                f"ButtonMonitor: button pressed for the first time"
                f"{' (latched)' if self._latch else ''}"
            )
        out = pressed or (self._latch and self._ever_pressed)
        self._pub.publish(Bool(data=out))


def main(args=None):
    rclpy.init(args=args)
    node = ButtonMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
