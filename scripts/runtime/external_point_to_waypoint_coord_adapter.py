#!/usr/bin/env python3
"""Relay an external planner PointStamped waypoint into Collab_QRC's contract.

Common-executor exploration benchmarks keep Nav2 MPPI, SLAM, map merge, robot
dynamics, and safety nodes fixed.  External planners only get to publish a
high-level waypoint.  This node is the narrow adapter from a planner-native
``geometry_msgs/PointStamped`` output to ``/<robot_ns>/way_point_coord``.
"""

from __future__ import annotations

import math
import sys

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class ExternalPointToWaypointCoordAdapter(Node):
    def __init__(self) -> None:
        super().__init__("external_point_to_waypoint_coord_adapter")
        self.declare_parameter("robot_namespace", "robot")
        self.declare_parameter("input_topic", "/robot/external_way_point")
        self.declare_parameter("output_topic", "")
        self.declare_parameter("default_frame_id", "map")
        self.declare_parameter("min_waypoint_separation", 0.20)
        self.declare_parameter("odometry_topic", "")
        self.declare_parameter("min_robot_waypoint_distance", 0.0)
        self.declare_parameter("source_planner", "external")

        ns = str(self.get_parameter("robot_namespace").value).strip().strip("/") or "robot"
        self._input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value).strip()
        self._output_topic = output_topic or f"/{ns}/way_point_coord"
        self._default_frame = str(self.get_parameter("default_frame_id").value).strip() or "map"
        self._min_sep = float(self.get_parameter("min_waypoint_separation").value)
        self._odom_topic = str(self.get_parameter("odometry_topic").value).strip()
        self._min_robot_dist = float(self.get_parameter("min_robot_waypoint_distance").value)
        self._source = str(self.get_parameter("source_planner").value)
        self._last_xy: tuple[float, float] | None = None
        self._robot_xy: tuple[float, float] | None = None

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(PointStamped, self._output_topic, qos)
        self.create_subscription(PointStamped, self._input_topic, self._on_point, qos)
        if self._odom_topic and self._min_robot_dist > 0.0:
            self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.get_logger().info(
            f"{self._source} adapter: {self._input_topic} -> {self._output_topic} "
            f"(min_sep={self._min_sep:.2f}m)")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._robot_xy = (float(p.x), float(p.y))

    def _on_point(self, msg: PointStamped) -> None:
        x = float(msg.point.x)
        y = float(msg.point.y)
        if self._robot_xy is not None and self._min_robot_dist > 0.0:
            if math.hypot(x - self._robot_xy[0], y - self._robot_xy[1]) < self._min_robot_dist:
                self.get_logger().debug(
                    f"ignored near-robot {self._source} waypoint ({x:+.2f}, {y:+.2f})")
                return
        if self._last_xy is not None:
            if math.hypot(x - self._last_xy[0], y - self._last_xy[1]) < self._min_sep:
                return
        self._last_xy = (x, y)

        out = PointStamped()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = self._default_frame
        out.header.stamp = self.get_clock().now().to_msg()
        out.point = msg.point
        self._pub.publish(out)
        self.get_logger().info(
            f"{self._source} waypoint -> ({x:+.2f}, {y:+.2f}, {msg.point.z:+.2f}) "
            f"frame={out.header.frame_id}")


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = ExternalPointToWaypointCoordAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
