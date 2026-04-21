#!/usr/bin/env python3
"""Monitors the door hinge angle from MuJoCo pose sensor output.

Subscribes to the DFKI auto-discovered door_panel_pose_sensor (PoseStamped)
and extracts the yaw component as the hinge angle.  Publishes:
  /door_task/door_state  (Float64) — hinge angle in radians [0, pi/2]
  /door_task/door_open   (Bool)    — True when angle > open_threshold
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


def _yaw_from_quat(q) -> float:
    """Extract yaw from geometry_msgs Quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class DoorMonitorNode(Node):
    def __init__(self):
        super().__init__("door_monitor")

        self.declare_parameter("pose_topic", "/mujoco_sim/door_panel_pose_sensor/pose")
        self.declare_parameter("door_state_topic", "/door_task/door_state")
        self.declare_parameter("door_open_topic", "/door_task/door_open")
        self.declare_parameter("open_threshold_rad", 0.5)
        self.declare_parameter("publish_rate", 20.0)

        pose_topic = str(self.get_parameter("pose_topic").value)
        state_topic = str(self.get_parameter("door_state_topic").value)
        open_topic = str(self.get_parameter("door_open_topic").value)
        self._open_thresh = float(self.get_parameter("open_threshold_rad").value)

        self._angle = 0.0
        self._state_pub = self.create_publisher(Float64, state_topic, 10)
        self._open_pub = self.create_publisher(Bool, open_topic, 10)

        self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10)

        rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"DoorMonitor: {pose_topic} -> {state_topic}, threshold={self._open_thresh:.2f} rad"
        )

    def _on_pose(self, msg: PoseStamped):
        self._angle = abs(_yaw_from_quat(msg.pose.orientation))

    def _tick(self):
        self._state_pub.publish(Float64(data=self._angle))
        self._open_pub.publish(Bool(data=self._angle > self._open_thresh))


def main(args=None):
    rclpy.init(args=args)
    node = DoorMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
