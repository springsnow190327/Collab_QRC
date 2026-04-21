#!/usr/bin/env python3
"""Relay RViz '2D Goal Pose' (PoseStamped) to PointStamped for reactive_nav.

Subscribes: /goal_pose (geometry_msgs/PoseStamped)
Publishes:  /{ns}/way_point_coord (geometry_msgs/PointStamped)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped


class RvizGoalRelay(Node):
    def __init__(self):
        super().__init__("rviz_goal_relay")
        self.declare_parameter("output_topic", "way_point_coord")
        out = self.get_parameter("output_topic").get_parameter_value().string_value

        self.pub = self.create_publisher(PointStamped, out, 10)
        self.sub = self.create_subscription(PoseStamped, "/goal_pose", self._cb, 10)
        self.get_logger().info(f"Relaying /goal_pose -> {out}")

    def _cb(self, msg: PoseStamped):
        pt = PointStamped()
        pt.header = msg.header
        pt.point = msg.pose.position
        self.pub.publish(pt)
        self.get_logger().info(
            f"Goal: ({pt.point.x:.2f}, {pt.point.y:.2f})"
        )


def main():
    rclpy.init()
    node = RvizGoalRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
