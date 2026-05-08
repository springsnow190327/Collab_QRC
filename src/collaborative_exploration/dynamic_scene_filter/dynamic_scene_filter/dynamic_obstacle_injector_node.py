from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class DynamicObstacleInjectorNode(Node):
    """Publish a small moving point cluster for filter validation."""

    def __init__(self) -> None:
        super().__init__("dynamic_obstacle_injector_node")
        self.declare_parameter("topic", "/robot_a/cloud_registered_body")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("speed_mps", 0.6)
        self.pub = self.create_publisher(PointCloud2, str(self.get_parameter("topic").value), 10)
        self.t0 = self.get_clock().now()
        period = 1.0 / max(0.1, float(self.get_parameter("rate_hz").value))
        self.create_timer(period, self._tick)

    def _tick(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.t0).nanoseconds * 1e-9
        speed = float(self.get_parameter("speed_mps").value)
        cx = -1.0 + speed * elapsed
        cy = 0.5 * math.sin(elapsed)
        points = []
        for i in range(40):
            dx = 0.04 * ((i % 8) - 3.5)
            dy = 0.04 * ((i // 8) - 2.0)
            points.append((cx + dx, cy + dy, 0.6))
        header = Header()
        header.stamp = now.to_msg()
        header.frame_id = str(self.get_parameter("frame_id").value)
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, points))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicObstacleInjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
