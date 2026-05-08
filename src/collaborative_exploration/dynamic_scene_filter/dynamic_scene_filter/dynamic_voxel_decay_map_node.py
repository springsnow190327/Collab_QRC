from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DynamicVoxelDecayMapNode(Node):
    """Summarize dynamic obstacle TTL health across robots."""

    def __init__(self) -> None:
        super().__init__("dynamic_voxel_decay_map_node")
        self.declare_parameter("metrics_topic", "/team_slam/dynamic_filter_metrics")
        self.declare_parameter("status_topic", "/team_slam/dynamic_filter_decay_status")
        self.latest: dict[str, dict] = {}
        self.create_subscription(String, str(self.get_parameter("metrics_topic").value), self._on_metrics, 20)
        self.pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
        self.create_timer(1.0, self._tick)

    def _on_metrics(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        robot = str(payload.get("robot_id", ""))
        if robot:
            self.latest[robot] = payload

    def _tick(self) -> None:
        dynamic = sum(int(v.get("dynamic_points_filtered", 0) or 0) for v in self.latest.values())
        static = sum(int(v.get("static_points_kept", 0) or 0) for v in self.latest.values())
        total = max(1, dynamic + static)
        payload = {
            "schema": "team_dynamic_filter_decay_status/v1",
            "robots": sorted(self.latest),
            "dynamic_points_filtered": dynamic,
            "static_points_kept": static,
            "dynamic_filter_ratio": round(dynamic / total, 5),
            "gt_used_runtime": False,
        }
        self.pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicVoxelDecayMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
