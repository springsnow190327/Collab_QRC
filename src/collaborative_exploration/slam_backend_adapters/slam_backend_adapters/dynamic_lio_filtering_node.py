from __future__ import annotations

import json
from typing import Iterable

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

from dynamic_scene_filter.temporal_voxel_filter import (
    DynamicFilterParams,
    Point3,
    TemporalVoxelFilter,
)


def _stamp_to_sec(msg) -> float:
    return float(msg.sec) + float(msg.nanosec) * 1e-9


def _cloud_points(msg: PointCloud2) -> list[Point3]:
    return [
        (float(p[0]), float(p[1]), float(p[2]))
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    ]


class DynamicLioFilteringNode(Node):
    """Dynamic-LIO integration facade.

    The node never publishes odometry. It either forwards Dynamic-LIO
    static/dynamic clouds from an external wrapper or uses the existing
    temporal voxel filter as an explicit fallback.
    """

    def __init__(self) -> None:
        super().__init__("dynamic_lio_filtering_node")
        self.declare_parameter("namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("dynamic_filter_backend", "temporal_voxel_fallback")
        self.declare_parameter("input_cloud_topic", "cloud_registered_body")
        self.declare_parameter("wrapper_static_topic", "dynamic_lio/cloud_static")
        self.declare_parameter("wrapper_dynamic_topic", "dynamic_lio/cloud_dynamic")
        self.declare_parameter("dynamic_voxel_size", 0.25)
        self.declare_parameter("dynamic_static_min_observations", 3)
        self.declare_parameter("dynamic_static_min_lifetime_sec", 2.0)
        self.declare_parameter("dynamic_decay_time_sec", 5.0)
        self.declare_parameter("dynamic_obstacle_ttl_sec", 2.0)
        self.declare_parameter("dynamic_max_static_velocity", 0.15)
        self.declare_parameter("dynamic_min_dynamic_velocity", 0.35)
        self.declare_parameter("dynamic_near_robot_ignore_radius", 0.4)

        raw_namespaces = self.get_parameter("namespaces").value
        self.namespaces = [str(ns).strip().strip("/") for ns in raw_namespaces if str(ns).strip()]
        self.backend = str(self.get_parameter("dynamic_filter_backend").value).strip()
        self.input_cloud_topic = str(self.get_parameter("input_cloud_topic").value).strip().strip("/")
        self.wrapper_static_topic = str(self.get_parameter("wrapper_static_topic").value).strip().strip("/")
        self.wrapper_dynamic_topic = str(self.get_parameter("wrapper_dynamic_topic").value).strip().strip("/")
        params = DynamicFilterParams(
            voxel_size=float(self.get_parameter("dynamic_voxel_size").value),
            static_min_observations=int(self.get_parameter("dynamic_static_min_observations").value),
            static_min_lifetime_sec=float(self.get_parameter("dynamic_static_min_lifetime_sec").value),
            decay_time_sec=float(self.get_parameter("dynamic_decay_time_sec").value),
            dynamic_obstacle_ttl_sec=float(self.get_parameter("dynamic_obstacle_ttl_sec").value),
            max_static_velocity=float(self.get_parameter("dynamic_max_static_velocity").value),
            min_dynamic_velocity=float(self.get_parameter("dynamic_min_dynamic_velocity").value),
            near_robot_ignore_radius=float(self.get_parameter("dynamic_near_robot_ignore_radius").value),
        )
        self.filters = {ns: TemporalVoxelFilter(params) for ns in self.namespaces}
        self.latest_counts = {
            ns: {"static": 0, "dynamic": 0, "ratio": 0.0, "wrapper_static": 0, "wrapper_dynamic": 0}
            for ns in self.namespaces
        }
        self.static_pubs = {
            ns: self.create_publisher(PointCloud2, f"/{ns}/cloud_static", 5) for ns in self.namespaces
        }
        self.dynamic_pubs = {
            ns: self.create_publisher(PointCloud2, f"/{ns}/cloud_dynamic", 5) for ns in self.namespaces
        }
        self.metrics_pub = self.create_publisher(String, "/team_slam/dynamic_filter_metrics", 10)
        for ns in self.namespaces:
            if self.backend == "temporal_voxel_fallback":
                self.create_subscription(
                    PointCloud2,
                    f"/{ns}/{self.input_cloud_topic}",
                    lambda msg, n=ns: self._on_raw_cloud(n, msg),
                    5,
                )
            elif self.backend in {"dynamic_lio_port", "dynamic_lio_wrapper"}:
                self.create_subscription(
                    PointCloud2,
                    f"/{ns}/{self.wrapper_static_topic}",
                    lambda msg, n=ns: self._forward_static(n, msg),
                    5,
                )
                self.create_subscription(
                    PointCloud2,
                    f"/{ns}/{self.wrapper_dynamic_topic}",
                    lambda msg, n=ns: self._forward_dynamic(n, msg),
                    5,
                )
        self.create_timer(1.0, self._publish_summary_metrics)
        self.get_logger().info(
            f"dynamic_lio_filtering_node up backend={self.backend} robots={self.namespaces}"
        )

    def _publish_cloud(self, pub, header: Header, points: Iterable[Point3]) -> None:
        out_header = Header()
        out_header.stamp = header.stamp
        out_header.frame_id = header.frame_id
        pub.publish(point_cloud2.create_cloud_xyz32(out_header, list(points)))

    def _on_raw_cloud(self, ns: str, msg: PointCloud2) -> None:
        stamp_sec = _stamp_to_sec(msg.header.stamp)
        result = self.filters[ns].classify_points(_cloud_points(msg), stamp_sec=stamp_sec)
        self._publish_cloud(self.static_pubs[ns], msg.header, result.static_points)
        self._publish_cloud(self.dynamic_pubs[ns], msg.header, result.dynamic_points)
        self.latest_counts[ns] = {
            "static": result.static_points_kept,
            "dynamic": result.dynamic_points_filtered,
            "ratio": result.dynamic_filter_ratio,
            "wrapper_static": self.latest_counts[ns]["wrapper_static"],
            "wrapper_dynamic": self.latest_counts[ns]["wrapper_dynamic"],
        }
        self._publish_metrics(ns, fallback_used=True, blocker="")

    def _forward_static(self, ns: str, msg: PointCloud2) -> None:
        self.static_pubs[ns].publish(msg)
        self.latest_counts[ns]["wrapper_static"] += 1
        self._publish_metrics(ns, fallback_used=False, blocker="")

    def _forward_dynamic(self, ns: str, msg: PointCloud2) -> None:
        self.dynamic_pubs[ns].publish(msg)
        self.latest_counts[ns]["wrapper_dynamic"] += 1
        self._publish_metrics(ns, fallback_used=False, blocker="")

    def _publish_summary_metrics(self) -> None:
        if self.backend not in {"dynamic_lio_port", "dynamic_lio_wrapper"}:
            return
        for ns, counts in self.latest_counts.items():
            blocker = ""
            if counts["wrapper_static"] == 0 and counts["wrapper_dynamic"] == 0:
                blocker = f"{self.backend}_cloud_outputs_not_received"
            self._publish_metrics(ns, fallback_used=False, blocker=blocker)

    def _publish_metrics(self, ns: str, *, fallback_used: bool, blocker: str) -> None:
        counts = self.latest_counts[ns]
        payload = {
            "schema": "team_dynamic_filter_metrics/v1",
            "robot_id": ns,
            "dynamic_filter_backend": self.backend,
            "dynamic_points_filtered": int(counts["dynamic"]),
            "static_points_kept": int(counts["static"]),
            "dynamic_filter_ratio": round(float(counts["ratio"]), 5),
            "stale_obstacle_decay_time_sec": (
                float(self.get_parameter("dynamic_obstacle_ttl_sec").value)
                if self.backend == "temporal_voxel_fallback" else None
            ),
            "fallback_used": bool(fallback_used),
            "blocker": blocker,
            "gt_used_runtime": False,
        }
        self.metrics_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicLioFilteringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
