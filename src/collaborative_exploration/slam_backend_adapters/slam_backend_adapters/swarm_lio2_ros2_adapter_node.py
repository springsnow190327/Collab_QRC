from __future__ import annotations

import json
import math
import copy
import time
from typing import Any

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .contracts import adapter_contract_for_mode


def _stamp_to_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class SwarmLio2Ros2Adapter(Node):
    def __init__(self) -> None:
        super().__init__("swarm_lio2_ros2_adapter_node")
        self.declare_parameter("namespace", "robot_a")
        self.declare_parameter("slam_backend", "swarm_lio2_shadow")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("dynamic_filter_backend", "none")
        self.declare_parameter("swarm_lio2_odom_topic", "")
        self.declare_parameter("swarm_lio2_cloud_static_topic", "")
        self.declare_parameter("swarm_lio2_cloud_map_topic", "")
        self.declare_parameter("swarm_lio2_relative_transform_topic", "")
        self.declare_parameter("input_odometry_topic", "")
        self.declare_parameter("input_cloud_static_topic", "")
        self.declare_parameter("input_cloud_dynamic_topic", "")
        self.declare_parameter("input_cloud_map_topic", "")
        self.declare_parameter("input_mutual_state_topic", "")
        self.declare_parameter("input_relative_transform_topic", "")
        self.declare_parameter("output_odom_frame_id", "")
        self.declare_parameter("output_child_frame_id", "")
        self.declare_parameter("output_map_frame_id", "")
        self.declare_parameter("output_static_cloud_frame_id", "")
        self.declare_parameter("output_map_cloud_frame_id", "")

        self.ns = str(self.get_parameter("namespace").value).strip().strip("/")
        self.mode = str(self.get_parameter("slam_backend").value).strip().lower()
        self.contract = adapter_contract_for_mode(self.mode, namespace=self.ns)
        self.dynamic_filter_backend = str(self.get_parameter("dynamic_filter_backend").value)
        self.base_frame = str(self.get_parameter("base_frame").value).strip() or "base_link"
        self.output_odom_frame_id = self._param_value("output_odom_frame_id", f"{self.ns}/odom")
        self.output_child_frame_id = self._param_value("output_child_frame_id", f"{self.ns}/base_link")
        self.output_map_frame_id = self._param_value("output_map_frame_id", f"{self.ns}/map")
        self.output_static_cloud_frame_id = self._param_value(
            "output_static_cloud_frame_id", f"{self.ns}/base_link"
        )
        self.output_map_cloud_frame_id = self._param_value(
            "output_map_cloud_frame_id", f"{self.ns}/map"
        )
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf and self.mode == "swarm_lio2_primary" else None
        self.last_seen: dict[str, float] = {}
        self.forward_counts: dict[str, int] = {
            "odometry": 0,
            "cloud_static": 0,
            "cloud_dynamic": 0,
            "cloud_map": 0,
            "mutual_state": 0,
            "relative_transform": 0,
        }

        self._make_publishers()
        self._make_subscriptions()
        rate = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish_metrics)
        self.get_logger().info(
            f"swarm_lio2_ros2_adapter_node up ns={self.ns} mode={self.mode}"
        )

    def _param_value(self, name: str, fallback: str) -> str:
        value = str(self.get_parameter(name).value).strip()
        return value if value else fallback

    def _param_topic(self, primary_name: str, legacy_name: str, fallback: str) -> str:
        primary = str(self.get_parameter(primary_name).value).strip()
        if primary:
            return primary
        legacy = str(self.get_parameter(legacy_name).value).strip()
        return legacy if legacy else fallback

    def _make_publishers(self) -> None:
        if self.mode == "swarm_lio2_shadow":
            self.odom_pub = self.create_publisher(Odometry, f"/{self.ns}/swarm_lio2/Odometry", 10)
            self.corrected_pub = None
            self.nav_odom_pub = None
            self.cloud_registered_pub = None
            self.static_pub = self.create_publisher(PointCloud2, f"/{self.ns}/swarm_lio2/cloud_static", 5)
            self.dynamic_pub = None
            self.map_pub = self.create_publisher(PointCloud2, f"/{self.ns}/swarm_lio2/cloud_map", 5)
            self.mutual_pub = self.create_publisher(String, f"/{self.ns}/swarm_lio2/mutual_state", 10)
            self.relative_pub = self.create_publisher(
                TransformStamped, f"/{self.ns}/swarm_lio2/relative_transform", 10
            )
            self.team_relative_pub = None
        else:
            self.odom_pub = self.create_publisher(Odometry, f"/{self.ns}/Odometry", 10)
            self.corrected_pub = self.create_publisher(Odometry, f"/{self.ns}/corrected_odom", 10)
            self.nav_odom_pub = self.create_publisher(Odometry, f"/{self.ns}/odom/nav", 10)
            self.cloud_registered_pub = self.create_publisher(
                PointCloud2, f"/{self.ns}/cloud_registered_body", 5
            )
            self.static_pub = self.create_publisher(PointCloud2, f"/{self.ns}/cloud_static", 5)
            self.dynamic_pub = self.create_publisher(PointCloud2, f"/{self.ns}/cloud_dynamic", 5)
            self.map_pub = None
            self.mutual_pub = None
            self.relative_pub = None
            self.team_relative_pub = self.create_publisher(
                TransformStamped, "/team_slam/swarm_lio2_relative_transform", 10
            )
        self.metrics_pub = self.create_publisher(String, "/team_slam/swarm_lio2_metrics", 10)

    def _make_subscriptions(self) -> None:
        raw_prefix = f"/{self.ns}/swarm_lio2_raw"
        self.create_subscription(
            Odometry,
            self._param_topic(
                "swarm_lio2_odom_topic", "input_odometry_topic", f"{raw_prefix}/Odometry"
            ),
            self._on_odom,
            20,
        )
        self.create_subscription(
            PointCloud2,
            self._param_topic(
                "swarm_lio2_cloud_static_topic",
                "input_cloud_static_topic",
                f"{raw_prefix}/cloud_static",
            ),
            self._on_static_cloud,
            5,
        )
        self.create_subscription(
            PointCloud2,
            self._param_value("input_cloud_dynamic_topic", f"{raw_prefix}/cloud_dynamic"),
            self._on_dynamic_cloud,
            5,
        )
        self.create_subscription(
            PointCloud2,
            self._param_topic(
                "swarm_lio2_cloud_map_topic", "input_cloud_map_topic", f"{raw_prefix}/cloud_map"
            ),
            self._on_map_cloud,
            5,
        )
        self.create_subscription(
            String,
            self._param_value("input_mutual_state_topic", f"{raw_prefix}/mutual_state"),
            self._on_mutual_state,
            10,
        )
        self.create_subscription(
            TransformStamped,
            self._param_topic(
                "swarm_lio2_relative_transform_topic",
                "input_relative_transform_topic",
                f"{raw_prefix}/relative_transform",
            ),
            self._on_relative_transform,
            10,
        )

    def _seen(self, key: str, stamp_sec: float | None = None) -> None:
        self.forward_counts[key] += 1
        self.last_seen[key] = stamp_sec if stamp_sec is not None else time.time()

    def _normalize_odom(self, msg: Odometry) -> Odometry:
        out = copy.deepcopy(msg)
        out.header.frame_id = self.output_odom_frame_id
        out.child_frame_id = self.output_child_frame_id
        return out

    def _normalize_static_cloud(self, msg: PointCloud2) -> PointCloud2:
        out = copy.deepcopy(msg)
        out.header.frame_id = self.output_static_cloud_frame_id
        return out

    def _normalize_map_cloud(self, msg: PointCloud2) -> PointCloud2:
        out = copy.deepcopy(msg)
        out.header.frame_id = self.output_map_cloud_frame_id
        return out

    def _normalize_relative_transform(self, msg: TransformStamped) -> TransformStamped:
        out = copy.deepcopy(msg)
        out.header.frame_id = out.header.frame_id or self.output_map_frame_id
        out.child_frame_id = out.child_frame_id or self.output_child_frame_id
        return out

    def _on_odom(self, msg: Odometry) -> None:
        out = self._normalize_odom(msg)
        self.odom_pub.publish(out)
        if self.corrected_pub is not None:
            self.corrected_pub.publish(out)
        if self.nav_odom_pub is not None:
            self.nav_odom_pub.publish(out)
        if self.tf_br is not None:
            tf = TransformStamped()
            tf.header = out.header
            tf.child_frame_id = out.child_frame_id
            tf.transform.translation.x = out.pose.pose.position.x
            tf.transform.translation.y = out.pose.pose.position.y
            tf.transform.translation.z = out.pose.pose.position.z
            tf.transform.rotation = out.pose.pose.orientation
            self.tf_br.sendTransform(tf)
        self._seen("odometry", _stamp_to_sec(msg.header.stamp))

    def _on_static_cloud(self, msg: PointCloud2) -> None:
        out = self._normalize_static_cloud(msg)
        self.static_pub.publish(out)
        if self.cloud_registered_pub is not None:
            self.cloud_registered_pub.publish(out)
        self._seen("cloud_static", _stamp_to_sec(msg.header.stamp))

    def _on_dynamic_cloud(self, msg: PointCloud2) -> None:
        if self.dynamic_pub is not None:
            self.dynamic_pub.publish(msg)
        self._seen("cloud_dynamic", _stamp_to_sec(msg.header.stamp))

    def _on_map_cloud(self, msg: PointCloud2) -> None:
        out = self._normalize_map_cloud(msg)
        if self.map_pub is not None:
            self.map_pub.publish(out)
        elif self.cloud_registered_pub is not None and self.forward_counts["cloud_static"] == 0:
            self.cloud_registered_pub.publish(out)
        self._seen("cloud_map", _stamp_to_sec(msg.header.stamp))

    def _on_mutual_state(self, msg: String) -> None:
        if self.mutual_pub is not None:
            self.mutual_pub.publish(msg)
        self._seen("mutual_state")

    def _on_relative_transform(self, msg: TransformStamped) -> None:
        out = self._normalize_relative_transform(msg)
        if self.relative_pub is not None:
            self.relative_pub.publish(out)
        if self.team_relative_pub is not None:
            self.team_relative_pub.publish(out)
        self._seen("relative_transform", _stamp_to_sec(msg.header.stamp))

    def _publish_metrics(self) -> None:
        odom_ok = self.forward_counts["odometry"] > 0
        relative_ok = (
            self.forward_counts["relative_transform"] > 0
            or self.forward_counts["mutual_state"] > 0
        )
        payload = {
            "schema": "swarm_lio2_metrics/v1",
            "robot_id": self.ns,
            "slam_backend": self.mode,
            "production_downstream_depends_on_swarm": (
                self.contract.production_downstream_depends_on_swarm
            ),
            "odometry_messages": self.forward_counts["odometry"],
            "cloud_static_messages": self.forward_counts["cloud_static"],
            "cloud_dynamic_messages": self.forward_counts["cloud_dynamic"],
            "cloud_map_messages": self.forward_counts["cloud_map"],
            "mutual_state_messages": self.forward_counts["mutual_state"],
            "relative_transform_messages": self.forward_counts["relative_transform"],
            "swarm_lio2_started": odom_ok,
            "swarm_lio2_odometry_valid": odom_ok,
            "swarm_lio2_relative_state_valid": relative_ok,
            "dynamic_filter_backend": self.dynamic_filter_backend,
            "fallback_used": False,
            "blocker": "" if odom_ok else "swarm_lio2_input_not_received",
            "gt_used_runtime": False,
        }
        self.metrics_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SwarmLio2Ros2Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
