from slam_backend_adapters.contracts import adapter_contract_for_mode
from slam_backend_adapters.dynamic_lio_filtering_node import DynamicLioFilteringNode
from slam_backend_adapters.erasor_adapter_node import ErasorAdapterNode
from slam_backend_adapters.swarm_lio2_ros2_adapter_node import SwarmLio2Ros2Adapter

import json
from pathlib import Path

from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class CapturePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


class Param:
    def __init__(self, value) -> None:
        self.value = value


def make_cloud(frame_id: str = "robot_a/lidar", points=None):
    header = Header()
    header.frame_id = frame_id
    header.stamp.sec = 12
    header.stamp.nanosec = 345
    return point_cloud2.create_cloud_xyz32(header, points or [(0.0, 0.0, 0.0)])


def test_shadow_contract_keeps_swarm_outputs_isolated() -> None:
    contract = adapter_contract_for_mode("swarm_lio2_shadow", namespace="robot_a")

    assert "/robot_a/swarm_lio2/Odometry" in contract.robot_topics
    assert "/robot_a/swarm_lio2/cloud_static" in contract.robot_topics
    assert "/robot_a/Odometry" not in contract.robot_topics
    assert "/robot_a/corrected_odom" not in contract.robot_topics
    assert "/team_slam/swarm_lio2_metrics" in contract.team_topics
    assert not contract.production_downstream_depends_on_swarm


def test_primary_contract_preserves_fast_lio_topic_surface() -> None:
    contract = adapter_contract_for_mode("swarm_lio2_primary", namespace="robot_b")

    assert "/robot_b/Odometry" in contract.robot_topics
    assert "/robot_b/corrected_odom" in contract.robot_topics
    assert "/robot_b/odom/nav" in contract.robot_topics
    assert "/robot_b/cloud_registered_body" in contract.robot_topics
    assert "/robot_b/cloud_static" in contract.robot_topics
    assert "/robot_b/cloud_dynamic" in contract.robot_topics
    assert "/team_slam/swarm_lio2_relative_transform" in contract.team_topics
    assert "/tf" in contract.team_topics
    assert contract.production_downstream_depends_on_swarm


def test_swarm_lio2_adapter_forwards_synthetic_primary_odometry_contract() -> None:
    node = SwarmLio2Ros2Adapter.__new__(SwarmLio2Ros2Adapter)
    node.odom_pub = CapturePublisher()
    node.corrected_pub = CapturePublisher()
    node.nav_odom_pub = CapturePublisher()
    node.tf_br = None
    node.base_frame = "base_link"
    node.forward_counts = {
        "odometry": 0,
        "cloud_static": 0,
        "cloud_dynamic": 0,
        "cloud_map": 0,
        "mutual_state": 0,
        "relative_transform": 0,
    }
    node.last_seen = {}
    node.output_odom_frame_id = "robot_a/odom"
    node.output_child_frame_id = "robot_a/base_link"

    msg = Odometry()
    msg.header.frame_id = ""
    msg.child_frame_id = ""
    msg.pose.pose.position.x = 1.25
    msg.pose.pose.orientation.w = 1.0

    SwarmLio2Ros2Adapter._on_odom(node, msg)

    assert len(node.odom_pub.messages) == 1
    assert len(node.corrected_pub.messages) == 1
    assert len(node.nav_odom_pub.messages) == 1
    assert node.forward_counts["odometry"] == 1
    assert node.odom_pub.messages[0].header.frame_id == "robot_a/odom"
    assert node.odom_pub.messages[0].child_frame_id == "robot_a/base_link"


def test_swarm_lio2_adapter_prefers_configurable_topic_aliases() -> None:
    node = SwarmLio2Ros2Adapter.__new__(SwarmLio2Ros2Adapter)
    params = {
        "swarm_lio2_odom_topic": "/custom/odom",
        "input_odometry_topic": "/legacy/odom",
        "swarm_lio2_cloud_static_topic": "",
        "input_cloud_static_topic": "/legacy/cloud_static",
    }
    node.get_parameter = lambda name: Param(params.get(name, ""))

    assert (
        SwarmLio2Ros2Adapter._param_topic(
            node, "swarm_lio2_odom_topic", "input_odometry_topic", "/fallback/odom"
        )
        == "/custom/odom"
    )
    assert (
        SwarmLio2Ros2Adapter._param_topic(
            node,
            "swarm_lio2_cloud_static_topic",
            "input_cloud_static_topic",
            "/fallback/cloud_static",
        )
        == "/legacy/cloud_static"
    )


def test_dynamic_lio_wrapper_forwards_clouds_and_metrics_contract() -> None:
    node = DynamicLioFilteringNode.__new__(DynamicLioFilteringNode)
    node.backend = "dynamic_lio_wrapper"
    node.static_pubs = {"robot_a": CapturePublisher()}
    node.dynamic_pubs = {"robot_a": CapturePublisher()}
    node.metrics_pub = CapturePublisher()
    node.latest_counts = {
        "robot_a": {
            "static": 0,
            "dynamic": 0,
            "ratio": 0.0,
            "wrapper_static": 0,
            "wrapper_dynamic": 0,
        }
    }
    node.get_parameter = lambda name: Param(2.0)
    static_cloud = make_cloud(points=[(0.0, 0.0, 0.0)])
    dynamic_cloud = make_cloud(points=[(1.0, 0.0, 0.0)])

    DynamicLioFilteringNode._forward_static(node, "robot_a", static_cloud)
    DynamicLioFilteringNode._forward_dynamic(node, "robot_a", dynamic_cloud)

    assert node.static_pubs["robot_a"].messages == [static_cloud]
    assert node.dynamic_pubs["robot_a"].messages == [dynamic_cloud]
    metrics = json.loads(node.metrics_pub.messages[-1].data)
    assert metrics["schema"] == "team_dynamic_filter_metrics/v1"
    assert metrics["robot_id"] == "robot_a"
    assert metrics["dynamic_filter_backend"] == "dynamic_lio_wrapper"
    assert metrics["fallback_used"] is False
    assert metrics["blocker"] == ""
    assert metrics["gt_used_runtime"] is False


def test_erasor_adapter_exports_synthetic_cleanup_artifacts_and_topics(tmp_path: Path) -> None:
    node = ErasorAdapterNode.__new__(ErasorAdapterNode)
    node.backend = "temporal_voxel_fallback"
    node.trigger_mode = "benchmark"
    node.export_dir = tmp_path
    node.erasor_executable = "erasor"
    node.latest_static = {"robot_a": make_cloud(points=[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])}
    node.latest_dynamic = {"robot_a": make_cloud(points=[(3.0, 0.0, 0.0)])}
    node.cleaned_pub = CapturePublisher()
    node.removed_pub = CapturePublisher()
    node.metrics_pub = CapturePublisher()

    ErasorAdapterNode._run_cleanup(node)

    assert (tmp_path / "initial_naive_map.pcd").exists()
    assert (tmp_path / "dense_global_map.pcd").exists()
    assert (tmp_path / "pcds" / "frame_000000.pcd").exists()
    assert (tmp_path / "poses_lidar2body.csv").exists()
    assert (tmp_path / "cleaned_static_map.pcd").exists()
    assert (tmp_path / "removed_dynamic_points.pcd").exists()
    metrics = json.loads((tmp_path / "erasor_metrics.json").read_text())
    assert metrics["schema"] == "erasor_metrics/v1"
    assert metrics["fallback_used"] is True
    assert metrics["control_loop_blocked"] is False
    assert metrics["gt_used_runtime"] is False
    assert metrics["naive_points"] == 2
    assert metrics["removed_dynamic_points"] == 1
    assert len(node.cleaned_pub.messages) == 1
    assert len(node.removed_pub.messages) == 1
    assert json.loads(node.metrics_pub.messages[-1].data)["cleaned_static_points"] == 2
