#!/usr/bin/env python3
"""Single Go2W VLM-in-the-loop Gazebo exploration launch with Cartographer SLAM.

Composes:
  1. Existing single_go2w_gazebo_cfpa2.launch.py (Gazebo + Go2W + CFPA2 nav)
     - with enable_slam=false so we provide SLAM via Cartographer
     - with goals redirected to a fallback topic
  2. Cartographer 3D SLAM (replaces ground truth odom relay)
  3. VLM explorer layer (skeleton extractor, map renderer, artifact detector,
     placeholder interaction tool, VLM coordinator)
  4. Waypoint mux (VLM override primary, CFPA2 fallback)

Usage:
  ros2 launch vlm_explorer single_vlm_gazebo.launch.py
  ros2 launch vlm_explorer single_vlm_gazebo.launch.py vlm_provider:=xai vlm_model:=grok-2-vision-latest
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _get(context, key: str) -> str:
    return LaunchConfiguration(key).perform(context)


def _launch_setup(context):
    use_sim_time = _as_bool(_get(context, "use_sim_time"))
    vlm_enabled = _as_bool(_get(context, "vlm_enabled"))
    vlm_provider = _get(context, "vlm_provider").strip().lower() or "auto"
    vlm_model = _get(context, "vlm_model").strip()
    vlm_replan_sec = float(_get(context, "vlm_replan_sec"))
    vlm_delay = float(_get(context, "vlm_delay"))
    vlm_goal_timeout_sec = float(_get(context, "vlm_goal_timeout_sec"))
    artifact_detection_mode = _get(context, "artifact_detection_mode").strip().lower() or "placeholder"
    map_backend = _get(context, "map_backend").strip().lower() or "scan"
    gui = _get(context, "gui")
    rviz = _get(context, "rviz")
    robot_ns = _get(context, "robot_namespace").strip().strip("/") or "robot"

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    vlm_pkg = get_package_share_directory("vlm_explorer")

    world = _get(context, "world").strip()
    if not world:
        world = os.path.join(vlm_pkg, "worlds", "vlm_exploration.world")
    octomap_cfg = os.path.join(
        get_package_share_directory("go2_real_bringup"),
        "config",
        "cartographer",
        "octomap_mapping.yaml",
    )

    if map_backend not in {"scan", "carto_binary", "carto_2d", "octomap"}:
        raise ValueError(
            f"Unsupported map_backend '{map_backend}' (expected scan, carto_binary, carto_2d or octomap)"
        )

    tf_remaps = [("/tf", f"/{robot_ns}/tf"), ("/tf_static", f"/{robot_ns}/tf_static")]
    carto_cfg_dir = os.path.join(vlm_pkg, "config")
    carto_cfg_basename = "cartographer_sim_3d.lua"
    if map_backend == "carto_2d":
        carto_cfg_basename = "cartographer_sim_2d.lua"

    actions = []

    # ── 1. Base launch: Gazebo + Go2W + CFPA2 (SLAM disabled) ────────
    #    We disable SLAM so the base launch doesn't start gt_odom_relay.
    #    Cartographer will provide /{ns}/odom/nav instead.
    single_launch = os.path.join(
        go2_gazebo_pkg, "launch", "single_go2w_gazebo_cfpa2.launch.py"
    )
    go2w_config_pkg = get_package_share_directory("go2w_config")
    nav_config_path = os.path.join(go2w_config_pkg, "config", "nav", "default_nav_vlm.yaml")

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(single_launch),
            launch_arguments={
                "robot_namespace": robot_ns,
                "use_sim_time": str(use_sim_time).lower(),
                "gui": gui,
                "rviz": "false",
                "cleanup_stale": "false",
                "enable_slam": "false",  # We provide SLAM via Cartographer
                "use_fast_lio": "false",
                # Mapper: off when Cartographer provides the map; on for scan backend
                "external_mapper": "false" if map_backend == "scan" else "true",
                "broadcast_tf": "false",
                "map_frame": "map",
                # Topic routing: CFPA2 → way_point_coord_base → mux → way_point_coord
                "cfpa2_goal_topic_suffix": "/way_point_coord_base",
                "nav_config": nav_config_path,
                "world": world,
                "spawn_x": _get(context, "spawn_x"),
                "spawn_y": _get(context, "spawn_y"),
                "spawn_yaw": _get(context, "spawn_yaw"),
                "cfpa2_w_ig": _get(context, "cfpa2_w_ig"),
                "cfpa2_w_c": _get(context, "cfpa2_w_c"),
                "cfpa2_w_momentum": _get(context, "cfpa2_w_momentum"),
                "cfpa2_min_utility": _get(context, "cfpa2_min_utility"),
            }.items(),
        )
    )

    # ── RViz with camera + VLM displays ──────────────────────────────
    if _as_bool(rviz):
        rviz_config = os.path.join(vlm_pkg, "rviz", "single_vlm_gazebo.rviz")
        actions.append(
            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2_vlm",
                        arguments=["-d", rviz_config],
                        parameters=[{"use_sim_time": use_sim_time}],
                        remappings=tf_remaps,
                        output="screen",
                    )
                ],
            )
        )

    # ── 2. Cartographer 3D SLAM ──────────────────────────────────────
    # CRITICAL: Start AFTER pose guard finishes to avoid teleportation
    # jumps corrupting the SLAM map.
    #   assets spawn @5s, pose_guard_hold=5s → guard ends @~10s
    #   robot_actions (nav/control) start @16s
    # Start Cartographer only after stand-up has finished:
    #   assets spawn @5s
    #   stand-up command starts @9s
    #   final stand-up pose completes @27s
    # Give a small buffer for residual settling before SLAM starts.
    carto_delay = 30.0

    carto_nodes = []

    # Cartographer node
    carto_nodes.append(
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            namespace=robot_ns,
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "-configuration_directory", carto_cfg_dir,
                "-configuration_basename", carto_cfg_basename,
            ],
            remappings=tf_remaps + [
                ("points2", f"/{robot_ns}/registered_scan_reliable"),
                ("imu", f"/{robot_ns}/imu/data"),
            ],
            output="screen",
        )
    )

    # NOTE:
    # We do not use Cartographer's occupancy grid by default for VLM exploration.
    # In this stack its 3D projected map tends to be "occupied + unknown" with
    # almost no explicit free cells, so CFPA2/default_nav cannot extract
    # frontiers or plan from it reliably. We keep Cartographer for pose/TF, and
    # use simple_scan_mapper_cpp by default to build a proper free/occupied/unknown
    # 2D exploration map from scan_3d + Cartographer pose.
    #
    # carto_2d is the alternative Cartographer-backed nav map path: run
    # Cartographer's 2D trajectory builder directly on the 3D PointCloud2 so the
    # resulting occupancy grid has explicit free-space carving semantics.
    if map_backend in {"carto_binary", "carto_2d"}:
        carto_nodes.append(
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="cartographer_occupancy_grid_node",
                namespace=robot_ns,
                arguments=[
                    f"-occupancy_grid_topic=/{robot_ns}/map_prob",
                    "-resolution=0.05",
                    "-publish_period_sec=0.5",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )
        )
        carto_nodes.append(
            Node(
                package="go2w_perception",
                executable="probability_grid_binarizer.py",
                name="probability_grid_binarizer",
                namespace=robot_ns,
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_topic": f"/{robot_ns}/map_prob"},
                    {"output_topic": f"/{robot_ns}/map"},
                    {"free_threshold": 25},
                    {"occupied_threshold": 50},
                    {"min_occupied_component_cells": 2},
                    {"fill_holes": True},
                    {"hole_neighbor_threshold": 7},
                ],
                output="screen",
            )
        )
    elif map_backend == "octomap":
        carto_nodes.append(
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                name="octomap_server_node",
                namespace=robot_ns,
                parameters=[octomap_cfg, {"use_sim_time": use_sim_time}],
                remappings=tf_remaps + [
                    ("cloud_in", f"/{robot_ns}/registered_scan_reliable"),
                    ("projected_map", f"/{robot_ns}/map"),
                ],
                output="screen",
            )
        )

    # carto_odom_bridge: convert Cartographer TF (map→base_link) to Odometry
    # The nav stack expects /{ns}/odom/nav with frame_id=world, child=base_link
    carto_nodes.append(
        Node(
            package="go2w_perception",
            executable="carto_odom_bridge.py",
            name="carto_odom_bridge",
            namespace=robot_ns,
            parameters=[
                {"use_sim_time": use_sim_time},
                {"parent_frame": "map"},
                {"child_frame": "base_link"},
                {"output_topic": f"/{robot_ns}/odom/nav"},
                {"output_frame_id": "world"},
                {"output_child_frame_id": "base_link"},
                {"rate": 50.0},
            ],
            remappings=tf_remaps,
            output="screen",
        )
    )

    actions.append(TimerAction(period=carto_delay, actions=carto_nodes))

    # ── 3. Waypoint mux: VLM override primary, CFPA2 fallback ────────
    actions.append(
        TimerAction(
            period=max(8.0, carto_delay + 2.0),
            actions=[
                Node(
                    package="go2_gazebo_sim",
                    executable="waypoint_mux.py",
                    name="vlm_waypoint_mux",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"namespaces": [robot_ns]},
                        {"primary_input_suffix": "/vlm_way_point"},
                        {"fallback_input_suffix": "/way_point_coord_base"},
                        {"output_suffix": "/way_point_coord"},
                        {"primary_timeout_sec": vlm_goal_timeout_sec},
                        {"output_rate": 8.0},
                        {"hold_last_output": False},
                        {"stamp_now": True},
                    ],
                    output="screen",
                )
            ],
        )
    )

    # ── 4. VLM nodes — delayed until Cartographer + nav stabilize ────
    map_topic = f"/{robot_ns}/map"
    vlm_nodes = []

    vlm_nodes.append(
        Node(
            package="vlm_explorer",
            executable="skeleton_extractor_node",
            name="skeleton_extractor",
            parameters=[
                {"use_sim_time": use_sim_time},
                {
                    "map_topic": map_topic,
                    "skeleton_marker_topic": "/vlm/skeleton_markers",
                    "skeleton_image_topic": "/vlm/skeleton_image",
                    "frame_id": "world",
                    "rate": 1.0,
                    "free_threshold": 50,
                    "downsample": 2,
                },
            ],
            output="screen",
        )
    )

    vlm_nodes.append(
        Node(
            package="vlm_explorer",
            executable="map_renderer_node",
            name="map_renderer",
            parameters=[
                {"use_sim_time": use_sim_time},
                {
                    "map_topic": map_topic,
                    "robot_namespaces": [robot_ns],
                    "skeleton_image_topic": "/vlm/skeleton_image",
                    "artifact_detections_topic": "/vlm/artifact_detections",
                    "rendered_map_topic": "/vlm/rendered_map",
                    "scene_json_topic": "/vlm/scene_json",
                    "frame_id": "world",
                    "rate": 1.0,
                },
            ],
            output="screen",
        )
    )

    vlm_nodes.append(
        Node(
            package="vlm_explorer",
            executable="artifact_detector_node",
            name="artifact_detector",
            parameters=[
                {"use_sim_time": use_sim_time},
                {
                    "robot_namespaces": [robot_ns],
                    "detections_topic": "/vlm/artifact_detections",
                    "mode": artifact_detection_mode,
                    "rate": 1.0 if artifact_detection_mode == "placeholder" else 2.0,
                    "label": "artifact",
                    "placeholder_detections_json": "[]",
                },
            ],
            output="screen",
        )
    )

    vlm_nodes.append(
        Node(
            package="vlm_explorer",
            executable="interaction_tool_node",
            name="interaction_tool",
            parameters=[
                {"use_sim_time": use_sim_time},
                {
                    "requests_topic": "/vlm/tool_requests",
                    "status_topic": "/vlm/tool_status",
                    "mode": "placeholder",
                    "response_delay_sec": 0.0,
                },
            ],
            output="screen",
        )
    )

    vlm_nodes.append(
        Node(
            package="vlm_explorer",
            executable="vlm_coordinator_node",
            name="vlm_coordinator",
            parameters=[
                {"use_sim_time": use_sim_time},
                {
                    "robot_namespaces": [robot_ns],
                    "rendered_map_topic": "/vlm/rendered_map",
                    "scene_json_topic": "/vlm/scene_json",
                    "artifact_detections_topic": "/vlm/artifact_detections",
                    "tool_requests_topic": "/vlm/tool_requests",
                    "tool_status_topic": "/vlm/tool_status",
                    "goal_topic_suffix": "/vlm_way_point",
                    "frame_id": "world",
                    "replan_period_sec": vlm_replan_sec,
                    "goal_repeat_period_sec": 0.5,
                    "primary_goal_ttl_sec": vlm_goal_timeout_sec,
                    "vlm_enabled": vlm_enabled,
                    "vlm_provider": vlm_provider,
                    "vlm_model": vlm_model,
                    "vlm_temperature": 0.1,
                    "vlm_max_tokens": 768,
                    "vlm_timeout_sec": 15.0,
                    "artifact_reach_radius_m": 1.0,
                },
            ],
            output="screen",
        )
    )

    actions.append(TimerAction(period=vlm_delay, actions=vlm_nodes))

    return actions


def generate_launch_description():
    vlm_pkg = get_package_share_directory("vlm_explorer")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="robot"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "vlm_enabled",
                default_value="true",
                description="Enable low-frequency VLM goal overrides. Baseline explorer remains active via fallback mux.",
            ),
            DeclareLaunchArgument("vlm_provider", default_value="auto"),
            DeclareLaunchArgument("vlm_model", default_value=""),
            DeclareLaunchArgument("vlm_replan_sec", default_value="20.0"),
            DeclareLaunchArgument("vlm_goal_timeout_sec", default_value="2.0"),
            DeclareLaunchArgument(
                "artifact_detection_mode",
                default_value="placeholder",
                description="placeholder or green_hsv",
            ),
            DeclareLaunchArgument(
                "map_backend",
                default_value="scan",
                description="Occupancy backend: scan, carto_binary, carto_2d or octomap",
            ),
            DeclareLaunchArgument(
                "vlm_delay",
                default_value="35.0",
                description="Seconds to wait before starting VLM nodes (let Cartographer + nav stabilize)",
            ),
            DeclareLaunchArgument("spawn_x", default_value="4.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("cfpa2_w_ig", default_value="0.5"),
            DeclareLaunchArgument("cfpa2_w_c", default_value="0.8"),
            DeclareLaunchArgument("cfpa2_w_momentum", default_value="2.5"),
            DeclareLaunchArgument("cfpa2_min_utility", default_value="-1.0"),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(vlm_pkg, "worlds", "vlm_exploration.world"),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
