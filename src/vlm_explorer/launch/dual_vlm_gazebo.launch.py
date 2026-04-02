#!/usr/bin/env python3
"""Dual-robot VLM-in-the-loop Gazebo exploration launch.

Composes:
  1. Existing dual_go2w_modular.launch.py (Gazebo + 2x Go2W + CFPA2 nav stack)
  2. VLM explorer layer (skeleton extractor, map renderer, green detector, VLM coordinator)

Usage:
  ros2 launch vlm_explorer dual_vlm_gazebo.launch.py
  ros2 launch vlm_explorer dual_vlm_gazebo.launch.py vlm_enabled:=false gui:=false
  ros2 launch vlm_explorer dual_vlm_gazebo.launch.py vlm_provider:=anthropic vlm_model:=claude-sonnet-4-20250514
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
    vlm_provider = _get(context, "vlm_provider").strip()
    vlm_model = _get(context, "vlm_model").strip()
    vlm_replan_sec = float(_get(context, "vlm_replan_sec"))
    gui = _get(context, "gui")
    rviz = _get(context, "rviz")

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    vlm_pkg = get_package_share_directory("vlm_explorer")

    world = _get(context, "world").strip()
    if not world:
        world = os.path.join(vlm_pkg, "worlds", "vlm_exploration.world")

    actions = []

    # ── 1. Include existing dual Go2W Gazebo launch ──────────────────
    dual_launch = os.path.join(go2_gazebo_pkg, "launch", "dual_go2w_modular.launch.py")
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dual_launch),
            launch_arguments={
                "profile": "coordinated",
                "planner_backend": "cfpa2",
                "use_sim_time": str(use_sim_time).lower(),
                "gui": gui,
                "rviz": rviz,
                "cleanup_stale": "true",
                "use_fast_lio": "false",
                "use_shared_map": "true",
                "shared_map_topic": "/world/map",
                "enable_internal_shared_map_fuser": "true",
                "world": world,
                "robot_a_spawn_x": _get(context, "robot_a_spawn_x"),
                "robot_a_spawn_y": _get(context, "robot_a_spawn_y"),
                "robot_a_spawn_yaw": _get(context, "robot_a_spawn_yaw"),
                "robot_b_spawn_x": _get(context, "robot_b_spawn_x"),
                "robot_b_spawn_y": _get(context, "robot_b_spawn_y"),
                "robot_b_spawn_yaw": _get(context, "robot_b_spawn_yaw"),
            }.items(),
        )
    )

    # ── 2. VLM explorer nodes (delayed to let SLAM/nav stabilize) ────
    ns_a = "robot_a"
    ns_b = "robot_b"
    vlm_nodes = []

    # Skeleton extractor: runs on the fused shared map
    vlm_nodes.append(
        Node(
            package="vlm_explorer",
            executable="skeleton_extractor_node",
            name="skeleton_extractor",
            parameters=[
                {"use_sim_time": use_sim_time},
                {
                    "map_topic": "/world/map",
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

    # Map renderer: annotates the fused map with robot poses, skeleton, green markers
    vlm_nodes.append(
        Node(
            package="vlm_explorer",
            executable="map_renderer_node",
            name="map_renderer",
            parameters=[
                {"use_sim_time": use_sim_time},
                {
                    "map_topic": "/world/map",
                    "robot_namespaces": [ns_a, ns_b],
                    "skeleton_image_topic": "/vlm/skeleton_image",
                    "green_detections_topic": "/vlm/green_detections",
                    "rendered_map_topic": "/vlm/rendered_map",
                    "scene_json_topic": "/vlm/scene_json",
                    "frame_id": "world",
                    "rate": 1.0,
                },
            ],
            output="screen",
        )
    )

    # Green marker detector: HSV-based detection from both robot cameras
    vlm_nodes.append(
        Node(
            package="vlm_explorer",
            executable="green_marker_detector_node",
            name="green_marker_detector",
            parameters=[
                {"use_sim_time": use_sim_time},
                {
                    "robot_namespaces": [ns_a, ns_b],
                    "detections_topic": "/vlm/green_detections",
                    "rate": 2.0,
                    "hsv_h_low": 35,
                    "hsv_h_high": 85,
                    "hsv_s_low": 80,
                    "hsv_v_low": 80,
                    "min_blob_pixels": 50,
                    "assumed_depth_m": 2.0,
                    "camera_hfov_rad": 2.0944,
                    "dedup_radius_m": 0.8,
                },
            ],
            output="screen",
        )
    )

    # VLM coordinator: queries VLM and publishes waypoints
    # NOTE: This publishes to /{ns}/way_point_coord which is the same topic
    # that CFPA2 publishes to. When VLM is enabled, the VLM coordinator's
    # more frequent goal republishing (2 Hz) will effectively override CFPA2.
    # When VLM is disabled, CFPA2 goals pass through normally.
    vlm_nodes.append(
        Node(
            package="vlm_explorer",
            executable="vlm_coordinator_node",
            name="vlm_coordinator",
            parameters=[
                {"use_sim_time": use_sim_time},
                {
                    "robot_namespaces": [ns_a, ns_b],
                    "rendered_map_topic": "/vlm/rendered_map",
                    "scene_json_topic": "/vlm/scene_json",
                    "green_detections_topic": "/vlm/green_detections",
                    "goal_topic_suffix": "/vlm_way_point",
                    "frame_id": "world",
                    "replan_period_sec": vlm_replan_sec,
                    "vlm_enabled": vlm_enabled,
                    "vlm_provider": vlm_provider,
                    "vlm_model": vlm_model,
                    "vlm_temperature": 0.2,
                    "vlm_max_tokens": 1024,
                    "vlm_max_retries": 3,
                    "green_reach_radius_m": 1.0,
                },
            ],
            output="screen",
        )
    )

    # Delay VLM nodes to let the base stack stabilize
    actions.append(TimerAction(period=25.0, actions=vlm_nodes))

    return actions


def generate_launch_description():
    vlm_pkg = get_package_share_directory("vlm_explorer")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "vlm_enabled",
                default_value="true",
                description="Enable VLM queries (needs API key in env). Falls back to dummy planner if no key.",
            ),
            DeclareLaunchArgument("vlm_provider", default_value="openai", description="openai or anthropic"),
            DeclareLaunchArgument("vlm_model", default_value="gpt-4o"),
            DeclareLaunchArgument("vlm_replan_sec", default_value="20.0", description="VLM replanning period"),
            DeclareLaunchArgument("robot_a_spawn_x", default_value="1.0"),
            DeclareLaunchArgument("robot_a_spawn_y", default_value="0.5"),
            DeclareLaunchArgument("robot_a_spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("robot_b_spawn_x", default_value="1.0"),
            DeclareLaunchArgument("robot_b_spawn_y", default_value="-0.5"),
            DeclareLaunchArgument("robot_b_spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(vlm_pkg, "worlds", "vlm_exploration.world"),
                description="Gazebo world file",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
