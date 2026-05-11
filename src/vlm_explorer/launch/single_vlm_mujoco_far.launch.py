#!/usr/bin/env python3
"""Single Go2W VLM-in-the-loop MuJoCo exploration launch for the FAR backend.

Composes:
  1. Existing single_go2w_mujoco_cfpa2.launch.py (MuJoCo + Go2W + CFPA2 nav)
     - with FAR execution backend
     - with enable_slam=false so we provide SLAM via Cartographer
  2. Cartographer 3D SLAM (replaces ground truth odom relay)
  3. VLM explorer layer (skeleton extractor, map renderer, green detector, VLM coordinator)

Usage:
  ros2 launch vlm_explorer single_vlm_mujoco_far.launch.py
  ros2 launch vlm_explorer single_vlm_mujoco_far.launch.py vlm_model:=gpt-4o-mini
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
    florence2_enabled = _as_bool(_get(context, "florence2_enabled"))
    florence2_model_id = _get(context, "florence2_model_id").strip()
    florence2_device = _get(context, "florence2_device").strip()
    florence2_goal_prompt = _get(context, "florence2_goal_prompt").strip()
    florence2_detection_rate = float(_get(context, "florence2_detection_rate"))
    florence2_grounding_rate = float(_get(context, "florence2_grounding_rate"))
    vlm_model = _get(context, "vlm_model").strip()
    mission_prompt = _get(context, "mission_prompt").strip()
    vlm_replan_sec = float(_get(context, "vlm_replan_sec"))
    vlm_goal_timeout_sec = float(_get(context, "vlm_goal_timeout_sec"))
    vlm_delay = float(_get(context, "vlm_delay"))
    slam_stack_start_delay = float(_get(context, "slam_stack_start_delay"))
    skeleton_rate = float(_get(context, "vlm_skeleton_rate"))
    renderer_rate = float(_get(context, "vlm_renderer_rate"))
    green_rate = float(_get(context, "vlm_green_rate"))
    enable_3d_viz = _as_bool(_get(context, "enable_3d_viz"))
    checker_enabled = _as_bool(_get(context, "checker_enabled"))
    checker_deadline_sec = float(_get(context, "checker_deadline_sec"))
    checker_coverage_pass_pct = float(_get(context, "checker_coverage_pass_pct"))
    checker_collision_range_m = float(_get(context, "checker_collision_range_m"))
    checker_use_boundary_roi = _as_bool(_get(context, "checker_use_boundary_roi"))
    checker_fallback_known_bbox_roi = _as_bool(_get(context, "checker_fallback_known_bbox_roi"))
    octomap_filter_min_z = float(_get(context, "octomap_filter_min_z"))
    octomap_filter_max_z = float(_get(context, "octomap_filter_max_z"))
    octomap_filter_min_radius = float(_get(context, "octomap_filter_min_radius"))
    octomap_filter_max_radius = float(_get(context, "octomap_filter_max_radius"))
    octomap_filter_startup_drop_sec = float(_get(context, "octomap_filter_startup_drop_sec"))
    octomap_filter_max_input_age_sec = float(_get(context, "octomap_filter_max_input_age_sec"))
    octomap_filter_max_abs_yaw_rate = float(_get(context, "octomap_filter_max_abs_yaw_rate"))
    octomap_bridge_transform_wait_sec = float(_get(context, "octomap_bridge_transform_wait_sec"))
    octomap_bridge_max_cloud_age_sec = float(_get(context, "octomap_bridge_max_cloud_age_sec"))
    octomap_sensor_max_range = float(_get(context, "octomap_sensor_max_range"))
    slam_source = _get(context, "slam_source").strip().lower() or "cartographer"
    nav_execution_backend = _get(context, "nav_execution_backend").strip().lower() or "far"
    frontier_backend = _get(context, "frontier_backend").strip().lower() or "cfpa2"
    map_backend = _get(context, "map_backend").strip().lower() or "carto_2d"
    gui = _get(context, "gui")
    rviz = _get(context, "rviz")
    robot_ns = _get(context, "robot_namespace").strip().strip("/") or "robot"
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    vlm_pkg = get_package_share_directory("vlm_explorer")

    world = _get(context, "world").strip()
    if not world:
        world = os.path.join(vlm_pkg, "worlds", "vlm_exploration.world")
    if map_backend not in {"carto_binary", "carto_2d", "octomap"}:
        raise ValueError(
            f"Unsupported map_backend '{map_backend}' (expected carto_binary, carto_2d or octomap)"
        )
    if slam_source not in {"cartographer", "fast_lio", "ground_truth"}:
        raise ValueError(
            f"Unsupported slam_source '{slam_source}' (expected cartographer, fast_lio or ground_truth)"
        )
    # Back-compat aliases: the reactive/RRT*/MPPI planners were removed
    # on 2026-04-24. Silently upgrade old names to their replacements.
    if nav_execution_backend == "reactive":
        nav_execution_backend = "default"
    elif nav_execution_backend in {"rrt_star", "far_rrt_star", "mppi"}:
        nav_execution_backend = "astar"
    if nav_execution_backend not in {"far", "astar", "default"}:
        raise ValueError(
            "Unsupported nav_execution_backend "
            f"'{nav_execution_backend}' (expected far, astar or default)"
        )
    if frontier_backend not in {"cfpa2", "simple_frontier"}:
        raise ValueError(
            f"Unsupported frontier_backend '{frontier_backend}' (expected cfpa2 or simple_frontier)"
        )

    tf_remaps = [("/tf", f"/{robot_ns}/tf"), ("/tf_static", f"/{robot_ns}/tf_static")]
    carto_cfg_dir = os.path.join(vlm_pkg, "config")
    carto_cfg_basename = "cartographer_sim_3d.lua"
    if map_backend == "carto_2d":
        carto_cfg_basename = "cartographer_sim_2d.lua"
    mapper_frame = "map" if slam_source == "cartographer" else "world"

    actions = []

    # ── 1. Base launch: MuJoCo + Go2W + CFPA2 ─────────────────────────
    #    cartographer mode: SLAM disabled here; carto_odom_bridge provides /{ns}/odom/nav.
    #    fast_lio mode: base launch runs Fast-LIO + slam_odom_relay.
    #    ground_truth mode: base launch runs gt_odom_relay directly from p3d GT.
    single_launch = os.path.join(go2_gazebo_pkg, "launch", "single_go2w_mujoco_cfpa2.launch.py")
    go2w_config_pkg = get_package_share_directory("go2w_config")
    # 2026-05-09: default_nav_vlm_far.yaml deleted along with default_nav.py /
    # astar_nav_node. VLM Phase 1 only ships a `far` backend now; reuse the
    # production FAR real config since FAR reads the same params on sim/real.
    nav_config_path = os.path.join(go2w_config_pkg, "config", "nav", "far_planner_real.yaml")

    # FAR terrain analysis needs map-frame 3D cloud.
    # - Fast-LIO: registered_scan_reliable is already in map frame.
    # - Cartographer/GT: cloud is in livox_mid360 frame; we add a frame bridge below.
    if slam_source == "fast_lio":
        far_scan_topic = f"/{robot_ns}/registered_scan_reliable"
    else:
        far_scan_topic = f"/{robot_ns}/registered_scan_map"

    # MuJoCo MJCF model path: use the VLM exploration world scene
    mujoco_model_path = _get(context, "mujoco_model_path").strip()
    if not mujoco_model_path:
        mujoco_model_path = os.path.join(
            get_package_share_directory("go2_gazebo_sim"), "mujoco", "vlm_exploration_scene.xml"
        )

    base_args = {
        "robot_namespace": robot_ns,
        "use_sim_time": str(use_sim_time).lower(),
        "gui": gui,
        "rviz": "false",
        "cleanup_stale": "false",
        "enable_slam": "false" if slam_source == "cartographer" else "true",
        "use_fast_lio": "true" if slam_source == "fast_lio" else "false",
        # Cartographer owns the full map→odom→base_link TF chain
        # (provide_odom_frame=true).  Odom bridge TF disabled to prevent
        # dual-parent conflict on base_link.
        "odom_bridge_publish_tf": "false",
        "map_frame": mapper_frame,
        # Topic routing: CFPA2 → way_point_coord_base → mux → way_point_coord
        "cfpa2_goal_topic_suffix": "/way_point_coord_base",
        "cfpa2_switch_hysteresis": "0.06",
        "nav_config": nav_config_path,
        "world": world,
        "mujoco_model_path": mujoco_model_path,
        "spawn_x": _get(context, "spawn_x"),
        "spawn_y": _get(context, "spawn_y"),
        "spawn_yaw": _get(context, "spawn_yaw"),
        "cfpa2_w_ig": _get(context, "cfpa2_w_ig"),
        "cfpa2_w_c": _get(context, "cfpa2_w_c"),
        "cfpa2_w_momentum": _get(context, "cfpa2_w_momentum"),
        "cfpa2_min_utility": _get(context, "cfpa2_min_utility"),
        # nav backend pass-through (already normalised above)
        "nav_backend": nav_execution_backend,
        "registered_scan_topic": far_scan_topic,
        "far_max_speed": "0.5",
        "far_robot_id": "0",
    }

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(single_launch),
            launch_arguments=base_args.items(),
        )
    )

    # In non-Cartographer modes mapper publishes /robot/map in "world". Keep
    # RViz's fixed frame "map" valid by providing an identity world->map link.
    if mapper_frame == "world":
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_map_identity",
                arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
                remappings=[("/tf_static", f"/{robot_ns}/tf_static")],
                output="screen",
            )
        )

    # ── RViz with camera + VLM displays ──────────────────────────────
    if _as_bool(rviz):
        rviz_config = os.path.join(vlm_pkg, "rviz", "single_vlm_mujoco.rviz")
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

    # ── 2. SLAM / odom provider ──────────────────────────────────────
    # Deterministic startup: wait for stand-up completion signal.

    carto_nodes = []

    if slam_source == "cartographer":
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

        # Cartographer occupancy grid: publishes probabilistic map, then binarizer
        # converts to standard free/occupied/unknown for CFPA2 and nav.
        if map_backend in {"carto_binary", "carto_2d"}:
            carto_nodes.append(
                Node(
                    package="cartographer_ros",
                    executable="cartographer_occupancy_grid_node",
                    name="cartographer_occupancy_grid_node",
                    namespace=robot_ns,
                    remappings=[("map", "map_prob")],
                    arguments=[
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
                        {"free_threshold": 49},
                        {"occupied_threshold": 65},
                        {"min_occupied_component_cells": 2},
                        {"fill_holes": True},
                        {"hole_neighbor_threshold": 7},
                    ],
                    output="screen",
                )
            )
        elif map_backend == "octomap":
            # Re-express the cloud in the physical LiDAR frame before OctoMap.
            # OctoMap uses cloud header.frame_id as BOTH the point transform frame
            # and the sensor ray origin frame. If cloud_in is base_link, rays are
            # cast from base center (wrong) instead of the LiDAR mount, creating
            # radial spoke artifacts in projected 2D maps.
            octomap_cloud_topic = f"/{robot_ns}/registered_scan_sensor"
            octomap_filtered_topic = f"/{robot_ns}/registered_scan_octomap"
            carto_nodes.append(
                Node(
                    package="go2w_perception",
                    executable="pointcloud_frame_bridge.py",
                    name="octomap_cloud_frame_bridge",
                    namespace=robot_ns,
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"input_topic": f"/{robot_ns}/registered_scan_reliable"},
                        {"output_topic": octomap_cloud_topic},
                        {"target_frame": "livox_mid360"},
                        {"tf_timeout_sec": 0.05},
                        {"transform_wait_sec": octomap_bridge_transform_wait_sec},
                        {"max_cloud_age_sec": octomap_bridge_max_cloud_age_sec},
                    ],
                    remappings=tf_remaps,
                    output="screen",
                )
            )
            # Filter in sensor frame before OctoMap insertion.
            # This rejects floor/self points that create radial spokes in 2D
            # projection when the platform has pitch/roll during motion.
            carto_nodes.append(
                Node(
                    package="go2w_perception",
                    executable="pointcloud_octomap_filter.py",
                    name="octomap_cloud_filter",
                    namespace=robot_ns,
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"input_topic": octomap_cloud_topic},
                        {"output_topic": octomap_filtered_topic},
                        {"min_z": octomap_filter_min_z},
                        {"max_z": octomap_filter_max_z},
                        {"min_radius": octomap_filter_min_radius},
                        {"max_radius": octomap_filter_max_radius},
                        {"startup_drop_sec": octomap_filter_startup_drop_sec},
                        {"max_input_age_sec": octomap_filter_max_input_age_sec},
                        {"drop_out_of_order": True},
                        {"odom_topic": f"/{robot_ns}/odom/nav"},
                        {"max_abs_yaw_rate": octomap_filter_max_abs_yaw_rate},
                    ],
                    output="screen",
                )
            )

            # Feed OctoMap the original sensor-frame cloud (livox_mid360).
            # OctoMap uses cloud->header.frame_id to determine BOTH the point
            # transform AND the sensor origin for raycasting via a single
            # lookupTransform(world_frame, cloud_frame, stamp).  Pre-transforming
            # to "map" frame makes frame_id="map", so the sensor origin becomes
            # (0,0,0) — completely wrong.  OctoMap has a built-in 1 s TF wait,
            # so timing is handled internally.
            carto_nodes.append(
                Node(
                    package="octomap_server",
                    executable="octomap_server_node",
                    name="octomap_server_node",
                    namespace=robot_ns,
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "resolution": 0.05,
                        "frame_id": "map",
                        "base_frame_id": "base_link",
                        "sensor_model.max_range": octomap_sensor_max_range,
                        "sensor_model.hit": 0.65,
                        "sensor_model.miss": 0.30,
                        "sensor_model.min": 0.12,
                        "sensor_model.max": 0.97,
                        # Cartographer's map frame can be pitch/roll-tilted during
                        # bringup, so keep cloud insertion Z wide. Constrain only the
                        # 2D projection band used for /map.
                        "point_cloud_min_z": -0.8,
                        "point_cloud_max_z": 1.4,
                        # Suppress the near-robot ground ring by ignoring low-Z voxels
                        # in the projected 2D occupancy map.
                        "occupancy_min_z": -0.10,
                        "occupancy_max_z": 1.20,
                        # Ground rejection is handled explicitly in the
                        # sensor-frame octomap_cloud_filter stage above.
                        "filter_ground_plane": False,
                        # Incremental projection can stick at all-unknown in this
                        # Cartographer+Gazebo path; use full projection updates.
                        "incremental_2D_projection": False,
                        "publish_free_space": True,
                        "use_height_map": False,
                        "compress_map": True,
                        "latch": False,
                        "filter_speckles": True,
                    }],
                    remappings=tf_remaps + [
                        ("cloud_in", octomap_filtered_topic),
                        ("projected_map", f"/{robot_ns}/map"),
                    ],
                    output="screen",
                )
            )

        # pointcloud_frame_bridge: transform registered_scan from livox_mid360 to map
        # frame for FAR terrain analysis (terrainAnalysis/localPlanner assume map-frame input).
        if nav_execution_backend == "far":
            carto_nodes.append(
                Node(
                    package="go2w_perception",
                    executable="pointcloud_frame_bridge.py",
                    name="registered_scan_frame_bridge",
                    namespace=robot_ns,
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"input_topic": f"/{robot_ns}/registered_scan_reliable"},
                        {"output_topic": f"/{robot_ns}/registered_scan_map"},
                        {"target_frame": "map"},
                        {"tf_timeout_sec": 0.15},
                        {"transform_wait_sec": 0.10},
                        {"max_cloud_age_sec": 0.80},
                    ],
                    remappings=tf_remaps,
                    output="screen",
                )
            )

        # carto_odom_bridge: convert Cartographer TF (map→base_link) to Odometry
        # Publish odom in the mapper frame so downstream visualization and planning
        # are all expressed in the same frame as the exploration map.
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
                    {"output_frame_id": "map"},
                    {"output_child_frame_id": "base_link"},
                    {"rate": 50.0},
                ],
                remappings=tf_remaps,
                output="screen",
            )
        )

        # Avoid shell `ros2 topic echo --once` waits: they are brittle across
        # repeated bringup/teardown and can miss one-shot events, leaving
        # Cartographer never starting. Start SLAM stack deterministically.
        actions.append(
            TimerAction(
                period=max(0.0, slam_stack_start_delay),
                actions=carto_nodes,
            )
        )

    # All robot namespaces for shared VLM nodes
    all_robot_ns = [robot_ns]

    # ── 2b. OctoMap 3D visualization (parallel, viz-only) ─────────
    #   Runs a separate OctoMap server on the sensor-frame cloud for 3D
    #   RViz visualization while the 2D pipeline handles nav.
    if enable_3d_viz and map_backend != "octomap":
        viz_cloud_topic = f"/{robot_ns}/registered_scan_reliable"
        actions.append(
            TimerAction(
                period=max(0.0, slam_stack_start_delay) + 5.0,
                actions=[
                    Node(
                        package="octomap_server",
                        executable="octomap_server_node",
                        name="octomap_3d_viz",
                        namespace=robot_ns,
                        parameters=[{
                            "use_sim_time": use_sim_time,
                            "resolution": 0.08,
                            "frame_id": mapper_frame,
                            "base_frame_id": "base_link",
                            "sensor_model.max_range": 8.0,
                            "sensor_model.hit": 0.65,
                            "sensor_model.miss": 0.35,
                            "sensor_model.min": 0.12,
                            "sensor_model.max": 0.97,
                            "point_cloud_min_z": -0.5,
                            "point_cloud_max_z": 1.5,
                            "occupancy_min_z": -0.10,
                            "occupancy_max_z": 1.20,
                            "filter_ground_plane": False,
                            "incremental_2D_projection": False,
                            "publish_free_space": True,
                            "use_height_map": False,
                            "compress_map": True,
                            "latch": False,
                            "filter_speckles": True,
                        }],
                        remappings=tf_remaps + [
                            ("cloud_in", viz_cloud_topic),
                            ("projected_map", f"/{robot_ns}/octomap_viz/projected_map"),
                        ],
                        output="screen",
                    )
                ],
            )
        )

    # ── 3. Waypoint mux: VLM override primary, CFPA2 fallback ────────
    actions.append(
        TimerAction(
            period=8.0,
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
                    "frame_id": mapper_frame,
                    "rate": max(0.1, skeleton_rate),
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
                    "robot_namespaces": all_robot_ns,
                    "skeleton_image_topic": "/vlm/skeleton_image",
                    "green_detections_topic": "/vlm/green_detections",
                    "rendered_map_topic": "/vlm/rendered_map",
                    "scene_json_topic": "/vlm/scene_json",
                    "frame_id": mapper_frame,
                    "rate": max(0.1, renderer_rate),
                },
            ],
            output="screen",
        )
    )

    vlm_nodes.append(
        Node(
            package="vlm_explorer",
            executable="green_marker_detector_node",
            name="green_marker_detector",
            parameters=[
                {"use_sim_time": use_sim_time},
                {
                    "robot_namespaces": all_robot_ns,
                    "detections_topic": "/vlm/green_detections",
                    "rate": max(0.1, green_rate),
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

    vlm_nodes.append(
        Node(
            package="vlm_explorer",
            executable="red_block_detector_node",
            name="red_block_detector",
            parameters=[
                {"use_sim_time": use_sim_time},
                {
                    "robot_namespaces": all_robot_ns,
                    "detections_topic": "/vlm/artifact_detections",
                    "rate": 2.0,
                    "hsv_h_low1": 0,
                    "hsv_h_high1": 10,
                    "hsv_h_low2": 170,
                    "hsv_h_high2": 180,
                    "hsv_s_low": 100,
                    "hsv_v_low": 80,
                    "min_blob_pixels": 50,
                    "assumed_depth_m": 2.0,
                    "camera_hfov_rad": 2.0944,
                    "dedup_radius_m": 0.8,
                    "marker_topic": "/vlm/artifact_markers",
                    "marker_frame_id": mapper_frame,
                },
            ],
            output="screen",
        )
    )

    # Ground truth artifact scorer
    vlm_nodes.append(
        Node(
            package="vlm_explorer",
            executable="artifact_scorer_node",
            name="artifact_scorer",
            parameters=[
                {"use_sim_time": use_sim_time},
                {
                    "detections_topic": "/vlm/artifact_detections",
                    "score_topic": "/vlm/artifact_score",
                    "gt_marker_topic": "/vlm/artifact_gt_markers",
                    "marker_frame_id": mapper_frame,
                    "match_radius_m": 1.5,
                    "rate": 0.5,
                },
            ],
            output="screen",
        )
    )

    if florence2_enabled:
        vlm_nodes.append(
            Node(
                package="vlm_explorer",
                executable="florence2_detector_node",
                name="florence2_detector",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {
                        "robot_namespaces": all_robot_ns,
                        "detections_topic": "/vlm/artifact_detections",
                        "descriptions_topic": "/vlm/scene_descriptions",
                        "goal_prompt": florence2_goal_prompt,
                        "model_id": florence2_model_id,
                        "device": florence2_device,
                        "detection_rate": florence2_detection_rate,
                        "grounding_rate": florence2_grounding_rate,
                        "description_cooldown_sec": 10.0,
                        "grounding_confidence_threshold": 0.3,
                        "assumed_depth_m": 2.0,
                        "camera_hfov_rad": 2.0944,
                        "dedup_radius_m": 0.8,
                        "min_bbox_area_frac": 0.002,
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
                    "robot_namespaces": all_robot_ns,
                    "rendered_map_topic": "/vlm/rendered_map",
                    "scene_json_topic": "/vlm/scene_json",
                    "green_detections_topic": "/vlm/green_detections",
                    "goal_topic_suffix": "/vlm_way_point",
                    "frame_id": mapper_frame,
                    "replan_period_sec": vlm_replan_sec,
                    "vlm_enabled": vlm_enabled,
                    "vlm_model": vlm_model,
                    "vlm_temperature": 0.2,
                    "vlm_max_tokens": 1024,
                    "vlm_max_retries": 3,
                    "green_reach_radius_m": 1.0,
                    "mission_prompt": mission_prompt,
                },
            ],
            output="screen",
        )
    )

    # Goal flow is runtime-gated by autonomy_enabler.
    # Low-overhead mode: when vlm_enabled=false, keep CFPA2 exploration
    # path active and skip VLM perception/planning nodes.
    if vlm_enabled:
        actions.append(TimerAction(period=max(0.0, vlm_delay), actions=vlm_nodes))

    if checker_enabled:
        # Guard: exploration_pass_fail_checker.py may not be installed yet
        obs_pkg_dir = get_package_share_directory("go2w_observability")
        checker_exe = os.path.join(
            os.path.dirname(obs_pkg_dir), "..", "lib", "go2w_observability",
            "exploration_pass_fail_checker.py"
        )
        if os.path.isfile(checker_exe):
            actions.append(
                TimerAction(
                    period=10.0,
                    actions=[
                        Node(
                            package="go2w_observability",
                            executable="exploration_pass_fail_checker.py",
                            name="exploration_pass_fail_checker",
                            namespace=robot_ns,
                            parameters=[
                                {"use_sim_time": use_sim_time},
                                {"robot_namespace": robot_ns},
                                {"map_topic": f"/{robot_ns}/map"},
                                {"stop_topic": f"/{robot_ns}/stop"},
                                {"scan_topic": f"/{robot_ns}/scan_3d"},
                                {"boundary_topic": f"/{robot_ns}/navigation_boundary"},
                                {"pass_topic": f"/{robot_ns}/exploration_pass"},
                                {"summary_topic": f"/{robot_ns}/exploration_pass_summary"},
                                {"deadline_sec": checker_deadline_sec},
                                {"coverage_pass_pct": checker_coverage_pass_pct},
                                {"collision_range_m": checker_collision_range_m},
                                {"count_stop_events": False},
                                {"coverage_occ_counts": True},
                                {"use_boundary_roi": checker_use_boundary_roi},
                                {"fallback_known_bbox_roi": checker_fallback_known_bbox_roi},
                                {"start_on_first_map": True},
                            ],
                            output="screen",
                        )
                    ],
                )
            )

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
                "slam_source",
                default_value="cartographer",
                description="SLAM / odom provider: cartographer, fast_lio, or ground_truth",
            ),
            DeclareLaunchArgument(
                "vlm_enabled",
                default_value="true",
                description="Enable VLM queries (needs OPENAI_API_KEY). Falls back to dummy planner if unset.",
            ),
            DeclareLaunchArgument("vlm_model", default_value="gpt-4o"),
            DeclareLaunchArgument(
                "florence2_enabled",
                default_value="false",
                description="Enable Florence-2 local visual detector (requires ~1.5GB VRAM).",
            ),
            DeclareLaunchArgument(
                "florence2_model_id",
                default_value="microsoft/Florence-2-large",
                description="HuggingFace model ID for Florence-2.",
            ),
            DeclareLaunchArgument(
                "florence2_device",
                default_value="cuda",
                description="Torch device for Florence-2 (cuda or cpu).",
            ),
            DeclareLaunchArgument(
                "florence2_goal_prompt",
                default_value="",
                description="Goal object description for phrase grounding (e.g. 'red fire extinguisher').",
            ),
            DeclareLaunchArgument("florence2_detection_rate", default_value="2.0"),
            DeclareLaunchArgument("florence2_grounding_rate", default_value="1.0"),
            DeclareLaunchArgument(
                "mission_prompt",
                default_value="",
                description="Vague user mission for VLM coordinator (e.g. 'find small colored objects').",
            ),
            DeclareLaunchArgument("vlm_replan_sec", default_value="10.0"),
            DeclareLaunchArgument("vlm_goal_timeout_sec", default_value="2.0"),
            DeclareLaunchArgument(
                "nav_execution_backend",
                default_value="far",
                description="Navigation execution backend: far | astar | default. "
                            "Legacy aliases reactive/rrt_star/far_rrt_star/mppi are "
                            "silently upgraded for back-compat.",
            ),
            DeclareLaunchArgument(
                "frontier_backend",
                default_value="simple_frontier",
                description="Goal-generation backend: cfpa2 or simple_frontier",
            ),
            DeclareLaunchArgument(
                "map_backend",
                default_value="carto_2d",
                description="Occupancy backend: carto_binary, carto_2d or octomap",
            ),
            DeclareLaunchArgument(
                "vlm_delay",
                default_value="10.0",
                description="Extra delay after SLAM-ready before starting VLM nodes.",
            ),
            DeclareLaunchArgument(
                "slam_stack_start_delay",
                default_value="14.0",
                description="Delay before starting Cartographer/carto_odom_bridge. Must be after CHAMP EKF is publishing odom→base_link TF (~t=7).",
            ),
            DeclareLaunchArgument("vlm_skeleton_rate", default_value="0.5"),
            DeclareLaunchArgument("vlm_renderer_rate", default_value="0.5"),
            DeclareLaunchArgument("vlm_green_rate", default_value="1.0"),
            DeclareLaunchArgument(
                "enable_3d_viz",
                default_value="true",
                description="Run a parallel OctoMap server for 3D RViz visualization (ignored when map_backend=octomap).",
            ),
            DeclareLaunchArgument("checker_enabled", default_value="true"),
            DeclareLaunchArgument("checker_deadline_sec", default_value="220.0"),
            DeclareLaunchArgument("checker_coverage_pass_pct", default_value="95.0"),
            DeclareLaunchArgument("checker_collision_range_m", default_value="0.22"),
            DeclareLaunchArgument(
                "octomap_filter_min_z",
                default_value="0.05",
                description="Sensor-frame min z for octomap cloud prefilter.",
            ),
            DeclareLaunchArgument(
                "octomap_filter_max_z",
                default_value="0.45",
                description="Sensor-frame max z for octomap cloud prefilter.",
            ),
            DeclareLaunchArgument(
                "octomap_filter_min_radius",
                default_value="0.40",
                description="Sensor-frame min XY radius (self-hit rejection) for octomap prefilter.",
            ),
            DeclareLaunchArgument(
                "octomap_filter_max_radius",
                default_value="6.0",
                description="Sensor-frame max XY radius for octomap prefilter.",
            ),
            DeclareLaunchArgument(
                "octomap_filter_startup_drop_sec",
                default_value="8.0",
                description="Drop octomap cloud during startup transients for this many seconds.",
            ),
            DeclareLaunchArgument(
                "octomap_filter_max_input_age_sec",
                default_value="0.20",
                description="Drop octomap clouds older than this age (sim-time seconds).",
            ),
            DeclareLaunchArgument(
                "octomap_filter_max_abs_yaw_rate",
                default_value="0.30",
                description="Drop octomap clouds when |yaw_rate| exceeds this (rad/s).",
            ),
            DeclareLaunchArgument(
                "octomap_bridge_transform_wait_sec",
                default_value="0.06",
                description="TF catch-up wait for octomap frame bridge (seconds).",
            ),
            DeclareLaunchArgument(
                "octomap_bridge_max_cloud_age_sec",
                default_value="0.30",
                description="Drop bridged clouds older than this while waiting for TF.",
            ),
            DeclareLaunchArgument(
                "octomap_sensor_max_range",
                default_value="6.0",
                description="Octomap ray max range (meters).",
            ),
            DeclareLaunchArgument(
                "checker_use_boundary_roi",
                default_value="true",
                description="If true, coverage is measured inside /navigation_boundary ROI.",
            ),
            DeclareLaunchArgument(
                "checker_fallback_known_bbox_roi",
                default_value="true",
                description="If true and ROI unavailable, use known-cell bbox for coverage denominator.",
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
            DeclareLaunchArgument(
                "mujoco_model_path",
                default_value="",
                description="Path to MuJoCo MJCF model (defaults to vlm_exploration.xml).",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
