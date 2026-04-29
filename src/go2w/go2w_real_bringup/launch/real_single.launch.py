#!/usr/bin/env python3
"""Single-robot real Unitree runtime (Go2W or Go2).

Composes: SLAM → core bringup → navigation (CFPA2|FAR|default|astar) → safety → observability.

robot_model:=go2w  (default) uses go2w_config/nav tuning.
robot_model:=go2           uses go2w_real_bringup/config/nav tuning
                           (tighter footprint, walking-gait speed ceiling).

Usage:
  ros2 launch go2w_real_bringup real_single.launch.py                      # go2w + carto_l1 + cfpa2
  ros2 launch go2w_real_bringup real_single.launch.py robot_model:=go2
  ros2 launch go2w_real_bringup real_single.launch.py slam:=fastlio_mid360
  ros2 launch go2w_real_bringup real_single.launch.py nav_backend:=far
  ros2 launch go2w_real_bringup real_single.launch.py map_backend:=carto_2d
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(text: str) -> bool:
    return str(text).strip().lower() in {"1", "true", "yes", "on"}


def _get(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _launch_setup(context):
    robot_ns = _get(context, "robot_namespace").strip().strip("/") or "robot"
    robot_model = _get(context, "robot_model").strip().lower() or "go2w"
    slam = _get(context, "slam").strip().lower() or "carto_l1"
    carto_mode = _get(context, "carto_mode").strip().lower() or "2d"
    nav_backend = _get(context, "nav_backend").strip().lower() or "nav2_mppi"
    # Back-compat aliases — reactive/mppi/RRT* planners were deleted 2026-04-24.
    # NOTE: bare "mppi" remains an alias to "astar" (legacy), NOT to "nav2_mppi".
    # Use nav_backend="nav2_mppi" (or nav=nav2_mppi via real_autonomy.sh) for
    # the Nav2 + MPPIController + behavior_server stack shipped 2026-04-29.
    if nav_backend == "reactive":
        nav_backend = "default"
    elif nav_backend in ("rrt_star", "far_rrt_star", "mppi"):
        nav_backend = "astar"
    map_backend = _get(context, "map_backend").strip().lower() or "carto_2d"
    obstacle_avoidance = _get(context, "obstacle_avoidance")
    enable_manual_fallback = _get(context, "enable_manual_fallback")
    waypoint_input_suffix = _get(context, "waypoint_input_suffix").strip() or "/way_point_coord"
    if not waypoint_input_suffix.startswith("/"):
        waypoint_input_suffix = "/" + waypoint_input_suffix

    if robot_model not in {"go2w", "go2"}:
        raise ValueError(f"Unsupported robot_model '{robot_model}' (expected go2w or go2)")

    # Consistency: carto_2d mapper needs Cartographer in 2D mode.
    # Other combos are fine (carto_binary + 3d projects 3D submaps to 2D grid).
    if map_backend == "carto_2d" and carto_mode != "2d":
        carto_mode = "2d"

    bringup_share = get_package_share_directory("go2w_real_bringup")
    go2w_config_pkg = get_package_share_directory("go2w_config")
    cfpa2_pkg = get_package_share_directory("cfpa2_collaborative_autonomy")

    slam_launch = os.path.join(bringup_share, "launch", "slam.launch.py")
    core_launch = os.path.join(bringup_share, "launch", "real_bringup_core.launch.py")
    nav_launch = os.path.join(go2w_config_pkg, "launch", "navigation.launch.py")
    safety_launch = os.path.join(go2w_config_pkg, "launch", "safety.launch.py")
    obs_launch = os.path.join(go2w_config_pkg, "launch", "observability.launch.py")

    # When SLAM owns the binary occupancy grid (carto_binary|carto_2d) or we're running
    # Fast-LIO (no occupancy grid at all), navigation must use an external mapper.
    use_external_mapper = map_backend != "scan" or slam == "fastlio_mid360"

    # Nav + FAR configs: Go2W-tuned stays in go2w_config; Go2-tuned ships in real_bringup.
    if robot_model == "go2":
        real_nav_cfg_dir = os.path.join(bringup_share, "config", "nav")
        if nav_backend == "far":
            nav_config = os.path.join(real_nav_cfg_dir, "far_planner_real_go2.yaml")
        else:
            nav_config = os.path.join(real_nav_cfg_dir, "default_nav_real_go2.yaml")
        max_linear_speed = "0.60"
        far_max_speed = "0.40"
    else:  # go2w
        if nav_backend == "far":
            nav_config = os.path.join(go2w_config_pkg, "config", "nav", "far_planner_real.yaml")
        else:
            nav_config = os.path.join(go2w_config_pkg, "config", "nav", "default_nav_single_go2w.yaml")
        max_linear_speed = "0.30"
        far_max_speed = "0.30"

    actions = [
        # ── SLAM (Cartographer + L1  OR  Fast-LIO + Mid360) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                "slam": slam,
                "carto_mode": carto_mode,
            }.items(),
        ),
        # ── Real-only core (transform_everything, bridges, mux, sport) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_launch),
            launch_arguments={
                "robot_namespace": robot_ns,
                "map_backend": map_backend,
                "slam": slam,
                "obstacle_avoidance": obstacle_avoidance,
                "enable_manual_fallback": enable_manual_fallback,
                "run_transform_everything": "true" if slam == "carto_l1" else "false",
                "execute_controller": _get(context, "execute_controller"),
                "manual_timeout_sec": _get(context, "manual_timeout_sec"),
                "auto_timeout_sec": _get(context, "auto_timeout_sec"),
                "manual_linear_threshold": _get(context, "manual_linear_threshold"),
                "manual_angular_threshold": _get(context, "manual_angular_threshold"),
                "joy_dev": _get(context, "joy_dev"),
            }.items(),
        ),
        # ── Planner (default | astar | far) + CFPA2 frontier picker ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch),
            launch_arguments={
                "robot_namespace": robot_ns,
                "robot_model": robot_model,  # nav2_mppi branch picks yaml by this
                "use_sim_time": "false",
                "map_frame": "map",
                "remap_tf": "false",
                "nav_backend": nav_backend,
                "scan_topic": f"/{robot_ns}/scan_3d",
                "odom_topic": f"/{robot_ns}/odom/nav",
                # Caller can override this (e.g. real_single_tare_real.launch.py
                # sets it to /{ns}/registered_scan_map, which a topic_tools
                # relay feeds from Fast-LIO's un-namespaced /cloud_registered).
                "registered_scan_topic": (
                    _get(context, "registered_scan_topic").strip()
                    or "/utlidar/transformed_cloud"
                ),
                "waypoint_input_suffix": waypoint_input_suffix,
                "nav_config": nav_config,
                "cfpa2_config": os.path.join(cfpa2_pkg, "config", "cfpa2_single_robot.yaml"),
                "cfpa2_w_ig": "0.5",
                "cfpa2_w_c": "0.8",
                "cfpa2_w_momentum": "2.5",
                "cfpa2_min_utility": "-1.0",
                "cfpa2_switch_hysteresis": "0.06",
                "max_linear_speed": max_linear_speed,
                "far_max_speed": far_max_speed,
                "require_settle_before_motion": "false",
                "nav_map_topic": f"/{robot_ns}/map",
                # Passthroughs for the real-CMU-TARE path (real_single_tare_real
                # launch sets explore=false and redirects FAR's I/O to leave it
                # idle while TARE publishes directly to /{ns}/way_point).
                "explore": _get(context, "explore"),
                "far_goal_topic": _get(context, "far_goal_topic"),
                "far_way_point_out": _get(context, "far_way_point_out"),
                # Real-robot Fast-LIO is launched un-namespaced (slam.launch.py
                # has no PushRosNamespace around fastlio_mapping), so it
                # publishes /Odometry. Tell the adapter to subscribe absolutely.
                "fast_lio_input_topic": "/Odometry",
                # Real has the legacy static chain map→camera_init→body→base_link
                # (slam.launch.py) plus a new map→odom identity. base_link
                # already has a parent (body); adapter publishing odom→base_link
                # would multi-parent it. Adapter still publishes the topic
                # /robot/odom/nav, just not TF.
                "fast_lio_publish_tf": "false",
            }.items(),
        ),
        # ── Safety ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(safety_launch),
            launch_arguments={
                "robot_namespace": robot_ns,
                "use_sim_time": "false",
                "scan_topic": f"/{robot_ns}/scan_3d",
                "autonomy_startup_delay": "4.0",
            }.items(),
        ),
        # ── Observability ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(obs_launch),
            launch_arguments={
                "robot_namespace": robot_ns,
                "use_sim_time": "false",
                "experiment_name": "real_go2w",
            }.items(),
        ),
    ]

    # ── Mapper selection ──
    # Cartographer path: /robot/map_prob → binarizer → /robot/map (in core launch)
    # Fast-LIO path:     /cloud_registered → octomap_server → /robot/map (here)
    # scan path:         /robot/scan_3d   → simple_scan_mapper_cpp → /robot/map
    if map_backend == "scan" and not use_external_mapper:
        actions.append(
            Node(
                package="go2_nav_algorithms",
                executable="simple_scan_mapper_cpp",
                namespace=robot_ns,
                name="simple_scan_mapper_cpp",
                parameters=[
                    os.path.join(go2w_config_pkg, "config", "nav", "simple_scan_mapper_single_go2w.yaml"),
                    {"use_sim_time": False, "broadcast_tf": False},
                ],
                remappings=[
                    ("scan", f"/{robot_ns}/scan_3d"),
                    ("odom", f"/{robot_ns}/odom/nav"),
                    ("map", f"/{robot_ns}/map"),
                ],
                output="screen",
            )
        )
    elif slam == "fastlio_mid360":
        # Fast-LIO publishes /cloud_registered (world-frame) but no OccupancyGrid.
        # octomap_server builds a voxel grid and projects to 2D → /robot/map.
        # Ground-return rejection is critical: Mid-360 at ~0.57m on Go2W,
        # ~0.39m on Go2; a z-band filter is the last line of defense against
        # rays sweeping the floor.
        # Also serves as the 3D voxel grid source when rviz_3d=true
        # (octomap_server publishes /robot/octomap_{binary,full,point_cloud_centers}).
        octo_min_z = 0.20 if robot_model == "go2w" else 0.30
        actions.append(
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                namespace=robot_ns,
                name="octomap_map_gen",
                parameters=[{
                    "use_sim_time": False,
                    "resolution": 0.05,
                    "frame_id": "map",
                    "base_frame_id": "base_link",
                    "sensor_model.max_range": 8.0,
                    "sensor_model.hit": 0.8,
                    "sensor_model.miss": 0.35,
                    "sensor_model.min": 0.12,
                    "sensor_model.max": 0.97,
                    # Widened z-band: we now let RANSAC segment ground rather
                    # than rejecting it with a world-frame z-filter. The
                    # z-band just prevents absurdly low / high points (under
                    # the floor, above the ceiling) from reaching the octree
                    # and confusing RANSAC.
                    "point_cloud_min_z": -0.50,
                    "point_cloud_max_z":  2.00,
                    # Safety net AFTER ground filter runs — anything still
                    # below ground level in world frame is noise.
                    "occupancy_min_z":  0.05,
                    "occupancy_max_z":  1.80,
                    # ── RANSAC ground-plane removal (fixes ramp littering) ──
                    # On a slope, a flat-z filter can't distinguish "ramp
                    # surface above 0.30 m in world" from "wall at 0.30 m".
                    # RANSAC segments a plane in BASE_LINK frame, where the
                    # ramp is approximately horizontal (because base_link
                    # tilts with the robot's chassis via Fast-LIO → body →
                    # base_link TF chain). Ground points are dropped entirely.
                    "filter_ground_plane": True,
                    "ground_filter.distance": 0.06,       # 6 cm inlier tolerance (a bit looser than default 4 cm — Mid-360 noisy at range)
                    "ground_filter.angle": 0.262,         # 15° max plane tilt (safety margin vs base_link; default 8.6° can miss on ramp-to-flat transitions)
                    "ground_filter.plane_distance": 0.10, # 10 cm — max ground-plane offset from sensor z baseline
                    # Isolated voxel removal — kills outlier speckles from
                    # specular reflections / moving props. No speckles = no
                    # phantom obstacles from transient reflections.
                    "filter_speckles": True,
                    # latch=True → octomap publishers use TRANSIENT_LOCAL QoS,
                    # matching reactive_nav + frontier_3d_markers + RViz Map
                    # display which all expect TRANSIENT_LOCAL. With latch=False
                    # (VOLATILE) ROS 2 silently drops every message due to QoS
                    # mismatch, so /robot/map never expands in RViz even though
                    # octomap is building it internally.
                    "latch": True,
                    # NOTE: octomap_server does NOT declare a transform_tolerance
                    # param (the tf2 MessageFilter tolerance is hardcoded to 5s
                    # in source). We leave this here as documentation but it's a
                    # no-op for this node version.
                    "transform_tolerance": 0.5,
                }],
                remappings=[
                    # CRITICAL: subscribe to the BODY-frame sweep, not the world
                    # one. /cloud_registered (world frame "camera_init") makes
                    # octomap's TF(map ← camera_init) resolve to IDENTITY every
                    # cycle, which pins sensor_origin to (0, 0, 0) forever. As
                    # the robot moves away from origin, (point − origin).norm()
                    # exceeds sensor_model.max_range (8 m) for every new scan
                    # point → occupied endpoints get truncated away → map stops
                    # growing the moment the robot leaves an 8 m ball around
                    # its spawn pose.
                    # /cloud_registered_body has frame_id="body" (set by
                    # fastlio_mid360.yaml scan_bodyframe_pub_en=true), so the
                    # TF(map ← body) lookup returns Fast-LIO's DYNAMIC pose
                    # and sensor_origin tracks the real robot. Raycasting +
                    # max_range culling then work correctly.
                    ("cloud_in", "/cloud_registered_body"),
                    ("projected_map", f"/{robot_ns}/map"),
                ],
                output="screen",
            )
        )

    # ── 3D voxel grid for the rviz_3d viewer ──
    # carto_l1 doesn't include an octomap (Cartographer is doing 2D). Spawn a
    # viz-only octomap_server subscribed to /utlidar/transformed_cloud so the
    # 3D RViz has /robot/octomap_{binary,full,point_cloud_centers}. It does
    # NOT publish a projected_map (Cartographer already owns /robot/map).
    rviz_3d_enabled = _get(context, "rviz_3d").strip().lower() in {"1", "true", "yes", "on"}
    if rviz_3d_enabled and slam == "carto_l1":
        actions.append(
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                namespace=robot_ns,
                name="octomap_viz",
                parameters=[{
                    "use_sim_time": False,
                    "resolution": 0.05,
                    "frame_id": "map",
                    "base_frame_id": "base_link",
                    "sensor_model.max_range": 8.0,
                    "sensor_model.hit": 0.8,
                    "sensor_model.miss": 0.35,
                    "sensor_model.min": 0.12,
                    "sensor_model.max": 0.97,
                    # Wider z-band so RANSAC can see the ground to segment it.
                    "point_cloud_min_z": -0.50,
                    "point_cloud_max_z":  2.00,
                    "occupancy_min_z":   0.05,
                    "occupancy_max_z":   1.80,
                    # Same ramp-aware ground removal as the Fast-LIO octomap —
                    # viz octomap also needs to drop ramp surfaces so the 3D
                    # voxel view isn't littered with false-positive ground
                    # obstacles when robot is on a slope.
                    "filter_ground_plane": True,
                    "ground_filter.distance": 0.06,
                    "ground_filter.angle": 0.262,
                    "ground_filter.plane_distance": 0.10,
                    "filter_speckles": True,
                    # latch=True → octomap publishers use TRANSIENT_LOCAL QoS,
                    # matching reactive_nav + frontier_3d_markers + RViz Map
                    # display which all expect TRANSIENT_LOCAL. With latch=False
                    # (VOLATILE) ROS 2 silently drops every message due to QoS
                    # mismatch, so /robot/map never expands in RViz even though
                    # octomap is building it internally.
                    "latch": True,
                    # TF buffer tolerance: default 0.1s is tight — Fast-LIO
                    # occasionally publishes /tf 50-100ms late under CPU load,
                    # which makes octomap silently drop clouds. 0.5s covers
                    # the worst observed jitter; downside is larger cold-start
                    # latency (negligible).
                    "transform_tolerance": 0.5,
                }],
                remappings=[
                    ("cloud_in", "/utlidar/transformed_cloud"),
                    # Route projected_map OUT OF THE WAY so we don't clobber
                    # Cartographer's /robot/map used by the planner.
                    ("projected_map", f"/{robot_ns}/octomap_projected_viz"),
                ],
                output="screen",
            )
        )

    # ── Frontier visualisation markers (always useful on real) ──
    actions.append(
        Node(
            package="go2w_perception",
            executable="frontier_3d_markers.py",
            namespace=robot_ns,
            name="frontier_3d_markers",
            parameters=[
                {
                    "map_topic": f"/{robot_ns}/map",
                    "marker_topic": f"/{robot_ns}/frontier_cylinders",
                    "frame_id": "map",
                    "free_threshold": 0,
                    "occ_threshold": 50,
                    "obstacle_clearance_m": 0.30,
                    # Match CFPA2's `cfpa2_frontier_min_cluster_area_m2`
                    # (0.08 m²). Previous 0.5 m² hid every legitimate frontier
                    # in indoor geometry — CFPA2 reported fronts=4 while the
                    # viz showed clusters=0, leaving the operator blind.
                    "min_cluster_area_m2": 0.08,
                    "cylinder_height": 0.8,
                    "cylinder_radius": 0.12,
                }
            ],
            output="screen",
        )
    )

    # ── RViz2 (2D top-down) ──
    # Shows: /robot/map (occupancy), /robot/scan_3d (laser), /robot/frontier_cylinders
    # (frontier targets), reactive_nav / FAR path, red robot-pose triangle, TF tree.
    # Config ships in go2w_real_bringup/config/rviz/.
    rviz = _get(context, "rviz").strip().lower() in {"1", "true", "yes", "on"}
    if rviz:
        rviz_cfg_name = _get(context, "rviz_config").strip() or "autonomy.rviz"
        rviz_path = os.path.join(bringup_share, "config", "rviz", rviz_cfg_name)
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_2d",
                arguments=["-d", rviz_path],
                output="log",
            )
        )

    # ── RViz2 (3D voxel view) — second window alongside the 2D one ──
    # Perspective 3D camera. Shows the octomap voxel cloud (coloured by z),
    # live registered scan (fastlio only), red robot triangle, TF tree.
    # Spawned as a SEPARATE process so you can reposition / close it without
    # affecting the 2D view.
    if rviz_3d_enabled:
        rviz3d_path = os.path.join(bringup_share, "config", "rviz", "3d_view.rviz")
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_3d",
                arguments=["-d", rviz3d_path],
                output="log",
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="robot"),
            DeclareLaunchArgument("robot_model", default_value="go2w",
                                   description="go2w (wheeled-legged) or go2 (no-wheel)"),
            DeclareLaunchArgument("slam", default_value="carto_l1",
                                   description="carto_l1 or fastlio_mid360"),
            DeclareLaunchArgument("carto_mode", default_value="2d",
                                   description="Cartographer 2d (default) or 3d (ignored for Fast-LIO). Auto-forced to 2d when map_backend=carto_2d."),
            DeclareLaunchArgument("nav_backend", default_value="nav2_mppi",
                                   description="nav2_mppi (default since 2026-04-29 — Nav2 + "
                                               "SmacPlannerHybrid + MPPI + behavior_server + "
                                               "fast_lio_tf_adapter + stuck_watchdog) | default | "
                                               "astar | far. Legacy aliases reactive → default, "
                                               "rrt_star/far_rrt_star/mppi → astar."),
            DeclareLaunchArgument("map_backend", default_value="carto_2d",
                                   description="carto_2d (default — Cartographer 2D grid + binarizer) | carto_binary (carto 3D→2D grid + binarizer) | scan (simple_scan_mapper, no free-space carving, no decay)"),
            DeclareLaunchArgument("obstacle_avoidance", default_value="true"),
            DeclareLaunchArgument("execute_controller", default_value="true",
                                   description="false = dry-run; planner + mux run but sport API never receives cmd_vel"),
            DeclareLaunchArgument("enable_manual_fallback", default_value="true"),
            DeclareLaunchArgument("waypoint_input_suffix", default_value="/way_point_coord"),
            DeclareLaunchArgument("manual_timeout_sec", default_value="0.35"),
            DeclareLaunchArgument("auto_timeout_sec", default_value="0.60"),
            DeclareLaunchArgument("manual_linear_threshold", default_value="0.02"),
            DeclareLaunchArgument("manual_angular_threshold", default_value="0.05"),
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("rviz", default_value="true",
                                   description="Launch 2D top-down RViz2 with autonomy.rviz (or rviz_config override)"),
            DeclareLaunchArgument("rviz_config", default_value="autonomy.rviz",
                                   description="RViz 2D config filename inside config/rviz/ (autonomy.rviz | cartographer.rviz | cartographer_grid.rviz | octomap.rviz)"),
            DeclareLaunchArgument("rviz_3d", default_value="true",
                                   description="Launch a second 3D-perspective RViz2 with octomap voxels + registered cloud. For carto_l1 a viz-only octomap_server is spawned; for fastlio_mid360 the existing map-gen octomap is reused."),
            # real-CMU-TARE passthroughs (see navigation.launch.py).
            DeclareLaunchArgument("explore", default_value="true",
                                   description="When false, skip CFPA2 — the caller supplies goals (e.g. from real CMU TARE)."),
            DeclareLaunchArgument("far_goal_topic", default_value=""),
            DeclareLaunchArgument("far_way_point_out", default_value=""),
            DeclareLaunchArgument("registered_scan_topic", default_value="",
                                   description="World-frame registered cloud topic for terrain_analysis + "
                                               "FAR + TARE. Empty → use the default /utlidar/transformed_cloud. "
                                               "The tare_real launch overrides this with /{ns}/registered_scan_map."),
            OpaqueFunction(function=_launch_setup),
        ]
    )
