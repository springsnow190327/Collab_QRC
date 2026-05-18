# orin_nano_phase4_full.launch.py — Full autonomy stack for the Jetson side of
# the HIL test bench. Pairs with the desktop running MuJoCo + sensors + CHAMP.
#
# What runs on Jetson (this launch):
#   1. fast_lio (slam_node)                — SLAM from /robot/velodyne_points + /robot/imu/data
#   2. elevation_mapping_cupy              — Kalman 2.5D heightmap + CNN traversability
#   3. filter_chain_runner (trav_cost_filters) — 10-stage grid_map filter chain
#   4. grid_map_to_occupancy_grid          — trav_fused layer → OccupancyGrid (fixed-frame)
#   5. nav2: controller_server (MPPI), planner_server (SmacLattice), behavior_server,
#      bt_navigator, lifecycle_manager_navigation
#
# What runs on desktop (separate launch):
#   - MuJoCo + mujoco_sensor_bridge        — sensors / GT pose / cmd_vel routing
#   - CHAMP (quadruped_controller_node)    — cmd_vel → joint torques in MuJoCo
#   - RViz                                  — visualizer (subscribes Jetson topics)
#
# Launch args:
#   robot_namespace      default "robot"
#   use_sim_time         default "true"
#   explore              default "false"  — set "true" to also start cfpa2_single_robot_node
#   trav_weight_file     default elevation_mapping_cupy core weights (override for fine-tuned)
#   nav2_yaml_file       default nav2_go2_full_stack.yaml (must exist under config/nav/)
#
# Usage (on Jetson):
#   ros2 launch /tmp/orin_nano_phase4_full.launch.py
#   ros2 launch /tmp/orin_nano_phase4_full.launch.py explore:=true
import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    elevation_cupy_share = get_package_share_directory("elevation_mapping_cupy")
    trav_share = get_package_share_directory("trav_cost_filters")
    cfpa2_share = get_package_share_directory("cfpa2_collaborative_autonomy")
    fast_lio_share = get_package_share_directory("fast_lio")

    emap_core = os.path.join(elevation_cupy_share, "config", "core", "core_param.yaml")
    emap_setup = os.path.join(trav_share, "config", "elevation_mapping.yaml")
    filter_chain_yaml = os.path.join(trav_share, "config", "grid_map_filters.yaml")
    default_weights = os.path.join(elevation_cupy_share, "config", "core", "weights.dat")
    cfpa2_yaml = os.path.join(cfpa2_share, "config", "cfpa2_single_robot.yaml")

    ws = "/home/johnpork233/jetson_ws"
    nav2_yaml_dir = os.path.join(ws, "config", "nav")
    velodyne_yaml = os.path.join(fast_lio_share, "config", "velodyne.yaml")

    robot_ns = "robot"
    use_sim_time = True

    tf_remaps = [
        ("/tf", f"/{robot_ns}/tf"),
        ("/tf_static", f"/{robot_ns}/tf_static"),
    ]

    # ── Args ─────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("explore", default_value="false"),
        DeclareLaunchArgument("trav_weight_file", default_value=default_weights),
        DeclareLaunchArgument("nav2_yaml_file", default_value="nav2_go2_full_stack.yaml"),
    ]

    nodes = []

    # ── 1. fast_lio (slam_node) ─────────────────────────────────────
    nodes.append(
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="slam_node",
            namespace=robot_ns,
            parameters=[velodyne_yaml,
                        {"use_sim_time": use_sim_time,
                         "preprocess.scan_line": 16,
                         "preprocess.blind": 0.5,
                         "pcd_save.pcd_save_en": False}],
            remappings=[
                ("/velodyne_points", f"/{robot_ns}/velodyne_points"),
                ("/imu/data", f"/{robot_ns}/imu/data"),
                ("/Odometry", f"/{robot_ns}/Odometry"),
                ("/cloud_registered", f"/{robot_ns}/cloud_registered_camera_init"),
                ("/cloud_registered_body", f"/{robot_ns}/cloud_registered_body"),
                ("/cloud_effected", f"/{robot_ns}/cloud_effected"),
                ("/Laser_map", f"/{robot_ns}/Laser_map"),
                ("/path", f"/{robot_ns}/path"),
            ] + tf_remaps,
            output="screen",
        )
    )

    # ── 2. elevation_mapping_cupy ───────────────────────────────────
    # ELEVATION_MAPPING_FORCE_CUPY=1 env is set in the runner script,
    # which bypasses torch path (Orin Nano has no torch installed).
    nodes.append(
        Node(
            package="elevation_mapping_cupy",
            executable="elevation_mapping_node.py",
            name="elevation_mapping",
            namespace=robot_ns,
            parameters=[emap_core, emap_setup,
                        {"use_sim_time": use_sim_time,
                         "weight_file": LaunchConfiguration("trav_weight_file")}],
            remappings=[
                # elevation_mapping_cupy hardcodes the publisher topic to
                # /<node_name>/elevation_map_raw → remap into the namespace.
                ("/elevation_mapping/elevation_map_raw",
                 f"/{robot_ns}/elevation_map_raw"),
            ] + tf_remaps,
            respawn=True, respawn_delay=3.0,
            output="screen",
        )
    )

    # ── 3. filter_chain_runner ──────────────────────────────────────
    nodes.append(
        Node(
            package="trav_cost_filters",
            executable="filter_chain_runner",
            name="filter_chain_runner",
            namespace=robot_ns,
            parameters=[filter_chain_yaml, {"use_sim_time": use_sim_time}],
            respawn=True, respawn_delay=3.0,
            output="screen",
        )
    )

    # ── 4. grid_map_to_occupancy_grid ───────────────────────────────
    # Params dict copied from nav_test_3d_explore.launch.py (ops2-tuned, 2026-05-18).
    nodes.append(
        Node(
            package="trav_cost_filters",
            executable="grid_map_to_occupancy_grid",
            name="grid_map_to_occupancy_grid",
            namespace=robot_ns,
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_topic": "elevation_map_filtered",
                "output_topic": "traversability_grid",
                "traversability_layer": "trav_fused",
                "free_threshold": 0.60,
                "lethal_threshold": 0.05,
                "elevation_cost_enabled": False,
                "elevation_layer": "elevation",
                "elevation_cost_min_h": 0.05,
                "elevation_cost_max_h": 1.50,
                "elevation_cost_max_value": 90,
                "cliff_proximity_cost_enabled": False,
                "cliff_step_layer": "step_height",
                "cliff_proximity_radius_m": 0.25,
                "cliff_step_threshold_m": 0.30,
                "cliff_step_saturation_m": 0.45,
                "cliff_proximity_cost_max_value": 90,
                "upper_bound_clearance_enabled": True,
                "upper_bound_layer": "upper_bound",
                "upper_bound_overhang_threshold_m": 0.30,
                "upper_bound_clear_cost": 0,
                "seed_robot_footprint": True,
                "robot_frame": "base_link",
                "robot_seed_radius_m": 2.0,
                "seed_max_clear_cost": 50,
                "ramp_override_enabled": True,
                "slope_layer": "slope",
                "step_residual_layer": "step_residual",
                "ramp_min_slope_rad": 0.20943951023931956,
                "ramp_max_slope_rad": 0.5235987755982988,
                "ramp_max_step_residual_m": 0.06,
                "fixed_grid_enabled": True,
                "fixed_origin_x": -100.0,
                "fixed_origin_y": -100.0,
                "fixed_width_cells": 2000,
                "fixed_height_cells": 2000,
                "unknown_clears_history": False,
                "occupied_cost_threshold": 100,
                "free_cost_threshold": 30,
                "occupied_confirm_hits": 2,
                "occupied_clear_hits": 0,
                "max_hit_count": 8,
                "workspace_mask_enabled": False,
            }],
            remappings=tf_remaps,
            respawn=True, respawn_delay=3.0,
            output="screen",
        )
    )

    # ── 5. Nav2 stack ────────────────────────────────────────────────
    # The dual-sim yaml uses /robot_a/ and /robot_b/ topic prefixes — rewrite
    # them to /robot/ for our single-robot HIL. Same logic as
    # nav_test_mujoco_fastlio.launch.py.
    nav2_base_path = os.path.join(nav2_yaml_dir, "nav2_go2_full_stack.yaml")
    with open(nav2_base_path) as f:
        yaml_text = f.read()
    yaml_text = re.sub(r"/robot_[ab]/", f"/{robot_ns}/", yaml_text)
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=f"_{robot_ns}_nav2.yaml", delete=False)
    tmp.write(yaml_text)
    tmp.close()
    # Go2 (no wheels) BT XMLs — strip `spin` recovery since behavior_server
    # does not expose spin for the legged platform (would fail bt_navigator
    # configure with "Action server spin not available").
    bt_dir = os.path.join(ws, "config", "nav", "behavior_trees")
    nav2_params = RewrittenYaml(
        source_file=tmp.name,
        root_key=robot_ns,
        param_rewrites={
            "use_sim_time": "true",
            "robot_base_frame": "base_link",
            "default_nav_to_pose_bt_xml":
                os.path.join(bt_dir, "navigate_to_pose_no_spin_recovery.xml"),
            "default_nav_through_poses_bt_xml":
                os.path.join(bt_dir, "navigate_through_poses_no_spin_recovery.xml"),
        },
        convert_types=True,
    )

    # Go2 (no wheels) needs cmd_vel → cmd_vel_legged remap so CHAMP receives it.
    nav2_cmd_remap = [("cmd_vel", "cmd_vel_legged")]

    nav2_group = GroupAction([
        PushRosNamespace(robot_ns),
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server",
             parameters=[nav2_params],
             remappings=tf_remaps + nav2_cmd_remap, output="screen"),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server",
             parameters=[nav2_params],
             remappings=tf_remaps, output="screen"),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server",
             parameters=[nav2_params],
             remappings=tf_remaps + nav2_cmd_remap, output="screen"),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator",
             parameters=[nav2_params],
             remappings=tf_remaps, output="screen"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation",
             parameters=[{
                 "use_sim_time": True,
                 "autostart": True,
                 "node_names": ["controller_server", "planner_server",
                                "behavior_server", "bt_navigator"],
             }],
             output="screen"),
    ])

    # Delay nav2 so SLAM + trav are warm.
    nav2_delayed = TimerAction(period=8.0, actions=[nav2_group])

    # ── 6. CFPA2 (optional) ──────────────────────────────────────────
    cfpa2_node = Node(
        package="cfpa2_collaborative_autonomy",
        executable="cfpa2_single_robot_node",
        name="cfpa2_single_robot",
        parameters=[cfpa2_yaml, {
            "use_sim_time": True,
            "robot_namespace": robot_ns,
            "namespaces": [robot_ns],
            "goal_topic_suffix": "/way_point_coord",
            "planning_map_topic_suffix": "/global_costmap/costmap",
            "marker_frame_override": "map",
        }],
        output="screen",
    )

    cfpa2_delayed = TimerAction(
        period=15.0,
        actions=[
            GroupAction(
                actions=[cfpa2_node],
                # Only spawn when explore:=true. Doing this via if/else on the
                # LaunchConfiguration value is awkward in pure ROS 2 launch;
                # leaving the gate as an env var the script handles before
                # invoking ros2 launch (see runner shell wrapper).
            ),
        ],
    )

    return LaunchDescription(args + nodes + [nav2_delayed])
