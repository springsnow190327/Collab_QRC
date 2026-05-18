# orin_nano_bag_test.launch.py — Jetson real-time verification via bag replay.
#
# Replays a real Go2 walk bag (Noetic-converted ROS 2 db3) at wall-clock 1.0x
# to measure if Jetson can sustain the downstream autonomy stack at real rates.
#
# What we replay (from bag):
#   /robot/cloud_registered_body   (real Mid-360 cloud, fast_lio body frame)
#   /robot/Odometry                (real fast_lio output)
#   /livox/imu                     (real Mid-360 IMU)
#   /tf, /tf_static                (real TF chain — includes lidar mount tilt)
#
# What we SKIP:
#   - fast_lio (bag has Odometry)
#   - fast_lio_tf_adapter (bag has TF)
#   - mujoco_* (no sim)
#   - 3 tilt static publishers (bag's tf_static handles)
#
# What we LAUNCH (wall-clock, use_sim_time=false):
#   - elevation_mapping_cupy        (subscribes cloud_registered_body)
#   - grid_map_to_occupancy_grid    (CNN direct, no filter_chain)
#   - Nav2 stack (controller_server, planner_server, behavior_server, bt_navigator, lifecycle_manager)
#
# Use:
#   bash /tmp/run_jetson_bag_test.sh
#   ros2 bag play /tmp/bag_test/<bag> --rate 1.0
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml
import re
import tempfile


WS = "/home/johnpork233/jetson_ws"
ROBOT_NS = "robot"
USE_SIM_TIME = False  # ← WALL CLOCK for real-time verification


def generate_launch_description():
    elevation_cupy_share = get_package_share_directory("elevation_mapping_cupy")
    trav_share = get_package_share_directory("trav_cost_filters")
    cfpa2_share = get_package_share_directory("cfpa2_collaborative_autonomy")

    emap_core = os.path.join(elevation_cupy_share, "config", "core", "core_param.yaml")
    emap_setup = os.path.join(trav_share, "config", "elevation_mapping.yaml")
    default_weights = os.path.join(elevation_cupy_share, "config", "core", "weights.dat")
    nav2_yaml_path = os.path.join(WS, "config", "nav", "nav2_go2_full_stack.yaml")
    bt_dir = os.path.join(WS, "config", "nav", "behavior_trees")

    # TF coming from bag's /tf and /tf_static (already wall-clock stamped from real Go2).
    # Our nodes' TF listener uses namespaced topic: tf is fine because all nodes are in /robot ns.
    tf_remaps = [
        ("/tf", f"/{ROBOT_NS}/tf"),
        ("/tf_static", f"/{ROBOT_NS}/tf_static"),
    ]

    actions = []

    # 1. elevation_mapping_cupy — consume bag's cloud_registered_body
    actions.append(
        Node(
            package="elevation_mapping_cupy",
            executable="elevation_mapping_node.py",
            name="elevation_mapping", namespace=ROBOT_NS,
            parameters=[emap_core, emap_setup,
                        {"use_sim_time": USE_SIM_TIME,
                         "weight_file": default_weights}],
            remappings=[
                ("/elevation_mapping/elevation_map_raw",
                 f"/{ROBOT_NS}/elevation_map_raw"),
            ] + tf_remaps,
            additional_env={"ELEVATION_MAPPING_FORCE_CUPY": "1"},
            respawn=False,
            output="screen",
        )
    )

    # 2. grid_map_to_occupancy_grid — read CNN trav directly from raw map
    actions.append(
        Node(
            package="trav_cost_filters", executable="grid_map_to_occupancy_grid",
            name="grid_map_to_occupancy_grid", namespace=ROBOT_NS,
            parameters=[{
                "use_sim_time": USE_SIM_TIME,
                "input_topic": "elevation_map_raw",
                "output_topic": "traversability_grid",
                "traversability_layer": "traversability",  # CNN direct
                "free_threshold": 0.60,
                "lethal_threshold": 0.05,
                "elevation_cost_enabled": False,
                "upper_bound_clearance_enabled": True,
                "upper_bound_layer": "upper_bound",
                "upper_bound_overhang_threshold_m": 0.30,
                "upper_bound_clear_cost": 0,
                "seed_robot_footprint": True,
                "robot_frame": "base_link",
                "robot_seed_radius_m": 2.0,
                "seed_max_clear_cost": 50,
                "ramp_override_enabled": False,
                "fixed_grid_enabled": True,
                "fixed_origin_x": -100.0, "fixed_origin_y": -100.0,
                "fixed_width_cells": 2000, "fixed_height_cells": 2000,
                "unknown_clears_history": False,
                "occupied_cost_threshold": 100, "free_cost_threshold": 30,
                "occupied_confirm_hits": 2, "occupied_clear_hits": 0,
                "max_hit_count": 8,
                "workspace_mask_enabled": False,
            }],
            remappings=tf_remaps,
            respawn=False, output="screen",
        )
    )

    # 3. Nav2 stack — rewrite yaml for ns=robot + base_link + Go2 BT XMLs
    with open(nav2_yaml_path) as f:
        yaml_text = f.read()
    yaml_text = re.sub(r"/robot_[ab]/", f"/{ROBOT_NS}/", yaml_text)
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=f"_{ROBOT_NS}_nav2.yaml", delete=False)
    tmp.write(yaml_text); tmp.close()
    nav2_params = RewrittenYaml(
        source_file=tmp.name, root_key=ROBOT_NS,
        param_rewrites={
            "use_sim_time": "false",
            "robot_base_frame": "base_link",
            "default_nav_to_pose_bt_xml":
                os.path.join(bt_dir, "navigate_to_pose_no_spin_recovery.xml"),
            "default_nav_through_poses_bt_xml":
                os.path.join(bt_dir, "navigate_through_poses_no_spin_recovery.xml"),
        },
        convert_types=True,
    )

    # No CHAMP on this test — just produce cmd_vel and measure rate. No remap needed.
    nav2_group = GroupAction([
        PushRosNamespace(ROBOT_NS),
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", parameters=[nav2_params],
             remappings=tf_remaps, output="screen"),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", parameters=[nav2_params],
             remappings=tf_remaps, output="screen"),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", parameters=[nav2_params],
             remappings=tf_remaps, output="screen"),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", parameters=[nav2_params],
             remappings=tf_remaps, output="screen"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation",
             parameters=[{
                 "use_sim_time": False, "autostart": True,
                 "node_names": ["controller_server", "planner_server",
                                "behavior_server", "bt_navigator"],
             }],
             output="screen"),
    ])
    actions.append(TimerAction(period=5.0, actions=[nav2_group]))

    return LaunchDescription(actions)
