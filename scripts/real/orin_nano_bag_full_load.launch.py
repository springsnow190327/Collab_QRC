# orin_nano_bag_full_load.launch.py — FULL stack Jetson real-time test via bag replay.
#
# Replays a real Go2 walk bag with ONLY raw sensor topics, forcing the Jetson
# to do all SLAM + perception + planning work itself. Measures whether the
# full pipeline keeps up with wall-clock 1.0x bag rate.
#
# What we replay (from bag):
#   /livox/imu                     (real Mid-360 IMU, 200 Hz)
#   /livox/lidar                   (real Mid-360 lidar, CustomMsg, 10 Hz)
#   NOTE: /tf, /tf_static, /robot/Odometry, /robot/cloud_registered_body are
#         intentionally EXCLUDED in the bag-play wrapper. Point-LIO builds the
#         SLAM tree from scratch using only raw IMU+lidar.
#
# What we LAUNCH (wall-clock, use_sim_time=false):
#   1. point_lio                       — SLAM, subscribes /livox/{lidar,imu}
#                                         publishes camera_init→body TF +
#                                         /robot/Odometry +
#                                         /robot/cloud_registered_body
#   2. 3x static TFs                   — map→camera_init, body→base_link,
#                                         map→odom (REP-105 compat for Nav2)
#   3. fast_lio_tf_adapter             — topic relay /Odometry → /odom/nav
#                                         (publish_tf=false; statics own TF)
#   4. elevation_mapping_cupy           — 2.5D heightmap + CNN traversability
#   5. grid_map_to_occupancy_grid       — CNN trav direct (filter_chain bypassed)
#   6. Nav2 stack                       — controller (MPPI), planner (Smac),
#                                         behaviors, BT, lifecycle
#   7. cfpa2_single_robot_node          — frontier allocator (autonomous goals)
#   8. cfpa2_to_nav2_bridge             — /way_point → /goal_pose
#
# Use:
#   bash /tmp/run_jetson_bag_full_load.sh
#   ros2 bag play <bag> --topics /livox/imu /livox/lidar --rate 1.0
import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, TimerAction
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml


WS = "/home/johnpork233/jetson_ws"
ROBOT_NS = "robot"
USE_SIM_TIME = True  # bag-play with --clock drives ros::Time; nodes must honour /clock
                     # so dynamic-TF stamps (Point-LIO emit time) live in same time
                     # domain as TF lookups (elevation_mapping, Nav2). RTF is still
                     # measured against wall-clock by bag-play itself.


def generate_launch_description():
    elevation_cupy_share = get_package_share_directory("elevation_mapping_cupy")
    trav_share = get_package_share_directory("trav_cost_filters")
    cfpa2_share = get_package_share_directory("cfpa2_collaborative_autonomy")
    fast_lio_share = get_package_share_directory("fast_lio")

    emap_core = os.path.join(elevation_cupy_share, "config", "core", "core_param.yaml")
    emap_setup = os.path.join(trav_share, "config", "elevation_mapping.yaml")
    default_weights = os.path.join(elevation_cupy_share, "config", "core", "weights.dat")
    cfpa2_yaml = os.path.join(cfpa2_share, "config", "cfpa2_single_robot.yaml")
    # Switched from Point-LIO to fast_lio: Point-LIO's dual-thread / iVox design
    # assumes strong single-core perf (paper benchmarks: Intel i7-10700K @ 3.8GHz).
    # On Jetson Orin Nano (Cortex-A78AE @ 1.5GHz, ~40% per-core vs i7), Point-LIO
    # falls behind real-time (6.4 Hz output vs 10 Hz Mid-360 input) → IMU
    # integration runs ahead of lidar correction → pose diverges (-148m in 6s
    # observed). Same bag with fast_lio onboard the real Go2 (same CPU class)
    # produced sensible pose (1 m/s walking, 14m total displacement over 50s).
    fast_lio_yaml = os.path.join(fast_lio_share, "config", "mid360.yaml")
    nav2_yaml_path = os.path.join(WS, "config", "nav", "nav2_go2_full_stack.yaml")
    bt_dir = os.path.join(WS, "config", "nav", "behavior_trees")

    tf_remaps = [
        ("/tf", f"/{ROBOT_NS}/tf"),
        ("/tf_static", f"/{ROBOT_NS}/tf_static"),
    ]

    actions = []

    # 1. fast_lio SLAM — subscribe raw CustomMsg lidar + raw IMU.
    # Onboard the real Go2 with the bag's recorded run, fast_lio sustained
    # ~10 Hz Odometry with sensible pose tracking (1 m/s, 14m over 50s).
    # On the bench Orin Nano 8GB (same Cortex-A78AE class as the real bot's
    # Orin NX 16GB), we expect similar throughput.
    actions.append(
        Node(
            package="fast_lio", executable="fastlio_mapping",
            name="laserMapping", namespace=ROBOT_NS,
            parameters=[fast_lio_yaml,
                        {"use_sim_time": USE_SIM_TIME,
                         "common.lid_topic": "/livox/lidar",
                         "common.imu_topic": "/livox/imu",
                         "preprocess.lidar_type": 1,  # 1 = Livox CustomMsg
                         "pcd_save.pcd_save_en": False,
                         "mapping.extrinsic_est_en": False}],
            remappings=[
                ("/Odometry", f"/{ROBOT_NS}/Odometry"),
                ("/cloud_registered_body", f"/{ROBOT_NS}/cloud_registered_body"),
                ("/cloud_registered", f"/{ROBOT_NS}/cloud_registered_camera_init"),
                ("/cloud_effected", f"/{ROBOT_NS}/cloud_effected"),
                ("/Laser_map", f"/{ROBOT_NS}/Laser_map"),
                ("/path", f"/{ROBOT_NS}/path"),
                ("/tf", f"/{ROBOT_NS}/tf"),
                ("/tf_static", f"/{ROBOT_NS}/tf_static"),
            ],
            output="screen",
        )
    )

    # 2. THREE static TFs — same pattern as orin_nano_hil_jetson.launch.py.
    actions.append(
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="map_to_camera_init", namespace=ROBOT_NS,
             arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0",
                        "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                        "--frame-id", "map", "--child-frame-id", "camera_init"],
             remappings=tf_remaps, output="screen"))
    actions.append(
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="body_to_base_link", namespace=ROBOT_NS,
             arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0",
                        "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                        "--frame-id", "body", "--child-frame-id", "base_link"],
             remappings=tf_remaps, output="screen"))
    actions.append(
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="map_to_odom_identity", namespace=ROBOT_NS,
             arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0",
                        "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                        "--frame-id", "map", "--child-frame-id", "odom"],
             remappings=tf_remaps, output="screen"))

    # 3. fast_lio_tf_adapter — topic relay only (publish_tf=false).
    # Point-LIO already publishes TF; this just exposes /odom/nav for stuck_watchdog
    # and CFPA2 clean-odom consumers. NO GT bootstrap (no MuJoCo).
    actions.append(
        ExecuteProcess(
            cmd=[
                "python3", "-u",
                os.path.join(WS, "scripts", "runtime", "fast_lio_tf_adapter.py"),
                "--ros-args",
                "-r", f"__ns:=/{ROBOT_NS}",
                "-r", f"/tf:=/{ROBOT_NS}/tf",
                "-r", f"/tf_static:=/{ROBOT_NS}/tf_static",
                "-p", f"use_sim_time:={str(USE_SIM_TIME).lower()}",
                "-p", f"namespace:={ROBOT_NS}",
                "-p", "input_topic:=Odometry",
                "-p", "output_topic:=odom/nav",
                "-p", "output_frame_id:=map",
                "-p", "output_child_frame_id:=base_link",
                "-p", "publish_tf:=false",
                "-p", "bootstrap_from_gt:=false",
            ],
            name="fast_lio_tf_adapter",
            output="screen",
        )
    )

    # 4. elevation_mapping_cupy — delayed 15s so Point-LIO can complete IMU init
    # AND publish the first few SLAM scans before a heavy subscriber attaches.
    # Without this delay, Point-LIO has an intermittent SIGSEGV (exit -11) right
    # after IMU Initializing: 100.0%, observed ~50% of launches on Orin Nano.
    # The crash window appears to be when the first /cloud_registered_body
    # subscriber (elevation_mapping) races with Point-LIO's first publish call.
    actions.append(TimerAction(period=15.0, actions=[
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
            respawn=False, output="screen",
        )
    ]))

    # 5. grid_map_to_occupancy_grid (C++ port) — CNN trav direct (filter_chain
    # bypassed for Jetson). Python version was CPU-bound at 0.59 Hz on Orin
    # Nano; C++ port `grid_map_to_occupancy_grid_cpp` has identical params and
    # topic contract. To fall back to Python, change executable back to
    # `grid_map_to_occupancy_grid`.
    # Delayed 18s so elevation_map_raw is publishing before subscriber starts.
    actions.append(TimerAction(period=18.0, actions=[
        Node(
            package="trav_cost_filters", executable="grid_map_to_occupancy_grid_cpp",
            name="grid_map_to_occupancy_grid", namespace=ROBOT_NS,
            parameters=[{
                "use_sim_time": USE_SIM_TIME,
                "input_topic": "elevation_map_raw",
                "output_topic": "traversability_grid",
                "traversability_layer": "traversability",
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
    ]))

    # 6. Nav2 stack — rewrite yaml for ns=robot + base_link + Go2 BT XMLs.
    with open(nav2_yaml_path) as f:
        yaml_text = f.read()
    yaml_text = re.sub(r"/robot_[ab]/", f"/{ROBOT_NS}/", yaml_text)
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=f"_{ROBOT_NS}_nav2.yaml", delete=False)
    tmp.write(yaml_text); tmp.close()
    nav2_params = RewrittenYaml(
        source_file=tmp.name, root_key=ROBOT_NS,
        param_rewrites={
            "use_sim_time": str(USE_SIM_TIME).lower(),
            "robot_base_frame": "base_link",
            "default_nav_to_pose_bt_xml":
                os.path.join(bt_dir, "navigate_to_pose_no_spin_recovery.xml"),
            "default_nav_through_poses_bt_xml":
                os.path.join(bt_dir, "navigate_through_poses_no_spin_recovery.xml"),
        },
        convert_types=True,
    )

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
                 "use_sim_time": USE_SIM_TIME, "autostart": True,
                 "node_names": ["controller_server", "planner_server",
                                "behavior_server", "bt_navigator"],
             }],
             output="screen"),
    ])
    # Nav2 delayed 22s (after Point-LIO SLAM steady + trav grid up)
    actions.append(TimerAction(period=22.0, actions=[nav2_group]))

    # 7. CFPA2 single-robot frontier allocator — autonomous exploration.
    actions.append(
        TimerAction(period=28.0, actions=[
            Node(
                package="cfpa2_collaborative_autonomy",
                executable="cfpa2_single_robot_node",
                name="cfpa2_single_robot", namespace=ROBOT_NS,
                parameters=[cfpa2_yaml, {
                    "use_sim_time": USE_SIM_TIME,
                    "robot_namespace": ROBOT_NS,
                    "map_topic": "traversability_grid",
                    "verbose_logs": False,
                }],
                remappings=tf_remaps,
                output="screen",
            )
        ])
    )

    # 8. CFPA2 → Nav2 bridge (/way_point_coord → /goal_pose action goal).
    # Standalone python node (not a console_script) — run via ExecuteProcess.
    actions.append(
        TimerAction(period=30.0, actions=[
            ExecuteProcess(
                cmd=[
                    "python3", "-u",
                    os.path.join(WS, "scripts", "runtime", "cfpa2_to_nav2_bridge.py"),
                    "--ros-args",
                    "-r", f"__ns:=/{ROBOT_NS}",
                    "-r", f"/tf:=/{ROBOT_NS}/tf",
                    "-r", f"/tf_static:=/{ROBOT_NS}/tf_static",
                    "-p", f"use_sim_time:={str(USE_SIM_TIME).lower()}",
                    "-p", f"namespace:={ROBOT_NS}",
                    "-p", "waypoint_topic:=way_point_coord",
                ],
                name="cfpa2_to_nav2_bridge",
                output="screen",
            )
        ])
    )

    return LaunchDescription(actions)
