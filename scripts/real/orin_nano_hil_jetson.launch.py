# orin_nano_hil_jetson.launch.py — Jetson half of the Orin Nano HIL test bench.
#
# Companion to scripts/launch/nav_test_hil_desktop.sh (desktop side).
#
# Runs on Jetson Orin Nano Super 8GB:
#   1. fast_lio (slam_node)             — SLAM, outputs /robot/Odometry (camera_init)
#   2. fast_lio_tf_adapter              — converts camera_init pose → clean
#                                          map → base_link TF, optional GT bootstrap
#   3. base_link → body static          — Mid-360 mount offset (Z=+0.28m)
#                                          (desktop also publishes a slightly different
#                                          imu→body; this static is the authoritative
#                                          base_link→body used by elevation_mapping)
#   4. elevation_mapping_cupy           — 2.5D heightmap + CNN traversability
#   5. filter_chain_runner              — grid_map analytical filters
#   6. grid_map_to_occupancy_grid       — trav_fused → OccupancyGrid
#   7. Nav2 stack                       — controller (MPPI), planner (Smac), behaviors, BT
#
# Frame conventions (must match the desktop side):
#   map → base_link        : fast_lio_tf_adapter publishes
#   base_link → body       : static below (Mid-360 mount Z=0.28)
#   base → FL_*, etc.      : CHAMP / robot_state_publisher on desktop
#   world → map            : desktop publishes (slam_odom_relay static identity)
#
# CRITICAL env var (set by the runner shell wrapper):
#   ELEVATION_MAPPING_FORCE_CUPY=1     # bypasses torch path (Orin sm_87, no torch)
import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml


WS = "/home/johnpork233/jetson_ws"
ROBOT_NS = "robot"
USE_SIM_TIME = True


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
    velodyne_yaml = os.path.join(fast_lio_share, "config", "velodyne.yaml")
    nav2_yaml_path = os.path.join(WS, "config", "nav", "nav2_go2_full_stack.yaml")
    bt_dir = os.path.join(WS, "config", "nav", "behavior_trees")

    tf_remaps = [
        ("/tf", f"/{ROBOT_NS}/tf"),
        ("/tf_static", f"/{ROBOT_NS}/tf_static"),
    ]

    args = [
        DeclareLaunchArgument("explore", default_value="false"),
        DeclareLaunchArgument("trav_weight_file", default_value=default_weights),
        DeclareLaunchArgument("bootstrap_from_gt", default_value="true"),
    ]

    actions = []

    # 1. fast_lio (SLAM)
    actions.append(
        Node(
            package="fast_lio", executable="fastlio_mapping",
            name="slam_node", namespace=ROBOT_NS,
            parameters=[velodyne_yaml,
                        {"use_sim_time": USE_SIM_TIME,
                         "preprocess.scan_line": 16,
                         "preprocess.blind": 0.5,
                         "pcd_save.pcd_save_en": False,
                         # SIM SETUP DIFFERS FROM REAL: in MuJoCo, IMU site
                         # is at body (identity quat, gravity-aligned), but
                         # lidar site has quat=(0.991160, -0.018244, 0.131392,
                         # 0.002418) ≈ rpy(-2.11°, +15.10°, 0°). Real Mid-360
                         # has IMU+lidar co-mounted in one housing so identity
                         # extrinsic_R works; in sim we MUST compensate via
                         # extrinsic_R = R_lidar→body (R_body_lidar in ROS conv).
                         # This is the rotation that maps points FROM lidar
                         # frame TO body frame: p_body = R · p_lidar + T.
                         # rpy=(-2.11°, +15.10°, 0°) = Rz(0)Ry(0.263)Rx(-0.037):
                         "mapping.extrinsic_R": [
                             0.965473, -0.009591,  0.260328,
                             0.000000,  0.999322,  0.036818,
                            -0.260505, -0.035547,  0.964818],
                         "mapping.extrinsic_est_en": False}],
            remappings=[
                ("/velodyne_points", f"/{ROBOT_NS}/velodyne_points"),
                # mujoco_imu_sensor publishes /robot/imu_imu_sensor/imu directly.
                ("/imu/data", f"/{ROBOT_NS}/imu_imu_sensor/imu"),
                ("/Odometry", f"/{ROBOT_NS}/Odometry"),
                # Let fast_lio publish its NATIVE camera_init→body TF. The 3
                # static TFs below tilt-align it into map→base_link (real
                # onboard_slam.sh pattern). NO MORE TF SINK.
                ("/tf", f"/{ROBOT_NS}/tf"),
                ("/tf_static", f"/{ROBOT_NS}/tf_static"),
                ("/cloud_registered_body", f"/{ROBOT_NS}/cloud_registered_body"),
                ("/cloud_registered", f"/{ROBOT_NS}/cloud_registered_camera_init"),
                ("/cloud_effected", f"/{ROBOT_NS}/cloud_effected"),
                ("/Laser_map", f"/{ROBOT_NS}/Laser_map"),
                ("/path", f"/{ROBOT_NS}/path"),
            ],
            output="screen",
        )
    )

    # 2. THREE static TFs — adapted from real onboard_slam.sh.
    # SIM differs from real: because extrinsic_R compensates the lidar mount
    # tilt INSIDE fast_lio (see above), fast_lio's body frame is already
    # gravity-aligned. So we use IDENTITY for both tilt statics (would
    # double-tilt the chain otherwise). map→odom identity remains for
    # REP-105 compatibility with Nav2.
    actions.append(
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="map_to_camera_init", namespace=ROBOT_NS,
            arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0",
                       "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                       "--frame-id", "map",
                       "--child-frame-id", "camera_init"],
            remappings=tf_remaps,
            output="screen",
        )
    )
    actions.append(
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="body_to_base_link_fastlio", namespace=ROBOT_NS,
            arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0",
                       "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                       "--frame-id", "body",
                       "--child-frame-id", "base_link"],
            remappings=tf_remaps,
            output="screen",
        )
    )
    actions.append(
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="map_to_odom_identity", namespace=ROBOT_NS,
            arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0",
                       "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                       "--frame-id", "map", "--child-frame-id", "odom"],
            remappings=tf_remaps,
            output="screen",
        )
    )

    # 3. fast_lio_tf_adapter — TOPIC RELAY ONLY (publish_tf=false). The 3 static
    # TFs above own the map→base_link chain; the adapter just relays
    # /Odometry → /odom/nav for stuck_watchdog/cfpa2 if they need a clean odom
    # topic. Bootstrap from GT keeps the SLAM frame anchored to world coords.
    actions.append(
        ExecuteProcess(
            cmd=[
                "python3", "-u",
                os.path.join(WS, "scripts", "runtime", "fast_lio_tf_adapter.py"),
                "--ros-args",
                "-r", f"__ns:=/{ROBOT_NS}",
                "-r", f"/tf:=/{ROBOT_NS}/tf",
                "-r", f"/tf_static:=/{ROBOT_NS}/tf_static",
                "-p", "use_sim_time:=true",
                "-p", f"namespace:={ROBOT_NS}",
                "-p", "input_topic:=Odometry",
                "-p", "output_topic:=odom/nav",
                "-p", "output_frame_id:=map",
                "-p", "output_child_frame_id:=base_link",
                "-p", "publish_tf:=false",     # static chain owns TF
                "-p", "bootstrap_from_gt:=true",
                "-p", "gt_topic:=odom/ground_truth",
            ],
            name="fast_lio_tf_adapter",
            output="screen",
        )
    )

    # 4. elevation_mapping_cupy
    # NOTE: launch DSL's Node action does NOT fully inherit shell env. The
    # ELEVATION_MAPPING_FORCE_CUPY export in run_jetson_hil.sh is invisible
    # to the spawned process — pass it via additional_env so the patched
    # backend selector (elevation_mapping.py:158) actually picks cupy on Orin.
    actions.append(
        Node(
            package="elevation_mapping_cupy",
            executable="elevation_mapping_node.py",
            name="elevation_mapping", namespace=ROBOT_NS,
            parameters=[emap_core, emap_setup,
                        {"use_sim_time": USE_SIM_TIME,
                         "weight_file": LaunchConfiguration("trav_weight_file")}],
            remappings=[
                ("/elevation_mapping/elevation_map_raw",
                 f"/{ROBOT_NS}/elevation_map_raw"),
            ] + tf_remaps,
            additional_env={"ELEVATION_MAPPING_FORCE_CUPY": "1"},
            respawn=False,
            output="screen",
        )
    )

    # 5. filter_chain_runner — DISABLED on Orin Nano.
    # The 23-stage analytical chain is CPU-bound on Orin Nano: 0.28 Hz output
    # vs 5 Hz input (each frame ~3.5s single-core 94% CPU). Trav grid ends
    # up showing 3-second-old elevation → looks noisy in RViz even though
    # CNN traversability is already in elevation_map_raw at full rate.
    # We bypass the chain and consume the CNN trav layer directly below.

    # 6. grid_map_to_occupancy_grid (C++ port) — reads CNN trav directly from
    # raw map. Bypass filter_chain: input_topic = elevation_map_raw,
    # layer = traversability. ramp_override needs slope+step_residual from
    # filter_chain → DISABLED. upper_bound_clearance still works (upper_bound
    # layer is in raw map).
    #
    # C++ port is the production executable (commit 0393668): 4.6× faster
    # than the Python `grid_map_to_occupancy_grid` (0.59 → 2.72 Hz on
    # Orin Nano, CPU 99% → 4%). Same param contract. Fall back to Python
    # by changing executable below to `grid_map_to_occupancy_grid`.
    actions.append(
        Node(
            package="trav_cost_filters", executable="grid_map_to_occupancy_grid_cpp",
            name="grid_map_to_occupancy_grid", namespace=ROBOT_NS,
            parameters=[{
                "use_sim_time": USE_SIM_TIME,
                "input_topic": "elevation_map_raw",        # was elevation_map_filtered
                "output_topic": "traversability_grid",
                "traversability_layer": "traversability",  # was trav_fused (CNN direct)
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
                "ramp_override_enabled": False,            # was True (filter_chain dependency)
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

    # 7. Nav2 stack — rewrite the dual-sim yaml's /robot_[ab]/ topics → /robot/,
    # force base_link, point to the Go2 no-spin BT XMLs.
    with open(nav2_yaml_path) as f:
        yaml_text = f.read()
    yaml_text = re.sub(r"/robot_[ab]/", f"/{ROBOT_NS}/", yaml_text)
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=f"_{ROBOT_NS}_nav2.yaml", delete=False)
    tmp.write(yaml_text); tmp.close()
    nav2_params = RewrittenYaml(
        source_file=tmp.name, root_key=ROBOT_NS,
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

    # CHAMP listens to cmd_vel_legged for Go2 (no wheels) — remap Nav2 output.
    nav2_cmd_remap = [("cmd_vel", "cmd_vel_legged")]

    nav2_group = GroupAction([
        PushRosNamespace(ROBOT_NS),
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", parameters=[nav2_params],
             remappings=tf_remaps + nav2_cmd_remap, output="screen"),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", parameters=[nav2_params],
             remappings=tf_remaps, output="screen"),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", parameters=[nav2_params],
             remappings=tf_remaps + nav2_cmd_remap, output="screen"),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", parameters=[nav2_params],
             remappings=tf_remaps, output="screen"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation",
             parameters=[{
                 "use_sim_time": True, "autostart": True,
                 "node_names": ["controller_server", "planner_server",
                                "behavior_server", "bt_navigator"],
             }],
             output="screen"),
    ])
    # Delay Nav2 so TF + trav grid are warm.
    actions.append(TimerAction(period=10.0, actions=[nav2_group]))

    return LaunchDescription(args + actions)
