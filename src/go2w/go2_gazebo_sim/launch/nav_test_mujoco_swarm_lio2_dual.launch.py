#!/usr/bin/env python3
"""Dual-robot MuJoCo + Swarm-LIO2 (ROS1 in docker) + FAR nav launch.

Cousin of nav_test_mujoco_fastlio_dual.launch.py — same MuJoCo+CHAMP
base, same FAR+CFPA2 stack, but Fast-LIO2 is replaced by the dockerized
ROS1 Swarm-LIO2 stack and the TF chain is reshaped accordingly.

Per-robot wiring (X = a / b, N = 1 / 2):
  - input relays:  /{ns}/registered_scan_reliable → /robot_X/velodyne_points
                   /{ns}/imu/data                 → /robot_X/imu/data
  - output relay:  /robot_X/swarm_lio2_raw/Odometry → /{ns}/Odometry
  - swarm_lio_tf_adapter (one instance per ns, CLI-renamed):
      publishes dynamic TF quadN/world → quadN_aft_mapped
      rewrites /robot_X/swarm_lio2_raw/cloud_static → /{ns}/cloud_registered_body
  - static TFs (sim: all identity, no IMU mount tilt):
      world → map → quadN/world          (… dynamic …)  quadN_aft_mapped → base_frame
      map  → odom                        (phantom for nav consumers)

This means slam_odom_relay + pointcloud_frame_bridge keep their topics
intact — the swarm_lio_tf_adapter rewrites the cloud_static frame to
quadN_aft_mapped so the body-cloud → map-cloud bridge resolves via our
static chain, and Fast-LIO's hardcoded `body` frame is gone.

Docker compose stack (docker/ros1_hybrid_slam/) is started once by this
launch and torn down on shutdown. bridge.yaml already exposes both
/robot_a/swarm_lio2_raw/* and /robot_b/swarm_lio2_raw/* and the
container entrypoint runs two laserMapping processes (drone_id=1/2), so
no docker-side changes are needed.

Default scene is demo3_dual.xml (24×16m, two Go2Ws).
"""
from __future__ import annotations

import os
import sys

_ws_root = os.path.abspath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "..", "..", "..", ".."
))

_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch_ros.actions import Node

from modules import _find_mujoco_plugin_dir
from modules.assets import build_dual_robot_stack, build_namespaced_robot_description
from modules.dual_urdf import build_robot_b_urdf
from modules.dual_urdf_nav import build_dual_nav_urdf
from modules.launch_helpers import (
    as_bool as _as_bool,
    build_cleanup_stale_cmd as _build_cleanup_stale_cmd,
    get_launch_arg as _get,
    load_yaml_params as _load_yaml_params,
)


_DOCKER_COMPOSE_DIR = os.path.join(_ws_root, "docker", "ros1_hybrid_slam")
_SWARM_LIO_TF_ADAPTER = os.path.join(_ws_root, "scripts", "runtime", "swarm_lio_tf_adapter.py")


def _build_sensor_bridges(ns: str, mjcf_path: str, base_body: str, imu_site: str,
                          pose_sensor: str, imu_sensor: str, links_config: str,
                          use_sim_time: bool):
    """Per-robot MuJoCo sensor bridges (ground-truth odom + foot contacts).

    Differs from fastlio_dual: `publish_tf=False`. With Swarm-LIO2 owning
    the map → base_link chain via the swarm_lio_tf_adapter dynamic TF,
    we MUST NOT let mujoco_odom_bridge publish a competing odom→base_link
    TF (golden rule: one owner per TF link).
    """
    return [
        Node(
            package="mujoco_sensor_bridge",
            executable="mujoco_odom_bridge",
            namespace=ns,
            name="mujoco_odom_bridge",
            parameters=[{
                "use_sim_time": use_sim_time,
                "mjcf_path": mjcf_path,
                "publish_rate": 50.0,
                "base_body_name": base_body,
                "odom_frame": "odom",
                "base_frame": base_body,
                # MUST be False — swarm_lio_tf_adapter owns the dynamic TF.
                "publish_tf": False,
                "pose_topic": f"/mujoco_sim/{pose_sensor}/pose",
                "imu_topic": f"/mujoco_sim/{imu_sensor}/imu",
                "republish_imu_topic": "imu/data",
            }],
            remappings=[
                ("/tf", f"/{ns}/tf"),
                ("/tf_static", f"/{ns}/tf_static"),
            ],
            output="screen",
        ),
        Node(
            package="mujoco_sensor_bridge",
            executable="mujoco_contact_node",
            namespace=ns,
            name="mujoco_contact_bridge",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"mjcf_path": mjcf_path},
                {"publish_rate": 50.0},
                links_config,
            ],
            output="screen",
        ),
    ]


def _build_swarm_lio2_nav_stack(
    *,
    ns: str,
    drone_id: int,
    mujoco_lidar_topic: str,
    base_frame: str,
    use_sim_time: bool,
    slam_delay: float,
    nav_delay: float,
    go2w_config_pkg: str,
    local_planner_paths_dir: str,
    far_tuning_yaml: str,
    far_default_yaml: str,
):
    """Per-robot Swarm-LIO2 (via docker relays) + octomap + FAR nav stack.

    Same shape as fastlio_dual's _build_fastlio_nav_stack but:
      * Fast-LIO node deleted (docker stack does SLAM externally).
      * Replaced with 2 input relays + 1 output relay + swarm_lio_tf_adapter.
      * `body→base_frame` static replaced with `map→quadN/world` +
        `quadN_aft_mapped→base_frame`.
    """
    tf_remaps = [("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")]
    actions = []

    robot_token = "robot_a" if drone_id == 1 else "robot_b"
    quad_world = f"quad{drone_id}/world"
    quad_aft = f"quad{drone_id}_aft_mapped"

    # ── QoS bridge: BE LiDAR → Reliable ──
    actions.append(
        Node(
            package="go2w_perception",
            executable="qos_bridge.py",
            namespace=ns,
            name="qos_bridge",
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_topic": mujoco_lidar_topic,
                "output_topic": f"/{ns}/registered_scan_reliable",
                "input_reliability": "best_effort",
                "output_reliability": "reliable",
            }],
            output="screen",
        )
    )

    # ── pointcloud_adapter: BE-cloud → 16-ring velodyne layout (kept for parity
    #    with fastlio_dual; even though docker swarm_lio2 reads /robot_X/
    #    velodyne_points from the relay below, the topic published by
    #    pointcloud_adapter is unused here. Left in to keep tooling that
    #    introspects /{ns}/velodyne_points alive.) ──
    actions.append(
        Node(
            package="go2w_perception",
            executable="pointcloud_adapter.py",
            namespace=ns,
            name="pointcloud_adapter",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"input_topic": f"/{ns}/registered_scan_reliable"},
                {"output_topic": f"/{ns}/velodyne_points"},
                {"num_rings": 16},
            ],
            output="screen",
        )
    )

    # ── pointcloud_to_laserscan (visual + 2D consumers) ──
    actions.append(
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            namespace=ns,
            name="pc_to_laserscan",
            parameters=[{
                "use_sim_time": use_sim_time,
                "target_frame": base_frame,
                "transform_tolerance": 0.1,
                "min_height": 0.05,
                "max_height": 0.60,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0087,
                "scan_time": 0.1,
                "range_min": 0.3,
                "range_max": 30.0,
                "use_inf": True,
            }],
            remappings=[
                ("cloud_in", f"/{ns}/registered_scan_reliable"),
                ("scan", f"/{ns}/scan_3d"),
            ] + tf_remaps,
            output="screen",
        )
    )

    # ── Static TFs: world ≡ map ≡ quadN/world ≡ odom (all identity) ──
    # Dynamic quadN/world → quadN_aft_mapped is published by
    # swarm_lio_tf_adapter from the relayed Odometry. We wrap that with
    # two statics so nav consumers can resolve map→base_frame.
    static_tfs = [
        ("world", "map"),
        ("map", "odom"),
        ("map", quad_world),
        (quad_aft, base_frame),
    ]
    for parent, child in static_tfs:
        sanitized = f"{parent}_to_{child}".replace("/", "_").replace("-", "_")
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                namespace=ns,
                name=f"{sanitized}_tf",
                arguments=[
                    "--frame-id", parent, "--child-frame-id", child,
                    "--x", "0", "--y", "0", "--z", "0",
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                ],
                remappings=[("/tf_static", f"/{ns}/tf_static")],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )
        )

    # ── Sim → docker input relays (ros1_bridge handles the rest) ──
    # NO IMU relay: ns == robot_token here (`robot_a` / `robot_b`), so
    # `/<ns>/imu/data → /<robot_token>/imu/data` would be a self-loop —
    # topic_tools relay republishes its own input and the rate explodes.
    # mujoco_odom_bridge already publishes IMU directly to /{ns}/imu/data
    # which bridge.yaml subscribes to on the ROS2 side. Lidar still needs
    # a relay because qos_bridge outputs `registered_scan_reliable` but
    # bridge.yaml expects `velodyne_points`.
    actions.append(
        TimerAction(period=slam_delay - 12.0, actions=[
            Node(
                package="topic_tools", executable="relay",
                namespace=ns, name=f"sim_lidar_to_swarm_lio2_in_{robot_token}",
                arguments=[
                    f"/{ns}/registered_scan_reliable",
                    f"/{robot_token}/velodyne_points",
                ],
                output="log",
            ),
        ])
    )

    # ── Docker → ROS2 output relay: swarm_lio Odometry → /{ns}/Odometry ──
    actions.append(
        TimerAction(period=slam_delay - 10.0, actions=[
            Node(
                package="topic_tools", executable="relay",
                namespace=ns, name=f"swarm_lio2_odom_to_sim_{robot_token}",
                arguments=[
                    f"/{robot_token}/swarm_lio2_raw/Odometry",
                    f"/{ns}/Odometry",
                ],
                output="log",
            ),
        ])
    )

    # ── swarm_lio_tf_adapter: dynamic quadN/world → quadN_aft_mapped +
    #    cloud frame rewrite. Node name CLI-renamed per robot to avoid
    #    multi-instance collision (the adapter source hardcodes node name
    #    as "swarm_lio_tf_adapter"). ──
    actions.append(
        TimerAction(period=slam_delay - 10.0, actions=[
            ExecuteProcess(
                cmd=[
                    "python3", _SWARM_LIO_TF_ADAPTER,
                    "--ros-args",
                    "-r", f"__node:=swarm_lio_tf_adapter_{robot_token}",
                    "-p", f"odom_input_topic:=/{robot_token}/swarm_lio2_raw/Odometry",
                    "-p", f"cloud_input_topic:=/{robot_token}/swarm_lio2_raw/cloud_static",
                    "-p", f"cloud_output_topic:=/{ns}/cloud_registered_body",
                    "-p", f"cloud_output_frame_id:={quad_aft}",
                    "-p", "publish_tf:=true",
                    "-r", f"/tf:=/{ns}/tf",
                    "-r", f"/tf_static:=/{ns}/tf_static",
                ],
                name=f"swarm_lio_tf_adapter_{robot_token}",
                output="screen",
            ),
        ])
    )

    # ── slam_odom_relay: /{ns}/Odometry → /{ns}/odom/nav (FAR consumes nav) ──
    actions.append(
        TimerAction(period=slam_delay, actions=[
            Node(
                package="go2w_perception",
                executable="slam_odom_relay.py",
                namespace=ns,
                name="slam_odom_relay",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "input_topic": f"/{ns}/Odometry",
                    "gt_topic": f"/{ns}/odom/ground_truth",
                    "output_topic": f"/{ns}/odom/nav",
                    "output_frame_id": "world",
                    "output_child_frame_id": base_frame,
                    "bootstrap_from_gt": True,
                    "require_gt_for_alignment": True,
                }],
                remappings=tf_remaps,
                output="screen",
            ),
        ])
    )

    # ── pointcloud_frame_bridge: cloud_registered_body (quadN_aft_mapped) →
    #    /{ns}/registered_scan_map (map frame) — feeds FAR / terrain_analysis ──
    actions.append(
        TimerAction(period=slam_delay + 0.5, actions=[
            Node(
                package="go2w_perception",
                executable="pointcloud_frame_bridge.py",
                namespace=ns,
                name="registered_scan_frame_bridge",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_topic": f"/{ns}/cloud_registered_body"},
                    {"output_topic": f"/{ns}/registered_scan_map"},
                    {"target_frame": "map"},
                    {"tf_timeout_sec": 0.15},
                    {"transform_wait_sec": 0.10},
                    {"max_cloud_age_sec": 0.80},
                ],
                remappings=tf_remaps,
                output="screen",
            ),
        ])
    )

    # ── Octomap: /{ns}/map OccupancyGrid (cloud_in = raw reliable scan) ──
    octomap_node = Node(
        package="octomap_server",
        executable="octomap_server_node",
        namespace=ns,
        name="octomap_server",
        parameters=[{
            "use_sim_time": use_sim_time,
            "resolution": 0.05,
            "frame_id": "map",
            "base_frame_id": base_frame,
            "sensor_model.max_range": 6.0,
            "sensor_model.hit": 0.8,
            "sensor_model.miss": 0.35,
            "sensor_model.min": 0.12,
            "sensor_model.max": 0.97,
            "point_cloud_min_z": 0.20,
            "point_cloud_max_z": 1.10,
            "occupancy_min_z": 0.20,
            "occupancy_max_z": 1.00,
            "filter_ground_plane": False,
            "filter_speckles": False,
            "compress_map": True,
            "latch": True,
            "publish_free_space": False,
        }],
        remappings=[
            ("cloud_in", f"/{ns}/registered_scan_reliable"),
            ("projected_map", f"/{ns}/map"),
        ] + tf_remaps,
        output="screen",
    )
    actions.append(TimerAction(period=slam_delay + 1.0, actions=[octomap_node]))

    # ── FAR stack (verbatim from fastlio_dual) ──
    far_scan_topic = f"/{ns}/registered_scan_map"
    far_odom_topic = f"/{ns}/odom/nav"
    far_max_speed = 0.2

    far_nodes = [
        Node(
            package="sensor_scan_generation",
            executable="sensorScanGeneration",
            namespace=ns,
            name="sensor_scan_generation",
            arguments=["--ros-args", "--log-level", "WARN"],
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[
                ("/state_estimation", far_odom_topic),
                ("/registered_scan", far_scan_topic),
                ("/state_estimation_at_scan", f"/{ns}/state_estimation_at_scan"),
                ("/sensor_scan", f"/{ns}/sensor_scan"),
            ] + tf_remaps,
            output="screen",
        ),
        Node(
            package="terrain_analysis",
            executable="terrainAnalysis",
            namespace=ns,
            name="terrain_analysis",
            arguments=["--ros-args", "--log-level", "WARN"],
            parameters=[{"use_sim_time": use_sim_time, "maxRelZ": 0.8}],
            remappings=[
                ("/state_estimation", far_odom_topic),
                ("/registered_scan", far_scan_topic),
                ("/joy", f"/{ns}/joy"),
                ("/map_clearing", f"/{ns}/map_clearing"),
                ("/terrain_map", f"/{ns}/terrain_map"),
            ],
            output="screen",
        ),
        Node(
            package="terrain_analysis_ext",
            executable="terrainAnalysisExt",
            namespace=ns,
            name="terrain_analysis_ext",
            arguments=["--ros-args", "--log-level", "WARN"],
            parameters=[{"use_sim_time": use_sim_time, "maxRelZ": 0.8}],
            remappings=[
                ("/state_estimation", far_odom_topic),
                ("/registered_scan", far_scan_topic),
                ("/joy", f"/{ns}/joy"),
                ("/cloud_clearing", f"/{ns}/cloud_clearing"),
                ("/terrain_map", f"/{ns}/terrain_map"),
                ("/terrain_map_ext", f"/{ns}/terrain_map_ext"),
            ],
            output="screen",
        ),
        Node(
            package="far_planner",
            executable="far_planner",
            namespace=ns,
            name="far_planner",
            parameters=[
                _load_yaml_params(far_default_yaml),
                far_tuning_yaml,
                {
                    "use_sim_time": use_sim_time,
                    "graph_msger/robot_id": 0 if ns == "robot_a" else 1,
                },
            ],
            remappings=[
                ("/odom_world", far_odom_topic),
                ("/terrain_cloud", f"/{ns}/terrain_map_ext"),
                ("/scan_cloud", f"/{ns}/terrain_map"),
                ("/terrain_local_cloud", far_scan_topic),
                ("/goal_point", f"/{ns}/way_point_coord"),
                ("/way_point", f"/{ns}/way_point"),
                ("/joy", f"/{ns}/joy"),
                ("/navigation_boundary", f"/{ns}/navigation_boundary"),
                ("/runtime", f"/{ns}/far_runtime"),
                ("/planning_time", f"/{ns}/far_planning_time"),
                ("/robot_vgraph", f"/{ns}/robot_vgraph"),
                ("/decoded_vgraph", f"/{ns}/decoded_vgraph"),
            ] + tf_remaps,
            output="screen",
        ),
        Node(
            package="local_planner",
            executable="localPlanner",
            namespace=ns,
            name="localPlanner",
            parameters=[{
                "use_sim_time": use_sim_time,
                "pathFolder": local_planner_paths_dir,
                "vehicleLength": 0.70,
                "vehicleWidth": 0.40,
                "sensorOffsetX": 0.0,
                "sensorOffsetY": 0.0,
                "twoWayDrive": True,
                "laserVoxelSize": 0.05,
                "terrainVoxelSize": 0.2,
                "useTerrainAnalysis": True,
                "checkObstacle": True,
                "checkRotObstacle": True,
                "adjacentRange": 4.0,
                "obstacleHeightThre": 0.20,
                "groundHeightThre": 0.1,
                "costHeightThre": 0.1,
                "costScore": 0.02,
                "useCost": False,
                "pointPerPathThre": 2,
                "minRelZ": -0.5,
                "maxRelZ": 1.2,
                "maxSpeed": far_max_speed,
                "dirWeight": 0.5,
                "dirThre": 90.0,
                "dirToVehicle": False,
                "pathScale": 1.0,
                "minPathScale": 0.75,
                "pathScaleStep": 0.25,
                "pathScaleBySpeed": False,
                "minPathRange": 1.0,
                "pathRangeStep": 0.5,
                "pathRangeBySpeed": False,
                "pathCropByGoal": True,
                "autonomyMode": True,
                "autonomySpeed": far_max_speed,
                "joyToSpeedDelay": 2.0,
                "joyToCheckObstacleDelay": 5.0,
                "goalClearRange": 0.5,
                "goalX": 0.0,
                "goalY": 0.0,
            }],
            remappings=[
                ("/state_estimation", far_odom_topic),
                ("/registered_scan", far_scan_topic),
                ("/way_point", f"/{ns}/way_point"),
                ("/terrain_map", f"/{ns}/terrain_map"),
                ("/overall_map", f"/{ns}/terrain_map"),
                ("/joy", f"/{ns}/joy"),
                ("/path", f"/{ns}/local_path"),
                ("/freePaths", f"/{ns}/free_paths"),
            ],
            output="screen",
        ),
        Node(
            package="local_planner",
            executable="pathFollower",
            namespace=ns,
            name="pathFollower",
            parameters=[{
                "use_sim_time": use_sim_time,
                "sensorOffsetX": 0.0,
                "sensorOffsetY": 0.0,
                "pubSkipNum": 1,
                "twoWayDrive": True,
                "lookAheadDis": 0.8,
                "yawRateGain": 1.0,
                "stopYawRateGain": 0.8,
                "maxYawRate": 45.0,
                "maxSpeed": far_max_speed,
                "maxAccel": 2.0,
                "switchTimeThre": 1.0,
                "dirDiffThre": 1.2,
                "omniDirDiffThre": 1.5,
                "noRotSpeed": 10.0,
                "stopDisThre": 0.25,
                "slowDwnDisThre": 0.50,
                "useInclRateToSlow": False,
                "inclRateThre": 120.0,
                "slowRate1": 0.25,
                "slowRate2": 0.5,
                "slowTime1": 2.0,
                "slowTime2": 2.0,
                "useInclToStop": False,
                "inclThre": 45.0,
                "stopTime": 5.0,
                "noRotAtStop": False,
                "noRotAtGoal": True,
                "autonomyMode": True,
                "autonomySpeed": far_max_speed,
                "joyToSpeedDelay": 2.0,
                "goalCloseDis": 0.6,
                "is_real_robot": False,
            }],
            remappings=[
                ("/state_estimation", far_odom_topic),
                ("/path", f"/{ns}/local_path"),
                ("/cmd_vel", f"/{ns}/cmd_vel_stamped"),
                ("/joy", f"/{ns}/joy"),
                ("/speed", f"/{ns}/speed"),
                ("/stop", f"/{ns}/stop"),
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            namespace=ns,
            name="far_vehicle_tf",
            arguments=["0", "0", "0", "0", "0", "0", "sensor", "vehicle"],
            remappings=tf_remaps,
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            namespace=ns,
            name="far_camera_tf",
            arguments=["0", "0", "0", "-1.5707963", "0", "-1.5707963", "sensor", "camera"],
            remappings=tf_remaps,
            output="screen",
        ),
        Node(
            package="go2w_perception",
            executable="twist_bridge.py",
            namespace=ns,
            name="twist_bridge",
            remappings=[
                ("/cmd_vel_stamped", f"/{ns}/cmd_vel_stamped"),
                ("/cmd_vel", f"/{ns}/cmd_vel"),
            ],
            output="screen",
        ),
        Node(
            package="go2w_control",
            executable="go2w_hybrid_cmd_router.py",
            namespace=ns,
            name="go2w_hybrid_cmd_router",
            parameters=[
                os.path.join(
                    go2w_config_pkg, "config", "control",
                    "go2w_hybrid_motion.yaml",
                ),
                {
                    "use_sim_time": use_sim_time,
                    "wheel_state_topic": "/mujoco_sim/joint_states",
                    "wheel_joint_names": (
                        ["FL_foot_joint", "FR_foot_joint",
                         "RL_foot_joint", "RR_foot_joint"]
                        if ns == "robot_a" else
                        ["b_FL_foot_joint", "b_FR_foot_joint",
                         "b_RL_foot_joint", "b_RR_foot_joint"]
                    ),
                    "wheel_command_topic":
                        f"/mujoco_sim/{ns}_wheel_velocity_controller/commands",
                },
            ],
            output="screen",
        ),
    ]
    actions.append(TimerAction(period=nav_delay, actions=far_nodes))

    return actions


def _launch_setup(context):
    use_sim_time = True
    gui = _as_bool(_get(context, "gui"))
    explore = _as_bool(_get(context, "explore"))
    cleanup_stale = _as_bool(_get(context, "cleanup_stale"))
    mujoco_model_path = _get(context, "mujoco_model_path").strip()
    session_duration_sec = float(_get(context, "session_duration_sec"))
    session_output_dir = _get(context, "session_output_dir").strip()
    scene_area_m2 = float(_get(context, "scene_area_m2"))
    collision_output = _get(context, "collision_output_path").strip()

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    go2w_config_pkg = get_package_share_directory("go2w_config")
    cfpa2_pkg = get_package_share_directory("cfpa2_collaborative_autonomy")
    champ_base_pkg = get_package_share_directory("champ_base")
    far_pkg = get_package_share_directory("far_planner")
    local_planner_pkg = get_package_share_directory("local_planner")

    if not mujoco_model_path:
        mujoco_model_path = os.path.join(go2_gazebo_pkg, "mujoco", "demo3_dual.xml")

    ros2_control_config = os.path.join(
        go2_gazebo_pkg, "config", "ros_control", "ros_control_dual_mujoco_nav.yaml"
    )

    # ── URDF generation ──
    base_robot_description = xacro.process_file(
        os.path.join(go2_gazebo_pkg, "urdf", "go2w", "go2w_description_3d_lidar.xacro"),
    ).documentElement.toxml()
    combined_urdf = build_dual_nav_urdf(base_robot_description)
    robot_a_urdf = build_namespaced_robot_description(
        base_robot_description, "robot_a",
        os.path.join(go2_gazebo_pkg, "config", "ros_control", "ros_control_go2w_robot_a.yaml"),
    )
    robot_b_urdf = build_robot_b_urdf(base_robot_description)

    joints_a = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "joints.yaml")
    joints_b = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "joints_robot_b.yaml")
    links_a = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "links.yaml")
    links_b = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "links_robot_b.yaml")
    gait_config = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "gait.yaml")
    ekf_base = os.path.join(champ_base_pkg, "config", "ekf", "base_to_footprint.yaml")
    ekf_odom = os.path.join(champ_base_pkg, "config", "ekf", "footprint_to_odom.yaml")

    far_tuning_yaml = os.path.join(go2w_config_pkg, "config", "nav", "far_planner_tuning.yaml")
    far_default_yaml = os.path.join(far_pkg, "config", "default.yaml")
    local_planner_paths_dir = os.path.join(local_planner_pkg, "paths")

    mujoco_plugin_dir = _find_mujoco_plugin_dir()
    sim_ns = "mujoco_sim"

    actions = [LogInfo(msg="[swarm_lio2_dual] starting dual-robot Swarm-LIO2 nav")]

    # ── Docker compose lifecycle: brings up ROS1 master + 2× swarm_lio2 +
    #    ros1_bridge. Pre-built image expected; rebuild during launch would
    #    race the MuJoCo bringup. ──
    actions.append(LogInfo(msg=(
        f"[swarm_lio2_dual] docker compose up -d at {_DOCKER_COMPOSE_DIR}. "
        "Expect /robot_a/swarm_lio2_raw/Odometry and "
        "/robot_b/swarm_lio2_raw/Odometry on ROS2 within ~15s."
    )))
    actions.append(
        ExecuteProcess(
            cmd=["docker", "compose", "up", "-d"],
            cwd=_DOCKER_COMPOSE_DIR,
            output="screen",
        )
    )
    actions.append(
        RegisterEventHandler(
            OnShutdown(on_shutdown=[
                LogInfo(msg="[swarm_lio2_dual] tearing down docker compose stack"),
                ExecuteProcess(
                    cmd=["docker", "compose", "down"],
                    cwd=_DOCKER_COMPOSE_DIR,
                    output="screen",
                ),
            ])
        )
    )

    if cleanup_stale:
        actions.append(
            ExecuteProcess(cmd=["bash", "-lc", _build_cleanup_stale_cmd()], output="screen")
        )

    # ── T=3: MuJoCo ──
    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        namespace=sim_ns,
        parameters=[
            {"robot_description": combined_urdf},
            ros2_control_config,
            {"robot_model_path": mujoco_model_path},
            # Lowered 500 → 200 (2026-05-11): see swarm_lio2_mixed launch for
            # full rationale. tl;dr CPU saturated at 500 Hz dual-robot, IMU
            # rate to swarm_lio2 collapsed and odom diverged.
            {"simulation_frequency": 200.0},
            {"real_time_factor": 1.0},
            {"clock_publisher_frequency": 100.0},
            {"show_gui": gui},
        ],
        remappings=[
            (f"/{sim_ns}/controller_manager/robot_description", f"/{sim_ns}/robot_description"),
        ],
        additional_env={"MUJOCO_PLUGIN_DIR": mujoco_plugin_dir},
        output="screen",
    )
    actions.append(TimerAction(period=3.0, actions=[mujoco_node]))

    # ── T=5: Sensor bridges (publish_tf=False — swarm_lio_tf_adapter owns TF) ──
    sensor_actions = []
    sensor_actions.extend(
        _build_sensor_bridges(
            ns="robot_a", mjcf_path=mujoco_model_path,
            base_body="base_link", imu_site="imu",
            pose_sensor="base_link_site_pose_sensor",
            imu_sensor="imu_imu_sensor",
            links_config=links_a,
            use_sim_time=use_sim_time,
        )
    )
    sensor_actions.extend(
        _build_sensor_bridges(
            ns="robot_b", mjcf_path=mujoco_model_path,
            base_body="b_base_link", imu_site="b_imu",
            pose_sensor="b_base_link_site_pose_sensor",
            imu_sensor="b_imu_imu_sensor",
            links_config=links_b,
            use_sim_time=use_sim_time,
        )
    )
    actions.append(TimerAction(period=5.0, actions=sensor_actions))

    # ── T=7 / T=10: Per-robot CHAMP stacks ──
    robot_a_stack = build_dual_robot_stack(
        ns="robot_a",
        spawn_x="4.0", spawn_y="2.0", spawn_yaw="0.0",
        use_sim_time=use_sim_time,
        robot_description=robot_a_urdf,
        joints_config=joints_a, links_config=links_a,
        gait_config=gait_config,
        ekf_base_to_footprint=ekf_base,
        ekf_footprint_to_odom=ekf_odom,
        activate_controllers_on_spawn=True,
        stand_up_joint_preset="go2",
        cmd_vel_input_topic="cmd_vel_legged",
        wheel_controller_name="robot_a_wheel_velocity_controller",
        use_mujoco=True,
        controller_manager_name=f"/{sim_ns}/controller_manager",
    )
    actions.append(TimerAction(period=7.0, actions=robot_a_stack))

    robot_b_stack = build_dual_robot_stack(
        ns="robot_b",
        spawn_x="4.0", spawn_y="-6.0", spawn_yaw="0.0",
        use_sim_time=use_sim_time,
        robot_description=robot_b_urdf,
        joints_config=joints_b, links_config=links_b,
        gait_config=gait_config,
        ekf_base_to_footprint=ekf_base,
        ekf_footprint_to_odom=ekf_odom,
        activate_controllers_on_spawn=True,
        stand_up_joint_preset="go2",
        stand_up_joint_prefix="b_",
        cmd_vel_input_topic="cmd_vel_legged",
        wheel_controller_name="robot_b_wheel_velocity_controller",
        use_mujoco=True,
        controller_manager_name=f"/{sim_ns}/controller_manager",
    )
    actions.append(TimerAction(period=10.0, actions=robot_b_stack))

    # ── Per-robot Swarm-LIO2 (via docker relays) + FAR nav stacks ──
    slam_delay = 20.0   # docker side has ~10s warm-up by now
    nav_delay = slam_delay + 5.0

    actions.extend(
        _build_swarm_lio2_nav_stack(
            ns="robot_a", drone_id=1,
            mujoco_lidar_topic="/mujoco_sim/mujoco_lidar_sensor/registered_scan",
            base_frame="base_link",
            use_sim_time=use_sim_time,
            slam_delay=slam_delay, nav_delay=nav_delay,
            go2w_config_pkg=go2w_config_pkg,
            local_planner_paths_dir=local_planner_paths_dir,
            far_tuning_yaml=far_tuning_yaml,
            far_default_yaml=far_default_yaml,
        )
    )
    actions.extend(
        _build_swarm_lio2_nav_stack(
            ns="robot_b", drone_id=2,
            mujoco_lidar_topic="/mujoco_sim/b_mujoco_lidar_sensor/registered_scan",
            base_frame="b_base_link",
            use_sim_time=use_sim_time,
            slam_delay=slam_delay, nav_delay=nav_delay,
            go2w_config_pkg=go2w_config_pkg,
            local_planner_paths_dir=local_planner_paths_dir,
            far_tuning_yaml=far_tuning_yaml,
            far_default_yaml=far_default_yaml,
        )
    )

    # ── CFPA2 dual-robot coordinator ──
    if explore:
        cfpa2_config_path = os.path.join(cfpa2_pkg, "config", "cfpa2_coordinator.yaml")
        if not os.path.exists(cfpa2_config_path):
            cfpa2_config_path = os.path.join(cfpa2_pkg, "config", "cfpa2_single_robot.yaml")
        actions.append(
            TimerAction(
                period=nav_delay + 2.0,
                actions=[
                    Node(
                        package="cfpa2_collaborative_autonomy",
                        executable="cfpa2_coordinator_node",
                        name="cfpa2_coordinator",
                        parameters=[
                            cfpa2_config_path,
                            {
                                "use_sim_time": use_sim_time,
                                "namespaces": ["robot_a", "robot_b"],
                                "goal_topic_suffix": "/way_point_coord",
                                "marker_frame_override": "map",
                            },
                        ],
                        output="screen",
                    ),
                ],
            )
        )

    # ── Inter-robot collision monitor ──
    collision_monitor_script = os.path.join(_ws_root, "scripts/runtime/dual_robot_collision_monitor.py")
    collision_args = ["python3", "-u", collision_monitor_script]
    if collision_output:
        collision_args += ["--output", collision_output]
    actions.append(
        TimerAction(
            period=3.5,
            actions=[
                ExecuteProcess(
                    cmd=collision_args,
                    name="dual_robot_collision_monitor",
                    output="screen",
                ),
            ],
        )
    )

    # ── Session reporter(s) ──
    if session_duration_sec > 0 and session_output_dir:
        os.makedirs(session_output_dir, exist_ok=True)
        reporter_script = os.path.join(_ws_root, "scripts/bench/session_reporter.py")
        last_reporter = None
        for ns in ("robot_a", "robot_b"):
            out_path = os.path.join(session_output_dir, f"{ns}.json")
            proc = ExecuteProcess(
                cmd=[
                    "python3", "-u", reporter_script,
                    "--duration", str(session_duration_sec),
                    "--namespace", ns,
                    "--output", out_path,
                    "--scene-area-m2", str(scene_area_m2),
                ],
                name=f"session_reporter_{ns}",
                output="screen",
            )
            actions.append(TimerAction(period=nav_delay + 3.0, actions=[proc]))
            last_reporter = proc
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=last_reporter,
                    on_exit=[
                        LogInfo(msg="[swarm_lio2_dual] session reporter exited — shutdown"),
                        Shutdown(reason="session complete"),
                    ],
                )
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("explore", default_value="true"),
        DeclareLaunchArgument("cleanup_stale", default_value="true"),
        DeclareLaunchArgument(
            "mujoco_model_path", default_value="",
            description="Path to MJCF scene. Defaults to demo3_dual.xml.",
        ),
        DeclareLaunchArgument("session_duration_sec", default_value="0.0"),
        DeclareLaunchArgument("session_output_dir", default_value=""),
        DeclareLaunchArgument("scene_area_m2", default_value="384.0"),
        DeclareLaunchArgument(
            "collision_output_path",
            default_value="/tmp/dual_robot_collision_report.json",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
