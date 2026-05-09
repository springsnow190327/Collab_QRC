#!/usr/bin/env python3
"""MuJoCo nav test — swarm_lio2 (ROS1 in docker) + CHAMP + Nav2 SE2 + CFPA2.

Single Go2 (no wheels) only. Validates the swarm_lio2 SLAM swap before real
hardware. Cousin of nav_test_mujoco_fastlio.launch.py — same MuJoCo+CHAMP
base, same Nav2/CFPA2 stack, but Fast-LIO2 is replaced by the dockerized
ROS1 Swarm-LIO2 stack and the TF chain is reshaped accordingly.

Key differences vs nav_test_mujoco_fastlio.launch.py
─────────────────────────────────────────────────────
  - `enable_slam=false, use_fast_lio=false` on the base launch — we
    don't run ROS2 fast_lio_mapping; the docker stack does the SLAM.
  - `odom_bridge_publish_tf=false` — mujoco_odom_bridge must NOT
    publish odom→base_link TF, or it competes with swarm_lio2's
    map→base_link chain on the same `base_link` frame.
  - Docker compose stack (docker/ros1_hybrid_slam/) is started by this
    launch and torn down on shutdown.
  - 4 ROS2 topic relays bridge sim→docker→sim:
      /<ns>/registered_scan_reliable → /robot_a/velodyne_points  (in)
      /<ns>/imu/data                 → /robot_a/imu/data         (in)
      /robot_a/swarm_lio2_raw/Odometry         → /<ns>/Odometry  (out)
    The cloud_static output runs through swarm_lio_tf_adapter (frame_id
    rewrite — see node docstring for the upstream bug).
  - swarm_lio_tf_adapter publishes the dynamic TF
      quad1/world → quad1_aft_mapped
    from the bridged Odometry, preserving the message's own frame names.
  - Static TFs wrap that dynamic with the gravity-aligned chain.

TF chain (sim — no IMU mount tilt, all statics identity)
─────────────────────────────────────────────────────────
   map ── (static identity) ── quad1/world
                                   │
                  (dynamic, swarm_lio_tf_adapter)
                                   │
                                   ▼
                            quad1_aft_mapped
                                   │
                          (static identity)
                                   │
                                   ▼
                              base_link ── (RSP) ── URDF tree

   map ── (static identity) ── odom        # phantom for Nav2 local_costmap

Diagnostic:
  ros2 run tf2_ros tf2_echo map base_link \\
      --ros-args -r /tf:=/robot/tf -r /tf_static:=/robot/tf_static
  ros2 topic hz /robot_a/swarm_lio2_raw/Odometry      # docker → ROS2
  ros2 topic hz /robot/Odometry                       # post-relay
  ros2 topic hz /robot/cloud_registered_body          # post-frame-rewrite
  docker compose -f docker/ros1_hybrid_slam/docker-compose.yml ps

Usage:
  ros2 launch go2_gazebo_sim nav_test_mujoco_swarm_lio2_se2.launch.py
  ros2 launch go2_gazebo_sim nav_test_mujoco_swarm_lio2_se2.launch.py gui:=false
  ros2 launch go2_gazebo_sim nav_test_mujoco_swarm_lio2_se2.launch.py rviz:=true
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_ws_root = os.path.abspath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "..", "..", "..", ".."
))
_DOCKER_COMPOSE_DIR = os.path.join(_ws_root, "docker", "ros1_hybrid_slam")


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _get(context, key: str) -> str:
    return LaunchConfiguration(key).perform(context)


def _launch_setup(context):
    use_sim_time = True
    robot_ns = _get(context, "robot_namespace").strip().strip("/") or "robot"
    gui = _get(context, "gui")
    rviz = _as_bool(_get(context, "rviz"))
    explore = _as_bool(_get(context, "explore"))
    spawn_x = _get(context, "spawn_x")
    spawn_y = _get(context, "spawn_y")
    spawn_yaw = _get(context, "spawn_yaw")
    mujoco_model_path = _get(context, "mujoco_model_path").strip()

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    go2w_config_pkg = get_package_share_directory("go2w_config")
    cfpa2_pkg = get_package_share_directory("cfpa2_collaborative_autonomy")

    if not mujoco_model_path:
        mujoco_model_path = os.path.join(
            go2_gazebo_pkg, "mujoco", "demo1_go2_real.xml"
        )

    tf_remaps = [("/tf", f"/{robot_ns}/tf"), ("/tf_static", f"/{robot_ns}/tf_static")]

    actions = []

    # ── Docker compose lifecycle: start swarm_lio2 (ROS1) + ros1_bridge ──
    # `up -d` runs detached so this launch keeps moving; shutdown handler
    # below tears it back down on Ctrl+C.
    actions.append(LogInfo(msg=(
        "[swarm_lio2_se2] starting docker compose stack at "
        f"{_DOCKER_COMPOSE_DIR}. Topics expected on ROS2 within ~10s: "
        "/robot_a/swarm_lio2_raw/Odometry, /robot_a/swarm_lio2_raw/cloud_static."
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
                LogInfo(msg="[swarm_lio2_se2] tearing down docker compose stack"),
                ExecuteProcess(
                    cmd=["docker", "compose", "down"],
                    cwd=_DOCKER_COMPOSE_DIR,
                    output="screen",
                ),
            ])
        )
    )

    # ── 1. Base platform: MuJoCo + CHAMP + sensors + perception (NO SLAM) ──
    base_launch = os.path.join(
        go2_gazebo_pkg, "launch", "single_go2w_mujoco_cfpa2.launch.py"
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                "robot_namespace": robot_ns,
                "use_sim_time": "true",
                "gui": gui,
                "rviz": "false",
                "cleanup_stale": "true",
                "enable_assets": "true",
                "enable_perception": "true",
                # SLAM intentionally OFF — docker swarm_lio2 owns map→base_link.
                "enable_slam": "false",
                "use_fast_lio": "false",
                "enable_control": "true",
                "enable_navigation": "false",  # we add navigation below
                # CRITICAL: mujoco_odom_bridge must NOT publish TF, or it
                # competes with swarm_lio2's map→base_link chain on the
                # `base_link` parent. It still publishes /odom topic and
                # the IMU data, which we relay into the docker stack.
                "odom_bridge_publish_tf": "false",
                "mujoco_model_path": mujoco_model_path,
                "spawn_x": spawn_x,
                "spawn_y": spawn_y,
                "spawn_yaw": spawn_yaw,
                "has_wheels": "false",  # Go2 only
            }.items(),
        )
    )

    # ── 2. Sim → docker input relays (run inside the robot's TF namespace) ──
    # Docker stack swarm_lio reads /robot_a/velodyne_points + /robot_a/imu/data
    # (defaults from ROBOT_A_SWARM_LIO2_LIDAR_TOPIC / _IMU_TOPIC env vars).
    # ros1_bridge bridges these names natively (see bridge.yaml). We just
    # rename the sim's existing reliable-QoS scan + IMU streams onto them.
    sim_lidar_topic = f"/{robot_ns}/registered_scan_reliable"
    sim_imu_topic = f"/{robot_ns}/imu/data"
    actions += [
        TimerAction(period=8.0, actions=[
            Node(
                package="topic_tools", executable="relay",
                name="sim_lidar_to_swarm_lio2_in",
                arguments=[sim_lidar_topic, "/robot_a/velodyne_points"],
                output="log",
            ),
            Node(
                package="topic_tools", executable="relay",
                name="sim_imu_to_swarm_lio2_in",
                arguments=[sim_imu_topic, "/robot_a/imu/data"],
                output="log",
            ),
        ]),
    ]

    # ── 3. Docker → ROS2 output relays (post-bridge) ──
    # /robot_a/swarm_lio2_raw/Odometry → /<ns>/Odometry: feeds
    # fast_lio_tf_adapter (publish_tf=false), which republishes as
    # /<ns>/odom/nav. swarm_lio_tf_adapter below covers TF + cloud frame.
    actions += [
        TimerAction(period=10.0, actions=[
            Node(
                package="topic_tools", executable="relay",
                name="swarm_lio2_odom_to_sim",
                arguments=[
                    "/robot_a/swarm_lio2_raw/Odometry",
                    f"/{robot_ns}/Odometry",
                ],
                output="log",
            ),
        ]),
    ]

    # ── 4. swarm_lio_tf_adapter: TF (quad1/world → quad1_aft_mapped)
    #       + cloud frame_id rewrite (→ /<ns>/cloud_registered_body) ──
    swarm_lio_tf_adapter_path = os.path.join(
        _ws_root, "scripts", "runtime", "swarm_lio_tf_adapter.py"
    )
    actions.append(
        TimerAction(period=10.0, actions=[
            ExecuteProcess(
                cmd=[
                    "python3", swarm_lio_tf_adapter_path,
                    "--ros-args",
                    "-p", "odom_input_topic:=/robot_a/swarm_lio2_raw/Odometry",
                    "-p", "cloud_input_topic:=/robot_a/swarm_lio2_raw/cloud_static",
                    "-p", f"cloud_output_topic:=/{robot_ns}/cloud_registered_body",
                    "-p", "cloud_output_frame_id:=quad1_aft_mapped",
                    "-p", "publish_tf:=true",
                    "-r", f"/tf:=/{robot_ns}/tf",
                    "-r", f"/tf_static:=/{robot_ns}/tf_static",
                ],
                output="screen",
            ),
        ])
    )

    # ── 5. Static TFs that wrap the swarm_lio dynamic ──
    # Sim has no IMU mount tilt — all statics are identity. (On real, the
    # map→quad1/world static would carry the forward tilt and
    # quad1_aft_mapped→base_link the inverse, mirroring the fastlio_mid360
    # chain in slam.launch.py.)
    actions += [
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="map_to_quad1_world",
            arguments=[
                "--frame-id", "map", "--child-frame-id", "quad1/world",
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            ],
            remappings=tf_remaps, output="log",
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="quad1_aft_mapped_to_base_link",
            arguments=[
                "--frame-id", "quad1_aft_mapped", "--child-frame-id", "base_link",
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            ],
            remappings=tf_remaps, output="log",
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="map_to_odom_identity",
            arguments=[
                "--frame-id", "map", "--child-frame-id", "odom",
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            ],
            remappings=tf_remaps, output="log",
        ),
        # `world` is the conventional Nav2 root in our sim; alias to map so
        # any consumer that asks for either gets the same identity.
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="world_to_map_identity",
            arguments=[
                "--frame-id", "world", "--child-frame-id", "map",
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            ],
            remappings=tf_remaps, output="log",
        ),
    ]

    # ── 6. Octomap_server (Go2-tuned bands; cloud from frame-rewriter) ──
    # Go2 (non-W) sensor at z≈0.39 m. Match nav_test_mujoco_fastlio.launch.py
    # values for octo_*_min_z (Go2 path). Subscribe to the rewritten cloud
    # so frame_id is `quad1_aft_mapped` — TF lookup map → quad1_aft_mapped
    # resolves through our static + dynamic chain.
    actions.append(
        TimerAction(period=12.0, actions=[
            Node(
                package="octomap_server", executable="octomap_server_node",
                namespace=robot_ns, name="octomap_map_gen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "resolution": 0.05,
                    "frame_id": "map",
                    "base_frame_id": "base_link",
                    "sensor_model.max_range": 8.0,
                    "sensor_model.hit": 0.8,
                    "sensor_model.miss": 0.35,
                    "sensor_model.min": 0.12,
                    "sensor_model.max": 0.97,
                    # Go2 (legged, sensor z≈0.39 m) — copy of fastlio sim Go2 path.
                    "point_cloud_min_z": 0.30,
                    "point_cloud_max_z": 1.10,
                    "occupancy_min_z": 0.30,
                    "occupancy_max_z": 1.00,
                    "filter_ground_plane": False,
                    "filter_speckles": True,
                    "compress_map": True,
                    "latch": True,
                    "publish_free_space": False,
                }],
                remappings=[
                    ("cloud_in", f"/{robot_ns}/cloud_registered_body"),
                    ("projected_map", f"/{robot_ns}/map"),
                ] + tf_remaps,
                output="screen",
            ),
        ])
    )

    # ── 7. Navigation: Nav2 MPPI + SE2 holonomic overlay + CFPA2 ──
    nav_launch = os.path.join(go2w_config_pkg, "launch", "navigation.launch.py")
    actions.append(
        TimerAction(period=14.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch),
                launch_arguments={
                    "robot_namespace": robot_ns,
                    "robot_model": "go2",
                    "use_sim_time": "true",
                    "map_frame": "map",
                    "remap_tf": "true",
                    "nav_backend": "nav2_mppi",
                    "scan_topic": f"/{robot_ns}/scan_3d",
                    "odom_topic": f"/{robot_ns}/odom/nav",
                    "registered_scan_topic": f"/{robot_ns}/registered_scan_reliable",
                    "waypoint_input_suffix": "/way_point_coord",
                    "nav_config": os.path.join(
                        go2w_config_pkg, "config", "nav", "default_nav_single_go2w.yaml"
                    ),
                    "cfpa2_config": os.path.join(
                        cfpa2_pkg, "config", "cfpa2_single_robot.yaml"
                    ),
                    "nav_map_topic": f"/{robot_ns}/map",
                    "explore": "true" if explore else "false",
                    # fast_lio_tf_adapter: input is the relayed swarm_lio
                    # Odometry (we map /robot_a/swarm_lio2_raw/Odometry →
                    # /<ns>/Odometry above). publish_tf=false because
                    # swarm_lio_tf_adapter already broadcasts the dynamic
                    # quad1/world→quad1_aft_mapped TF; this just emits
                    # /<ns>/odom/nav for the nav stack.
                    "fast_lio_input_topic": "Odometry",
                    "fast_lio_publish_tf": "false",
                    # SE2 sim overlay (nav2_se2_holonomic_overlay_sim.yaml).
                    "nav2_yaml_extra_override": "nav2_se2_holonomic_overlay_sim.yaml",
                    "max_linear_speed": "0.30",
                    # ── These are all declared with default_value="" by
                    # single_go2w_mujoco_cfpa2.launch.py (lines ~789-793) and
                    # share the LaunchContext, which suppresses
                    # navigation.launch.py's own non-empty defaults. We must
                    # pass them explicitly so the float() casts in
                    # navigation.launch.py:_setup don't blow up on "".
                    "far_max_speed": "0.30",
                    "far_robot_id": "0",
                    "cfpa2_w_ig": "0.5",
                    "cfpa2_w_c": "0.8",
                    "cfpa2_w_momentum": "2.5",
                    "cfpa2_min_utility": "-1.0",
                }.items(),
            ),
        ])
    )

    # ── 8. RViz (optional) ──
    if rviz:
        rviz_cfg = os.path.join(go2w_config_pkg, "config", "rviz", "autonomy.rviz")
        actions.append(
            TimerAction(period=18.0, actions=[
                Node(
                    package="rviz2", executable="rviz2",
                    name="rviz2_swarm_lio2_se2",
                    arguments=["-d", rviz_cfg],
                    parameters=[{"use_sim_time": use_sim_time}],
                    remappings=tf_remaps,
                    output="screen",
                ),
            ])
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="robot"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("explore", default_value="true"),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "mujoco_model_path", default_value="",
            description="Absolute path to MuJoCo MJCF (.xml). Empty → demo1_go2_real.xml.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
