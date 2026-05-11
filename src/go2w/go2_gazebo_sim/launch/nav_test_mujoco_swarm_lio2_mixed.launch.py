#!/usr/bin/env python3
"""Heterogeneous dual-robot MuJoCo + Swarm-LIO2 (docker) + Nav2 SE2 + CFPA2.

Cousin of nav_test_mujoco_fastlio_mixed.launch.py — Go2W (robot_a) + Go2
(robot_b), one MuJoCo process, one combined URDF. Fast-LIO2 swapped for
the dockerized ROS1 Swarm-LIO2 stack (drone_id=1 / 2).

Compared to the fastlio_mixed version the surgical diffs are:
  * docker compose lifecycle (started once, torn down on shutdown).
  * No `fast_lio` node; `fast_lio_tf_adapter` ALSO removed (it would
    contend for ownership of map↔base_link with the swarm adapter).
  * Per-robot relays:
      input:  /{ns}/registered_scan_reliable → /robot_X/velodyne_points
              /{ns}/imu/data                 → /robot_X/imu/data
      output: /robot_X/swarm_lio2_raw/Odometry → /{ns}/Odometry
  * Per-robot swarm_lio_tf_adapter (CLI-renamed per ns to avoid
    hardcoded-node-name collision); broadcasts dynamic
    `quadN/world → quadN_aft_mapped` and rewrites the cloud_static
    PointCloud2 to frame_id=`quadN_aft_mapped`.
  * Per-robot slam_odom_relay publishes /{ns}/odom/nav in frame `odom`
    so Nav2's local_costmap (global_frame: odom) resolves directly.
  * Static TFs: world→map→quadN/world (chained), quadN_aft_mapped→base_frame,
    map→odom (phantom). The Fast-LIO `body` static is gone.
  * SLAM-agnostic blocks (peer self_filter, map_augmenter, map_merge,
    CFPA2, dual_robot_collision_monitor, RViz, debug filter, Nav2 MPPI
    stack with SE2 overlay) are preserved verbatim.

Default scene: demo3_mixed.xml (Go2W at (4,2), Go2 at (4,-6), 24×16m).
"""
from __future__ import annotations

import os
import sys

_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)

_ws_root = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", "..", ".."))

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
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

from modules import _find_mujoco_plugin_dir
from modules.assets import build_dual_robot_stack, build_namespaced_robot_description
from modules.dual_urdf import build_mixed_mujoco_urdf, build_robot_b_urdf
from modules.launch_helpers import (
    as_bool as _as_bool,
    build_cleanup_stale_cmd as _build_cleanup_stale_cmd,
    get_launch_arg as _get,
    load_yaml_params as _load_yaml_params,
)


_DOCKER_COMPOSE_DIR = os.path.join(_ws_root, "docker", "ros1_hybrid_slam")
_SWARM_LIO_TF_ADAPTER = os.path.join(_ws_root, "scripts", "runtime", "swarm_lio_tf_adapter.py")


# Debug filter — same surface as fastlio_mixed.
_NAV_DEBUG_KEEP_EXECUTABLES = (
    "far_planner",
    "localPlanner",
    "pathFollower",
    "far_status_adapter",
    "twist_bridge",
    "go2w_hybrid_cmd_router",
    "cfpa2_coordinator",
    "dual_robot_collision_monitor",
    "map_augmenter",
    "robot_self_filter",
    "swarm_lio_tf_adapter",
)
_NAV_DEBUG_VERBOSE_NODES = (
    "far_planner",
    "localPlanner",
    "pathFollower",
    "cfpa2_coordinator",
)


def _exec_string(node) -> str:
    raw = getattr(node, "_Node__node_executable", None)
    if raw is None:
        raw = getattr(node, "_ExecuteLocal__cmd", None) \
            or getattr(node, "_ExecuteProcess__cmd", None)
    if raw is None:
        return ""
    if isinstance(raw, str):
        return raw
    try:
        return " ".join(
            getattr(s, "text", str(s)) for s in (raw if isinstance(raw, list) else [raw])
        )
    except Exception:
        return str(raw)


def _silence_action(act) -> None:
    try:
        act._ExecuteLocal__output = [TextSubstitution(text="log")]
    except Exception:
        try:
            act._ExecuteProcess__output = [TextSubstitution(text="log")]
        except Exception:
            pass


def _drop_log_level_to_warn(act) -> None:
    extra = [
        TextSubstitution(text="--ros-args"),
        TextSubstitution(text="--log-level"),
        TextSubstitution(text="WARN"),
    ]
    args = getattr(act, "_Node__arguments", None)
    if isinstance(args, list):
        args.extend(extra)
    else:
        try:
            act._Node__arguments = list(extra)
        except Exception:
            pass


def _filter_actions_to_nav_only(actions) -> None:
    from launch.actions import (
        ExecuteProcess as _ExecuteProcess,
        TimerAction as _TimerAction,
        RegisterEventHandler as _RegEvt,
    )

    def _visit(act):
        if isinstance(act, _ExecuteProcess):
            es = _exec_string(act)
            if not any(keep in es for keep in _NAV_DEBUG_KEEP_EXECUTABLES):
                _silence_action(act)
            elif any(verbose in es for verbose in _NAV_DEBUG_VERBOSE_NODES):
                _drop_log_level_to_warn(act)
            return
        if isinstance(act, _TimerAction):
            for child in (act.actions or []):
                _visit(child)
            return
        if isinstance(act, _RegEvt):
            tgt = getattr(act.event_handler, "_target_action", None)
            if tgt is not None:
                _visit(tgt)
            return

    for a in actions:
        _visit(a)


def _build_sensor_bridges(ns: str, mjcf_path: str, base_body: str, imu_site: str,
                          pose_sensor: str, imu_sensor: str, links_config: str,
                          use_sim_time: bool):
    """Per-robot MuJoCo sensor bridges.

    publish_tf=False — swarm_lio_tf_adapter owns the dynamic TF.
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
    nav_backend: str,
    slam_delay: float,
    nav_delay: float,
    go2w_config_pkg: str,
    local_planner_paths_dir: str,
    far_tuning_yaml: str,
    far_default_yaml: str,
    has_wheels: bool = True,
    peer_namespaces: list | None = None,
    holonomic_profile: str = "off",
):
    """Per-robot Swarm-LIO2 (via docker relays) + octomap + nav stack.

    Same shape as fastlio_mixed's _build_fastlio_nav_stack, surgical diffs:
      * Fast-LIO node + fast_lio_tf_adapter removed.
      * 2× input relay + 1× output relay + swarm_lio_tf_adapter + slam_odom_relay added.
      * TF: world → map → quadN/world (… dynamic …) quadN_aft_mapped → base_frame.

    nav_backend: "nav2_mppi" | "far" | "none". Matches fastlio_mixed.
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

    # ── robot_self_filter: strip peer-robot LiDAR returns ──
    if peer_namespaces:
        actions.append(
            Node(
                package="go2w_perception",
                executable="robot_self_filter",
                namespace=ns,
                name="robot_self_filter",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "input_topic": "registered_scan_reliable",
                    "output_topic": "registered_scan_octomap",
                    "peer_namespaces": list(peer_namespaces),
                    "peer_pose_topic": "/odom/ground_truth",
                    "peer_filter_radius_m": 1.20,
                    # See CLAUDE.md golden rule #13. Docker bridge adds
                    # extra hops; keep generous.
                    "peer_pose_stale_sec": 5.0,
                    "stats_log_period_sec": 10.0,
                }],
                output="screen",
            )
        )
    else:
        actions.append(
            Node(
                package="topic_tools",
                executable="relay",
                namespace=ns,
                name="cloud_passthrough",
                arguments=["registered_scan_reliable", "registered_scan_octomap"],
                output="screen",
            )
        )

    # ── pointcloud_adapter: kept for tooling parity; consumes filtered cloud
    #    if peer filter present, else raw. Output unused by SLAM here (docker
    #    swarm_lio2 reads from /robot_X/velodyne_points via relay below). ──
    fastlio_input_topic = (
        f"/{ns}/registered_scan_octomap"
        if peer_namespaces
        else f"/{ns}/registered_scan_reliable"
    )
    actions.append(
        Node(
            package="go2w_perception",
            executable="pointcloud_adapter.py",
            namespace=ns,
            name="pointcloud_adapter",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"input_topic": fastlio_input_topic},
                {"output_topic": f"/{ns}/velodyne_points"},
                {"num_rings": 16},
            ],
            output="screen",
        )
    )

    # ── pointcloud_to_laserscan ──
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
                ("cloud_in",
                 f"/{ns}/registered_scan_octomap" if peer_namespaces
                 else f"/{ns}/registered_scan_reliable"),
                ("scan", f"/{ns}/scan_3d"),
            ] + tf_remaps,
            output="screen",
        )
    )

    # ── Static TFs ──
    # world → map → quadN/world wraps the dynamic adapter output;
    # quadN_aft_mapped → base_frame projects swarm_lio's child frame into
    # our URDF tree. map → odom is a phantom identity that local_costmap
    # (global_frame: odom) traverses to find base_frame.
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

    # ── Sim → docker input relays ──
    # NO IMU relay: ns == robot_token here (`robot_a` / `robot_b`), so
    # `/<ns>/imu/data → /<robot_token>/imu/data` would be a self-loop
    # (topic_tools relay republishes its own input → rate explodes,
    # observed 2157 Hz which destabilised swarm_lio2's odom estimate).
    # mujoco_odom_bridge already publishes IMU to /{ns}/imu/data and
    # bridge.yaml subscribes to that name directly.
    # Lidar still needs a relay: qos_bridge writes registered_scan_reliable
    # but bridge.yaml expects velodyne_points. NOTE: feeds the RAW reliable
    # scan into swarm_lio2, not the peer-filtered cloud — swarm_lio2's ICP
    # is robust to peer-body points (they show as moving outliers).
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

    # ── Docker → ROS2 output relay ──
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

    # ── swarm_lio_tf_adapter: dynamic quadN/world → quadN_aft_mapped TF +
    #    cloud_static frame rewrite. CLI-renamed per robot. ──
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

    # ── slam_odom_relay: /{ns}/Odometry → /{ns}/odom/nav (Nav2 odom input) ──
    # Force output frame to `odom` so local_costmap.global_frame=odom
    # resolves without any extra hop. With GT bootstrap, the relay aligns
    # swarm_lio's `quadN/world` origin to MuJoCo GT so map frame matches
    # the truth and the static map→odom identity is honest.
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
                    "output_frame_id": "odom",
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
    #    /{ns}/registered_scan_map (map frame) for FAR/terrain_analysis ──
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

    # ── Octomap (consumes peer-filtered cloud, emits /{ns}/map_raw) ──
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
            ("cloud_in", f"/{ns}/registered_scan_octomap"),
            ("projected_map", f"/{ns}/map_raw"),
        ] + tf_remaps,
        output="screen",
    )
    actions.append(TimerAction(period=slam_delay + 1.0, actions=[octomap_node]))

    # ── map_augmenter: enrich /{ns}/map_raw with /merged_map → /{ns}/map ──
    map_augmenter_script = os.path.join(_ws_root, "scripts/runtime/map_augmenter.py")
    actions.append(
        TimerAction(
            period=slam_delay + 2.0,
            actions=[
                ExecuteProcess(
                    cmd=["python3", "-u", map_augmenter_script,
                         "--ros-args",
                         "-r", f"__ns:=/{ns}",
                         "-p", "use_sim_time:=true",
                         "-p", "local_map_topic:=map_raw",
                         "-p", "merged_map_topic:=/merged_map",
                         "-p", "augmented_map_topic:=map",
                         "-p", "heartbeat_rate_hz:=1.0"],
                    name=f"map_augmenter_{ns}",
                    output="screen",
                ),
            ],
        )
    )

    # ── Nav2 MPPI branch ────────────────────────────────────────────────
    if nav_backend == "nav2_mppi":
        from launch_ros.actions import PushRosNamespace
        from nav2_common.launch import RewrittenYaml

        _nav2_yaml_filename = (
            "nav2_go2w_full_stack.yaml" if has_wheels
            else "nav2_go2_full_stack.yaml"
        )
        nav2_yaml = os.path.join(
            go2w_config_pkg, "config", "nav", _nav2_yaml_filename
        )
        rewritten_nav2 = RewrittenYaml(
            source_file=nav2_yaml,
            root_key=ns,
            param_rewrites={"use_sim_time": str(use_sim_time).lower()},
            convert_types=True,
        )

        nav2_params = [rewritten_nav2]
        if holonomic_profile == "se2_holonomic":
            overlay_path = os.path.join(
                go2w_config_pkg, "config", "nav",
                "nav2_se2_holonomic_overlay_sim.yaml",
            )
            overlay_rewritten = RewrittenYaml(
                source_file=overlay_path,
                root_key=ns,
                param_rewrites={"use_sim_time": str(use_sim_time).lower()},
                convert_types=True,
            )
            nav2_params.append(overlay_rewritten)

        nav2_inner_nodes = [
            PushRosNamespace(ns),
            Node(
                package="nav2_controller", executable="controller_server",
                name="controller_server",
                parameters=nav2_params,
                remappings=tf_remaps, output="screen",
            ),
            Node(
                package="nav2_planner", executable="planner_server",
                name="planner_server",
                parameters=nav2_params,
                remappings=tf_remaps, output="screen",
            ),
            Node(
                package="nav2_behaviors", executable="behavior_server",
                name="behavior_server",
                parameters=nav2_params,
                remappings=tf_remaps, output="screen",
            ),
            Node(
                package="nav2_bt_navigator", executable="bt_navigator",
                name="bt_navigator",
                parameters=nav2_params,
                remappings=tf_remaps, output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager", executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                parameters=nav2_params,
                output="screen",
            ),
        ]

        bridge_path = os.path.join(_ws_root, "scripts/runtime/cfpa2_to_nav2_bridge.py")
        bridge_node = ExecuteProcess(
            cmd=[
                "python3", "-u", bridge_path,
                "--ros-args",
                "-p", f"namespace:={ns}",
                "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
                "-p", "waypoint_topic:=way_point_coord",
            ],
            name=f"cfpa2_to_nav2_bridge_{ns}",
            output="screen",
        )

        path_relay_path = os.path.join(_ws_root, "scripts/runtime/path_relay.py")
        path_relay_node = ExecuteProcess(
            cmd=[
                "python3", "-u", path_relay_path,
                "--ros-args",
                "-p", f"namespace:={ns}",
                "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
            ],
            name=f"path_relay_{ns}",
            output="screen",
        )

        stuck_watchdog_path = os.path.join(_ws_root, "scripts/runtime/stuck_watchdog.py")
        stuck_watchdog_node = ExecuteProcess(
            cmd=[
                "python3", "-u", stuck_watchdog_path,
                "--ros-args",
                "-p", f"namespace:={ns}",
                "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
            ],
            name=f"stuck_watchdog_{ns}",
            output="screen",
        )

        if has_wheels:
            router_node = Node(
                package="go2w_control",
                executable="go2w_hybrid_cmd_router.py",
                namespace=ns,
                name="go2w_hybrid_cmd_router",
                parameters=[
                    os.path.join(go2w_config_pkg, "config", "control",
                                 "go2w_hybrid_motion.yaml"),
                    {
                        "use_sim_time": use_sim_time,
                        "wheel_command_topic":
                            f"/mujoco_sim/{ns}_wheel_velocity_controller/commands",
                        "wheel_state_topic": "/mujoco_sim/joint_states",
                    },
                ],
                output="screen",
            )
        else:
            router_node = None

        bag_dir = f"/tmp/nav2_run_{ns}"
        _wheel_bag_topics = (
            f"/{ns}/cmd_vel_legged "
            f"/mujoco_sim/{ns}_wheel_velocity_controller/commands "
            f"/{ns}/mobility_mode "
        ) if has_wheels else ""
        bag_record = ExecuteProcess(
            cmd=[
                "bash", "-lc",
                f"rm -rf {bag_dir} && "
                "ros2 bag record "
                f"-o {bag_dir} "
                f"/{ns}/cmd_vel "
                f"{_wheel_bag_topics}"
                f"/mujoco_sim/{ns}_joint_group_effort_controller/joint_trajectory "
                "/mujoco_sim/joint_states "
                f"/{ns}/odom/nav "
                f"/{ns}/plan "
                f"/{ns}/way_point_coord "
                f"/{ns}/goal_pose "
                "/collision_events"
            ],
            name=f"nav2_bag_record_{ns}",
            output="screen",
        )

        from launch.actions import GroupAction
        _nav2_actions = [
            GroupAction(actions=nav2_inner_nodes),
            bridge_node,
            path_relay_node,
            stuck_watchdog_node,
            bag_record,
        ]
        if router_node is not None:
            _nav2_actions.insert(3, router_node)
        actions.append(
            TimerAction(period=nav_delay, actions=_nav2_actions)
        )
        return actions

    if nav_backend != "far":
        return actions  # "none" or unknown — skip nav layer

    # ── FAR fallback (kept verbatim from fastlio_mixed for parity) ──
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
                _load_yaml_params(far_tuning_yaml),
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
                "vehicleLength": 0.3, "vehicleWidth": 0.7,
                "sensorOffsetX": 0.0, "sensorOffsetY": 0.0,
                "twoWayDrive": False,
                "laserVoxelSize": 0.05, "terrainVoxelSize": 0.2,
                "useTerrainAnalysis": True,
                "checkObstacle": True, "checkRotObstacle": True,
                "adjacentRange": 3.0,
                "obstacleHeightThre": 0.02,
                "groundHeightThre": 0.1, "costHeightThre": 0.1,
                "costScore": 0.10, "useCost": True,
                "pointPerPathThre": 1,
                "minRelZ": -0.5, "maxRelZ": 0.8,
                "maxSpeed": far_max_speed,
                "dirWeight": 0.02, "dirThre": 90.0, "dirToVehicle": False,
                "pathScale": 0.75, "minPathScale": 0.5,
                "pathScaleStep": 0.25, "pathScaleBySpeed": True,
                "minPathRange": 1.0, "pathRangeStep": 0.5, "pathRangeBySpeed": True,
                "pathCropByGoal": True,
                "autonomyMode": True, "autonomySpeed": far_max_speed,
                "joyToSpeedDelay": 2.0, "joyToCheckObstacleDelay": 5.0,
                "goalClearRange": 0.5, "goalX": 0.0, "goalY": 0.0,
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
                "sensorOffsetX": 0.0, "sensorOffsetY": 0.0,
                "pubSkipNum": 1, "twoWayDrive": False,
                "lookAheadDis": 0.5,
                "yawRateGain": 1.5, "stopYawRateGain": 1.5,
                "maxYawRate": 80.0, "maxSpeed": far_max_speed,
                "maxAccel": 2.0, "switchTimeThre": 1.0,
                "dirDiffThre": 0.4, "omniDirDiffThre": 1.5,
                "noRotSpeed": 10.0,
                "stopDisThre": 0.15, "slowDwnDisThre": 0.75,
                "useInclRateToSlow": False, "inclRateThre": 120.0,
                "slowRate1": 0.25, "slowRate2": 0.5,
                "slowTime1": 2.0, "slowTime2": 2.0,
                "useInclToStop": False, "inclThre": 45.0, "stopTime": 5.0,
                "noRotAtStop": False, "noRotAtGoal": True,
                "autonomyMode": True, "autonomySpeed": far_max_speed,
                "joyToSpeedDelay": 2.0,
                "goalCloseDis": 0.4, "is_real_robot": False,
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
            name="base_link_to_vehicle_bridge",
            arguments=["0", "0", "0", "0", "0", "0", base_frame, "vehicle"],
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
    ]
    if has_wheels:
        far_nodes.append(
            Node(
                package="go2w_control",
                executable="go2w_hybrid_cmd_router.py",
                namespace=ns,
                name="go2w_hybrid_cmd_router",
                parameters=[
                    os.path.join(go2w_config_pkg, "config", "control",
                                 "go2w_hybrid_motion.yaml"),
                    {
                        "use_sim_time": use_sim_time,
                        "wheel_command_topic":
                            f"/mujoco_sim/{ns}_wheel_velocity_controller/commands",
                        "wheel_state_topic": "/mujoco_sim/joint_states",
                    },
                ],
                output="screen",
            )
        )
    actions.append(TimerAction(period=nav_delay, actions=far_nodes))

    return actions


def _launch_setup(context):
    use_sim_time = True
    gui = _as_bool(_get(context, "gui"))
    rviz = _as_bool(_get(context, "rviz"))
    explore = _as_bool(_get(context, "explore"))
    cleanup_stale = _as_bool(_get(context, "cleanup_stale"))
    debug = _as_bool(_get(context, "debug"))
    map_merge_enabled = _as_bool(_get(context, "map_merge"))
    mujoco_model_path = _get(context, "mujoco_model_path").strip()
    session_duration_sec = float(_get(context, "session_duration_sec"))
    session_output_dir = _get(context, "session_output_dir").strip()
    scene_area_m2 = float(_get(context, "scene_area_m2"))
    collision_output = _get(context, "collision_output_path").strip()
    nav_backend_a = (_get(context, "nav_backend_a").strip().lower() or "nav2_mppi")
    nav_backend_b = (_get(context, "nav_backend_b").strip().lower() or "nav2_mppi")
    holonomic_profile_a = (_get(context, "holonomic_profile_a").strip().lower() or "se2_holonomic")
    holonomic_profile_b = (_get(context, "holonomic_profile_b").strip().lower() or "se2_holonomic")
    _holonomic_allowed = {"off", "se2_holonomic"}
    if holonomic_profile_a not in _holonomic_allowed:
        raise ValueError(f"holonomic_profile_a must be one of {_holonomic_allowed}, got '{holonomic_profile_a}'")
    if holonomic_profile_b not in _holonomic_allowed:
        raise ValueError(f"holonomic_profile_b must be one of {_holonomic_allowed}, got '{holonomic_profile_b}'")
    _alias = {"rrt_star": "nav2_mppi", "far_rrt_star": "nav2_mppi",
              "mppi": "nav2_mppi", "reactive": "nav2_mppi",
              "default": "nav2_mppi", "astar": "nav2_mppi",
              "hybrid": "nav2_mppi", "hybrid_astar": "nav2_mppi",
              "nav2": "nav2_mppi", "nav2_hybrid_astar": "nav2_mppi"}
    nav_backend_a = _alias.get(nav_backend_a, nav_backend_a)
    nav_backend_b = _alias.get(nav_backend_b, nav_backend_b)
    _allowed = {"far", "nav2_mppi", "none"}
    if nav_backend_a not in _allowed:
        raise ValueError(f"nav_backend_a must be one of {_allowed}, got '{nav_backend_a}'")
    if nav_backend_b not in _allowed:
        raise ValueError(f"nav_backend_b must be one of {_allowed}, got '{nav_backend_b}'")

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    go2w_config_pkg = get_package_share_directory("go2w_config")
    cfpa2_pkg = get_package_share_directory("cfpa2_collaborative_autonomy")
    champ_base_pkg = get_package_share_directory("champ_base")
    far_pkg = get_package_share_directory("far_planner")
    local_planner_pkg = get_package_share_directory("local_planner")

    if not mujoco_model_path:
        mujoco_model_path = os.path.join(go2_gazebo_pkg, "mujoco", "demo3_mixed.xml")

    ros2_control_config = os.path.join(
        go2_gazebo_pkg, "config", "ros_control", "ros_control_mixed_mujoco_nav.yaml"
    )

    # ── URDF (Go2W for A, Go2 for B) ──
    go2w_urdf = xacro.process_file(
        os.path.join(go2_gazebo_pkg, "urdf", "go2w", "go2w_description_3d_lidar.xacro"),
    ).documentElement.toxml()
    go2_urdf = xacro.process_file(
        os.path.join(go2_gazebo_pkg, "urdf", "go2", "go2_description_3d_lidar.xacro"),
    ).documentElement.toxml()
    combined_urdf = build_mixed_mujoco_urdf(go2w_urdf, go2_urdf)
    robot_a_urdf = build_namespaced_robot_description(
        go2w_urdf, "robot_a",
        os.path.join(go2_gazebo_pkg, "config", "ros_control", "ros_control_go2w_robot_a.yaml"),
    )
    robot_b_urdf = build_robot_b_urdf(go2_urdf)

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

    actions = [LogInfo(msg="[swarm_lio2_mixed] starting heterogeneous Swarm-LIO2 nav (Go2W + Go2)")]

    # ── Docker compose lifecycle ──
    actions.append(LogInfo(msg=(
        f"[swarm_lio2_mixed] docker compose up -d at {_DOCKER_COMPOSE_DIR}. "
        "Expect /robot_{a,b}/swarm_lio2_raw/Odometry on ROS2 within ~15s."
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
                LogInfo(msg="[swarm_lio2_mixed] tearing down docker compose stack"),
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
            # Lowered 500 → 200 (2026-05-11): dual-robot + docker swarm_lio2
            # + dual Nav2 saturated CPU at 500 Hz, /clock fell below 1 Hz
            # real-time and IMU starved swarm_lio2 → odom diverged to MJ km.
            # 200 Hz gives swarm_lio2 enough IMU (≥50 Hz after odom_bridge)
            # while leaving headroom for the rest of the stack.
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

    # ── T=5: Sensor bridges (publish_tf=False) ──
    sensor_actions = []
    sensor_actions.extend(
        _build_sensor_bridges(
            ns="robot_a", mjcf_path=mujoco_model_path,
            base_body="base_link", imu_site="imu",
            pose_sensor="base_link_site_pose_sensor",
            imu_sensor="imu_imu_sensor",
            links_config=links_a, use_sim_time=use_sim_time,
        )
    )
    sensor_actions.extend(
        _build_sensor_bridges(
            ns="robot_b", mjcf_path=mujoco_model_path,
            base_body="b_base_link", imu_site="b_imu",
            pose_sensor="b_base_link_site_pose_sensor",
            imu_sensor="b_imu_imu_sensor",
            links_config=links_b, use_sim_time=use_sim_time,
        )
    )
    actions.append(TimerAction(period=5.0, actions=sensor_actions))

    # ── T=7 / T=10: CHAMP stacks ──
    robot_a_stack = build_dual_robot_stack(
        ns="robot_a",
        spawn_x="4.0", spawn_y="2.0", spawn_yaw="0.0",
        use_sim_time=use_sim_time,
        robot_description=robot_a_urdf,
        joints_config=joints_a, links_config=links_a, gait_config=gait_config,
        ekf_base_to_footprint=ekf_base, ekf_footprint_to_odom=ekf_odom,
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
        joints_config=joints_b, links_config=links_b, gait_config=gait_config,
        ekf_base_to_footprint=ekf_base, ekf_footprint_to_odom=ekf_odom,
        activate_controllers_on_spawn=True,
        stand_up_joint_preset="go2",
        stand_up_joint_prefix="b_",
        cmd_vel_input_topic="cmd_vel",
        wheel_controller_name=None,
        use_mujoco=True,
        controller_manager_name=f"/{sim_ns}/controller_manager",
    )
    actions.append(TimerAction(period=10.0, actions=robot_b_stack))

    # ── Per-robot Swarm-LIO2 + nav stacks ──
    slam_delay = 20.0
    nav_delay = slam_delay + 5.0

    actions.extend(
        _build_swarm_lio2_nav_stack(
            ns="robot_a", drone_id=1,
            mujoco_lidar_topic="/mujoco_sim/mujoco_lidar_sensor/registered_scan",
            base_frame="base_link",
            use_sim_time=use_sim_time,
            nav_backend=nav_backend_a,
            slam_delay=slam_delay, nav_delay=nav_delay,
            go2w_config_pkg=go2w_config_pkg,
            local_planner_paths_dir=local_planner_paths_dir,
            far_tuning_yaml=far_tuning_yaml,
            far_default_yaml=far_default_yaml,
            has_wheels=True,
            peer_namespaces=["robot_b"],
            holonomic_profile=holonomic_profile_a,
        )
    )
    actions.extend(
        _build_swarm_lio2_nav_stack(
            ns="robot_b", drone_id=2,
            mujoco_lidar_topic="/mujoco_sim/b_mujoco_lidar_sensor/registered_scan",
            base_frame="b_base_link",
            use_sim_time=use_sim_time,
            nav_backend=nav_backend_b,
            slam_delay=slam_delay, nav_delay=nav_delay,
            go2w_config_pkg=go2w_config_pkg,
            local_planner_paths_dir=local_planner_paths_dir,
            far_tuning_yaml=far_tuning_yaml,
            far_default_yaml=far_default_yaml,
            has_wheels=False,
            peer_namespaces=["robot_a"],
            holonomic_profile=holonomic_profile_b,
        )
    )

    # ── CFPA2 coordinator ──
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
                                "use_shared_map": True,
                                "shared_map_topic": "/merged_map",
                                "shared_map_wait_sec": 35.0,
                            },
                        ],
                        output="screen",
                    ),
                ],
            )
        )

    # ── Safety monitor ──
    collision_monitor_script = os.path.join(_ws_root, "scripts/runtime/dual_robot_collision_monitor.py")
    collision_args = [
        "python3", "-u", collision_monitor_script,
        "--robots", "robot_a", "robot_b",
        "--scene-area-m2", str(scene_area_m2),
    ]
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

    # ── Session reporter ──
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
                        LogInfo(msg="[swarm_lio2_mixed] session reporter exited — shutdown"),
                        Shutdown(reason="session complete"),
                    ],
                )
            )
        )

    # ── multirobot_map_merge with GT bootstrap ──
    if map_merge_enabled:
        bootstrap_script = os.path.join(_ws_root, "scripts/runtime/bootstrap_map_merge_poses.py")
        merge_params_path = "/tmp/map_merge_params.yaml"
        bootstrap_proc = ExecuteProcess(
            cmd=[
                "python3", "-u", bootstrap_script,
                "--robots", "robot_a", "robot_b",
                "--gt-topic-suffix", "odom/ground_truth",
                "--output", merge_params_path,
                "--timeout-sec", "30",
                "--merged-map-topic", "merged_map",
                "--merging-rate", "2.0",
                "--discovery-rate", "0.5",
            ],
            name="bootstrap_map_merge_poses",
            output="screen",
        )
        map_merge_node = Node(
            package="multirobot_map_merge",
            executable="map_merge",
            name="map_merge",
            parameters=[merge_params_path, {"use_sim_time": use_sim_time}],
            output="screen",
        )
        actions.append(TimerAction(period=slam_delay + 2.0, actions=[bootstrap_proc]))

        def _on_bootstrap_exit(event, _context):
            rc = getattr(event, "returncode", None)
            if rc == 0:
                return [map_merge_node]
            return [LogInfo(msg=(
                f"[map_merge] bootstrap_map_merge_poses exited with code {rc}; "
                f"skipping map_merge (no valid init poses)."
            ))]

        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=bootstrap_proc,
                    on_exit=_on_bootstrap_exit,
                )
            )
        )

    # ── RViz ──
    if rviz:
        actions.append(
            TimerAction(
                period=slam_delay,
                actions=[
                    Node(
                        package="go2w_perception",
                        executable="multi_tf_relay",
                        name="multi_tf_relay",
                        parameters=[
                            {"use_sim_time": use_sim_time},
                            {"sources": ["robot_a", "robot_b"]},
                        ],
                        output="screen",
                    ),
                    ExecuteProcess(
                        cmd=[
                            "bash", "-c",
                            "unset XDG_DATA_HOME GSETTINGS_SCHEMA_DIR GTK_PATH LOCPATH "
                            "SNAP SNAP_NAME SNAP_INSTANCE_NAME SNAP_REVISION "
                            "SNAP_LIBRARY_PATH SNAP_USER_DATA SNAP_USER_COMMON; "
                            "exec rviz2 -d \"$1\" "
                            "--ros-args -p use_sim_time:=true "
                            "--log-level rviz2:=WARN "
                            "--log-level rviz_common:=WARN "
                            "--log-level rviz_default_plugins:=WARN",
                            "--",
                            os.path.join(go2_gazebo_pkg, "rviz", "nav_test_mixed.rviz"),
                        ],
                        name="rviz2_nav_test_mixed",
                        output="log",
                    ),
                ],
            )
        )

    if debug:
        _filter_actions_to_nav_only(actions)
        actions.insert(0, LogInfo(msg=(
            "[swarm_lio2_mixed] debug:=true — non-nav nodes routed to log file; "
            "only path-planner / controller output on stdout. Visible: "
            + ", ".join(_NAV_DEBUG_KEEP_EXECUTABLES)
        )))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("explore", default_value="true"),
        DeclareLaunchArgument("cleanup_stale", default_value="true"),
        DeclareLaunchArgument(
            "map_merge", default_value="true",
            description="Run multirobot_map_merge with GT-bootstrapped initial poses.",
        ),
        DeclareLaunchArgument(
            "mujoco_model_path", default_value="",
            description="Path to MJCF scene. Defaults to demo3_mixed.xml.",
        ),
        DeclareLaunchArgument("session_duration_sec", default_value="0.0"),
        DeclareLaunchArgument("session_output_dir", default_value=""),
        DeclareLaunchArgument("scene_area_m2", default_value="384.0"),
        DeclareLaunchArgument(
            "collision_output_path",
            default_value="/tmp/dual_robot_collision_report.json",
        ),
        DeclareLaunchArgument(
            "nav_backend_a", default_value="nav2_mppi",
            description="Nav backend for robot_a (Go2W). 'nav2_mppi' | 'far' | 'none'.",
        ),
        DeclareLaunchArgument(
            "nav_backend_b", default_value="nav2_mppi",
            description="Nav backend for robot_b (Go2). 'nav2_mppi' | 'far' | 'none'.",
        ),
        DeclareLaunchArgument(
            "holonomic_profile_a", default_value="se2_holonomic",
            description="Nav2 SE2-holonomic overlay for robot_a. 'off' | 'se2_holonomic'.",
        ),
        DeclareLaunchArgument(
            "holonomic_profile_b", default_value="se2_holonomic",
            description="Nav2 SE2-holonomic overlay for robot_b. 'off' | 'se2_holonomic'.",
        ),
        DeclareLaunchArgument(
            "debug", default_value="false",
            description="Route non-nav nodes to log files; keep planner/controller/safety on stdout.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
