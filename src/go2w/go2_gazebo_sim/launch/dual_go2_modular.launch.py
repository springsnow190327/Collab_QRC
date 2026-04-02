"""Canonical dual-Go2 modular Gazebo launch.

Profiles:
- autonomy
- coordinated
- mtare_ros2
- pointlio_debug
"""

import os
import shlex
import sys

import xacro
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))
from modules.assets import build_dual_robot_stack, build_namespaced_robot_description
from modules.control import build_autonomy_enabler_node, build_default_nav_node, build_wall_checker_node
from modules.navigation import build_geometric_frontier_node
from modules.orchestration import build_rviz_node
from modules.perception import build_qos_bridge_node
from modules.slam import build_slam_odom_relay_node
from go2_nav_algorithms.pipeline_components import build_pointcloud_to_laserscan_node, build_simple_scan_mapper_cpp_node


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _get(context, key: str) -> str:
    return LaunchConfiguration(key).perform(context)


def _normalize_planner_backend(value: str) -> str:
    backend = str(value).strip().lower()
    supported = {
        "auto",
        "none",
        "coordinated",
        "go2_nav_algorithms",
        "cfpa2",
        "mtare_ros2",
        "ros1_mtare",
        "far_ros2",
        "tare_ros2_exact",
        "gbplanner2",
    }
    if backend not in supported:
        raise ValueError(
            "Unsupported planner_backend "
            f"'{value}'. Use one of: {', '.join(sorted(supported))}."
        )
    return backend


def _build_cleanup_stale_command() -> str:
    patterns = [
        "ros2 launch go2_gazebo_sim dual_go2_modular.launch.py",
        "ros2 launch go2_gazebo_sim dual_go2w_modular.launch.py",
        "[g]zserver",
        "(^|/)gzclient( |$)",
        "(^|/)gazebo( |$)",
        "/go2_nav_algorithms/lib/go2_nav_algorithms/simple_scan_mapper_cpp",
        "/go2_nav_algorithms/lib/go2_nav_algorithms/simple_frontier_explorer.py",
        "/go2w_observability/lib/go2w_observability/dual_map_coverage_visualizer.py",
        "/go2_gazebo_sim/lib/go2_gazebo_sim/shared_map_fuser.py",
        "/go2w_control/lib/go2w_control/default_nav.py",
        "/go2w_control/lib/go2w_control/autonomy_enabler.py",
        "/go2w_perception/lib/go2w_perception/twist_bridge.py",
        "/go2w_control/lib/go2w_control/go2w_hybrid_cmd_router.py",
        "/go2w_perception/lib/go2w_perception/qos_bridge.py",
        "/go2w_observability/lib/go2w_observability/robot_status_monitor.py",
        "/go2w_spawn/lib/go2w_spawn/initial_pose_guard.py",
        "/go2w_spawn/lib/go2w_spawn/spawn_entity_direct.py",
        "/go2w_perception/lib/go2w_perception/pointcloud_adapter.py",
        "/go2w_perception/lib/go2w_perception/slam_odom_relay.py",
        "/cfpa2_collaborative_autonomy/lib/cfpa2_collaborative_autonomy/cfpa2_coordinator_node",
        "/champ_base/lib/champ_base/quadruped_controller_node",
        "/champ_base/lib/champ_base/state_estimation_node",
        "/robot_localization/ekf_node",
        "/robot_state_publisher",
        "/champ_gazebo/lib/champ_gazebo/contact_sensor",
        "/opt/ros/.*/lib/controller_manager/spawner",
        "/fast_lio/lib/fast_lio/fastlio_mapping",
        "/pointcloud_to_laserscan_node",
    ]

    command = [
        "SELF=$$; PARENT=$PPID; ",
        "kill_pattern(){ ",
        "  PATTERN=\"$1\"; ",
        "  SIGNAL=\"$2\"; ",
        "  for PID in $(pgrep -f \"$PATTERN\" 2>/dev/null || true); do ",
        "    [ \"$PID\" = \"$SELF\" ] && continue; ",
        "    [ \"$PID\" = \"$PARENT\" ] && continue; ",
        "    kill -\"$SIGNAL\" \"$PID\" 2>/dev/null || true; ",
        "  done; ",
        "}; ",
    ]

    for pattern in patterns:
        command.append(f"kill_pattern {shlex.quote(pattern)} TERM; ")
    command.append("sleep 1; ")
    for pattern in patterns:
        command.append(f"kill_pattern {shlex.quote(pattern)} KILL; ")
    command.append("sleep 0.5")
    return "".join(command)


def _build_robot_variant_config(*, go2_gazebo_pkg: str, go2_config_pkg: str, robot_variant: str):
    variant = robot_variant.strip().lower() or "go2"
    if variant == "go2":
        return {
            "robot_variant": "go2",
            "description_path": os.path.join(go2_gazebo_pkg, "urdf", "go2_description_3d_lidar.xacro"),
            "ros_control_robot_a": os.path.join(
                go2_gazebo_pkg, "config", "ros_control", "ros_control_robot_a.yaml"
            ),
            "ros_control_robot_b": os.path.join(
                go2_gazebo_pkg, "config", "ros_control", "ros_control_robot_b.yaml"
            ),
            "joints_config": os.path.join(go2_config_pkg, "config", "joints", "joints.yaml"),
            "links_config": os.path.join(go2_config_pkg, "config", "links", "links.yaml"),
            "gait_config": os.path.join(go2_config_pkg, "config", "gait", "gait.yaml"),
            "stand_up_joint_preset": "go2",
        }

    if variant == "go2w":
        try:
            get_package_share_directory("go2w_description")
        except PackageNotFoundError as exc:
            raise RuntimeError(
                "robot_variant=go2w requires package 'go2w_description'. "
                "Build the top-level workspace so src/unitree_go2w_ros2/src/go2w_description is installed."
            ) from exc
        return {
            "robot_variant": "go2w",
            "description_path": os.path.join(
                go2_gazebo_pkg, "urdf", "go2w", "go2w_description_3d_lidar.xacro"
            ),
            "ros_control_robot_a": os.path.join(
                go2_gazebo_pkg, "config", "ros_control", "ros_control_go2w_robot_a.yaml"
            ),
            "ros_control_robot_b": os.path.join(
                go2_gazebo_pkg, "config", "ros_control", "ros_control_go2w_robot_b.yaml"
            ),
            "joints_config": os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "joints.yaml"),
            "links_config": os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "links.yaml"),
            "gait_config": os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "gait.yaml"),
            "stand_up_joint_preset": "go2w",
        }

    raise RuntimeError(
        f"Unsupported robot_variant '{robot_variant}'. Use robot_variant:=go2 or robot_variant:=go2w."
    )


def _wait_controllers_loaded(ns: str):
    return ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            (
                "until ros2 control list_controllers -c "
                f"/{ns}/controller_manager 2>/dev/null | "
                "awk '"
                f"/{ns}_joint_states_controller/ && tolower($0) ~ /(inactive|active|configured)/ {{a=1}} "
                f"/{ns}_joint_group_effort_controller/ && tolower($0) ~ /(inactive|active|configured)/ {{b=1}} "
                "END {exit !(a && b)}'; "
                "do sleep 0.25; done"
            ),
        ],
        output="screen",
    )


def _build_mtare_feeder_nodes(*, ns: str, use_sim_time: bool, nav_odom_topic: str):
    scan_topic = f"/{ns}/registered_scan_reliable"
    return [
        Node(
            package="sensor_scan_generation",
            executable="sensorScanGeneration",
            name=f"{ns}_sensor_scan_generation",
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[
                ("/state_estimation", nav_odom_topic),
                ("/registered_scan", scan_topic),
                ("/state_estimation_at_scan", f"/{ns}/state_estimation_at_scan"),
                ("/sensor_scan", f"/{ns}/sensor_scan"),
            ],
            output="screen",
        ),
        Node(
            package="terrain_analysis",
            executable="terrainAnalysis",
            name=f"{ns}_terrain_analysis",
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[
                ("/state_estimation", nav_odom_topic),
                ("/registered_scan", scan_topic),
                ("/joy", f"/{ns}/joy"),
                ("/map_clearing", f"/{ns}/map_clearing"),
                ("/terrain_map", f"/{ns}/terrain_map"),
            ],
            output="screen",
        ),
        Node(
            package="terrain_analysis_ext",
            executable="terrainAnalysisExt",
            name=f"{ns}_terrain_analysis_ext",
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[
                ("/state_estimation", nav_odom_topic),
                ("/registered_scan", scan_topic),
                ("/joy", f"/{ns}/joy"),
                ("/cloud_clearing", f"/{ns}/cloud_clearing"),
                ("/terrain_map", f"/{ns}/terrain_map"),
                ("/terrain_map_ext", f"/{ns}/terrain_map_ext"),
            ],
            output="screen",
        ),
    ]


def _build_far_planner_node(
    *,
    ns: str,
    use_sim_time: bool,
    nav_odom_topic: str,
    tf_remaps: list[tuple[str, str]],
    world_frame: str,
    robot_id: int,
    use_shared_graph_bus: bool,
):
    graph_remaps = [
        ("/robot_vgraph", "/mtare/robot_vgraph"),
        ("/decoded_vgraph", "/mtare/decoded_vgraph"),
    ] if use_shared_graph_bus else [
        ("/robot_vgraph", f"/{ns}/robot_vgraph"),
        ("/decoded_vgraph", f"/{ns}/decoded_vgraph"),
    ]
    return Node(
        package="far_planner",
        executable="far_planner",
        namespace=ns,
        name="far_planner",
        parameters=[
            os.path.join(get_package_share_directory("far_planner"), "config", "default.yaml"),
            {"use_sim_time": use_sim_time},
            {"world_frame": world_frame},
            {"graph_msger/robot_id": robot_id},
        ],
        remappings=[
            ("/odom_world", nav_odom_topic),
            ("/terrain_cloud", f"/{ns}/terrain_map_ext"),
            ("/scan_cloud", f"/{ns}/terrain_map"),
            ("/terrain_local_cloud", f"/{ns}/registered_scan"),
            ("/goal_point", f"/{ns}/goal_point"),
            ("/way_point", f"/{ns}/way_point_far"),
            ("/joy", f"/{ns}/joy"),
            ("/navigation_boundary", f"/{ns}/navigation_boundary"),
            ("/runtime", f"/{ns}/far_runtime"),
            ("/planning_time", f"/{ns}/far_planning_time"),
            *graph_remaps,
            *tf_remaps,
        ],
        output="screen",
    )


def _build_graph_decoder_node(*, use_sim_time: bool, use_shared_graph_bus: bool):
    remappings = []
    if use_shared_graph_bus:
        remappings = [
            ("/robot_vgraph", "/mtare/robot_vgraph"),
            ("decoded_vgraph", "/mtare/decoded_vgraph"),
        ]
    return Node(
        package="graph_decoder",
        executable="graph_decoder",
        name="graph_decoder_exact",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=remappings,
        output="screen",
    )


def _build_mtare_behavior_executive_node(*, use_sim_time: bool):
    return Node(
        package="mtare_ros2",
        executable="mtare_behavior_executive_cpp",
        name="mtare_behavior_executive_cpp",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"namespaces": ["robot_a", "robot_b"]},
            {"tare_input_suffix": "/way_point_tare"},
            {"waypoint_output_suffix": "/way_point_coord"},
            {"planner_mode_output_suffix": "/planner_mode"},
            {"enable_hysteresis_guard": False},
            {"enable_switch_lock_guard": False},
            {"enable_stale_guard": False},
            {"source_timeout_sec": 2.0},
            {"output_rate_hz": 8.0},
        ],
        output="screen",
    )


def _robot_autonomy_actions(
    *,
    ns: str,
    use_sim_time: bool,
    use_fast_lio: bool,
    slam_config: str,
    enable_frontier: bool,
    goal_topic: str,
    frontier_goal_topic: str,
    startup_delay_sec: float,
    enable_perception: bool,
    enable_slam: bool,
    enable_control: bool,
    enable_navigation: bool,
    default_nav_profile: str,
    hybrid_motion_config: str | None = None,
    wheel_controller_name: str | None = None,
    hybrid_motion_extra_params: dict | None = None,
):
    tf_remaps = [("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")]
    nav_odom_topic = f"/{ns}/odom/nav"
    planning_scan_topic = f"/{ns}/scan_3d"

    actions = []

    if enable_control:
        actions.append(
            Node(
                package="go2w_perception",
                executable="twist_bridge.py",
                namespace=ns,
                remappings=[("/cmd_vel_stamped", f"/{ns}/cmd_vel_stamped"), ("/cmd_vel", f"/{ns}/cmd_vel")],
                output="screen",
            )
        )
        if hybrid_motion_config and wheel_controller_name:
            hybrid_params = [
                hybrid_motion_config,
                {"wheel_command_topic": f"{wheel_controller_name}/commands"},
            ]
            if hybrid_motion_extra_params:
                hybrid_params.append(hybrid_motion_extra_params)
            actions.append(
                Node(
                    package="go2w_control",
                    executable="go2w_hybrid_cmd_router.py",
                    namespace=ns,
                    name="go2w_hybrid_cmd_router",
                    parameters=hybrid_params,
                    output="screen",
                )
            )
            if enable_navigation:
                actions.append(
                    build_wall_checker_node(
                        ns=ns,
                        use_sim_time=use_sim_time,
                        extra_params={
                            "scan_topic": planning_scan_topic,
                            "stop_topic": f"/{ns}/stop",
                            "mode_topic": "mobility_mode",
                            "safety_dist": 0.28,
                            "check_angle_deg": 30.0,
                            "wheel_safety_dist": 0.50,
                            "wheel_check_angle_deg": 70.0,
                            "wheel_min_close_points": 4,
                        },
                        name="wheel_wall_collision_checker",
                    )
                )

    if enable_perception:
        actions.append(
            build_qos_bridge_node(
                ns=ns,
                use_sim_time=use_sim_time,
                extra_params={
                    "input_topic": f"/{ns}/registered_scan",
                    "output_topic": f"/{ns}/registered_scan_reliable",
                },
            )
        )

    if enable_perception and enable_slam and use_fast_lio:
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
        actions.append(
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                namespace=ns,
                name="slam_node",
                parameters=[slam_config, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("/velodyne_points", f"/{ns}/velodyne_points"),
                    ("/imu/data", f"/{ns}/imu/data"),
                    ("/Odometry", f"/{ns}/Odometry"),
                ],
                output="screen",
            )
        )

    if enable_slam:
        if use_fast_lio:
            actions.append(
                build_slam_odom_relay_node(
                    ns=ns,
                    use_sim_time=use_sim_time,
                    name="slam_odom_relay",
                    input_topic=f"/{ns}/Odometry",
                    gt_topic=f"/{ns}/odom/ground_truth",
                    output_topic=nav_odom_topic,
                    output_frame_id="world",
                    output_child_frame_id="base_link",
                    bootstrap_from_gt=True,
                    require_gt_for_alignment=True,
                )
            )
        else:
            actions.append(
                build_slam_odom_relay_node(
                    ns=ns,
                    use_sim_time=use_sim_time,
                    name="gt_odom_relay",
                    input_topic=f"/{ns}/odom/ground_truth",
                    output_topic=nav_odom_topic,
                    output_frame_id="world",
                    output_child_frame_id="base_link",
                )
            )

    if enable_perception:
        actions.append(
            build_pointcloud_to_laserscan_node(
                ns=ns,
                use_sim_time=use_sim_time,
                extra_params={
                    "target_frame": "base_link",
                    "min_height": 0.05,
                    "max_height": 0.60,
                    "range_max": 12.0,
                },
                remappings=tf_remaps
                + [
                    ("cloud_in", f"/{ns}/registered_scan_reliable"),
                    ("scan", planning_scan_topic),
                ],
            )
        )

    if enable_navigation:
        actions.append(
            build_simple_scan_mapper_cpp_node(
                ns=ns,
                use_sim_time=use_sim_time,
                profile="geometric_frontier_dual.yaml",
                extra_params={
                    "scan_topic": planning_scan_topic,
                    "odom_topic": nav_odom_topic,
                    "map_topic": f"/{ns}/map",
                    "map_frame": "world",
                    "startup_delay": 0.0,
                    "max_range": 12.0,
                    "max_clear_distance": 2.0,
                    "max_scan_odom_dt": 0.10,
                },
                remappings=tf_remaps,
            )
        )

        if enable_frontier:
            actions.append(
                build_geometric_frontier_node(
                    ns=ns,
                    use_sim_time=use_sim_time,
                    profile="geometric_frontier_dual.yaml",
                    extra_params={
                        "odom_topic": nav_odom_topic,
                        "map_topic": f"/{ns}/map",
                        "prefer_costmap": False,
                        "costmap_topic": "",
                        "frontier_goal_topic": frontier_goal_topic,
                        "frontier_marker_topic": f"/{ns}/frontier_goal_marker",
                        "frontier_regions_topic": f"/{ns}/frontier_markers",
                        "frontier_replan_topic": f"/{ns}/frontier_replan",
                        "startup_delay": 0.0,
                        "max_map_odom_dt": 0.5,
                    },
                    remappings=tf_remaps,
                )
            )

    if enable_control:
        actions.append(
            build_autonomy_enabler_node(
                ns=ns,
                use_sim_time=use_sim_time,
                extra_params={"startup_delay": 8.0, "rate": 10.0},
                remappings=[("/way_point", goal_topic), ("/joy", f"/{ns}/joy")],
            )
        )
        actions.append(
            build_default_nav_node(
                ns=ns,
                use_sim_time=use_sim_time,
                profile=default_nav_profile,
                extra_params={
                    "frontier_replan_topic": f"/{ns}/frontier_replan",
                    "stop_topic": f"/{ns}/stop",
                },
                remappings=[
                    ("/way_point", goal_topic),
                    ("/odom/ground_truth", nav_odom_topic),
                    ("/scan", planning_scan_topic),
                    ("/cmd_vel_stamped", f"/{ns}/cmd_vel_stamped"),
                    ("/nav_status", f"/{ns}/nav_status"),
                    ("/planned_path", f"/{ns}/planned_path"),
                    ("/robot_trajectory", f"/{ns}/robot_trajectory"),
                    ("/final_goal_marker", f"/{ns}/final_goal_marker"),
                ],
            )
        )

    if not actions:
        return []

    return [TimerAction(period=startup_delay_sec, actions=actions)]


def _build_dual_profile_actions(context, *, launch_name: str, robot_config: dict[str, str], default_nav_profile: str):
    profile = _get(context, "profile").strip().lower()

    if profile == "pointlio_debug":
        go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
        return [
            LogInfo(msg=f"[{launch_name}] profile=pointlio_debug -> delegating to pointlio_debug_core.launch.py"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(go2_gazebo_pkg, "launch", "pointlio_debug_core.launch.py")
                ),
                launch_arguments={
                    "gui": _get(context, "gui"),
                    "autonomous": _get(context, "pointlio_autonomous"),
                    "spawn_x": _get(context, "pointlio_spawn_x"),
                    "spawn_y": _get(context, "pointlio_spawn_y"),
                    "spawn_z": _get(context, "pointlio_spawn_z"),
                    "spawn_heading": _get(context, "pointlio_spawn_heading"),
                }.items(),
            ),
        ]

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    mtare_ros2_pkg = get_package_share_directory("mtare_ros2")
    go2_config_pkg = get_package_share_directory("go2_config")
    champ_base_pkg = get_package_share_directory("champ_base")
    champ_gazebo_pkg = get_package_share_directory("champ_gazebo")

    use_sim_time = _as_bool(_get(context, "use_sim_time"))
    gui = _as_bool(_get(context, "gui"))
    rviz = _as_bool(_get(context, "rviz"))
    cleanup_stale = _as_bool(_get(context, "cleanup_stale"))
    use_fast_lio = _as_bool(_get(context, "use_fast_lio"))
    pointcloud_noise_enabled = _as_bool(_get(context, "pointcloud_noise_enabled"))
    pointcloud_noise_mean = _get(context, "pointcloud_noise_mean").strip() or "0.0"
    pointcloud_noise_stddev = _get(context, "pointcloud_noise_stddev").strip() or "0.015"
    enable_frontier_aux = _as_bool(_get(context, "enable_frontier_aux"))
    use_shared_map = _as_bool(_get(context, "use_shared_map"))
    enable_internal_shared_map_fuser = _as_bool(_get(context, "enable_internal_shared_map_fuser"))

    enable_assets = _as_bool(_get(context, "enable_assets"))
    enable_perception = _as_bool(_get(context, "enable_perception"))
    enable_slam = _as_bool(_get(context, "enable_slam"))
    enable_control = _as_bool(_get(context, "enable_control"))
    enable_navigation = _as_bool(_get(context, "enable_navigation"))

    planner_backend = _normalize_planner_backend(_get(context, "planner_backend"))
    if planner_backend == "auto":
        if profile == "coordinated":
            planner_backend = "coordinated"
        elif profile == "mtare_ros2":
            planner_backend = "mtare_ros2"
        else:
            planner_backend = "none"
    require_shared_graph = _as_bool(_get(context, "require_shared_graph"))

    world = _get(context, "world")
    robot_a_spawn_x = _get(context, "robot_a_spawn_x")
    robot_a_spawn_y = _get(context, "robot_a_spawn_y")
    robot_a_spawn_yaw = _get(context, "robot_a_spawn_yaw")
    robot_b_spawn_x = _get(context, "robot_b_spawn_x")
    robot_b_spawn_y = _get(context, "robot_b_spawn_y")
    robot_b_spawn_yaw = _get(context, "robot_b_spawn_yaw")

    gazebo_config = os.path.join(champ_gazebo_pkg, "config", "gazebo.yaml")
    rviz_config_robot_a = os.path.join(go2_gazebo_pkg, "rviz", "dual_robot_a.rviz")
    rviz_config_robot_b = os.path.join(go2_gazebo_pkg, "rviz", "dual_robot_b.rviz")
    rviz_config_mtare_shared = os.path.join(go2_gazebo_pkg, "rviz", "dual_mtare_shared.rviz")
    mtare_params_file = os.path.join(mtare_ros2_pkg, "config", "mtare_ros2.yaml")
    cfpa2_params_file = ""
    if planner_backend == "cfpa2":
        try:
            cfpa2_pkg = get_package_share_directory("cfpa2_collaborative_autonomy")
        except PackageNotFoundError as exc:
            raise RuntimeError(
                "planner_backend=cfpa2 requires package 'cfpa2_collaborative_autonomy'. "
                "Build the workspace after adding src/cfpa2_collaborative_autonomy."
            ) from exc
        cfpa2_params_file = os.path.join(cfpa2_pkg, "config", "cfpa2_coordinator.yaml")
    exact_far_world_frame = _get(context, "exact_far_world_frame").strip() or "map"

    use_tare_ros2_exact = planner_backend == "tare_ros2_exact"
    missing_exact_packages: list[str] = []
    missing_shared_graph_packages: list[str] = []
    if use_tare_ros2_exact:
        for pkg_name in ("far_planner", "go2_tare_planner_ros2"):
            try:
                get_package_share_directory(pkg_name)
            except PackageNotFoundError:
                missing_exact_packages.append(pkg_name)
        for pkg_name in ("graph_decoder", "visibility_graph_msg"):
            try:
                get_package_share_directory(pkg_name)
            except PackageNotFoundError:
                missing_shared_graph_packages.append(pkg_name)
        if missing_exact_packages:
            raise RuntimeError(
                "planner_backend=tare_ros2_exact missing required package(s): "
                + ", ".join(missing_exact_packages)
            )
        if missing_shared_graph_packages and require_shared_graph:
            raise RuntimeError(
                "planner_backend=tare_ros2_exact requires graph_decoder and visibility_graph_msg "
                "(set require_shared_graph:=false to allow degraded mode). Missing: "
                + ", ".join(missing_shared_graph_packages)
            )
    use_shared_graph_bus = use_tare_ros2_exact and not missing_shared_graph_packages

    robot_description_mappings = {}
    if robot_config["robot_variant"] == "go2w":
        robot_description_mappings = {
            "pointcloud_noise_enabled": "true" if pointcloud_noise_enabled else "false",
            "pointcloud_noise_mean": pointcloud_noise_mean,
            "pointcloud_noise_stddev": pointcloud_noise_stddev,
        }

    doc = xacro.process_file(robot_config["description_path"], mappings=robot_description_mappings)
    base_robot_description = doc.documentElement.toxml()

    robot_description_a = build_namespaced_robot_description(
        base_robot_description, "robot_a", robot_config["ros_control_robot_a"]
    )
    robot_description_b = build_namespaced_robot_description(
        base_robot_description, "robot_b", robot_config["ros_control_robot_b"]
    )

    joints_config = robot_config["joints_config"]
    links_config = robot_config["links_config"]
    gait_config = robot_config["gait_config"]
    stand_up_joint_preset = robot_config["stand_up_joint_preset"]
    hybrid_motion_config = None
    hybrid_motion_extra_params = None
    cmd_vel_input_topic = "cmd_vel"
    wheel_controller_names = {"robot_a": None, "robot_b": None}
    rsp_publish_frequency = 200.0
    if robot_config["robot_variant"] == "go2w":
        hybrid_motion_config = os.path.join(get_package_share_directory("go2w_config"), "config", "control", "go2w_hybrid_motion.yaml")
        hybrid_motion_extra_params = {"publish_rate": 15.0}
        cmd_vel_input_topic = "cmd_vel_legged"
        wheel_controller_names = {
            "robot_a": "robot_a_wheel_velocity_controller",
            "robot_b": "robot_b_wheel_velocity_controller",
        }
        rsp_publish_frequency = 100.0  # Go2W: lower TF rate to save CPU
    ekf_base_to_footprint = os.path.join(champ_base_pkg, "config", "ekf", "base_to_footprint.yaml")
    ekf_footprint_to_odom = os.path.join(champ_base_pkg, "config", "ekf", "footprint_to_odom.yaml")
    slam_config = os.path.join(get_package_share_directory("go2w_config"), "config", "slam", "pointlio_gazebo.yaml")

    actions = []

    if cleanup_stale:
        actions.append(
            ExecuteProcess(
                cmd=["bash", "-lc", _build_cleanup_stale_command()],
                output="screen",
            )
        )

    actions.append(
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "gzserver",
                        "-s",
                        "libgazebo_ros_init.so",
                        "-s",
                        "libgazebo_ros_factory.so",
                        world,
                        "--ros-args",
                        "--params-file",
                        gazebo_config,
                    ],
                    output="screen",
                )
            ],
        )
    )

    if gui:
        actions.append(TimerAction(period=6.0, actions=[ExecuteProcess(cmd=["gzclient"], output="screen")]))

    if rviz:
        rviz_actions = []
        if profile == "mtare_ros2":
            rviz_actions.append(build_rviz_node(rviz_config_mtare_shared, use_sim_time, name="rviz2_mtare_shared"))
        else:
            rviz_actions.extend(
                [
                    build_rviz_node(rviz_config_robot_a, use_sim_time, name="rviz2_robot_a"),
                    build_rviz_node(rviz_config_robot_b, use_sim_time, name="rviz2_robot_b"),
                ]
            )
        actions.append(TimerAction(period=7.0, actions=rviz_actions))

    actions.append(
        Node(
            package="go2w_observability",
            executable="dual_map_coverage_visualizer.py",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"robot_a_map_topic": "/robot_a/map"},
                {"robot_b_map_topic": "/robot_b/map"},
                {"robot_a_odom_topic": "/robot_a/odom/nav"},
                {"robot_b_odom_topic": "/robot_b/odom/nav"},
                {"marker_topic": "/dual_robot/coverage_markers"},
                {"marker_frame": "world"},
                {"publish_rate": 1.0},
                {"min_map_value": 0},
                {"cell_stride": 1},
                {"robot_a_alpha": 0.20},
                {"robot_b_alpha": 0.20},
            ],
            output="screen",
        )
    )


    if use_shared_map and enable_internal_shared_map_fuser:
        actions.append(
            Node(
                package="go2_gazebo_sim",
                executable="shared_map_fuser.py",
                name="shared_map_fuser",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"map_a_topic": "/robot_a/map"},
                    {"map_b_topic": "/robot_b/map"},
                    {"output_topic": _get(context, "shared_map_topic")},
                    {"frame_id": "world"},
                    {"publish_rate": 2.0},
                ],
                output="screen",
            )
        )

    if profile == "autonomy":
        goal_topic = "/{ns}/way_point"
        frontier_goal = "/{ns}/way_point"
        frontier_enabled = True
        startup_delay = 24.0
    elif profile == "coordinated":
        goal_topic = "/{ns}/way_point_coord"
        frontier_goal = "/{ns}/way_point_raw"
        frontier_enabled = True
        startup_delay = 16.0
    else:
        goal_topic = "/{ns}/way_point_coord"
        frontier_goal = "/{ns}/way_point_raw"
        frontier_enabled = enable_frontier_aux or use_tare_ros2_exact
        startup_delay = 16.0

    robot_a_actions = []
    robot_b_actions = []
    robot_a_pose_guard_node = None
    robot_b_pose_guard_node = None

    if enable_assets:
        robot_a_stack_actions, robot_a_stack_handles = build_dual_robot_stack(
            ns="robot_a",
            spawn_x=robot_a_spawn_x,
            spawn_y=robot_a_spawn_y,
            spawn_yaw=robot_a_spawn_yaw,
            use_sim_time=use_sim_time,
            robot_description=robot_description_a,
            joints_config=joints_config,
            links_config=links_config,
            gait_config=gait_config,
            ekf_base_to_footprint=ekf_base_to_footprint,
            ekf_footprint_to_odom=ekf_footprint_to_odom,
            joint_state_spawner_delay_sec=1.0,
            effort_spawner_delay_sec=1.2,
            standup_delay_sec=4.0,
            pose_guard_hold_sec=12.0,
            activate_controllers_on_spawn=True,
            stand_up_joint_preset=stand_up_joint_preset,
            cmd_vel_input_topic=cmd_vel_input_topic,
            wheel_controller_name=wheel_controller_names["robot_a"],
            rsp_publish_frequency=rsp_publish_frequency,
            return_handles=True,
        )
        robot_b_stack_actions, robot_b_stack_handles = build_dual_robot_stack(
            ns="robot_b",
            spawn_x=robot_b_spawn_x,
            spawn_y=robot_b_spawn_y,
            spawn_yaw=robot_b_spawn_yaw,
            use_sim_time=use_sim_time,
            robot_description=robot_description_b,
            joints_config=joints_config,
            links_config=links_config,
            gait_config=gait_config,
            ekf_base_to_footprint=ekf_base_to_footprint,
            ekf_footprint_to_odom=ekf_footprint_to_odom,
            joint_state_spawner_delay_sec=1.6,
            effort_spawner_delay_sec=1.8,
            standup_delay_sec=4.8,
            pose_guard_hold_sec=13.0,
            activate_controllers_on_spawn=True,
            stand_up_joint_preset=stand_up_joint_preset,
            cmd_vel_input_topic=cmd_vel_input_topic,
            wheel_controller_name=wheel_controller_names["robot_b"],
            rsp_publish_frequency=rsp_publish_frequency,
            return_handles=True,
        )
        robot_a_actions += robot_a_stack_actions
        robot_b_actions += robot_b_stack_actions
        robot_a_pose_guard_node = robot_a_stack_handles.get("initial_pose_guard_node")
        robot_b_pose_guard_node = robot_b_stack_handles.get("initial_pose_guard_node")

    robot_a_actions += _robot_autonomy_actions(
        ns="robot_a",
        use_sim_time=use_sim_time,
        use_fast_lio=use_fast_lio,
        slam_config=slam_config,
        enable_frontier=frontier_enabled,
        goal_topic=goal_topic.format(ns="robot_a"),
        frontier_goal_topic=frontier_goal.format(ns="robot_a"),
        startup_delay_sec=startup_delay,
        enable_perception=enable_perception,
        enable_slam=enable_slam,
        enable_control=enable_control,
        enable_navigation=enable_navigation,
        default_nav_profile=default_nav_profile,
        hybrid_motion_config=hybrid_motion_config,
        wheel_controller_name=wheel_controller_names["robot_a"],
        hybrid_motion_extra_params=hybrid_motion_extra_params,
    )
    robot_b_actions += _robot_autonomy_actions(
        ns="robot_b",
        use_sim_time=use_sim_time,
        use_fast_lio=use_fast_lio,
        slam_config=slam_config,
        enable_frontier=frontier_enabled,
        goal_topic=goal_topic.format(ns="robot_b"),
        frontier_goal_topic=frontier_goal.format(ns="robot_b"),
        startup_delay_sec=startup_delay,
        enable_perception=enable_perception,
        enable_slam=enable_slam,
        enable_control=enable_control,
        enable_navigation=enable_navigation,
        default_nav_profile=default_nav_profile,
        hybrid_motion_config=hybrid_motion_config,
        wheel_controller_name=wheel_controller_names["robot_b"],
        hybrid_motion_extra_params=hybrid_motion_extra_params,
    )

    exact_aux_actions = []
    if enable_navigation and use_tare_ros2_exact:
        robot_a_nav_odom = "/robot_a/odom/nav"
        robot_b_nav_odom = "/robot_b/odom/nav"
        # FAR visibility-graph input topics are map-framed in Gazebo.
        robot_a_far_odom = "/robot_a/state_estimation_at_scan"
        robot_b_far_odom = "/robot_b/state_estimation_at_scan"
        robot_a_tf_remaps = [("/tf", "/robot_a/tf"), ("/tf_static", "/robot_a/tf_static")]
        robot_b_tf_remaps = [("/tf", "/robot_b/tf"), ("/tf_static", "/robot_b/tf_static")]

        robot_a_actions += _build_mtare_feeder_nodes(
            ns="robot_a", use_sim_time=use_sim_time, nav_odom_topic=robot_a_nav_odom
        )
        robot_b_actions += _build_mtare_feeder_nodes(
            ns="robot_b", use_sim_time=use_sim_time, nav_odom_topic=robot_b_nav_odom
        )

        exact_aux_actions += [
            Node(
                package="mtare_ros2",
                executable="mtare_topic_bridge.py",
                name="mtare_topic_bridge",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"robot_a_ns": "robot_a"},
                    {"robot_b_ns": "robot_b"},
                    {"robot_a_mtare_name": "wheeled0"},
                    {"robot_b_mtare_name": "wheeled1"},
                    {"state_estimation_in_suffix": "state_estimation_at_scan"},
                    {"state_estimation_out_suffix": "state_estimation_at_scan"},
                    {"key_pose_in_suffix": "state_estimation_at_scan"},
                    {"key_pose_out_suffix": "key_pose_to_map"},
                    {"registered_scan_in_suffix": "sensor_scan"},
                    {"registered_scan_out_suffix": "registered_scan"},
                    {"terrain_map_in_suffix": "terrain_map"},
                    {"terrain_map_out_suffix": "terrain_map"},
                    {"terrain_map_ext_in_suffix": "terrain_map_ext"},
                    {"terrain_map_ext_out_suffix": "terrain_map_ext"},
                    {"waypoint_in_suffix": "way_point"},
                    {"waypoint_out_suffix": "way_point_coord"},
                    {"frontier_markers_in_suffix": "frontier_markers"},
                    {"hold_last_waypoint": False},
                    {"waypoint_republish_hz": 0.0},
                    {"marker_frame_override": "world"},
                ],
                output="screen",
            ),
            _build_mtare_behavior_executive_node(use_sim_time=use_sim_time),
            Node(
                package="mtare_ros2",
                executable="mtare_coordinator.py",
                name="mtare_coordinator",
                parameters=[
                    mtare_params_file,
                    {"use_sim_time": use_sim_time},
                    {"namespaces": ["robot_a", "robot_b"]},
                    {"algorithm_mode": _get(context, "mtare_algorithm_mode")},
                    {"publish_rate": float(_get(context, "mtare_goal_publish_rate"))},
                    {"output_mode": "exact_split"},
                    {"goal_topic_suffix": "/way_point_coord"},
                    {"use_shared_map": use_shared_map},
                    {"shared_map_topic": _get(context, "shared_map_topic")},
                    {"shared_map_wait_sec": float(_get(context, "shared_map_wait_sec"))},
                    {"overlap_weight": float(_get(context, "mtare_overlap_weight"))},
                    {"communication_timeout_sec": float(_get(context, "mtare_communication_timeout_sec"))},
                    {"prediction_horizon_sec": float(_get(context, "mtare_prediction_horizon_sec"))},
                    {"pursuit_weight": float(_get(context, "mtare_pursuit_weight"))},
                    {"pursuit_switch_margin": float(_get(context, "mtare_pursuit_switch_margin"))},
                    {"exploration_gain_radius_cells": int(_get(context, "mtare_exploration_gain_radius_cells"))},
                    {"meeting_min_distance": float(_get(context, "mtare_meeting_min_distance"))},
                    {"teammate_stale_ttl_sec": float(_get(context, "mtare_teammate_stale_ttl_sec"))},
                    {"cfpa2_w_ig": float(_get(context, "cfpa2_w_ig"))},
                    {"cfpa2_w_c": float(_get(context, "cfpa2_w_c"))},
                    {"cfpa2_w_sw": float(_get(context, "cfpa2_w_sw"))},
                    {"cfpa2_lambda_overlap": float(_get(context, "cfpa2_lambda_overlap"))},
                    {"cfpa2_sigma_overlap_m": float(_get(context, "cfpa2_sigma_overlap_m"))},
                    {"cfpa2_stuck_lock_sec": float(_get(context, "cfpa2_stuck_lock_sec"))},
                    {"cfpa2_stuck_min_motion_m": float(_get(context, "cfpa2_stuck_min_motion_m"))},
                    {"cfpa2_stuck_blacklist_sec": float(_get(context, "cfpa2_stuck_blacklist_sec"))},
                    {"cfpa2_close_stop_radius_m": float(_get(context, "cfpa2_close_stop_radius_m"))},
                    {
                        "cfpa2_close_stop_speed_epsilon": float(
                            _get(context, "cfpa2_close_stop_speed_epsilon")
                        )
                    },
                    {"marker_frame_override": "world"},
                ],
                output="screen",
            ),
            _build_far_planner_node(
                ns="robot_a",
                use_sim_time=use_sim_time,
                nav_odom_topic=robot_a_far_odom,
                tf_remaps=robot_a_tf_remaps,
                world_frame=exact_far_world_frame,
                robot_id=1,
                use_shared_graph_bus=use_shared_graph_bus,
            ),
            _build_far_planner_node(
                ns="robot_b",
                use_sim_time=use_sim_time,
                nav_odom_topic=robot_b_far_odom,
                tf_remaps=robot_b_tf_remaps,
                world_frame=exact_far_world_frame,
                robot_id=2,
                use_shared_graph_bus=use_shared_graph_bus,
            ),
        ]
        if use_shared_graph_bus:
            exact_aux_actions.append(
                _build_graph_decoder_node(use_sim_time=use_sim_time, use_shared_graph_bus=True)
            )

    wait_robot_a = _wait_controllers_loaded("robot_a") if enable_assets else None
    wait_robot_b = _wait_controllers_loaded("robot_b") if enable_assets else None

    def _coordinator_start_actions(coordinator_node):
        if (
            (not enable_assets)
            or robot_a_pose_guard_node is None
            or robot_b_pose_guard_node is None
        ):
            return [
                LogInfo(msg="[planner_startup] pose-guard gating unavailable; using fallback 20s delayed start."),
                RegisterEventHandler(
                    OnProcessStart(
                        target_action=coordinator_node,
                        on_start=[LogInfo(msg="[planner_startup] coordinator process started (fallback path).")],
                    )
                ),
                TimerAction(period=20.0, actions=[coordinator_node]),
            ]

        pose_guard_state = {"robot_a": False, "robot_b": False, "started": False}

        def _make_pose_guard_exit_cb(ns: str):
            def _cb(_context, *_args, **_kwargs):
                pose_guard_state[ns] = True
                if pose_guard_state["started"]:
                    return []
                if pose_guard_state["robot_a"] and pose_guard_state["robot_b"]:
                    pose_guard_state["started"] = True
                    return [
                        LogInfo(
                            msg="[planner_startup] pose guards exited for robot_a and robot_b; "
                            "starting coordinator in 0.5s."
                        ),
                        TimerAction(period=0.5, actions=[coordinator_node]),
                    ]
                return [
                    LogInfo(
                        msg=f"[planner_startup] {ns} initial_pose_guard exited; waiting for the other robot."
                    )
                ]

            return _cb

        def _fallback_start_cb(_context, *_args, **_kwargs):
            if pose_guard_state["started"]:
                return []
            pose_guard_state["started"] = True
            return [
                LogInfo(
                    msg=(
                        "[planner_startup] pose-guard timeout exceeded; "
                        "starting coordinator via fallback timer."
                    )
                ),
                coordinator_node,
            ]

        return [
            RegisterEventHandler(
                OnProcessStart(
                    target_action=coordinator_node,
                    on_start=[LogInfo(msg="[planner_startup] coordinator process started after pose-guard gating.")],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=robot_a_pose_guard_node,
                    on_exit=[OpaqueFunction(function=_make_pose_guard_exit_cb("robot_a"))],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=robot_b_pose_guard_node,
                    on_exit=[OpaqueFunction(function=_make_pose_guard_exit_cb("robot_b"))],
                )
            ),
            TimerAction(period=30.0, actions=[OpaqueFunction(function=_fallback_start_cb)]),
        ]

    # Use deterministic staggered startup for the second robot.
    # The previous OnProcessExit(wait_robot_a) handoff proved unreliable under GUI load:
    # robot_a came up, wait_robot_a exited, but robot_b actions were never scheduled.
    if wait_robot_a is not None:
        actions.append(TimerAction(period=5.0, actions=robot_a_actions + [wait_robot_a]))
        actions.append(
            LogInfo(
                msg=f"[{launch_name}] scheduling robot_b stack with fixed delayed startup at 14s."
            )
        )
        actions.append(
            TimerAction(
                period=14.0,
                actions=robot_b_actions + ([wait_robot_b] if wait_robot_b is not None else []),
            )
        )
    else:
        actions.append(TimerAction(period=5.0, actions=robot_a_actions))
        actions.append(TimerAction(period=14.0, actions=robot_b_actions))

    if enable_navigation:
        if planner_backend in {"coordinated", "go2_nav_algorithms"}:
            actions.append(
                TimerAction(
                    period=20.0,
                    actions=[
                        Node(
                            package="go2_nav_algorithms",
                            executable="multi_robot_goal_assigner.py",
                            name="multi_robot_goal_assigner",
                            parameters=[
                                {"use_sim_time": use_sim_time},
                                {"namespaces": ["robot_a", "robot_b"]},
                                {"publish_rate": 2.0},
                                {"beta": 0.18},
                                {"sensor_range": 3.5},
                                {"frontier_stride": 2},
                                {"max_targets": 800},
                                {"switch_hysteresis": 0.05},
                                {"switch_min_dist": 0.35},
                                {"goal_topic_suffix": "/way_point_coord"},
                                {"use_shared_map": use_shared_map},
                                {"shared_map_topic": _get(context, "shared_map_topic")},
                                {"shared_map_wait_sec": float(_get(context, "shared_map_wait_sec"))},
                                {"algorithm_mode": _get(context, "coordinated_algorithm_mode")},
                            ],
                            output="screen",
                        )
                    ],
                )
            )
        elif planner_backend == "mtare_ros2":
            actions.append(
                TimerAction(
                    period=20.0,
                    actions=[
                        Node(
                            package="mtare_ros2",
                            executable="mtare_coordinator.py",
                            name="mtare_coordinator",
                            parameters=[
                                mtare_params_file,
                                {"use_sim_time": use_sim_time},
                                {"namespaces": ["robot_a", "robot_b"]},
                                {"algorithm_mode": _get(context, "mtare_algorithm_mode")},
                                {"publish_rate": float(_get(context, "mtare_goal_publish_rate"))},
                                {"switch_hysteresis": float(_get(context, "switch_hysteresis"))},
                                {"goal_lock_sec": float(_get(context, "goal_lock_sec"))},
                                {"goal_topic_suffix": "/way_point_coord"},
                                {"use_shared_map": use_shared_map},
                                {"shared_map_topic": _get(context, "shared_map_topic")},
                                {"shared_map_wait_sec": float(_get(context, "shared_map_wait_sec"))},
                                {"overlap_weight": float(_get(context, "mtare_overlap_weight"))},
                                {
                                    "communication_timeout_sec": float(
                                        _get(context, "mtare_communication_timeout_sec")
                                    )
                                },
                                {"prediction_horizon_sec": float(_get(context, "mtare_prediction_horizon_sec"))},
                                {"pursuit_weight": float(_get(context, "mtare_pursuit_weight"))},
                                {"pursuit_switch_margin": float(_get(context, "mtare_pursuit_switch_margin"))},
                                {
                                    "exploration_gain_radius_cells": int(
                                        _get(context, "mtare_exploration_gain_radius_cells")
                                    )
                                },
                                {"meeting_min_distance": float(_get(context, "mtare_meeting_min_distance"))},
                                {"teammate_stale_ttl_sec": float(_get(context, "mtare_teammate_stale_ttl_sec"))},
                                {"cfpa2_w_ig": float(_get(context, "cfpa2_w_ig"))},
                                {"cfpa2_w_c": float(_get(context, "cfpa2_w_c"))},
                                {"cfpa2_w_sw": float(_get(context, "cfpa2_w_sw"))},
                                {"cfpa2_lambda_overlap": float(_get(context, "cfpa2_lambda_overlap"))},
                                {"cfpa2_sigma_overlap_m": float(_get(context, "cfpa2_sigma_overlap_m"))},
                                {"cfpa2_stuck_lock_sec": float(_get(context, "cfpa2_stuck_lock_sec"))},
                                {"cfpa2_stuck_min_motion_m": float(_get(context, "cfpa2_stuck_min_motion_m"))},
                                {"cfpa2_stuck_blacklist_sec": float(_get(context, "cfpa2_stuck_blacklist_sec"))},
                                {"cfpa2_close_stop_radius_m": float(_get(context, "cfpa2_close_stop_radius_m"))},
                                {
                                    "cfpa2_close_stop_speed_epsilon": float(
                                        _get(context, "cfpa2_close_stop_speed_epsilon")
                                    )
                                },
                                {"marker_frame_override": "world"},
                            ],
                            output="screen",
                        )
                    ],
                )
            )
        elif planner_backend == "cfpa2":
            actions.extend(
                _coordinator_start_actions(
                    Node(
                        package="cfpa2_collaborative_autonomy",
                        executable="cfpa2_coordinator_node",
                        name="cfpa2_coordinator",
                        parameters=[
                            cfpa2_params_file,
                            {"use_sim_time": use_sim_time},
                            {"namespaces": ["robot_a", "robot_b"]},
                            {"algorithm_mode": "cfpa2"},
                            {"publish_rate": float(_get(context, "mtare_goal_publish_rate"))},
                            {"switch_hysteresis": float(_get(context, "switch_hysteresis"))},
                            {"goal_lock_sec": float(_get(context, "goal_lock_sec"))},
                            {"goal_topic_suffix": "/way_point_coord"},
                            {"use_shared_map": use_shared_map},
                            {"shared_map_topic": _get(context, "shared_map_topic")},
                            {"shared_map_wait_sec": float(_get(context, "shared_map_wait_sec"))},
                            {"overlap_weight": float(_get(context, "mtare_overlap_weight"))},
                            {
                                "communication_timeout_sec": float(
                                    _get(context, "mtare_communication_timeout_sec")
                                )
                            },
                            {"prediction_horizon_sec": float(_get(context, "mtare_prediction_horizon_sec"))},
                            {"pursuit_weight": float(_get(context, "mtare_pursuit_weight"))},
                            {"pursuit_switch_margin": float(_get(context, "mtare_pursuit_switch_margin"))},
                            {
                                "exploration_gain_radius_cells": int(
                                    _get(context, "mtare_exploration_gain_radius_cells")
                                )
                            },
                            {"meeting_min_distance": float(_get(context, "mtare_meeting_min_distance"))},
                            {"teammate_stale_ttl_sec": float(_get(context, "mtare_teammate_stale_ttl_sec"))},
                            {"cfpa2_w_ig": float(_get(context, "cfpa2_w_ig"))},
                            {"cfpa2_w_c": float(_get(context, "cfpa2_w_c"))},
                            {"cfpa2_w_sw": float(_get(context, "cfpa2_w_sw"))},
                            {"cfpa2_lambda_overlap": float(_get(context, "cfpa2_lambda_overlap"))},
                            {"cfpa2_sigma_overlap_m": float(_get(context, "cfpa2_sigma_overlap_m"))},
                            {"cfpa2_stuck_lock_sec": float(_get(context, "cfpa2_stuck_lock_sec"))},
                            {"cfpa2_stuck_min_motion_m": float(_get(context, "cfpa2_stuck_min_motion_m"))},
                            {"cfpa2_stuck_blacklist_sec": float(_get(context, "cfpa2_stuck_blacklist_sec"))},
                            {"cfpa2_close_stop_radius_m": float(_get(context, "cfpa2_close_stop_radius_m"))},
                            {
                                "cfpa2_close_stop_speed_epsilon": float(
                                    _get(context, "cfpa2_close_stop_speed_epsilon")
                                )
                            },
                            {"cfpa2_space_time_enabled": _as_bool(_get(context, "cfpa2_space_time_enabled"))},
                            {"cfpa2_space_time_horizon_sec": float(_get(context, "cfpa2_space_time_horizon_sec"))},
                            {"cfpa2_space_time_dt_sec": float(_get(context, "cfpa2_space_time_dt_sec"))},
                            {
                                "cfpa2_space_time_safety_radius_m": float(
                                    _get(context, "cfpa2_space_time_safety_radius_m")
                                )
                            },
                            {
                                "cfpa2_space_time_waypoint_lookahead_m": float(
                                    _get(context, "cfpa2_space_time_waypoint_lookahead_m")
                                )
                            },
                            {
                                "cfpa2_space_time_window_margin_m": float(
                                    _get(context, "cfpa2_space_time_window_margin_m")
                                )
                            },
                            {"cfpa2_space_time_max_expansions": int(_get(context, "cfpa2_space_time_max_expansions"))},
                            {
                                "cfpa2_space_time_assumed_speed_mps": float(
                                    _get(context, "cfpa2_space_time_assumed_speed_mps")
                                )
                            },
                            {
                                "cfpa2_space_time_max_speed_mps": float(
                                    _get(context, "cfpa2_space_time_max_speed_mps")
                                )
                            },
                            {
                                "cfpa2_frontier_min_cluster_area_m2": float(
                                    _get(context, "cfpa2_frontier_min_cluster_area_m2")
                                )
                            },
                            {
                                "cfpa2_frontier_obstacle_clearance_m": float(
                                    _get(context, "cfpa2_frontier_obstacle_clearance_m")
                                )
                            },
                            {"marker_frame_override": "world"},
                        ],
                        output="screen",
                    )
                )
            )
        elif planner_backend == "tare_ros2_exact":
            actions.append(
                LogInfo(
                    msg=(
                        f"[{launch_name}] planner_backend=tare_ros2_exact: "
                        "launching exact split coordinator + BE + FAR planners."
                    )
                )
            )
        elif planner_backend == "gbplanner2":
            gbp_config = os.path.join(
                get_package_share_directory("go2_nav_algorithms"),
                "config", "gbplanner2_dual.yaml",
            )
            for ns in ("robot_a", "robot_b"):
                actions.append(
                    TimerAction(
                        period=20.0,
                        actions=[
                            Node(
                                package="go2_nav_algorithms",
                                executable="gbplanner2_local.py",
                                name="gbplanner2_local",
                                namespace=ns,
                                parameters=[
                                    gbp_config,
                                    {"use_sim_time": use_sim_time},
                                    {"goal_topic": f"/{ns}/way_point_coord"},
                                    {"map_topic": f"/{ns}/map"},
                                    {"odom_topic": f"/{ns}/odom/nav"},
                                ],
                                remappings=[
                                    ("way_point_coord", f"/{ns}/way_point_coord"),
                                    ("map", f"/{ns}/map"),
                                    ("odom/nav", f"/{ns}/odom/nav"),
                                ],
                                output="screen",
                            ),
                        ],
                    )
                )
        elif planner_backend in {"ros1_mtare", "far_ros2"}:
            actions.append(
                LogInfo(
                    msg=(
                        f"[{launch_name}] planner_backend={planner_backend} is not native in Gazebo dual stack; "
                        "running without global coordinator."
                    )
                )
            )

    if exact_aux_actions:
        if wait_robot_b is not None:
            actions.append(
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=wait_robot_b,
                        on_exit=[TimerAction(period=2.0, actions=exact_aux_actions)],
                    )
                )
            )
        else:
            actions.append(TimerAction(period=20.0, actions=exact_aux_actions))

    actions.append(
        TimerAction(
            period=18.0,
            actions=[
                Node(
                    package="go2w_observability",
                    executable="robot_status_monitor.py",
                    name="robot_status_monitor",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"namespaces": ["robot_a", "robot_b"]},
                        {"report_rate": 0.1},
                        {"json_output": False},
                    ],
                    remappings=[
                        ("/robot_a/odom/ground_truth", "/robot_a/odom/nav"),
                        ("/robot_b/odom/ground_truth", "/robot_b/odom/nav"),
                        ("/robot_a/way_point", "/robot_a/way_point_coord"),
                        ("/robot_b/way_point", "/robot_b/way_point_coord"),
                    ],
                    output="screen",
                )
            ],
        )
    )

    actions.insert(
        0,
        LogInfo(
            msg=(
                f"[{launch_name}] "
                f"profile={profile} robot_variant={robot_config['robot_variant']} planner_backend={planner_backend} "
                f"use_fast_lio={use_fast_lio} pointcloud_noise={pointcloud_noise_enabled} "
                f"pointcloud_noise_stddev={pointcloud_noise_stddev} "
                f"use_tare_ros2_exact={use_tare_ros2_exact} "
                f"require_shared_graph={require_shared_graph} "
                f"use_shared_graph_bus={use_shared_graph_bus} "
                f"assets={enable_assets} perception={enable_perception} slam={enable_slam} "
                f"control={enable_control} navigation={enable_navigation}"
            )
        ),
    )
    if use_tare_ros2_exact and missing_shared_graph_packages and not require_shared_graph:
        actions.insert(
            1,
            LogInfo(
                msg=(
                    f"[{launch_name}] tare_ros2_exact degraded mode: "
                    f"missing shared graph packages={missing_shared_graph_packages} "
                    "(require_shared_graph:=false)."
                )
            ),
        )

    return actions


def generate_fixed_variant_launch_description(*, launch_name: str, robot_variant: str, default_nav_profile: str):
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    go2_config_pkg = get_package_share_directory("go2_config")
    robot_config = _build_robot_variant_config(
        go2_gazebo_pkg=go2_gazebo_pkg,
        go2_config_pkg=go2_config_pkg,
        robot_variant=robot_variant,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("profile", default_value="autonomy"),
            DeclareLaunchArgument("planner_backend", default_value="auto"),
            DeclareLaunchArgument("require_shared_graph", default_value="true"),
            DeclareLaunchArgument("exact_far_world_frame", default_value="map"),
            DeclareLaunchArgument("coordinated_algorithm_mode", default_value="committed"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("cleanup_stale", default_value="true"),
            DeclareLaunchArgument("use_fast_lio", default_value="false"),
            DeclareLaunchArgument("pointcloud_noise_enabled", default_value="false"),
            DeclareLaunchArgument("pointcloud_noise_mean", default_value="0.0"),
            DeclareLaunchArgument("pointcloud_noise_stddev", default_value="0.015"),
            DeclareLaunchArgument("enable_frontier_aux", default_value="false"),
            DeclareLaunchArgument("use_shared_map", default_value="false"),
            DeclareLaunchArgument("shared_map_topic", default_value="/disco_slam/global_map"),
            DeclareLaunchArgument("shared_map_wait_sec", default_value="8.0"),
            DeclareLaunchArgument("enable_internal_shared_map_fuser", default_value="true"),
            DeclareLaunchArgument("enable_assets", default_value="true"),
            DeclareLaunchArgument("enable_perception", default_value="true"),
            DeclareLaunchArgument("enable_slam", default_value="true"),
            DeclareLaunchArgument("enable_control", default_value="true"),
            DeclareLaunchArgument("enable_navigation", default_value="true"),
            DeclareLaunchArgument("mtare_algorithm_mode", default_value="mui_tare"),
            DeclareLaunchArgument("mtare_goal_publish_rate", default_value="2.0"),
            DeclareLaunchArgument("mtare_overlap_weight", default_value="1.0"),
            DeclareLaunchArgument("mtare_communication_timeout_sec", default_value="6.0"),
            DeclareLaunchArgument("mtare_prediction_horizon_sec", default_value="4.0"),
            DeclareLaunchArgument("mtare_pursuit_weight", default_value="2.0"),
            DeclareLaunchArgument("mtare_pursuit_switch_margin", default_value="0.10"),
            DeclareLaunchArgument("switch_hysteresis", default_value="0.05"),
            DeclareLaunchArgument("goal_lock_sec", default_value="5.0"),
            DeclareLaunchArgument("mtare_exploration_gain_radius_cells", default_value="4"),
            DeclareLaunchArgument("mtare_meeting_min_distance", default_value="1.5"),
            DeclareLaunchArgument("mtare_teammate_stale_ttl_sec", default_value="120.0"),
            DeclareLaunchArgument("cfpa2_w_ig", default_value="1.0"),
            DeclareLaunchArgument("cfpa2_w_c", default_value="0.6"),
            DeclareLaunchArgument("cfpa2_w_sw", default_value="0.2"),
            DeclareLaunchArgument("cfpa2_lambda_overlap", default_value="1.0"),
            DeclareLaunchArgument("cfpa2_sigma_overlap_m", default_value="0.0"),
            DeclareLaunchArgument("cfpa2_stuck_lock_sec", default_value="45.0"),
            DeclareLaunchArgument("cfpa2_stuck_min_motion_m", default_value="0.20"),
            DeclareLaunchArgument("cfpa2_stuck_blacklist_sec", default_value="60.0"),
            DeclareLaunchArgument("cfpa2_close_stop_radius_m", default_value="0.35"),
            DeclareLaunchArgument("cfpa2_close_stop_speed_epsilon", default_value="0.02"),
            DeclareLaunchArgument("cfpa2_space_time_enabled", default_value="true"),
            DeclareLaunchArgument("cfpa2_space_time_horizon_sec", default_value="5.0"),
            DeclareLaunchArgument("cfpa2_space_time_dt_sec", default_value="0.40"),
            DeclareLaunchArgument("cfpa2_space_time_safety_radius_m", default_value="0.45"),
            DeclareLaunchArgument("cfpa2_space_time_waypoint_lookahead_m", default_value="0.90"),
            DeclareLaunchArgument("cfpa2_space_time_window_margin_m", default_value="3.0"),
            DeclareLaunchArgument("cfpa2_space_time_max_expansions", default_value="12000"),
            DeclareLaunchArgument("cfpa2_space_time_assumed_speed_mps", default_value="0.25"),
            DeclareLaunchArgument("cfpa2_space_time_max_speed_mps", default_value="0.60"),
            DeclareLaunchArgument("cfpa2_frontier_min_cluster_area_m2", default_value="0.20"),
            DeclareLaunchArgument("cfpa2_frontier_obstacle_clearance_m", default_value="0.40"),
            DeclareLaunchArgument("robot_a_spawn_x", default_value="1.0"),
            DeclareLaunchArgument("robot_a_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_a_spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("robot_b_spawn_x", default_value="18.0"),
            DeclareLaunchArgument("robot_b_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_b_spawn_yaw", default_value="3.14159"),
            DeclareLaunchArgument("world", default_value=os.path.join(go2_gazebo_pkg, "worlds", "3.world")),
            DeclareLaunchArgument("pointlio_autonomous", default_value="false"),
            DeclareLaunchArgument("pointlio_spawn_x", default_value="2.5"),
            DeclareLaunchArgument("pointlio_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("pointlio_spawn_z", default_value="0.32"),
            DeclareLaunchArgument("pointlio_spawn_heading", default_value="0.0"),
            OpaqueFunction(
                function=lambda context, *_args, **_kwargs: _build_dual_profile_actions(
                    context,
                    launch_name=launch_name,
                    robot_config=robot_config,
                    default_nav_profile=default_nav_profile,
                )
            ),
        ]
    )


def generate_launch_description():
    return generate_fixed_variant_launch_description(
        launch_name="dual_go2_modular",
        robot_variant="go2",
        default_nav_profile="default_nav_dual.yaml",
    )
