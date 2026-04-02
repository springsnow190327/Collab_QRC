import os
import sys

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))

from _stack_components import build_dual_robot_stack, build_namespaced_robot_description


def _drift_monitor_node(ns: str, use_sim_time, spawn_x, spawn_y, spawn_yaw, sample_rate, settle_sec, analysis_duration_sec):
    return Node(
        package="go2w_spawn",
        executable="spawn_drift_monitor.py",
        name=f"{ns}_spawn_drift_monitor",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"entity_name": ns},
            {"spawn_x": spawn_x},
            {"spawn_y": spawn_y},
            {"spawn_z": 0.32},
            {"spawn_yaw": spawn_yaw},
            {"sample_rate": sample_rate},
            {"settle_sec": settle_sec},
            {"analysis_duration_sec": analysis_duration_sec},
            {"log_rate_hz": 1.0},
            {"model_states_topic": "/gazebo/model_states"},
            {"model_states_topic_fallback": "/model_states"},
        ],
        output="screen",
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


def _activate_controllers(ns: str):
    return ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            (
                "until ros2 control switch_controllers -c "
                f"/{ns}/controller_manager --activate "
                f"{ns}_joint_states_controller {ns}_joint_group_effort_controller "
                "> /dev/null 2>&1; do sleep 0.25; done"
            ),
        ],
        output="screen",
    )


def generate_launch_description():
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    go2_config_pkg = get_package_share_directory("go2_config")
    champ_base_pkg = get_package_share_directory("champ_base")
    champ_gazebo_pkg = get_package_share_directory("champ_gazebo")

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    cleanup_stale = LaunchConfiguration("cleanup_stale")

    robot_a_spawn_x = LaunchConfiguration("robot_a_spawn_x")
    robot_a_spawn_y = LaunchConfiguration("robot_a_spawn_y")
    robot_a_spawn_yaw = LaunchConfiguration("robot_a_spawn_yaw")
    robot_b_spawn_x = LaunchConfiguration("robot_b_spawn_x")
    robot_b_spawn_y = LaunchConfiguration("robot_b_spawn_y")
    robot_b_spawn_yaw = LaunchConfiguration("robot_b_spawn_yaw")

    analysis_duration_sec = LaunchConfiguration("analysis_duration_sec")
    settle_sec = LaunchConfiguration("settle_sec")
    sample_rate = LaunchConfiguration("sample_rate")
    world = LaunchConfiguration("world")

    gazebo_config = os.path.join(champ_gazebo_pkg, "config", "gazebo.yaml")
    description_path = os.path.join(go2_gazebo_pkg, "urdf", "go2_description_3d_lidar.xacro")
    doc = xacro.process_file(description_path)
    base_robot_description = doc.documentElement.toxml()

    ros_control_robot_a = os.path.join(go2_gazebo_pkg, "config", "ros_control", "ros_control_robot_a.yaml")
    ros_control_robot_b = os.path.join(go2_gazebo_pkg, "config", "ros_control", "ros_control_robot_b.yaml")

    robot_description_a = build_namespaced_robot_description(base_robot_description, "robot_a", ros_control_robot_a)
    robot_description_b = build_namespaced_robot_description(base_robot_description, "robot_b", ros_control_robot_b)

    joints_config = os.path.join(go2_config_pkg, "config", "joints", "joints.yaml")
    links_config = os.path.join(go2_config_pkg, "config", "links", "links.yaml")
    gait_config = os.path.join(go2_config_pkg, "config", "gait", "gait.yaml")
    ekf_base_to_footprint = os.path.join(champ_base_pkg, "config", "ekf", "base_to_footprint.yaml")
    ekf_footprint_to_odom = os.path.join(champ_base_pkg, "config", "ekf", "footprint_to_odom.yaml")

    cleanup_stale_processes = ExecuteProcess(
        condition=IfCondition(cleanup_stale),
        cmd=[
            "bash",
            "-lc",
            "pkill -f '[g]zserver' || true; "
            "pkill -f '^gzclient$' || true; "
            "pkill -f '/go2w_spawn/lib/go2w_spawn/[i]nitial_pose_guard.py' || true; "
            "pkill -f '/go2w_spawn/lib/go2w_spawn/[s]tand_up_slowly.py' || true; "
            "pkill -f '/go2w_spawn/lib/go2w_spawn/[s]pawn_drift_monitor.py' || true; "
            "sleep 1",
        ],
        output="screen",
    )

    start_gazebo_server = ExecuteProcess(
        cmd=[
            "gzserver",
            "-u",
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

    start_gazebo_client = ExecuteProcess(condition=IfCondition(gui), cmd=["gzclient"], output="screen")

    robot_a_actions = build_dual_robot_stack(
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
        pose_guard_hold_sec=8.5,
        activate_controllers_on_spawn=False,
    )

    robot_b_actions = build_dual_robot_stack(
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
        pose_guard_hold_sec=9.5,
        activate_controllers_on_spawn=False,
    )

    robot_a_monitor = _drift_monitor_node(
        "robot_a",
        use_sim_time,
        robot_a_spawn_x,
        robot_a_spawn_y,
        robot_a_spawn_yaw,
        sample_rate,
        settle_sec,
        analysis_duration_sec,
    )
    robot_b_monitor = _drift_monitor_node(
        "robot_b",
        use_sim_time,
        robot_b_spawn_x,
        robot_b_spawn_y,
        robot_b_spawn_yaw,
        sample_rate,
        settle_sec,
        analysis_duration_sec,
    )

    wait_robot_a_controllers_loaded = _wait_controllers_loaded("robot_a")
    wait_robot_b_controllers_loaded = _wait_controllers_loaded("robot_b")
    activate_robot_a_controllers = _activate_controllers("robot_a")
    activate_robot_b_controllers = _activate_controllers("robot_b")
    unpause_physics = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "until timeout 1s ros2 service call /unpause_physics std_srvs/srv/Empty '{}' >/dev/null 2>&1; do sleep 0.25; done",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("cleanup_stale", default_value="true"),
            DeclareLaunchArgument("robot_a_spawn_x", default_value="1.0"),
            DeclareLaunchArgument("robot_a_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_a_spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("robot_b_spawn_x", default_value="18.0"),
            DeclareLaunchArgument("robot_b_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_b_spawn_yaw", default_value="3.14159"),
            DeclareLaunchArgument("analysis_duration_sec", default_value="20.0"),
            DeclareLaunchArgument("settle_sec", default_value="1.0"),
            DeclareLaunchArgument("sample_rate", default_value="20.0"),
            DeclareLaunchArgument("world", default_value=os.path.join(go2_gazebo_pkg, "worlds", "3.world")),
            cleanup_stale_processes,
            TimerAction(period=3.0, actions=[start_gazebo_server]),
            start_gazebo_client,
            TimerAction(period=5.0, actions=robot_a_actions + [robot_a_monitor, wait_robot_a_controllers_loaded]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_robot_a_controllers_loaded,
                    on_exit=robot_b_actions + [robot_b_monitor, wait_robot_b_controllers_loaded],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_robot_b_controllers_loaded,
                    on_exit=[unpause_physics, activate_robot_a_controllers, activate_robot_b_controllers],
                )
            ),
        ]
    )
