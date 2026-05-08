#!/usr/bin/env python3
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def _workspace_root() -> Path:
    for candidate in (Path(__file__).resolve(), Path.cwd()):
        for parent in (candidate, *candidate.parents):
            if (parent / "scripts" / "launch" / "ros1_hybrid_slam_bridge.sh").is_file():
                return parent
    return Path.cwd()


def generate_launch_description():
    root = _workspace_root()
    fastdds_profile = root / "config" / "fastdds_no_shm.xml"
    if not os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE") and fastdds_profile.is_file():
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(fastdds_profile)

    slam_backend = LaunchConfiguration("slam_backend")
    dynamic_filter_backend = LaunchConfiguration("dynamic_filter_backend")
    static_map_cleanup_backend = LaunchConfiguration("static_map_cleanup_backend")
    start_ros1_slam_bridge = LaunchConfiguration("start_ros1_slam_bridge")
    bridge_script = root / "scripts" / "launch" / "ros1_hybrid_slam_bridge.sh"
    nav_launch = (
        Path(get_package_share_directory("go2_gazebo_sim"))
        / "launch"
        / "nav_test_mujoco_fastlio_mixed.launch.py"
    )

    return LaunchDescription([
        DeclareLaunchArgument("deployment_mode", default_value="sim_hybrid_ros1_slam_ros2_nav"),
        DeclareLaunchArgument("slam_backend", default_value="swarm_lio2_shadow"),
        DeclareLaunchArgument("dynamic_filter_backend", default_value="temporal_voxel_fallback"),
        DeclareLaunchArgument("static_map_cleanup_backend", default_value="none"),
        DeclareLaunchArgument("start_ros1_slam_bridge", default_value="true"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("explore", default_value="true"),
        DeclareLaunchArgument("require_swarm_loop_agreement", default_value="true"),
        DeclareLaunchArgument("swarm_loop_agreement_max_translation", default_value="0.5"),
        DeclareLaunchArgument("swarm_loop_agreement_max_yaw_deg", default_value="5.0"),
        LogInfo(msg="[sim_hybrid] ROS2 MuJoCo remains source of simulated sensors; ROS1 Noetic SLAM runs through Docker bridge."),
        ExecuteProcess(
            cmd=[
                str(bridge_script),
                "mode=sim",
                [TextSubstitution(text="slam_backend="), slam_backend],
                [TextSubstitution(text="dynamic_filter_backend="), dynamic_filter_backend],
                [TextSubstitution(text="static_map_cleanup_backend="), static_map_cleanup_backend],
            ],
            condition=IfCondition(start_ros1_slam_bridge),
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav_launch)),
            launch_arguments={
                "deployment_mode": LaunchConfiguration("deployment_mode"),
                "slam_backend": slam_backend,
                "dynamic_filter_backend": dynamic_filter_backend,
                "static_map_cleanup_backend": static_map_cleanup_backend,
                "gui": LaunchConfiguration("gui"),
                "rviz": LaunchConfiguration("rviz"),
                "explore": LaunchConfiguration("explore"),
                "loop_closure": "true",
                "inter_robot_loop_closure": "true",
                "relative_pose_source": "discovered",
                "map_merge": "true",
                "require_swarm_loop_agreement": LaunchConfiguration("require_swarm_loop_agreement"),
                "swarm_loop_agreement_max_translation": LaunchConfiguration("swarm_loop_agreement_max_translation"),
                "swarm_loop_agreement_max_yaw_deg": LaunchConfiguration("swarm_loop_agreement_max_yaw_deg"),
            }.items(),
        ),
    ])
