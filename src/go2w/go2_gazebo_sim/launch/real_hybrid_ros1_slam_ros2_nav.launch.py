#!/usr/bin/env python3
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def _workspace_root() -> Path:
    for candidate in (Path(__file__).resolve(), Path.cwd()):
        for parent in (candidate, *candidate.parents):
            if (parent / "scripts" / "launch" / "ros1_hybrid_slam_bridge.sh").is_file():
                return parent
    return Path.cwd()


def _swarm_adapter(ns: str, base_frame: str, slam_backend, dynamic_filter_backend, use_sim_time):
    return Node(
        package="slam_backend_adapters",
        executable="swarm_lio2_ros2_adapter_node",
        namespace=ns,
        name="swarm_lio2_ros2_adapter_node",
        parameters=[{
            "namespace": ns,
            "slam_backend": slam_backend,
            "dynamic_filter_backend": dynamic_filter_backend,
            "base_frame": base_frame,
            "use_sim_time": use_sim_time,
            "publish_tf": True,
        }],
        output="screen",
    )


def generate_launch_description():
    root = _workspace_root()
    fastdds_profile = root / "config" / "fastdds_no_shm.xml"
    if not os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE") and fastdds_profile.is_file():
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(fastdds_profile)

    slam_backend = LaunchConfiguration("slam_backend")
    dynamic_filter_backend = LaunchConfiguration("dynamic_filter_backend")
    static_map_cleanup_backend = LaunchConfiguration("static_map_cleanup_backend")
    start_ros1_slam_bridge = LaunchConfiguration("start_ros1_slam_bridge")
    start_nav2_stack = LaunchConfiguration("start_nav2_stack")
    use_sim_time = LaunchConfiguration("use_sim_time")
    bridge_script = root / "scripts" / "launch" / "ros1_hybrid_slam_bridge.sh"
    real_single_launch = (
        Path(get_package_share_directory("go2w_real_bringup"))
        / "launch"
        / "real_single.launch.py"
    )

    return LaunchDescription([
        DeclareLaunchArgument("deployment_mode", default_value="real_hybrid_ros1_slam_ros2_nav"),
        DeclareLaunchArgument("robot_namespace", default_value="robot_a"),
        DeclareLaunchArgument("robot_model", default_value="go2w"),
        DeclareLaunchArgument("slam_backend", default_value="swarm_lio2_shadow"),
        DeclareLaunchArgument("dynamic_filter_backend", default_value="temporal_voxel_fallback"),
        DeclareLaunchArgument("static_map_cleanup_backend", default_value="none"),
        DeclareLaunchArgument("start_ros1_slam_bridge", default_value="false"),
        DeclareLaunchArgument("start_nav2_stack", default_value="true"),
        DeclareLaunchArgument("nav_backend", default_value="nav2_mppi"),
        DeclareLaunchArgument("map_backend", default_value="carto_2d"),
        DeclareLaunchArgument("execute_controller", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_3d", default_value="true"),
        DeclareLaunchArgument("explore", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("require_swarm_loop_agreement", default_value="true"),
        DeclareLaunchArgument("swarm_loop_agreement_max_translation", default_value="0.5"),
        DeclareLaunchArgument("swarm_loop_agreement_max_yaw_deg", default_value="5.0"),
        LogInfo(msg="[real_hybrid] ROS1/Noetic SLAM is expected onboard; ROS2/Humble keeps Nav2 and team_loop_closure."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(real_single_launch)),
            condition=IfCondition(start_nav2_stack),
            launch_arguments={
                "robot_namespace": LaunchConfiguration("robot_namespace"),
                "robot_model": LaunchConfiguration("robot_model"),
                "slam": "fastlio_mid360",
                "carto_mode": "2d",
                "nav_backend": LaunchConfiguration("nav_backend"),
                "map_backend": LaunchConfiguration("map_backend"),
                "execute_controller": LaunchConfiguration("execute_controller"),
                "rviz": LaunchConfiguration("rviz"),
                "rviz_3d": LaunchConfiguration("rviz_3d"),
                "explore": LaunchConfiguration("explore"),
                "onboard_slam": "true",
            }.items(),
        ),
        ExecuteProcess(
            cmd=[
                str(bridge_script),
                "mode=real",
                [TextSubstitution(text="slam_backend="), slam_backend],
                [TextSubstitution(text="dynamic_filter_backend="), dynamic_filter_backend],
                [TextSubstitution(text="static_map_cleanup_backend="), static_map_cleanup_backend],
            ],
            condition=IfCondition(start_ros1_slam_bridge),
            output="screen",
        ),
        _swarm_adapter("robot_a", "base_link", slam_backend, dynamic_filter_backend, use_sim_time),
        _swarm_adapter("robot_b", "b_base_link", slam_backend, dynamic_filter_backend, use_sim_time),
        Node(
            package="slam_backend_adapters",
            executable="dynamic_lio_filtering_node",
            name="dynamic_lio_filtering_node",
            parameters=[{
                "use_sim_time": use_sim_time,
                "namespaces": ["robot_a", "robot_b"],
                "dynamic_filter_backend": dynamic_filter_backend,
            }],
            output="screen",
        ),
        Node(
            package="slam_backend_adapters",
            executable="erasor_adapter_node",
            name="erasor_adapter_node",
            parameters=[{
                "use_sim_time": use_sim_time,
                "namespaces": ["robot_a", "robot_b"],
                "static_map_cleanup_backend": static_map_cleanup_backend,
                "erasor_trigger_mode": "manual",
                "export_dir": "logs/erasor_real_hybrid",
            }],
            output="screen",
        ),
        Node(
            package="team_loop_closure",
            executable="loop_keyframe_exporter_node",
            name="loop_keyframe_exporter_node",
            parameters=[{
                "use_sim_time": use_sim_time,
                "namespaces": ["robot_a", "robot_b"],
                "cloud_topic": "cloud_static",
                "output_topic": "/team_slam/keyframes",
                "keyframe_cloud_topic": "/team_slam/static_keyframe_clouds",
            }],
            output="screen",
        ),
        Node(
            package="team_loop_closure",
            executable="cross_robot_loop_matcher_node",
            name="cross_robot_loop_matcher_node",
            parameters=[{
                "use_sim_time": use_sim_time,
                "keyframe_topic": "/team_slam/keyframes",
                "keyframe_cloud_topic": "/team_slam/static_keyframe_clouds",
                "candidate_topic": "/team_slam/cross_robot_candidates",
                "match_topic": "/team_slam/cross_robot_matches",
                "reference_robot": "robot_a",
                "target_robot": "robot_b",
                "registration_backend": "icp_2d",
            }],
            output="screen",
        ),
        Node(
            package="team_loop_closure",
            executable="robust_loop_selector_node",
            name="robust_loop_selector_node",
            parameters=[{
                "use_sim_time": use_sim_time,
                "match_topic": "/team_slam/cross_robot_matches",
                "robust_inliers_topic": "/team_slam/robust_loop_inliers",
                "reference_robot": "robot_a",
                "target_robot": "robot_b",
            }],
            output="screen",
        ),
        Node(
            package="team_loop_closure",
            executable="team_pose_graph_node",
            name="team_pose_graph_node",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robots": ["robot_a", "robot_b"],
                "keyframe_topic": "/team_slam/keyframes",
                "match_topic": "/team_slam/cross_robot_matches",
                "robust_inliers_topic": "/team_slam/robust_loop_inliers",
                "metrics_topic": "/team_slam/pose_graph_metrics",
                "team_pose_graph_backend": "auto",
                "export_dir": "logs/real_hybrid_team_pose_graph",
            }],
            output="screen",
        ),
        Node(
            package="team_loop_closure",
            executable="relative_transform_manager_node",
            name="relative_transform_manager_node",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robust_inliers_topic": "/team_slam/robust_loop_inliers",
                "pose_graph_metrics_topic": "/team_slam/pose_graph_metrics",
                "status_topic": "/team_slam/alignment_status",
                "relative_transform_topic": "/team_slam/relative_transform",
                "require_swarm_loop_agreement": LaunchConfiguration("require_swarm_loop_agreement"),
                "swarm_loop_relative_transform_topic": "/team_slam/swarm_lio2_relative_transform",
                "swarm_loop_agreement_max_translation": LaunchConfiguration("swarm_loop_agreement_max_translation"),
                "swarm_loop_agreement_max_yaw_deg": LaunchConfiguration("swarm_loop_agreement_max_yaw_deg"),
                "publish_tf": False,
            }],
            output="screen",
        ),
    ])
