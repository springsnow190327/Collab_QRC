#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _adapter(ns: str, base_frame: str, use_sim_time, dynamic_filter_backend):
    return Node(
        package="slam_backend_adapters",
        executable="swarm_lio2_ros2_adapter_node",
        namespace=ns,
        name="swarm_lio2_ros2_adapter_node",
        parameters=[{
            "namespace": ns,
            "slam_backend": "swarm_lio2_primary",
            "dynamic_filter_backend": dynamic_filter_backend,
            "base_frame": base_frame,
            "use_sim_time": use_sim_time,
            "publish_tf": True,
        }],
        remappings=[
            ("/tf", f"/{ns}/tf"),
            ("/tf_static", f"/{ns}/tf_static"),
        ],
        output="screen",
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    dynamic_filter_backend = LaunchConfiguration("dynamic_filter_backend")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("dynamic_filter_backend", default_value="temporal_voxel_fallback"),
        _adapter("robot_a", "base_link", use_sim_time, dynamic_filter_backend),
        _adapter("robot_b", "b_base_link", use_sim_time, dynamic_filter_backend),
    ])
