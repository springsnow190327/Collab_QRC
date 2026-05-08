#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("dynamic_filter_backend", default_value="temporal_voxel_fallback"),
        Node(
            package="slam_backend_adapters",
            executable="dynamic_lio_filtering_node",
            name="dynamic_lio_filtering_node",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "namespaces": ["robot_a", "robot_b"],
                "dynamic_filter_backend": LaunchConfiguration("dynamic_filter_backend"),
            }],
            output="screen",
        ),
    ])
