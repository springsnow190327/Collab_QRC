#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("static_map_cleanup_backend", default_value="erasor_wrapper"),
        DeclareLaunchArgument("erasor_trigger_mode", default_value="manual"),
        DeclareLaunchArgument("export_dir", default_value="logs/erasor"),
        Node(
            package="slam_backend_adapters",
            executable="erasor_adapter_node",
            name="erasor_adapter_node",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "namespaces": ["robot_a", "robot_b"],
                "static_map_cleanup_backend": LaunchConfiguration("static_map_cleanup_backend"),
                "erasor_trigger_mode": LaunchConfiguration("erasor_trigger_mode"),
                "export_dir": LaunchConfiguration("export_dir"),
            }],
            output="screen",
        ),
    ])
