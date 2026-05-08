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
            "slam_backend": "swarm_lio2_shadow",
            "dynamic_filter_backend": dynamic_filter_backend,
            "base_frame": base_frame,
            "output_odom_frame_id": f"{ns}/odom",
            "output_child_frame_id": f"{ns}/base_link",
            "output_map_frame_id": f"{ns}/map",
            "output_static_cloud_frame_id": f"{ns}/base_link",
            "output_map_cloud_frame_id": f"{ns}/map",
            "use_sim_time": use_sim_time,
            "publish_tf": False,
        }],
        output="screen",
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    dynamic_filter_backend = LaunchConfiguration("dynamic_filter_backend")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("dynamic_filter_backend", default_value="none"),
        _adapter("robot_a", "base_link", use_sim_time, dynamic_filter_backend),
        _adapter("robot_b", "base_link", use_sim_time, dynamic_filter_backend),
    ])
