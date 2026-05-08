#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    slam_backend = LaunchConfiguration("slam_backend")
    dynamic_filter_backend = LaunchConfiguration("dynamic_filter_backend")
    base_frame = LaunchConfiguration("base_frame")
    use_sim_time = LaunchConfiguration("use_sim_time")
    swarm_lio2_odom_topic = LaunchConfiguration("swarm_lio2_odom_topic")
    swarm_lio2_cloud_static_topic = LaunchConfiguration("swarm_lio2_cloud_static_topic")
    swarm_lio2_cloud_map_topic = LaunchConfiguration("swarm_lio2_cloud_map_topic")
    swarm_lio2_relative_transform_topic = LaunchConfiguration("swarm_lio2_relative_transform_topic")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="robot_a"),
        DeclareLaunchArgument("slam_backend", default_value="swarm_lio2_shadow"),
        DeclareLaunchArgument("dynamic_filter_backend", default_value="none"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("swarm_lio2_odom_topic", default_value=""),
        DeclareLaunchArgument("swarm_lio2_cloud_static_topic", default_value=""),
        DeclareLaunchArgument("swarm_lio2_cloud_map_topic", default_value=""),
        DeclareLaunchArgument("swarm_lio2_relative_transform_topic", default_value=""),
        Node(
            package="slam_backend_adapters",
            executable="swarm_lio2_ros2_adapter_node",
            namespace=namespace,
            name="swarm_lio2_ros2_adapter_node",
            parameters=[{
                "namespace": namespace,
                "slam_backend": slam_backend,
                "dynamic_filter_backend": dynamic_filter_backend,
                "base_frame": base_frame,
                "use_sim_time": use_sim_time,
                "swarm_lio2_odom_topic": swarm_lio2_odom_topic,
                "swarm_lio2_cloud_static_topic": swarm_lio2_cloud_static_topic,
                "swarm_lio2_cloud_map_topic": swarm_lio2_cloud_map_topic,
                "swarm_lio2_relative_transform_topic": swarm_lio2_relative_transform_topic,
            }],
            remappings=[
                ("/tf", ["/", namespace, "/tf"]),
                ("/tf_static", ["/", namespace, "/tf_static"]),
            ],
            output="screen",
        ),
    ])
