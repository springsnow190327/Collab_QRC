# pc2_to_livox.launch.py — PointCloud2 → livox CustomMsg converter.
#
# Defaults wire the MuJoCo lidar sim (registered_scan / cloud topic) to
# /livox/lidar so a Livox-native SLAM downstream (Point-LIO) consumes it.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("input_topic", default_value="registered_scan"),
        DeclareLaunchArgument("output_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("frame_id", default_value="body"),
        DeclareLaunchArgument("offset_time_span_us", default_value="100.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="pc2_to_livox",
            executable="pc2_to_livox_node",
            name="pc2_to_livox",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "frame_id": LaunchConfiguration("frame_id"),
                "offset_time_span_us": LaunchConfiguration("offset_time_span_us"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
    ])
