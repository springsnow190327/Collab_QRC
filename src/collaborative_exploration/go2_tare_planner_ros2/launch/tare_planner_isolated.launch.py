from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="go2_tare_planner_ros2",
                executable="tare_planner_node",
                name="tare_planner_node",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"namespaces": ["robot_a", "robot_b"]},
                    {"input_topic_suffix": "/way_point_seed"},
                    {"output_topic_suffix": "/way_point_tare"},
                    {"output_rate_hz": 5.0},
                ],
                output="screen",
            ),
        ]
    )
