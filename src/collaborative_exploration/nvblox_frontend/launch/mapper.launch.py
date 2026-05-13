"""Standalone launch for the nvblox_frontend mapper node.

Wires Point-LIO / Fast-LIO outputs into nvblox::Mapper and publishes:
  - /<robot_namespace>/traversability_grid  (2.5D nav_msgs/OccupancyGrid)
  - /<robot_namespace>/voxels_3d            (sparse nvblox_frontend_msgs/VoxelGrid3D)

Typical usage:
  ros2 launch nvblox_frontend mapper.launch.py \
       robot_namespace:=robot \
       cloud_topic:=/robot/cloud_registered_body \
       odom_topic:=/robot/odom/nav \
       use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("robot_namespace", default_value="robot"),
        DeclareLaunchArgument("use_sim_time",   default_value="true"),
        DeclareLaunchArgument("cloud_topic",    default_value="/cloud_registered_body"),
        DeclareLaunchArgument("odom_topic",     default_value="/Odometry"),
        DeclareLaunchArgument("world_frame",    default_value="map"),
        DeclareLaunchArgument("voxel_size_m",   default_value="0.10"),
        DeclareLaunchArgument("publish_period_s", default_value="0.5"),
        DeclareLaunchArgument("trav_xy_extent_m",  default_value="20.0"),
        DeclareLaunchArgument("voxel_xy_extent_m", default_value="20.0"),
        DeclareLaunchArgument("voxel_z_extent_m",  default_value="3.0"),
        DeclareLaunchArgument("voxel_z_origin_m",  default_value="-0.5"),
        DeclareLaunchArgument("slope_max_deg",     default_value="30.0"),
        DeclareLaunchArgument("step_max_m",        default_value="0.20"),
        DeclareLaunchArgument("robot_clearance_m", default_value="0.50"),
    ]

    ns = LaunchConfiguration("robot_namespace")

    mapper = Node(
        package="nvblox_frontend",
        executable="mapper_node",
        name="nvblox_frontend_mapper",
        namespace=ns,
        output="screen",
        parameters=[{
            "use_sim_time":      LaunchConfiguration("use_sim_time"),
            "cloud_topic":       LaunchConfiguration("cloud_topic"),
            "odom_topic":        LaunchConfiguration("odom_topic"),
            "world_frame":       LaunchConfiguration("world_frame"),
            "voxel_size_m":      LaunchConfiguration("voxel_size_m"),
            "publish_period_s":  LaunchConfiguration("publish_period_s"),
            "trav_xy_extent_m":  LaunchConfiguration("trav_xy_extent_m"),
            "voxel_xy_extent_m": LaunchConfiguration("voxel_xy_extent_m"),
            "voxel_z_extent_m":  LaunchConfiguration("voxel_z_extent_m"),
            "voxel_z_origin_m":  LaunchConfiguration("voxel_z_origin_m"),
            "slope_max_deg":     LaunchConfiguration("slope_max_deg"),
            "step_max_m":        LaunchConfiguration("step_max_m"),
            "robot_clearance_m": LaunchConfiguration("robot_clearance_m"),
        }],
    )

    return LaunchDescription([*args, mapper])
