#!/usr/bin/env python3
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_path = Path(get_package_share_directory("go2_gazebo_sim")) / "launch" / "dynamic_lio_filtering.launch.py"
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_path)))])
