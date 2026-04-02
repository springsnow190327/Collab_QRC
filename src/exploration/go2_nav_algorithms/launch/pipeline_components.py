"""Shared scan pipeline launch builders owned by go2_nav_algorithms."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


NAV_ALGORITHMS_PKG = "go2_nav_algorithms"


def nav_profile_path(filename: str) -> str:
    return os.path.join(get_package_share_directory(NAV_ALGORITHMS_PKG), "config", "nav", filename)


def build_pointcloud_to_laserscan_node(
    *,
    ns: str | None,
    use_sim_time,
    extra_params=None,
    remappings=None,
    name: str = "pointcloud_to_laserscan",
    condition=None,
):
    params = [
        {
            "use_sim_time": use_sim_time,
            "target_frame": "base_link",
            "transform_tolerance": 2.0,
            "min_height": 0.05,
            "max_height": 1.0,
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": 0.006135923151543,
            "scan_time": 0.1,
            "range_min": 0.05,
            "range_max": 20.0,
            "use_inf": True,
        }
    ]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "pointcloud_to_laserscan",
        "executable": "pointcloud_to_laserscan_node",
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    if condition is not None:
        kwargs["condition"] = condition
    return Node(**kwargs)


def build_simple_scan_mapper_cpp_node(
    *,
    ns: str | None,
    use_sim_time,
    profile: str | None = None,
    extra_params=None,
    remappings=None,
    name: str = "simple_scan_mapper_cpp",
    condition=None,
):
    params = [{"use_sim_time": use_sim_time}]
    if profile:
        params.insert(0, nav_profile_path(profile))
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": NAV_ALGORITHMS_PKG,
        "executable": "simple_scan_mapper_cpp",
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    if condition is not None:
        kwargs["condition"] = condition
    return Node(**kwargs)


def build_goal_assigner_passthrough_node(
    *,
    use_sim_time,
    namespaces,
    input_topic_suffix: str,
    output_topic_suffix: str,
    publish_rate: float,
    hold_last: bool,
    package: str = "go2_gazebo_sim",
    executable: str = "goal_assigner_passthrough.py",
    name: str = "goal_assigner_passthrough",
    condition=None,
):
    kwargs = {
        "package": package,
        "executable": executable,
        "name": name,
        "parameters": [
            {"use_sim_time": use_sim_time},
            {"namespaces": namespaces},
            {"input_topic_suffix": input_topic_suffix},
            {"output_topic_suffix": output_topic_suffix},
            {"publish_rate": publish_rate},
            {"hold_last": hold_last},
        ],
        "output": "screen",
    }
    if condition is not None:
        kwargs["condition"] = condition
    return Node(**kwargs)


def build_waypoint_mux_node(
    *,
    use_sim_time,
    namespaces,
    primary_input_suffix: str,
    fallback_input_suffix: str,
    output_suffix: str,
    primary_timeout_sec: float,
    output_rate: float,
    hold_last_output: bool,
    stamp_now: bool,
    package: str = "go2_gazebo_sim",
    executable: str = "waypoint_mux.py",
    name: str = "waypoint_mux",
    condition=None,
):
    kwargs = {
        "package": package,
        "executable": executable,
        "name": name,
        "parameters": [
            {"use_sim_time": use_sim_time},
            {"namespaces": namespaces},
            {"primary_input_suffix": primary_input_suffix},
            {"fallback_input_suffix": fallback_input_suffix},
            {"output_suffix": output_suffix},
            {"primary_timeout_sec": primary_timeout_sec},
            {"output_rate": output_rate},
            {"hold_last_output": hold_last_output},
            {"stamp_now": stamp_now},
        ],
        "output": "screen",
    }
    if condition is not None:
        kwargs["condition"] = condition
    return Node(**kwargs)
