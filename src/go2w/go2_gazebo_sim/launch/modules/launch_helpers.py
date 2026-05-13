"""Shared utilities for nav_test_mujoco_fastlio*.launch.py.

Extracted 2026-05-10: each of the three nav_test_mujoco_fastlio launches
(single / dual / mixed) had local copies of these four helpers. Differences
were trivial (variable names, slightly different docstrings) — the actual
implementations were identical-up-to-renaming.

The bigger per-launch helpers (`_build_sensor_bridges`, `_build_fastlio_nav_stack`)
are intentionally NOT extracted: mixed sets `publish_tf=False` (fast_lio_tf_adapter
owns the TF chain) while dual sets `publish_tf=True` (Fast-LIO seeds odom→base_link).
Merging them would either add flags that hide the divergence, or risk regressing
tested behavior.
"""
from __future__ import annotations

import yaml

from launch.substitutions import LaunchConfiguration


def as_bool(value: str) -> bool:
    """Parse a launch-arg string as bool ('true', '1', 'yes', 'on' → True)."""
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def get_launch_arg(context, key: str) -> str:
    """Resolve a LaunchConfiguration to its string value at context-eval time."""
    return LaunchConfiguration(key).perform(context)


def load_yaml_params(yaml_path: str) -> dict:
    """Load a ROS 2 YAML param file and return the ros__parameters dict.

    CMU autonomy stack YAML files are keyed by unqualified node name
    (e.g. ``far_planner:``), which doesn't match when the node is launched
    in a namespace. Strip the outer key and return just the parameter dict
    so it can be merged into the launch params list.
    """
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f) or {}
    for _node_name, inner in data.items():
        if isinstance(inner, dict) and "ros__parameters" in inner:
            return dict(inner["ros__parameters"])
    return data


# Pattern list mirrors benchmark_fastlio.sh's cleanup_procs so we cover
# everything that could pollute DDS discovery and break controller_manager
# service lookup in the next launch. Previously included "/go2w_perception/"
# but that patterned-killed the same launch's qos_bridge / pointcloud_adapter
# when they spawned at T=0 alongside cleanup_stale — breaking the
# LiDAR → Fast-LIO → octomap → /map → CFPA2 pipeline from step 1. External
# benchmark scripts (benchmark_fastlio.sh) handle stale perception procs
# before launching; no need to duplicate here.
_CLEANUP_PATTERNS = (
    "ros2 launch go2_gazebo_sim nav_test_mujoco",
    "mujoco_ros2_control",
    "/mujoco_sensor_bridge/",
    "/champ_base/",
    "/fast_lio/",
    "fastlio_mapping",
    "__node:=slam_node",
    "laserMapping",
    "fast_lio_tf_adapter",
    "cloud_world_offset_bridge",
    "/far_planner/",
    "/local_planner/",
    "/terrain_analysis",
    "/octomap_server/",
    "/cfpa2_collaborative_autonomy/",
    "/robot_state_publisher",
    "/robot_localization/",
    "/opt/ros/.*/lib/controller_manager/spawner",
    "session_reporter.py",
    "dual_robot_collision_monitor.py",
)


def build_cleanup_stale_cmd() -> str:
    """Return a shell snippet that kills leftover sim/nav processes from a prior run.

    Uses pgrep -f against `_CLEANUP_PATTERNS`, skipping the launch's own PID + parent.
    Sends SIGTERM, waits 1 s, sends SIGKILL. Also clears DDS shm files.
    """
    cmd = [
        "SELF=$$; PARENT=$PPID; ",
        "kill_pattern(){ ",
        "  PATTERN=\"$1\"; SIGNAL=\"$2\"; ",
        "  for PID in $(pgrep -f \"$PATTERN\" 2>/dev/null || true); do ",
        "    [ \"$PID\" = \"$SELF\" ] && continue; ",
        "    [ \"$PID\" = \"$PARENT\" ] && continue; ",
        "    kill -\"$SIGNAL\" \"$PID\" 2>/dev/null || true; ",
        "  done; ",
        "}; ",
    ]
    for p in _CLEANUP_PATTERNS:
        cmd.append(f"kill_pattern '{p}' TERM; ")
    cmd.append("sleep 1; ")
    for p in _CLEANUP_PATTERNS:
        cmd.append(f"kill_pattern '{p}' KILL; ")
    cmd.append(
        "rm -f /dev/shm/sem.fastrtps_* /dev/shm/sem.fastdds_* "
        "/dev/shm/fastrtps_* /dev/shm/fastdds_* 2>/dev/null || true; "
    )
    cmd.append("sleep 0.5")
    return "".join(cmd)
