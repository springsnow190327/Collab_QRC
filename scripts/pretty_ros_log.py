#!/usr/bin/env python3
"""Pretty-print ROS/launch console lines for humans while preserving raw logs."""

from __future__ import annotations

import os
import re
import sys
import time
from pathlib import Path

ROS_LINE = re.compile(r"^\[([^\]]+)\] \[(DEBUG|INFO|WARN|ERROR|FATAL)\] \[[^\]]+\] \[([^\]]+)\]: (.*)$")
LAUNCH_LINE = re.compile(r"^\[(DEBUG|INFO|WARN|ERROR|FATAL)\] \[[^\]]+\]: (.*)$")
PREFIX_LINE = re.compile(r"^\[([^\]]+)\] (.*)$")
LEVEL_IN_MSG = re.compile(r"\[(WARN|ERROR|FATAL)\]")

COLOR_ENABLED = sys.stdout.isatty()
COLORS = {
    "DEBUG": "\033[0;90m",
    "INFO": "\033[0;36m",
    "WARN": "\033[1;33m",
    "ERROR": "\033[1;31m",
    "FATAL": "\033[1;31m",
}
RESET = "\033[0m"
SESSION_DIR = os.environ.get("ROS_LOG_SESSION_DIR", "")
PIPELINE_LOG = None
STAGE_FILES: dict[str, object] = {}

if SESSION_DIR:
    session_path = Path(SESSION_DIR)
    session_path.mkdir(parents=True, exist_ok=True)
    (session_path / "stages").mkdir(parents=True, exist_ok=True)
    PIPELINE_LOG = (session_path / "pipeline.log").open("a", encoding="utf-8")


def normalize_tag(tag: str) -> str:
    return tag.strip().lower().replace("/", ".")


def classify(tag: str, message: str) -> str:
    tag_n = normalize_tag(tag)
    msg = message.lower()

    if tag_n in {"launch"}:
        return "LAUNCH"
    if any(k in tag_n for k in ("gzserver", "gzclient", "gazebo", "spawn_entity", "initial_pose_guard", "robot_state_publisher", "contact_sensor")):
        return "SIM"
    if any(k in tag_n for k in ("qos_bridge", "pointcloud_adapter", "pointcloud_to_laserscan", "sensor_transformer")):
        return "SENSE"
    if any(k in tag_n for k in ("cartographer", "fastlio", "slam_odom_relay", "carto_odom_bridge", "gt_odom_relay", "ekf", "state_estimation")):
        return "SLAM"
    if any(k in tag_n for k in ("simple_scan_mapper", "octomap", "probability_grid_binarizer", "map_renderer", "shared_map_fuser", "skeleton_extractor")):
        return "MAP"
    if any(k in tag_n for k in ("cfpa2", "frontier")):
        return "FRONTIER"
    if any(k in tag_n for k in ("async_grid_mppi_nav", "controller_server", "behavior_server", "lifecycle_manager_navigation")):
        return "PLAN" if "async_grid_mppi_nav" in tag_n else "NAV"
    if "default_nav" in tag_n:
        if any(k in msg for k in ("grid planner", "planned_", "plan_wps", "planner_mode", "a*", "d* lite", "goal update", "new goal")):
            return "PLAN"
        return "NAV"
    if any(k in tag_n for k in ("wall_collision_checker", "autonomy_enabler")):
        return "NAV"
    if any(k in tag_n for k in ("twist_bridge", "hybrid_cmd_router", "quadruped_controller", "cmd_router", "cmd_vel_safety_gate")):
        return "CMD"
    if any(k in tag_n for k in ("vlm_", "green_marker_detector")):
        return "VLM"
    if "rviz" in tag_n:
        return "VIZ"
    return "MISC"


def stage_writer(stage: str):
    if not SESSION_DIR:
        return None
    if stage not in STAGE_FILES:
        path = Path(SESSION_DIR) / "stages" / f"{stage.lower()}.log"
        STAGE_FILES[stage] = path.open("a", encoding="utf-8")
    return STAGE_FILES[stage]


def emit(level: str, tag: str, message: str) -> None:
    stamp = time.strftime("%H:%M:%S")
    stage = classify(tag, message)
    line = f"{stamp} {stage:<9} {level:<5} {tag:<28} | {message}"
    if COLOR_ENABLED:
        sys.stdout.write(f"{COLORS.get(level, '')}{line}{RESET}\n")
    else:
        sys.stdout.write(f"{line}\n")
    sys.stdout.flush()
    if PIPELINE_LOG is not None:
        PIPELINE_LOG.write(f"{line}\n")
        PIPELINE_LOG.flush()
    writer = stage_writer(stage)
    if writer is not None:
        writer.write(f"{line}\n")
        writer.flush()


for raw in sys.stdin:
    line = raw.rstrip("\n")

    match = ROS_LINE.match(line)
    if match:
        _, level, node_name, message = match.groups()
        emit(level, node_name, message)
        continue

    match = LAUNCH_LINE.match(line)
    if match:
        level, message = match.groups()
        emit(level, "launch", message)
        continue

    match = PREFIX_LINE.match(line)
    if match:
        proc_name, message = match.groups()
        level = "INFO"
        level_match = LEVEL_IN_MSG.search(message)
        if level_match:
            level = level_match.group(1)
        emit(level, proc_name, message)
        continue

    stamp = time.strftime("%H:%M:%S")
    out = f"{stamp} {'MISC':<9} {'INFO':<5} {'raw':<28} | {line}"
    sys.stdout.write(f"{out}\n")
    sys.stdout.flush()
    if PIPELINE_LOG is not None:
        PIPELINE_LOG.write(f"{out}\n")
        PIPELINE_LOG.flush()
    writer = stage_writer("MISC")
    if writer is not None:
        writer.write(f"{out}\n")
        writer.flush()

if PIPELINE_LOG is not None:
    PIPELINE_LOG.close()
for handle in STAGE_FILES.values():
    handle.close()
