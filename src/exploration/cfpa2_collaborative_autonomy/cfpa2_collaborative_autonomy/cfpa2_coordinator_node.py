#!/usr/bin/env python3
"""CFPA2 ROS2 coordinator with optional space-time A* waypointing."""

from __future__ import annotations

import ctypes
import heapq
import json
import math
import os
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any, Optional

import numpy as np

# ---------- C++ grid-ops acceleration (optional) ----------
_GRID_OPS_LIB = None
try:
    _here = os.path.dirname(os.path.abspath(__file__))
    _here_real = os.path.dirname(os.path.realpath(__file__))
    for _d in (_here, _here_real):
        _so = os.path.join(_d, "cfpa2_grid_ops.so")
        if os.path.isfile(_so):
            _GRID_OPS_LIB = ctypes.CDLL(_so)
            break
    _GRID_OPS_LIB.extract_frontiers.restype = ctypes.c_int
    _GRID_OPS_LIB.extract_frontiers.argtypes = [
        ctypes.POINTER(ctypes.c_int8),   # grid
        ctypes.c_int, ctypes.c_int,       # W, H
        ctypes.c_float, ctypes.c_float, ctypes.c_float,  # res, ox, oy
        ctypes.c_int,                     # stride
        ctypes.c_float,                   # min_cluster_area
        ctypes.c_int,                     # clearance_cells
        ctypes.c_int8, ctypes.c_int8, ctypes.c_int8,  # free_val, unknown_val, occ_threshold
        ctypes.POINTER(ctypes.c_float),   # out_x
        ctypes.POINTER(ctypes.c_float),   # out_y
        ctypes.c_int,                     # max_out
    ]
    _GRID_OPS_LIB.distance_transform.restype = None
    _GRID_OPS_LIB.distance_transform.argtypes = [
        ctypes.POINTER(ctypes.c_int8),   # grid
        ctypes.c_int, ctypes.c_int,       # W, H
        ctypes.c_int, ctypes.c_int,       # sx, sy
        ctypes.c_int8,                    # free_val
        ctypes.POINTER(ctypes.c_int),     # dist_out
    ]
    _GRID_OPS_LIB.batch_info_gain.restype = None
    _GRID_OPS_LIB.batch_info_gain.argtypes = [
        ctypes.POINTER(ctypes.c_int8),   # grid
        ctypes.c_int, ctypes.c_int,       # W, H
        ctypes.c_float, ctypes.c_float, ctypes.c_float,  # res, ox, oy
        ctypes.POINTER(ctypes.c_float),   # goal_x
        ctypes.POINTER(ctypes.c_float),   # goal_y
        ctypes.c_int,                     # n_goals
        ctypes.c_int,                     # radius
        ctypes.c_int8,                    # unknown_val
        ctypes.POINTER(ctypes.c_float),   # gains_out
    ]
except Exception:
    _GRID_OPS_LIB = None

import rclpy
from geometry_msgs.msg import Point, PointStamped
from .map_merge_utils import build_fallback_map, build_shared_with_local_patches
from .mdvrp_solver import first_goal_for_route, solve_mdvrp
from mtare_ros2.msg import GridWorldStatus
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker, MarkerArray


def _resolve_cfpa2_overlap_penalty_fn():
    candidates: list[Path] = []
    here = Path(__file__).resolve()

    # Source-tree path: .../src/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy
    if len(here.parents) >= 3:
        candidates.append(here.parents[2])

    # Generic fallback: search upward for repository root containing cfpa2_demo.
    for parent in here.parents[:8]:
        if (parent / "cfpa2_demo").is_dir():
            candidates.append(parent)

    for root in candidates:
        if not (root / "cfpa2_demo").is_dir():
            continue
        root_str = str(root)
        if root_str not in sys.path:
            sys.path.insert(0, root_str)
        try:
            from cfpa2_demo.core.allocator import overlap_penalty as cfpa2_overlap_penalty
        except Exception:
            continue
        return cfpa2_overlap_penalty
    return None


_CFPA2_OVERLAP_PENALTY_FN = _resolve_cfpa2_overlap_penalty_fn()


def select_first_route_goals(
    *,
    namespaces: list[str],
    routes: dict[int, list[int]],
    exploring_cells: list[tuple[float, float, float]],
    robot_xy: dict[str, tuple[float, float]],
    min_assign_distance: float,
) -> dict[str, tuple[float, float]]:
    goals: dict[str, tuple[float, float]] = {}
    for idx, ns in enumerate(namespaces):
        goals_for_robot = routes.get(idx, [])
        goal = first_goal_for_route(
            goals_for_robot,
            exploring_cells,
            robot_xy=robot_xy.get(ns),
            min_assign_distance=min_assign_distance,
        )
        if goal is not None:
            goals[ns] = goal
    return goals


class CFPA2Coordinator(Node):
    def __init__(
        self,
        *,
        node_name: str = "cfpa2_coordinator",
        default_namespaces: Optional[list[str]] = None,
        startup_label: str = "cfpa2_coordinator",
        planner_desc: str = "Coordinator",
    ) -> None:
        super().__init__(node_name)

        if default_namespaces is None:
            default_namespaces = ["robot_a", "robot_b"]
        self._startup_label = startup_label
        self._planner_desc = planner_desc

        self.declare_parameter("namespaces", default_namespaces)
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("beta", 0.18)
        self.declare_parameter("sensor_range", 3.5)
        self.declare_parameter("frontier_stride", 2)
        self.declare_parameter("max_targets", 800)
        self.declare_parameter("goal_topic_suffix", "/way_point_coord")
        self.declare_parameter("output_mode", "waypoint_coord")
        self.declare_parameter("tare_goal_topic_suffix", "/way_point_tare")
        self.declare_parameter("relocation_goal_topic_suffix", "/goal_point")
        self.declare_parameter("grid_world_status_topic_suffix", "/grid_world_status")
        self.declare_parameter("nav_status_topic_suffix", "/nav_status")
        self.declare_parameter("use_shared_map", False)
        self.declare_parameter("shared_map_topic", "/disco_slam/global_map")
        self.declare_parameter("shared_map_wait_sec", 8.0)
        self.declare_parameter("shared_map_local_patch_radius_m", 2.5)
        self.declare_parameter("free_value", 0)
        self.declare_parameter("unknown_value", -1)
        self.declare_parameter("occupancy_block_threshold", 50)
        self.declare_parameter("switch_hysteresis", 0.02)
        self.declare_parameter("switch_min_dist", 0.35)
        self.declare_parameter("min_assign_distance", 0.30)

        # Algorithm-selection and Level-1 stabilization controls.
        self.declare_parameter("algorithm_mode", "mtare")
        self.declare_parameter("goal_lock_sec", 5.0)
        self.declare_parameter("progress_window_sec", 3.0)
        self.declare_parameter("progress_min_delta_m", 0.15)
        self.declare_parameter("blacklist_fail_count", 2)
        self.declare_parameter("blacklist_ttl_sec", 30.0)
        self.declare_parameter("blacklist_key_resolution", 0.5)
        self.declare_parameter("reached_blacklist_dist", 0.30)
        self.declare_parameter("reached_blacklist_repeat_count", 3)
        self.declare_parameter("reached_blacklist_ttl_sec", 12.0)
        self.declare_parameter("overlap_weight", 1.0)
        self.declare_parameter("cfpa2_w_ig", 1.0)
        self.declare_parameter("cfpa2_w_c", 0.6)
        self.declare_parameter("cfpa2_w_sw", 0.2)
        self.declare_parameter("cfpa2_lambda_overlap", 1.0)
        self.declare_parameter("cfpa2_w_momentum", 0.8)
        self.declare_parameter("cfpa2_min_utility", -0.5)
        self.declare_parameter("cfpa2_sigma_overlap_m", 0.0)
        self.declare_parameter("cfpa2_stuck_lock_sec", 45.0)
        self.declare_parameter("cfpa2_stuck_min_motion_m", 0.20)
        self.declare_parameter("cfpa2_stuck_blacklist_sec", 60.0)
        self.declare_parameter("local_nav_status_stale_sec", 3.0)
        self.declare_parameter("local_nav_stall_blacklist_sec", 45.0)
        self.declare_parameter("cfpa2_close_stop_radius_m", 0.35)
        self.declare_parameter("cfpa2_close_stop_speed_epsilon", 0.02)
        self.declare_parameter("cfpa2_space_time_enabled", True)
        self.declare_parameter("cfpa2_space_time_horizon_sec", 5.0)
        self.declare_parameter("cfpa2_space_time_dt_sec", 0.40)
        self.declare_parameter("cfpa2_space_time_safety_radius_m", 0.45)
        self.declare_parameter("cfpa2_space_time_waypoint_lookahead_m", 0.9)
        self.declare_parameter("cfpa2_space_time_window_margin_m", 3.0)
        self.declare_parameter("cfpa2_space_time_max_expansions", 12000)
        self.declare_parameter("cfpa2_space_time_assumed_speed_mps", 0.25)
        self.declare_parameter("cfpa2_space_time_max_speed_mps", 0.60)
        self.declare_parameter("cfpa2_frontier_min_cluster_area_m2", 0.20)
        self.declare_parameter("cfpa2_frontier_obstacle_clearance_m", 0.40)
        self.declare_parameter("communication_timeout_sec", 6.0)
        self.declare_parameter("prediction_horizon_sec", 4.0)
        self.declare_parameter("pursuit_weight", 2.0)
        self.declare_parameter("pursuit_switch_margin", 0.1)
        self.declare_parameter("exploration_gain_radius_cells", 4)
        self.declare_parameter("meeting_min_distance", 1.5)
        self.declare_parameter("teammate_stale_ttl_sec", 120.0)
        self.declare_parameter("mui_resolve_period_sec", 5.0)
        self.declare_parameter("mui_mdvrp_time_limit_sec", 1.0)
        self.declare_parameter("mui_max_exploring_cells", 120)
        self.declare_parameter("mui_cell_merge_resolution_m", 1.0)
        self.declare_parameter("mui_unreachable_penalty_m", 200.0)
        self.declare_parameter("marker_frame_override", "world")
        self.declare_parameter("coordinator_map_topic", "/mtare/coordinator_map")
        self.declare_parameter("robot_markers_topic", "/mtare/robot_markers")
        self.declare_parameter("trajectory_max_points", 600)
        self.declare_parameter("trajectory_min_point_distance", 0.08)
        self.declare_parameter("robot_marker_scale", 0.35)
        self.declare_parameter("perf_enable", True)
        self.declare_parameter("perf_tick_window_size", 240)
        self.declare_parameter("perf_min_samples", 20)
        self.declare_parameter("perf_tick_warn_p95_ms", 150.0)
        self.declare_parameter("perf_cpu_warn_pct", 15.0)
        self.declare_parameter("adaptive_load_shedding_enabled", False)
        self.declare_parameter("adaptive_budget_utilization", 0.85)
        self.declare_parameter("adaptive_restore_utilization", 0.55)
        self.declare_parameter("adaptive_max_frontier_stride", 8)
        self.declare_parameter("adaptive_min_max_targets", 120)
        self.declare_parameter("adaptive_min_exploration_gain_radius_cells", 2)
        self.declare_parameter("adaptive_max_skip_ticks", 2)
        self.declare_parameter("debug_no_goal_logging", True)
        self.declare_parameter("debug_no_goal_log_interval_sec", 2.0)

        self.namespaces = [str(x) for x in self.get_parameter("namespaces").value]
        self.publish_rate = max(0.2, float(self.get_parameter("publish_rate").value))
        self.beta = float(self.get_parameter("beta").value)
        self.sensor_range = max(0.1, float(self.get_parameter("sensor_range").value))
        self.frontier_stride = max(1, int(self.get_parameter("frontier_stride").value))
        self.max_targets = max(50, int(self.get_parameter("max_targets").value))
        self.goal_topic_suffix = str(self.get_parameter("goal_topic_suffix").value)
        self.output_mode = str(self.get_parameter("output_mode").value).strip().lower()
        if self.output_mode not in {"waypoint_coord", "exact_split"}:
            self.get_logger().warn(
                f"Unknown output_mode='{self.output_mode}', falling back to waypoint_coord"
            )
            self.output_mode = "waypoint_coord"
        self.tare_goal_topic_suffix = str(self.get_parameter("tare_goal_topic_suffix").value)
        self.relocation_goal_topic_suffix = str(self.get_parameter("relocation_goal_topic_suffix").value)
        self.grid_world_status_topic_suffix = str(
            self.get_parameter("grid_world_status_topic_suffix").value
        )
        self.nav_status_topic_suffix = str(self.get_parameter("nav_status_topic_suffix").value)
        self.use_shared_map = bool(self.get_parameter("use_shared_map").value)
        self.shared_map_topic = str(self.get_parameter("shared_map_topic").value)
        self.shared_map_wait_sec = max(0.0, float(self.get_parameter("shared_map_wait_sec").value))
        self.shared_map_local_patch_radius_m = max(
            0.0, float(self.get_parameter("shared_map_local_patch_radius_m").value)
        )
        self.free_value = int(self.get_parameter("free_value").value)
        self.unknown_value = int(self.get_parameter("unknown_value").value)
        self.occ_thresh = int(self.get_parameter("occupancy_block_threshold").value)
        self.switch_hysteresis = max(0.0, float(self.get_parameter("switch_hysteresis").value))
        self.switch_min_dist = max(0.1, float(self.get_parameter("switch_min_dist").value))
        self.min_assign_distance = max(0.0, float(self.get_parameter("min_assign_distance").value))

        self.algorithm_mode = str(self.get_parameter("algorithm_mode").value).strip().lower()
        if self.algorithm_mode != "cfpa2":
            self.get_logger().warn(
                f"CFPA2Coordinator forcing algorithm_mode=cfpa2 (received '{self.algorithm_mode}')."
            )
            self.algorithm_mode = "cfpa2"

        self.goal_lock_sec = max(0.0, float(self.get_parameter("goal_lock_sec").value))
        self.progress_window_sec = max(0.5, float(self.get_parameter("progress_window_sec").value))
        self.progress_min_delta_m = max(0.0, float(self.get_parameter("progress_min_delta_m").value))
        self.blacklist_fail_count = max(1, int(self.get_parameter("blacklist_fail_count").value))
        self.blacklist_ttl_sec = max(0.0, float(self.get_parameter("blacklist_ttl_sec").value))
        self.blacklist_key_resolution = max(
            0.05,
            float(self.get_parameter("blacklist_key_resolution").value),
        )
        self.reached_blacklist_dist = max(0.0, float(self.get_parameter("reached_blacklist_dist").value))
        self.reached_blacklist_repeat_count = max(
            1,
            int(self.get_parameter("reached_blacklist_repeat_count").value),
        )
        self.reached_blacklist_ttl_sec = max(
            0.0,
            float(self.get_parameter("reached_blacklist_ttl_sec").value),
        )
        self.overlap_weight = max(0.0, float(self.get_parameter("overlap_weight").value))
        self.cfpa2_w_ig = float(self.get_parameter("cfpa2_w_ig").value)
        self.cfpa2_w_c = max(0.0, float(self.get_parameter("cfpa2_w_c").value))
        self.cfpa2_w_sw = max(0.0, float(self.get_parameter("cfpa2_w_sw").value))
        self.cfpa2_lambda_overlap = max(0.0, float(self.get_parameter("cfpa2_lambda_overlap").value))
        self.cfpa2_w_momentum = max(0.0, float(self.get_parameter("cfpa2_w_momentum").value))
        self.cfpa2_min_utility = float(self.get_parameter("cfpa2_min_utility").value)
        self.cfpa2_sigma_overlap_m = max(0.0, float(self.get_parameter("cfpa2_sigma_overlap_m").value))
        self.cfpa2_stuck_lock_sec = max(0.0, float(self.get_parameter("cfpa2_stuck_lock_sec").value))
        self.cfpa2_stuck_min_motion_m = max(
            0.0, float(self.get_parameter("cfpa2_stuck_min_motion_m").value)
        )
        self.cfpa2_stuck_blacklist_sec = max(
            0.0, float(self.get_parameter("cfpa2_stuck_blacklist_sec").value)
        )
        self.local_nav_status_stale_sec = max(
            0.0, float(self.get_parameter("local_nav_status_stale_sec").value)
        )
        self.local_nav_stall_blacklist_sec = max(
            0.0, float(self.get_parameter("local_nav_stall_blacklist_sec").value)
        )
        self.cfpa2_close_stop_radius_m = max(
            0.0, float(self.get_parameter("cfpa2_close_stop_radius_m").value)
        )
        self.cfpa2_close_stop_speed_epsilon = max(
            0.0, float(self.get_parameter("cfpa2_close_stop_speed_epsilon").value)
        )
        self.cfpa2_space_time_enabled = bool(self.get_parameter("cfpa2_space_time_enabled").value)
        self.cfpa2_space_time_horizon_sec = max(
            0.5, float(self.get_parameter("cfpa2_space_time_horizon_sec").value)
        )
        self.cfpa2_space_time_dt_sec = max(
            0.05, float(self.get_parameter("cfpa2_space_time_dt_sec").value)
        )
        self.cfpa2_space_time_safety_radius_m = max(
            0.0, float(self.get_parameter("cfpa2_space_time_safety_radius_m").value)
        )
        self.cfpa2_space_time_waypoint_lookahead_m = max(
            0.1, float(self.get_parameter("cfpa2_space_time_waypoint_lookahead_m").value)
        )
        self.cfpa2_space_time_window_margin_m = max(
            0.0, float(self.get_parameter("cfpa2_space_time_window_margin_m").value)
        )
        self.cfpa2_space_time_max_expansions = max(
            1000, int(self.get_parameter("cfpa2_space_time_max_expansions").value)
        )
        self.cfpa2_space_time_assumed_speed_mps = max(
            0.01, float(self.get_parameter("cfpa2_space_time_assumed_speed_mps").value)
        )
        self.cfpa2_space_time_max_speed_mps = max(
            self.cfpa2_space_time_assumed_speed_mps,
            float(self.get_parameter("cfpa2_space_time_max_speed_mps").value),
        )
        self.cfpa2_frontier_min_cluster_area_m2 = max(
            0.0, float(self.get_parameter("cfpa2_frontier_min_cluster_area_m2").value)
        )
        self.cfpa2_frontier_obstacle_clearance_m = max(
            0.0, float(self.get_parameter("cfpa2_frontier_obstacle_clearance_m").value)
        )
        self.communication_timeout_sec = max(0.0, float(self.get_parameter("communication_timeout_sec").value))
        self.prediction_horizon_sec = max(0.0, float(self.get_parameter("prediction_horizon_sec").value))
        self.pursuit_weight = max(0.0, float(self.get_parameter("pursuit_weight").value))
        self.pursuit_switch_margin = float(self.get_parameter("pursuit_switch_margin").value)
        self.exploration_gain_radius_cells = max(1, int(self.get_parameter("exploration_gain_radius_cells").value))
        self.meeting_min_distance = max(0.0, float(self.get_parameter("meeting_min_distance").value))
        self.teammate_stale_ttl_sec = max(0.0, float(self.get_parameter("teammate_stale_ttl_sec").value))
        self.mui_resolve_period_sec = max(0.2, float(self.get_parameter("mui_resolve_period_sec").value))
        self.mui_mdvrp_time_limit_sec = max(0.1, float(self.get_parameter("mui_mdvrp_time_limit_sec").value))
        self.mui_max_exploring_cells = max(10, int(self.get_parameter("mui_max_exploring_cells").value))
        self.mui_cell_merge_resolution_m = max(
            0.05,
            float(self.get_parameter("mui_cell_merge_resolution_m").value),
        )
        self.mui_unreachable_penalty_m = max(
            0.0,
            float(self.get_parameter("mui_unreachable_penalty_m").value),
        )
        self.marker_frame_override = str(self.get_parameter("marker_frame_override").value).strip()
        self.coordinator_map_topic = str(self.get_parameter("coordinator_map_topic").value).strip()
        self.robot_markers_topic = str(self.get_parameter("robot_markers_topic").value).strip()
        self.trajectory_max_points = max(10, int(self.get_parameter("trajectory_max_points").value))
        self.trajectory_min_point_distance = max(
            0.0, float(self.get_parameter("trajectory_min_point_distance").value)
        )
        self.robot_marker_scale = max(0.05, float(self.get_parameter("robot_marker_scale").value))
        self.perf_enable = bool(self.get_parameter("perf_enable").value)
        self.perf_tick_window_size = max(20, int(self.get_parameter("perf_tick_window_size").value))
        self.perf_min_samples = max(5, int(self.get_parameter("perf_min_samples").value))
        self.perf_tick_warn_p95_ms = max(0.0, float(self.get_parameter("perf_tick_warn_p95_ms").value))
        self.perf_cpu_warn_pct = max(0.0, float(self.get_parameter("perf_cpu_warn_pct").value))
        self.adaptive_load_shedding_enabled = bool(
            self.get_parameter("adaptive_load_shedding_enabled").value
        )
        self.adaptive_budget_utilization = min(
            1.5,
            max(0.2, float(self.get_parameter("adaptive_budget_utilization").value)),
        )
        self.adaptive_restore_utilization = min(
            self.adaptive_budget_utilization,
            max(0.1, float(self.get_parameter("adaptive_restore_utilization").value)),
        )
        self.adaptive_max_frontier_stride = max(
            self.frontier_stride,
            int(self.get_parameter("adaptive_max_frontier_stride").value),
        )
        self.adaptive_min_max_targets = min(
            self.max_targets,
            max(50, int(self.get_parameter("adaptive_min_max_targets").value)),
        )
        self.adaptive_min_exploration_gain_radius_cells = min(
            self.exploration_gain_radius_cells,
            max(1, int(self.get_parameter("adaptive_min_exploration_gain_radius_cells").value)),
        )
        self.adaptive_max_skip_ticks = max(
            0,
            int(self.get_parameter("adaptive_max_skip_ticks").value),
        )
        self.debug_no_goal_logging = bool(self.get_parameter("debug_no_goal_logging").value)
        self.debug_no_goal_log_interval_sec = max(
            0.2, float(self.get_parameter("debug_no_goal_log_interval_sec").value)
        )

        self.maps: dict[str, OccupancyGrid] = {}
        self.shared_map: Optional[OccupancyGrid] = None
        self.odoms: dict[str, Odometry] = {}
        self.grid_world_status: dict[str, GridWorldStatus] = {}
        self.grid_world_status_rx_time_ns: dict[str, int] = {}
        self.nav_status: dict[str, dict[str, Any]] = {}
        self.nav_status_rx_time_ns: dict[str, int] = {}
        self.last_goal: dict[str, tuple[float, float]] = {}
        self.last_goal_set_time_ns: dict[str, int] = {}
        self.goal_progress_samples: dict[str, deque[tuple[int, float]]] = {
            ns: deque() for ns in self.namespaces
        }
        self.goal_fail_counts: dict[str, dict[tuple[int, int], int]] = {
            ns: {} for ns in self.namespaces
        }
        self.goal_blacklist_until_ns: dict[str, dict[tuple[int, int], int]] = {
            ns: {} for ns in self.namespaces
        }
        self.reached_goal_repeat_count: dict[str, int] = {ns: 0 for ns in self.namespaces}
        self.reached_goal_last_key: dict[str, Optional[tuple[int, int]]] = {
            ns: None for ns in self.namespaces
        }
        self.last_policy_reason: dict[str, str] = {ns: "init" for ns in self.namespaces}
        self.odom_rx_time_ns: dict[str, int] = {}
        self.odom_velocity_xy: dict[str, tuple[float, float]] = {ns: (0.0, 0.0) for ns in self.namespaces}
        self.trajectory_history: dict[str, deque[tuple[float, float]]] = {
            ns: deque(maxlen=self.trajectory_max_points) for ns in self.namespaces
        }
        self.goal_lock_start_xy: dict[str, Optional[tuple[float, float]]] = {
            ns: None for ns in self.namespaces
        }
        self.cfpa2_last_stuck_event_ns: dict[str, int] = {ns: 0 for ns in self.namespaces}
        self.local_nav_last_stall_event_count: dict[str, int] = {
            ns: 0 for ns in self.namespaces
        }
        self._frontier_replan_last_bl_ns: dict[str, int] = {
            ns: 0 for ns in self.namespaces
        }

        self._warned_missing_shared_map = False
        self._shared_map_fallback_active = False
        self._warned_cfpa2_two_robot_only = False
        self._cfpa2_last_close_stop_log_ns = 0
        self._start_ns = self.get_clock().now().nanoseconds
        self._summary_interval_sec = 10.0
        self._last_summary_ns = 0
        self._last_prereq_warn_ns = 0
        self._last_no_goal_debug_ns = 0
        self._tick_period_ms = 1000.0 / self.publish_rate
        self._perf_tick_durations_ms: deque[float] = deque(maxlen=self.perf_tick_window_size)
        self._last_perf_summary_ns = 0
        self._perf_last_cpu_process_sec = time.process_time()
        self._perf_last_cpu_wall_ns = time.perf_counter_ns()
        self._adaptive_frontier_stride = self.frontier_stride
        self._adaptive_max_targets = self.max_targets
        self._adaptive_exploration_gain_radius_cells = self.exploration_gain_radius_cells
        self._adaptive_skip_ticks = 0
        self._adaptive_tick_skip_counter = 0
        self._mui_last_solve_ns = 0
        self._mui_last_cell_keys: set[tuple[int, int, int]] = set()
        self._mui_routes: dict[str, list[int]] = {ns: [] for ns in self.namespaces}
        self._mui_cover_by_others: dict[str, set[int]] = {ns: set() for ns in self.namespaces}

        self.goal_pubs = {}
        self.tare_goal_pubs = {}
        self.relocation_goal_pubs = {}
        self.goal_marker_pubs = {}
        coordinator_map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.coordinator_map_pub = self.create_publisher(
            OccupancyGrid,
            self.coordinator_map_topic,
            coordinator_map_qos,
        )
        self.robot_markers_pub = self.create_publisher(MarkerArray, self.robot_markers_topic, 10)
        for ns in self.namespaces:
            self.create_subscription(OccupancyGrid, f"/{ns}/map", lambda m, n=ns: self._map_cb(m, n), 1)
            self.create_subscription(Odometry, f"/{ns}/odom/nav", lambda m, n=ns: self._odom_cb(m, n), 10)
            self.create_subscription(
                GridWorldStatus,
                f"/{ns}{self.grid_world_status_topic_suffix}",
                lambda m, n=ns: self._grid_world_status_cb(m, n),
                10,
            )
            self.create_subscription(
                String,
                f"/{ns}{self.nav_status_topic_suffix}",
                lambda m, n=ns: self._nav_status_cb(m, n),
                10,
            )
            self.create_subscription(
                Empty,
                f"/{ns}/frontier_replan",
                lambda m, n=ns: self._frontier_replan_cb(n),
                10,
            )
            self.goal_pubs[ns] = self.create_publisher(PointStamped, f"/{ns}{self.goal_topic_suffix}", 10)
            self.tare_goal_pubs[ns] = self.create_publisher(PointStamped, f"/{ns}{self.tare_goal_topic_suffix}", 10)
            self.relocation_goal_pubs[ns] = self.create_publisher(
                PointStamped, f"/{ns}{self.relocation_goal_topic_suffix}", 10
            )
            self.goal_marker_pubs[ns] = self.create_publisher(Marker, f"/{ns}/mtare_goal_marker", 10)
        if self.use_shared_map:
            self.create_subscription(OccupancyGrid, self.shared_map_topic, self._shared_map_cb, 1)

        self.timer = self.create_timer(1.0 / self.publish_rate, self._tick)
        self.get_logger().info(f"[planner_startup] {self._startup_label} initialized.")
        self.get_logger().info(
            f"{self._planner_desc} started for {self.namespaces}\n"
            f"  mode={self.algorithm_mode}  output={self.output_mode}\n"
            f"  ── Utility weights ──\n"
            f"    w_ig={self.cfpa2_w_ig:.2f}  w_c={self.cfpa2_w_c:.2f}  "
            f"w_sw={self.cfpa2_w_sw:.2f}  w_momentum={self.cfpa2_w_momentum:.2f}  "
            f"min_utility={self.cfpa2_min_utility:.2f}\n"
            f"  ── Frontier ──\n"
            f"    sensor_range={self.sensor_range:.1f}m  "
            f"gain_radius={self.exploration_gain_radius_cells}cells  "
            f"min_cluster={self.cfpa2_frontier_min_cluster_area_m2:.2f}m²  "
            f"beta={self.beta:.2f}\n"
            f"  ── Assignment ──\n"
            f"    min_dist={self.min_assign_distance:.2f}m  "
            f"switch_hysteresis={self.switch_hysteresis:.3f}  "
            f"goal_lock={self.goal_lock_sec:.0f}s\n"
            f"  ── Stuck / blacklist ──\n"
            f"    stuck_lock={self.cfpa2_stuck_lock_sec:.0f}s  "
            f"stuck_motion={self.cfpa2_stuck_min_motion_m:.2f}m  "
            f"bl_ttl={self.blacklist_ttl_sec:.0f}s  "
            f"reached_bl_dist={self.reached_blacklist_dist:.2f}m"
        )

    def _map_cb(self, msg: OccupancyGrid, ns: str) -> None:
        self.maps[ns] = msg

    def _odom_cb(self, msg: Odometry, ns: str) -> None:
        self.odoms[ns] = msg
        self.odom_rx_time_ns[ns] = self.get_clock().now().nanoseconds
        self.odom_velocity_xy[ns] = (
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
        )
        self._append_trajectory(ns, msg)

    def _grid_world_status_cb(self, msg: GridWorldStatus, ns: str) -> None:
        self.grid_world_status[ns] = msg
        self.grid_world_status_rx_time_ns[ns] = self.get_clock().now().nanoseconds

    def _nav_status_cb(self, msg: String, ns: str) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return
        self.nav_status[ns] = payload
        self.nav_status_rx_time_ns[ns] = self.get_clock().now().nanoseconds

    def _frontier_replan_cb(self, ns: str) -> None:
        """Reactive nav signals it cannot reach the current goal."""
        now_ns = self.get_clock().now().nanoseconds
        current_goal = self.last_goal.get(ns)
        if current_goal is None:
            return

        key = self._goal_key(current_goal)

        # Skip if this goal is already blacklisted.
        if self.goal_blacklist_until_ns[ns].get(key, 0) > now_ns:
            return

        # Cooldown: don't blacklist more often than once per 10s per namespace.
        last_bl_ns = self._frontier_replan_last_bl_ns.get(ns, 0)
        if (now_ns - last_bl_ns) < int(10e9):
            return
        self._frontier_replan_last_bl_ns[ns] = now_ns

        bl_sec = max(self.local_nav_stall_blacklist_sec, 20.0)
        until_ns = now_ns + int(bl_sec * 1e9)
        self.goal_blacklist_until_ns[ns][key] = max(
            self.goal_blacklist_until_ns[ns].get(key, 0),
            until_ns,
        )
        self.goal_fail_counts[ns][key] = 0
        self.goal_progress_samples[ns].clear()
        self.get_logger().warn(
            f"{ns}: frontier_replan received — blacklisting current goal "
            f"({current_goal[0]:.2f},{current_goal[1]:.2f}) for {bl_sec:.1f}s."
        )

    def _shared_map_cb(self, msg: OccupancyGrid) -> None:
        self.shared_map = msg
        if self._shared_map_fallback_active:
            self.get_logger().info(
                f"Shared map received on {self.shared_map_topic}; switching to shared-map coordination."
            )
        self._warned_missing_shared_map = False
        self._shared_map_fallback_active = False

    @staticmethod
    def _grid_index(x: int, y: int, w: int) -> int:
        return y * w + x

    def _world_to_grid(self, msg: OccupancyGrid, wx: float, wy: float) -> Optional[tuple[int, int]]:
        gx = int((wx - msg.info.origin.position.x) / msg.info.resolution)
        gy = int((wy - msg.info.origin.position.y) / msg.info.resolution)
        if gx < 0 or gy < 0 or gx >= msg.info.width or gy >= msg.info.height:
            return None
        return (gx, gy)

    def _grid_to_world(self, msg: OccupancyGrid, gx: int, gy: int) -> tuple[float, float]:
        return (
            msg.info.origin.position.x + (gx + 0.5) * msg.info.resolution,
            msg.info.origin.position.y + (gy + 0.5) * msg.info.resolution,
        )

    def _is_free(self, data: list[int], idx: int) -> bool:
        v = data[idx]
        return v != self.unknown_value and 0 <= v < self.occ_thresh

    def _is_unknown(self, data: list[int], idx: int) -> bool:
        return data[idx] == self.unknown_value

    def _has_frontier_obstacle_clearance(
        self, data: list[int], gx: int, gy: int, w: int, h: int, radius_cells: int
    ) -> bool:
        """Return True iff a circle of radius_cells around (gx,gy) contains no occupied cell."""
        if radius_cells <= 0:
            return True
        r2 = radius_cells * radius_cells
        for dy in range(-radius_cells, radius_cells + 1):
            ny = gy + dy
            if ny < 0 or ny >= h:
                return False
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > r2:
                    continue
                nx = gx + dx
                if nx < 0 or nx >= w:
                    return False
                if data[self._grid_index(nx, ny, w)] >= self.occ_thresh:
                    return False
        return True

    def _build_fallback_map(self) -> Optional[OccupancyGrid]:
        return build_fallback_map(
            namespaces=self.namespaces,
            maps=self.maps,
            unknown_value=self.unknown_value,
            free_value=self.free_value,
            occ_threshold=self.occ_thresh,
        )

    def _build_shared_with_local_patches(self, shared_msg: OccupancyGrid) -> OccupancyGrid:
        return build_shared_with_local_patches(
            shared_map=shared_msg,
            namespaces=self.namespaces,
            maps=self.maps,
            odoms=self.odoms,
            local_patch_radius_m=self.shared_map_local_patch_radius_m,
            unknown_value=self.unknown_value,
            free_value=self.free_value,
            occ_threshold=self.occ_thresh,
        )

    def _extract_frontiers(self, msg: OccupancyGrid) -> list[tuple[float, float]]:
        w = int(msg.info.width)
        h = int(msg.info.height)
        res = max(1e-6, float(msg.info.resolution))
        s = max(1, self._adaptive_frontier_stride)
        min_area_m2 = self.cfpa2_frontier_min_cluster_area_m2
        clearance_cells = int(math.ceil(self.cfpa2_frontier_obstacle_clearance_m / res))
        max_targets = self._adaptive_max_targets

        if _GRID_OPS_LIB is not None:
            return self._extract_frontiers_cpp(msg, w, h, res, s, min_area_m2, clearance_cells, max_targets)
        return self._extract_frontiers_py(msg, w, h, res, s, min_area_m2, clearance_cells, max_targets)

    def _extract_frontiers_cpp(self, msg, w, h, res, s, min_area_m2, clearance_cells, max_targets):
        grid_np = np.array(msg.data, dtype=np.int8)
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        out_x = (ctypes.c_float * max_targets)()
        out_y = (ctypes.c_float * max_targets)()
        n = _GRID_OPS_LIB.extract_frontiers(
            grid_np.ctypes.data_as(ctypes.POINTER(ctypes.c_int8)),
            w, h, res, ox, oy,
            s, min_area_m2, clearance_cells,
            ctypes.c_int8(self.free_value),
            ctypes.c_int8(self.unknown_value),
            ctypes.c_int8(self.occ_thresh),
            out_x, out_y, max_targets,
        )
        return [(float(out_x[i]), float(out_y[i])) for i in range(n)]

    def _extract_frontiers_py(self, msg, w, h, res, s, min_area_m2, clearance_cells, max_targets):
        data = msg.data
        out: list[tuple[float, float]] = []
        neighbor8 = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1))

        frontier_mask = [False] * (w * h)
        frontier_cells: list[tuple[int, int]] = []
        for gy in range(1, h - 1):
            row = gy * w
            for gx in range(1, w - 1):
                idx = row + gx
                if not self._is_free(data, idx):
                    continue
                found_unknown = False
                for dx, dy in neighbor8:
                    nidx = (gy + dy) * w + (gx + dx)
                    if self._is_unknown(data, nidx):
                        found_unknown = True
                        break
                if not found_unknown:
                    continue
                frontier_mask[idx] = True
                frontier_cells.append((gx, gy))

        if not frontier_cells:
            return out

        visited = [False] * (w * h)
        q: deque[tuple[int, int]] = deque()
        for seed_x, seed_y in frontier_cells:
            seed_idx = self._grid_index(seed_x, seed_y, w)
            if visited[seed_idx] or not frontier_mask[seed_idx]:
                continue

            visited[seed_idx] = True
            q.append((seed_x, seed_y))
            component: list[tuple[int, int]] = []

            while q:
                cx, cy = q.popleft()
                component.append((cx, cy))

                for dx, dy in neighbor8:
                    nx = cx + dx
                    ny = cy + dy
                    if nx <= 0 or ny <= 0 or nx >= (w - 1) or ny >= (h - 1):
                        continue
                    nidx = self._grid_index(nx, ny, w)
                    if visited[nidx] or not frontier_mask[nidx]:
                        continue
                    visited[nidx] = True
                    q.append((nx, ny))

            cluster_area_m2 = len(component) * res * res
            if cluster_area_m2 + 1e-9 < min_area_m2:
                continue

            for i, (gx, gy) in enumerate(component):
                if (i % s) != 0:
                    continue
                if not self._has_frontier_obstacle_clearance(data, gx, gy, w, h, clearance_cells):
                    continue
                out.append(self._grid_to_world(msg, gx, gy))
                if len(out) >= max_targets:
                    return out
        return out

    def _distance_transform(self, msg: OccupancyGrid, start_w: tuple[float, float]) -> dict[int, int]:
        start = self._world_to_grid(msg, start_w[0], start_w[1])
        if start is None:
            return {}

        w = int(msg.info.width)
        h = int(msg.info.height)
        sx, sy = start

        if _GRID_OPS_LIB is not None:
            return self._distance_transform_cpp(msg, w, h, sx, sy)
        return self._distance_transform_py(msg, w, h, sx, sy)

    def _distance_transform_cpp(self, msg, w, h, sx, sy):
        grid_np = np.array(msg.data, dtype=np.int8)
        dist_np = np.full(w * h, -1, dtype=np.int32)
        _GRID_OPS_LIB.distance_transform(
            grid_np.ctypes.data_as(ctypes.POINTER(ctypes.c_int8)),
            w, h, sx, sy,
            ctypes.c_int8(self.free_value),
            dist_np.ctypes.data_as(ctypes.POINTER(ctypes.c_int)),
        )
        # Store flat array for fast lookup; _grid_path_cost_m uses dict[int, int]
        # Convert to dict only for reachable cells
        indices = np.where(dist_np >= 0)[0]
        return dict(zip(indices.tolist(), dist_np[indices].tolist()))

    def _distance_transform_py(self, msg, w, h, sx, sy):
        data = msg.data
        sidx = self._grid_index(sx, sy, w)

        if not self._is_free(data, sidx):
            found = None
            for r in range(1, 13):
                for dy in range(-r, r + 1):
                    ny = sy + dy
                    if ny < 0 or ny >= h:
                        continue
                    for dx in range(-r, r + 1):
                        nx = sx + dx
                        if nx < 0 or nx >= w:
                            continue
                        nidx = self._grid_index(nx, ny, w)
                        if self._is_free(data, nidx):
                            found = (nx, ny, nidx)
                            break
                    if found is not None:
                        break
                if found is not None:
                    break
            if found is None:
                return {}
            sx, sy, sidx = found

        q = deque([(sx, sy)])
        dist = {sidx: 0}
        while q:
            cx, cy = q.popleft()
            cidx = self._grid_index(cx, cy, w)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx = cx + dx
                ny = cy + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                nidx = self._grid_index(nx, ny, w)
                if nidx in dist:
                    continue
                if not self._is_free(data, nidx):
                    continue
                dist[nidx] = dist[cidx] + 1
                q.append((nx, ny))
        return dist

    def _merge_targets(self, target_lists: list[list[tuple[float, float]]], merge_resolution: float) -> list[tuple[float, float]]:
        out: list[tuple[float, float]] = []
        seen: set[tuple[int, int]] = set()
        q = max(0.05, float(merge_resolution))
        for targets in target_lists:
            for wx, wy in targets:
                key = (int(round(wx / q)), int(round(wy / q)))
                if key in seen:
                    continue
                seen.add(key)
                out.append((wx, wy))
                if len(out) >= self.max_targets:
                    return out
        return out

    def _goal_key(self, goal: tuple[float, float]) -> tuple[int, int]:
        q = self.blacklist_key_resolution
        return (int(round(goal[0] / q)), int(round(goal[1] / q)))

    def _prune_blacklist(self, ns: str, now_ns: int) -> None:
        entries = self.goal_blacklist_until_ns[ns]
        expired = [k for k, until_ns in entries.items() if until_ns <= now_ns]
        for key in expired:
            entries.pop(key, None)

    def _is_blacklisted(self, ns: str, goal: tuple[float, float], now_ns: int) -> bool:
        self._prune_blacklist(ns, now_ns)
        key = self._goal_key(goal)
        until_ns = self.goal_blacklist_until_ns[ns].get(key, 0)
        return until_ns > now_ns

    def _register_goal_failure(self, ns: str, goal: tuple[float, float], now_ns: int, reason: str) -> None:
        key = self._goal_key(goal)
        counts = self.goal_fail_counts[ns]
        counts[key] = counts.get(key, 0) + 1
        if counts[key] < self.blacklist_fail_count:
            return

        counts[key] = 0
        if self.blacklist_ttl_sec <= 0.0:
            return

        until_ns = now_ns + int(self.blacklist_ttl_sec * 1e9)
        self.goal_blacklist_until_ns[ns][key] = until_ns
        self.get_logger().warn(
            f"{ns}: blacklisting goal ({goal[0]:.2f},{goal[1]:.2f}) for {self.blacklist_ttl_sec:.1f}s "
            f"after repeated {reason} failures."
        )

    def _consume_local_nav_stall_blacklists(self, now_ns: int) -> set[str]:
        if self.local_nav_stall_blacklist_sec <= 0.0 or self.local_nav_status_stale_sec <= 0.0:
            return set()

        forced_switch: set[str] = set()
        stale_ns = int(self.local_nav_status_stale_sec * 1e9)
        for ns in self.namespaces:
            diag = self.nav_status.get(ns)
            rx_time_ns = self.nav_status_rx_time_ns.get(ns, 0)
            if diag is None or rx_time_ns <= 0 or (now_ns - rx_time_ns) > stale_ns:
                continue

            try:
                stall_event_count = int(diag.get("stall_event_count", 0))
            except (TypeError, ValueError):
                continue

            if stall_event_count <= self.local_nav_last_stall_event_count.get(ns, 0):
                continue
            self.local_nav_last_stall_event_count[ns] = stall_event_count

            current_goal = self.last_goal.get(ns)
            if current_goal is None:
                continue

            key = self._goal_key(current_goal)
            until_ns = now_ns + int(self.local_nav_stall_blacklist_sec * 1e9)
            self.goal_blacklist_until_ns[ns][key] = max(
                self.goal_blacklist_until_ns[ns].get(key, 0),
                until_ns,
            )
            self.goal_fail_counts[ns][key] = 0
            self.goal_progress_samples[ns].clear()
            forced_switch.add(ns)

            mode = str(diag.get("mode", "navigate"))
            stall_sec = diag.get("stall_sec", "-")
            self.get_logger().warn(
                f"{ns}: local nav deadlock event #{stall_event_count} "
                f"(mode={mode}, stall_sec={stall_sec}) blacklisting current goal "
                f"({current_goal[0]:.2f},{current_goal[1]:.2f}) for "
                f"{self.local_nav_stall_blacklist_sec:.1f}s."
            )

        return forced_switch

    def _distance_robot_to_goal(self, ns: str, goal: tuple[float, float]) -> float:
        od = self.odoms[ns]
        rx = float(od.pose.pose.position.x)
        ry = float(od.pose.pose.position.y)
        return math.hypot(goal[0] - rx, goal[1] - ry)

    def _goal_too_close(self, ns: str, goal: tuple[float, float]) -> bool:
        if self.min_assign_distance <= 0.0:
            return False
        return self._distance_robot_to_goal(ns, goal) <= self.min_assign_distance

    def _update_reached_goal_blacklist(self, ns: str, now_ns: int) -> None:
        if self.reached_blacklist_ttl_sec <= 0.0 or self.reached_blacklist_dist <= 0.0:
            return

        goal = self.last_goal.get(ns)
        if goal is None:
            self.reached_goal_last_key[ns] = None
            self.reached_goal_repeat_count[ns] = 0
            return

        key = self._goal_key(goal)
        dist = self._distance_robot_to_goal(ns, goal)
        if dist > self.reached_blacklist_dist:
            self.reached_goal_last_key[ns] = key
            self.reached_goal_repeat_count[ns] = 0
            return

        # Do not repeatedly extend an active blacklist entry for the same key.
        if self.goal_blacklist_until_ns[ns].get(key, 0) > now_ns:
            self.reached_goal_repeat_count[ns] = 0
            return

        if self.reached_goal_last_key[ns] == key:
            self.reached_goal_repeat_count[ns] += 1
        else:
            self.reached_goal_last_key[ns] = key
            self.reached_goal_repeat_count[ns] = 1

        if self.reached_goal_repeat_count[ns] < self.reached_blacklist_repeat_count:
            return

        self.reached_goal_repeat_count[ns] = 0
        until_ns = now_ns + int(self.reached_blacklist_ttl_sec * 1e9)
        self.goal_blacklist_until_ns[ns][key] = until_ns
        self.get_logger().warn(
            f"{ns}: blacklisting repeatedly reached goal ({goal[0]:.2f},{goal[1]:.2f}) "
            f"for {self.reached_blacklist_ttl_sec:.1f}s "
            f"after {self.reached_blacklist_repeat_count} near-goal repeats "
            f"(dist<={self.reached_blacklist_dist:.2f}m)."
        )

    def _goal_reachable(self, map_msg: OccupancyGrid, dist_map: dict[int, int], goal: tuple[float, float]) -> bool:
        g = self._world_to_grid(map_msg, goal[0], goal[1])
        if g is None:
            return False
        idx = self._grid_index(g[0], g[1], int(map_msg.info.width))
        return idx in dist_map

    def _update_progress_samples(self, ns: str, now_ns: int) -> None:
        goal = self.last_goal.get(ns)
        if goal is None:
            return

        samples = self.goal_progress_samples[ns]
        samples.append((now_ns, self._distance_robot_to_goal(ns, goal)))

        cutoff_ns = now_ns - int(self.progress_window_sec * 1e9)
        while len(samples) >= 2 and samples[0][0] < cutoff_ns:
            samples.popleft()

    def _progress_delta(self, ns: str) -> Optional[float]:
        samples = self.goal_progress_samples[ns]
        if len(samples) < 2:
            return None
        span_ns = samples[-1][0] - samples[0][0]
        if span_ns < int(0.5 * self.progress_window_sec * 1e9):
            return None
        return samples[0][1] - samples[-1][1]

    def _set_active_goal(self, ns: str, goal: tuple[float, float], now_ns: int) -> None:
        prev = self.last_goal.get(ns)
        if prev is None or math.hypot(prev[0] - goal[0], prev[1] - goal[1]) > 1e-6:
            self.last_goal_set_time_ns[ns] = now_ns
            self.goal_progress_samples[ns].clear()
            self.goal_lock_start_xy[ns] = self._robot_xy(ns) if ns in self.odoms else None
        elif self.goal_lock_start_xy.get(ns) is None and ns in self.odoms:
            self.goal_lock_start_xy[ns] = self._robot_xy(ns)
        self.last_goal[ns] = goal

    def _set_policy_reason(self, ns: str, reason: str) -> None:
        self.last_policy_reason[ns] = reason

    def _apply_switch_hysteresis(self, ns: str, goal: tuple[float, float], assignment_score: float) -> tuple[float, float]:
        last = self.last_goal.get(ns)
        if last is None:
            self._set_policy_reason(ns, "switch/no_previous_goal")
            return goal

        # Never hold a blacklisted goal — force switch to the new candidate.
        now_ns = self.get_clock().now().nanoseconds
        if self._is_blacklisted(ns, last, now_ns):
            self._set_policy_reason(ns, "switch/held_goal_blacklisted")
            return goal

        od = self.odoms[ns]
        rx = float(od.pose.pose.position.x)
        ry = float(od.pose.pose.position.y)
        dist_to_last = math.hypot(last[0] - rx, last[1] - ry)
        move = math.hypot(goal[0] - last[0], goal[1] - last[1])

        # Only apply hold logic while still traveling to the previous goal.
        if dist_to_last > self.switch_min_dist:
            if move < self.switch_min_dist:
                self._set_policy_reason(ns, "hold/hysteresis_small_move")
                return last
            if assignment_score < self.switch_hysteresis:
                self._set_policy_reason(ns, "hold/hysteresis_low_score")
                return last
        self._set_policy_reason(ns, "switch/hysteresis_ok")
        return goal

    def _apply_goal_policy(
        self,
        ns: str,
        candidate_goal: tuple[float, float],
        assignment_score: float,
        map_msg: OccupancyGrid,
        dist_map: dict[int, int],
        now_ns: int,
    ) -> tuple[float, float]:
        # Use committed-style policy for ALL modes (progress-based hold,
        # goal_lock, stall detection).  The weaker switch_hysteresis was
        # allowing premature goal switches mid-navigation.

        last = self.last_goal.get(ns)
        if last is None:
            self._set_policy_reason(ns, "switch/no_previous_goal")
            return candidate_goal

        if self._is_blacklisted(ns, candidate_goal, now_ns):
            self._set_policy_reason(ns, "hold/candidate_blacklisted")
            return last

        dist_to_last = self._distance_robot_to_goal(ns, last)
        reached_last = dist_to_last <= self.switch_min_dist
        last_reachable = self._goal_reachable(map_msg, dist_map, last)
        hard_failure = not last_reachable

        last_set_ns = self.last_goal_set_time_ns.get(ns, 0)
        lock_active = (
            self.goal_lock_sec > 0.0
            and last_set_ns > 0
            and (now_ns - last_set_ns) < int(self.goal_lock_sec * 1e9)
        )

        if lock_active and not hard_failure and not reached_last:
            self._set_policy_reason(ns, "hold/goal_lock_active")
            return last

        delta = self._progress_delta(ns)
        stalled = delta is not None and delta < self.progress_min_delta_m

        candidate_move = math.hypot(candidate_goal[0] - last[0], candidate_goal[1] - last[1])
        if candidate_move < self.switch_min_dist:
            self._set_policy_reason(ns, "hold/small_candidate_move")
            return last

        if not reached_last and not hard_failure:
            # Keep commitment while making sufficient progress.
            if not stalled:
                self._set_policy_reason(ns, "hold/progressing")
                return last
            # Only switch on weak assignment scores when already stalled.
            if assignment_score < self.switch_hysteresis:
                self._set_policy_reason(ns, "hold/stalled_but_low_score")
                return last

        if not reached_last and (hard_failure or stalled):
            reason = "unreachable" if hard_failure else "stalled"
            self._register_goal_failure(ns, last, now_ns, reason)
            self._set_policy_reason(ns, f"switch/{reason}")
            return candidate_goal

        self._set_policy_reason(ns, "switch/reached_or_improved")
        return candidate_goal

    def _maybe_log_summary(
        self,
        targets_total: int,
        per_ns_frontiers: dict[str, int],
        per_ns_reachable: dict[str, int],
        per_ns_assigned: dict[str, tuple[float, float]],
        per_ns_utilities: dict[str, dict[tuple[float, float], float]] | None = None,
    ) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_summary_ns == 0:
            self._last_summary_ns = now_ns
            return
        if (now_ns - self._last_summary_ns) < int(self._summary_interval_sec * 1e9):
            return
        self._last_summary_ns = now_ns

        parts = []
        for ns in self.namespaces:
            goal = per_ns_assigned.get(ns)
            goal_txt = "None" if goal is None else f"({goal[0]:.2f},{goal[1]:.2f})"
            dist_txt = "-"
            age_txt = "-"
            if goal is not None and ns in self.odoms:
                dist_txt = f"{self._distance_robot_to_goal(ns, goal):.2f}"
            set_ns = self.last_goal_set_time_ns.get(ns, 0)
            if set_ns > 0:
                age_txt = f"{max(0.0, (now_ns - set_ns) / 1e9):.1f}"
            policy = self.last_policy_reason.get(ns, "-")
            # Show assigned goal's utility if available
            util_txt = "-"
            top_txt = ""
            if per_ns_utilities and ns in per_ns_utilities:
                ns_utils = per_ns_utilities[ns]
                if goal is not None and goal in ns_utils:
                    util_txt = f"{ns_utils[goal]:.2f}"
                # Show top-3 candidates
                sorted_goals = sorted(ns_utils.items(), key=lambda kv: kv[1], reverse=True)[:3]
                if sorted_goals:
                    top_parts = [f"({g[0]:.1f},{g[1]:.1f})={s:.2f}" for g, s in sorted_goals]
                    top_txt = f" top3=[{' '.join(top_parts)}]"
            speed_txt = ""
            vx, vy = self.odom_velocity_xy.get(ns, (0.0, 0.0))
            spd = math.hypot(float(vx), float(vy))
            if spd > 0.03:
                speed_txt = f" spd={spd:.2f}"
            parts.append(
                f"{ns}: fronts={per_ns_frontiers.get(ns, 0)} "
                f"reach={per_ns_reachable.get(ns, 0)} "
                f"goal={goal_txt} d={dist_txt} u={util_txt} "
                f"age={age_txt}s{speed_txt} [{policy}]{top_txt}"
            )
        self.get_logger().info(
            f"ASSIGN [{self.algorithm_mode}] targets={targets_total}\n  " + "\n  ".join(parts)
        )

    def _map_cell_stats(self, msg: OccupancyGrid) -> tuple[int, int, int]:
        free_n = 0
        occ_n = 0
        unknown_n = 0
        for value in msg.data:
            v = int(value)
            if v == self.unknown_value:
                unknown_n += 1
            elif v >= self.occ_thresh:
                occ_n += 1
            elif v == self.free_value:
                free_n += 1
            else:
                unknown_n += 1
        return (free_n, occ_n, unknown_n)

    def _should_log_no_goal_debug(self, now_ns: int) -> bool:
        if not self.debug_no_goal_logging:
            return False
        if (now_ns - self._last_no_goal_debug_ns) < int(self.debug_no_goal_log_interval_sec * 1e9):
            return False
        self._last_no_goal_debug_ns = now_ns
        return True

    def _log_no_goal_debug(
        self,
        *,
        now_ns: int,
        reason: str,
        planning_map: OccupancyGrid,
        per_ns_targets: dict[str, list[tuple[float, float]]],
        dist_maps: Optional[dict[str, dict[int, int]]] = None,
        utilities_sizes: Optional[dict[str, int]] = None,
        candidate_goals: Optional[dict[str, tuple[float, float]]] = None,
        per_ns_assigned: Optional[dict[str, tuple[float, float]]] = None,
    ) -> None:
        if not self._should_log_no_goal_debug(now_ns):
            return

        p_free, p_occ, p_unk = self._map_cell_stats(planning_map)
        parts = [
            f"NO_GOAL[{reason}]",
            f"planning_map(free={p_free} occ={p_occ} unk={p_unk})",
            f"use_shared_map={self.use_shared_map}",
            f"shared_map_ready={self.shared_map is not None}",
        ]
        for ns in self.namespaces:
            frontier_n = len(per_ns_targets.get(ns, []))
            parts.append(f"{ns}: fronts={frontier_n}")
        self.get_logger().warn(
            f"NO_GOAL [{reason}] map(free={p_free} occ={p_occ} unk={p_unk}) | " + " | ".join(parts)
        )

    @staticmethod
    def _percentile(sorted_values: list[float], quantile: float) -> float:
        if not sorted_values:
            return 0.0
        if len(sorted_values) == 1:
            return sorted_values[0]
        q = min(1.0, max(0.0, quantile))
        idx = q * float(len(sorted_values) - 1)
        lo = int(math.floor(idx))
        hi = int(math.ceil(idx))
        if lo == hi:
            return sorted_values[lo]
        frac = idx - lo
        return sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac

    def _maybe_log_perf_summary(self) -> None:
        if not self.perf_enable:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self._last_perf_summary_ns == 0:
            self._last_perf_summary_ns = now_ns
            self._perf_last_cpu_process_sec = time.process_time()
            self._perf_last_cpu_wall_ns = time.perf_counter_ns()
            return
        if (now_ns - self._last_perf_summary_ns) < int(self._summary_interval_sec * 1e9):
            return
        self._last_perf_summary_ns = now_ns

        sample_count = len(self._perf_tick_durations_ms)
        if sample_count < self.perf_min_samples:
            self.get_logger().info(
                f"PERF coordinator: collecting samples ({sample_count}/{self.perf_min_samples})"
            )
            return

        samples = list(self._perf_tick_durations_ms)
        sorted_samples = sorted(samples)
        p50_ms = self._percentile(sorted_samples, 0.50)
        p95_ms = self._percentile(sorted_samples, 0.95)
        mean_ms = sum(samples) / float(sample_count)
        max_ms = sorted_samples[-1]
        over_budget = sum(1 for val in samples if val > self._tick_period_ms)

        current_cpu_process_sec = time.process_time()
        current_cpu_wall_ns = time.perf_counter_ns()
        delta_cpu_sec = max(0.0, current_cpu_process_sec - self._perf_last_cpu_process_sec)
        delta_wall_sec = max(1e-6, (current_cpu_wall_ns - self._perf_last_cpu_wall_ns) / 1e9)
        cpu_pct = 100.0 * delta_cpu_sec / delta_wall_sec
        self._perf_last_cpu_process_sec = current_cpu_process_sec
        self._perf_last_cpu_wall_ns = current_cpu_wall_ns

        self.get_logger().info(
            "PERF coordinator: "
            f"tick_ms[p50={p50_ms:.1f} p95={p95_ms:.1f} mean={mean_ms:.1f} max={max_ms:.1f} "
            f"budget={self._tick_period_ms:.1f} over_budget={over_budget}/{sample_count}] "
            f"cpu={cpu_pct:.1f}% "
            f"adaptive[stride={self._adaptive_frontier_stride} "
            f"targets={self._adaptive_max_targets} "
            f"gain_r={self._adaptive_exploration_gain_radius_cells} "
            f"skip={self._adaptive_skip_ticks}]"
        )

        if self.perf_tick_warn_p95_ms > 0.0 and p95_ms > self.perf_tick_warn_p95_ms:
            self.get_logger().warn(
                "PERF threshold exceeded: "
                f"tick p95 {p95_ms:.1f}ms > {self.perf_tick_warn_p95_ms:.1f}ms"
            )
        if self.perf_cpu_warn_pct > 0.0 and cpu_pct > self.perf_cpu_warn_pct:
            self.get_logger().warn(
                "PERF threshold exceeded: "
                f"CPU {cpu_pct:.1f}% > {self.perf_cpu_warn_pct:.1f}% single-core budget"
            )

        self._update_adaptive_load_shedding(
            p95_ms=p95_ms,
            cpu_pct=cpu_pct,
            over_budget=over_budget,
            sample_count=sample_count,
        )

    def _update_adaptive_load_shedding(
        self,
        *,
        p95_ms: float,
        cpu_pct: float,
        over_budget: int,
        sample_count: int,
    ) -> None:
        if not self.adaptive_load_shedding_enabled:
            return

        budget_ms = max(1.0, self._tick_period_ms)
        over_budget_ratio = float(over_budget) / max(1, sample_count)
        cpu_budget_pct = self.perf_cpu_warn_pct if self.perf_cpu_warn_pct > 0.0 else 100.0
        p95_util = p95_ms / budget_ms
        cpu_util = cpu_pct / max(1.0, cpu_budget_pct)
        overloaded = (
            p95_util > self.adaptive_budget_utilization
            or over_budget_ratio > 0.35
            or cpu_util > 1.0
        )
        healthy = (
            p95_util < self.adaptive_restore_utilization
            and over_budget_ratio < 0.10
            and cpu_util < 0.8
        )

        changed = False
        if overloaded:
            if self._adaptive_frontier_stride < self.adaptive_max_frontier_stride:
                self._adaptive_frontier_stride += 1
                changed = True
            if self._adaptive_max_targets > self.adaptive_min_max_targets:
                reduced_targets = max(
                    self.adaptive_min_max_targets,
                    int(math.floor(self._adaptive_max_targets * 0.75)),
                )
                if reduced_targets != self._adaptive_max_targets:
                    self._adaptive_max_targets = reduced_targets
                    changed = True
            if self._adaptive_exploration_gain_radius_cells > self.adaptive_min_exploration_gain_radius_cells:
                self._adaptive_exploration_gain_radius_cells -= 1
                changed = True
            if self._adaptive_skip_ticks < self.adaptive_max_skip_ticks:
                self._adaptive_skip_ticks += 1
                changed = True
            if changed:
                self.get_logger().warn(
                    "Adaptive CFPA2 load shedding increased: "
                    f"stride={self._adaptive_frontier_stride} "
                    f"targets={self._adaptive_max_targets} "
                    f"gain_r={self._adaptive_exploration_gain_radius_cells} "
                    f"skip={self._adaptive_skip_ticks} "
                    f"(p95={p95_ms:.1f}ms budget={budget_ms:.1f}ms cpu={cpu_pct:.1f}%)"
                )
            return

        if healthy:
            if self._adaptive_skip_ticks > 0:
                self._adaptive_skip_ticks -= 1
                changed = True
            elif self._adaptive_exploration_gain_radius_cells < self.exploration_gain_radius_cells:
                self._adaptive_exploration_gain_radius_cells += 1
                changed = True
            elif self._adaptive_max_targets < self.max_targets:
                self._adaptive_max_targets = min(
                    self.max_targets,
                    int(math.ceil(self._adaptive_max_targets / 0.75)),
                )
                changed = True
            elif self._adaptive_frontier_stride > self.frontier_stride:
                self._adaptive_frontier_stride -= 1
                changed = True
            if changed:
                self.get_logger().info(
                    "Adaptive CFPA2 load shedding relaxed: "
                    f"stride={self._adaptive_frontier_stride} "
                    f"targets={self._adaptive_max_targets} "
                    f"gain_r={self._adaptive_exploration_gain_radius_cells} "
                    f"skip={self._adaptive_skip_ticks}"
                )

    def _record_tick_perf(self, tick_start_ns: int) -> None:
        if not self.perf_enable:
            return
        elapsed_ms = max(0.0, (time.perf_counter_ns() - tick_start_ns) / 1e6)
        self._perf_tick_durations_ms.append(elapsed_ms)
        self._maybe_log_perf_summary()

    def _tick(self) -> None:
        tick_start_ns = time.perf_counter_ns()
        skipped = False
        try:
            if self._adaptive_skip_ticks > 0:
                if self._adaptive_tick_skip_counter < self._adaptive_skip_ticks:
                    self._adaptive_tick_skip_counter += 1
                    skipped = True
                    return
                self._adaptive_tick_skip_counter = 0
            self._tick_impl()
        finally:
            if not skipped:
                self._record_tick_perf(tick_start_ns)

    def _publish_goal(self, ns: str, map_msg: OccupancyGrid, goal_w: tuple[float, float]) -> None:
        if not self._goal_is_finite(goal_w):
            self.get_logger().warn(f"{ns}: dropping non-finite goal {goal_w}")
            return
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = map_msg.header.frame_id or "world"
        msg.point.x = goal_w[0]
        msg.point.y = goal_w[1]
        msg.point.z = 0.0
        if self.output_mode == "exact_split":
            policy_reason = self.last_policy_reason.get(ns, "")
            # Relocation targets are currently tied to pursuit decisions.
            if "pursuit" in policy_reason:
                self.relocation_goal_pubs[ns].publish(msg)
            else:
                self.tare_goal_pubs[ns].publish(msg)
        else:
            self.goal_pubs[ns].publish(msg)
        self._publish_goal_marker(ns=ns, frame_id=msg.header.frame_id, goal_w=goal_w)

    def _publish_goal_marker(self, ns: str, frame_id: str, goal_w: tuple[float, float]) -> None:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.marker_frame_override or frame_id or "world"
        marker.ns = "mtare_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(goal_w[0])
        marker.pose.position.y = float(goal_w[1])
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        color = self._ns_color(ns)
        marker.color.a = 0.95
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        self.goal_marker_pubs[ns].publish(marker)

    @staticmethod
    def _goal_is_finite(goal_w: tuple[float, float]) -> bool:
        return math.isfinite(float(goal_w[0])) and math.isfinite(float(goal_w[1]))

    def _ns_color(self, ns: str) -> tuple[float, float, float]:
        idx = self.namespaces.index(ns) if ns in self.namespaces else 0
        if idx == 0:
            return (1.0, 0.15, 0.15)
        if idx == 1:
            return (0.15, 1.0, 0.15)
        if idx % 2 == 0:
            return (0.2, 0.6, 1.0)
        return (1.0, 0.8, 0.2)

    def _append_trajectory(self, ns: str, odom_msg: Odometry) -> None:
        x = float(odom_msg.pose.pose.position.x)
        y = float(odom_msg.pose.pose.position.y)
        hist = self.trajectory_history[ns]
        if not hist:
            hist.append((x, y))
            return
        px, py = hist[-1]
        if math.hypot(x - px, y - py) >= self.trajectory_min_point_distance:
            hist.append((x, y))

    def _publish_coordinator_map(self, target_map: OccupancyGrid) -> None:
        self.coordinator_map_pub.publish(target_map)

    def _publish_robot_markers(self, target_map: OccupancyGrid) -> None:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        frame_id = self.marker_frame_override or target_map.header.frame_id or "world"

        for idx, ns in enumerate(self.namespaces):
            color = self._ns_color(ns)
            pose_id = 100 + idx
            traj_id = 200 + idx
            goal_id = 300 + idx
            label_id = 400 + idx

            od = self.odoms.get(ns)
            if od is not None:
                pose_marker = Marker()
                pose_marker.header.stamp = stamp
                pose_marker.header.frame_id = frame_id
                pose_marker.ns = "mtare_robot_pose"
                pose_marker.id = pose_id
                pose_marker.type = Marker.TRIANGLE_LIST
                pose_marker.action = Marker.ADD
                pose_marker.pose.position.x = float(od.pose.pose.position.x)
                pose_marker.pose.position.y = float(od.pose.pose.position.y)
                pose_marker.pose.position.z = float(od.pose.pose.position.z) + 0.02
                pose_marker.pose.orientation = od.pose.pose.orientation
                pose_marker.scale.x = 1.0
                pose_marker.scale.y = 1.0
                pose_marker.scale.z = 1.0
                pose_marker.color.a = 0.95
                pose_marker.color.r = color[0]
                pose_marker.color.g = color[1]
                pose_marker.color.b = color[2]
                front = Point()
                front.x = 0.70 * self.robot_marker_scale
                front.y = 0.0
                front.z = 0.03
                rear_left = Point()
                rear_left.x = -0.45 * self.robot_marker_scale
                rear_left.y = 0.45 * self.robot_marker_scale
                rear_left.z = 0.03
                rear_right = Point()
                rear_right.x = -0.45 * self.robot_marker_scale
                rear_right.y = -0.45 * self.robot_marker_scale
                rear_right.z = 0.03
                pose_marker.points = [front, rear_left, rear_right]
                markers.markers.append(pose_marker)

                label_marker = Marker()
                label_marker.header.stamp = stamp
                label_marker.header.frame_id = frame_id
                label_marker.ns = "mtare_robot_label"
                label_marker.id = label_id
                label_marker.type = Marker.TEXT_VIEW_FACING
                label_marker.action = Marker.ADD
                label_marker.pose.position.x = float(od.pose.pose.position.x)
                label_marker.pose.position.y = float(od.pose.pose.position.y)
                label_marker.pose.position.z = float(od.pose.pose.position.z) + 0.5
                label_marker.pose.orientation.w = 1.0
                label_marker.scale.z = 0.32
                label_marker.color.a = 1.0
                label_marker.color.r = color[0]
                label_marker.color.g = color[1]
                label_marker.color.b = color[2]
                label_marker.text = ns
                markers.markers.append(label_marker)

            traj_marker = Marker()
            traj_marker.header.stamp = stamp
            traj_marker.header.frame_id = frame_id
            traj_marker.ns = "mtare_robot_traj"
            traj_marker.id = traj_id
            traj_marker.type = Marker.LINE_STRIP
            traj_marker.action = Marker.ADD
            traj_marker.pose.orientation.w = 1.0
            traj_marker.scale.x = 0.08
            traj_marker.color.a = 0.95
            traj_marker.color.r = color[0]
            traj_marker.color.g = color[1]
            traj_marker.color.b = color[2]
            for x, y in self.trajectory_history[ns]:
                pt = Point()
                pt.x = float(x)
                pt.y = float(y)
                pt.z = 0.05
                traj_marker.points.append(pt)
            markers.markers.append(traj_marker)

            goal = self.last_goal.get(ns)
            if goal is not None:
                goal_marker = Marker()
                goal_marker.header.stamp = stamp
                goal_marker.header.frame_id = frame_id
                goal_marker.ns = "mtare_goal_points"
                goal_marker.id = goal_id
                goal_marker.type = Marker.SPHERE
                goal_marker.action = Marker.ADD
                goal_marker.pose.position.x = float(goal[0])
                goal_marker.pose.position.y = float(goal[1])
                goal_marker.pose.position.z = 0.08
                goal_marker.pose.orientation.w = 1.0
                goal_marker.scale.x = 0.24
                goal_marker.scale.y = 0.24
                goal_marker.scale.z = 0.24
                goal_marker.color.a = 0.95
                goal_marker.color.r = color[0]
                goal_marker.color.g = color[1]
                goal_marker.color.b = color[2]
                markers.markers.append(goal_marker)

        self.robot_markers_pub.publish(markers)

    def _robot_xy(self, ns: str) -> tuple[float, float]:
        od = self.odoms[ns]
        return (float(od.pose.pose.position.x), float(od.pose.pose.position.y))

    def _predict_teammate_xy(self, ns: str, now_ns: int) -> tuple[float, float]:
        rx, ry = self._robot_xy(ns)
        vx, vy = self.odom_velocity_xy.get(ns, (0.0, 0.0))
        dt = self.prediction_horizon_sec
        if ns in self.odom_rx_time_ns:
            age_sec = max(0.0, (now_ns - self.odom_rx_time_ns[ns]) / 1e9)
            dt += age_sec
        return (rx + vx * dt, ry + vy * dt)

    def _frontier_information_gain(self, msg: OccupancyGrid, goal: tuple[float, float]) -> float:
        g = self._world_to_grid(msg, goal[0], goal[1])
        if g is None:
            return 0.0
        gx, gy = g
        w = int(msg.info.width)
        h = int(msg.info.height)
        data = msg.data
        r = self._adaptive_exploration_gain_radius_cells
        gain = 0.0
        for yy in range(max(0, gy - r), min(h, gy + r + 1)):
            row = yy * w
            for xx in range(max(0, gx - r), min(w, gx + r + 1)):
                idx = row + xx
                if data[idx] == self.unknown_value:
                    gain += 1.0
        return gain

    def _batch_frontier_information_gain(
        self, msg: OccupancyGrid, goals: list[tuple[float, float]]
    ) -> list[float]:
        """Batch info-gain for all goals at once (C++ accelerated)."""
        if not goals:
            return []
        w = int(msg.info.width)
        h = int(msg.info.height)
        r = self._adaptive_exploration_gain_radius_cells
        n = len(goals)

        if _GRID_OPS_LIB is not None:
            grid_np = np.array(msg.data, dtype=np.int8)
            ox = float(msg.info.origin.position.x)
            oy = float(msg.info.origin.position.y)
            res = float(msg.info.resolution)
            gx_arr = (ctypes.c_float * n)(*(g[0] for g in goals))
            gy_arr = (ctypes.c_float * n)(*(g[1] for g in goals))
            gains_out = (ctypes.c_float * n)()
            _GRID_OPS_LIB.batch_info_gain(
                grid_np.ctypes.data_as(ctypes.POINTER(ctypes.c_int8)),
                w, h, res, ox, oy,
                gx_arr, gy_arr, n, r,
                ctypes.c_int8(self.unknown_value),
                gains_out,
            )
            return [float(gains_out[i]) for i in range(n)]

        # Python fallback
        return [self._frontier_information_gain(msg, g) for g in goals]

    def _grid_path_cost_m(
        self,
        msg: OccupancyGrid,
        dist_map: dict[int, int],
        goal: tuple[float, float],
    ) -> Optional[float]:
        g = self._world_to_grid(msg, goal[0], goal[1])
        if g is None:
            return None
        idx = self._grid_index(g[0], g[1], int(msg.info.width))
        if idx not in dist_map:
            return None
        return float(dist_map[idx]) * msg.info.resolution

    def _cfpa2_switch_penalty(self, ns: str, goal: tuple[float, float]) -> float:
        last = self.last_goal.get(ns)
        if last is None:
            return 0.0
        return 0.0 if self._goal_key(last) == self._goal_key(goal) else 1.0

    def _cfpa2_single_utility(
        self,
        *,
        ns: str,
        goal: tuple[float, float],
        map_msg: OccupancyGrid,
        dist_map: dict[int, int],
    ) -> float:
        dist_m = self._grid_path_cost_m(map_msg, dist_map, goal)
        if dist_m is None or dist_m <= 0.0:
            return -1e18
        info_gain = self._frontier_information_gain(map_msg, goal)
        # Reject frontiers with negligible info-gain (tiny slivers near walls)
        if info_gain < 3:
            return -1e18
        switch_penalty = self._cfpa2_switch_penalty(ns, goal)
        momentum_bonus = self._cfpa2_momentum_bonus(ns, goal)
        return (
            (self.cfpa2_w_ig * info_gain)
            - (self.cfpa2_w_c * dist_m)
            - (self.cfpa2_w_sw * switch_penalty)
            + (self.cfpa2_w_momentum * momentum_bonus)
        )

    def _cfpa2_overlap_penalty(self, goal_i: tuple[float, float], goal_j: tuple[float, float]) -> float:
        sigma = self.cfpa2_sigma_overlap_m if self.cfpa2_sigma_overlap_m > 0.0 else (2.0 * self.sensor_range)
        sigma = max(1e-3, sigma)
        if _CFPA2_OVERLAP_PENALTY_FN is not None:
            try:
                return float(_CFPA2_OVERLAP_PENALTY_FN(goal_i, goal_j, sigma))
            except Exception:
                pass

        dx = goal_i[0] - goal_j[0]
        dy = goal_i[1] - goal_j[1]
        d2 = (dx * dx) + (dy * dy)
        return math.exp(-d2 / (2.0 * sigma * sigma))

    def _cfpa2_momentum(self, ns: str) -> float:
        vx, vy = self.odom_velocity_xy.get(ns, (0.0, 0.0))
        return math.hypot(float(vx), float(vy))

    def _cfpa2_momentum_bonus(self, ns: str, goal: tuple[float, float]) -> float:
        """Heading + velocity momentum bonus.

        bonus = cos(heading_to_frontier) × (α + β × speed)

        α (base, 0.5): heading-only component — even when stopped, frontiers
          behind the robot get penalized.  This prevents backtracking at
          waypoint stops.
        β (velocity scale, 1.0): scales up the bonus when the robot is moving,
          making it very expensive to switch direction mid-stride.
        """
        odom = self.odoms.get(ns)
        if odom is None:
            return 0.0
        rx = float(odom.pose.pose.position.x)
        ry = float(odom.pose.pose.position.y)
        dx = goal[0] - rx
        dy = goal[1] - ry
        d = math.hypot(dx, dy)
        if d < 0.1:
            return 0.0

        # Extract yaw from quaternion
        q = odom.pose.pose.orientation
        siny = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
        cosy = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
        yaw = math.atan2(siny, cosy)

        # cos(angle between heading and frontier direction)
        cos_angle = (math.cos(yaw) * dx + math.sin(yaw) * dy) / d

        # Velocity boost
        vx, vy = self.odom_velocity_xy.get(ns, (0.0, 0.0))
        speed = math.hypot(float(vx), float(vy))

        alpha = 0.5   # base heading weight (always active)
        beta = 1.0    # velocity scale
        return cos_angle * (alpha + beta * speed)

    def _find_nearest_free_cell(
        self,
        map_msg: OccupancyGrid,
        cell: tuple[int, int],
        search_radius_cells: int,
    ) -> Optional[tuple[int, int]]:
        w = int(map_msg.info.width)
        h = int(map_msg.info.height)
        data = map_msg.data
        gx, gy = cell
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            return None
        if self._is_free(data, self._grid_index(gx, gy, w)):
            return (gx, gy)

        best: Optional[tuple[int, int]] = None
        best_d2 = float("inf")
        for r in range(1, search_radius_cells + 1):
            for dy in range(-r, r + 1):
                ny = gy + dy
                if ny < 0 or ny >= h:
                    continue
                for dx in range(-r, r + 1):
                    nx = gx + dx
                    if nx < 0 or nx >= w:
                        continue
                    nidx = self._grid_index(nx, ny, w)
                    if not self._is_free(data, nidx):
                        continue
                    d2 = (nx - gx) * (nx - gx) + (ny - gy) * (ny - gy)
                    if d2 < best_d2:
                        best_d2 = d2
                        best = (nx, ny)
            if best is not None:
                break
        return best

    @staticmethod
    def _dilate_grid_cell(
        gx: int,
        gy: int,
        radius_cells: int,
        w: int,
        h: int,
    ) -> list[tuple[int, int]]:
        if radius_cells <= 0:
            return [(gx, gy)]
        out: list[tuple[int, int]] = []
        r2 = radius_cells * radius_cells
        for dy in range(-radius_cells, radius_cells + 1):
            ny = gy + dy
            if ny < 0 or ny >= h:
                continue
            for dx in range(-radius_cells, radius_cells + 1):
                nx = gx + dx
                if nx < 0 or nx >= w:
                    continue
                if (dx * dx + dy * dy) > r2:
                    continue
                out.append((nx, ny))
        return out

    def _predict_other_robot_blocks(
        self,
        *,
        map_msg: OccupancyGrid,
        ns: str,
        planned_goals: dict[str, tuple[float, float]],
        steps: int,
        dt_sec: float,
        safety_radius_cells: int,
    ) -> list[set[tuple[int, int]]]:
        blocked_by_t: list[set[tuple[int, int]]] = [set() for _ in range(steps + 1)]
        w = int(map_msg.info.width)
        h = int(map_msg.info.height)

        for other in self.namespaces:
            if other == ns or other not in self.odoms:
                continue

            ox, oy = self._robot_xy(other)
            target = planned_goals.get(other) or self.last_goal.get(other) or (ox, oy)
            tx, ty = float(target[0]), float(target[1])

            dx = tx - ox
            dy = ty - oy
            dist = math.hypot(dx, dy)
            dir_x = (dx / dist) if dist > 1e-6 else 0.0
            dir_y = (dy / dist) if dist > 1e-6 else 0.0

            raw_speed = math.hypot(*self.odom_velocity_xy.get(other, (0.0, 0.0)))
            speed = raw_speed if raw_speed > 0.05 else self.cfpa2_space_time_assumed_speed_mps
            speed = max(self.cfpa2_space_time_assumed_speed_mps, min(speed, self.cfpa2_space_time_max_speed_mps))

            for k in range(steps + 1):
                t = dt_sec * k
                travel = min(dist, speed * t)
                px = ox + dir_x * travel
                py = oy + dir_y * travel
                g = self._world_to_grid(map_msg, px, py)
                if g is None:
                    continue
                for cell in self._dilate_grid_cell(g[0], g[1], safety_radius_cells, w, h):
                    blocked_by_t[k].add(cell)

        return blocked_by_t

    def _space_time_astar_cells(
        self,
        *,
        ns: str,
        map_msg: OccupancyGrid,
        final_goal: tuple[float, float],
        planned_goals: dict[str, tuple[float, float]],
    ) -> Optional[list[tuple[int, int]]]:
        if ns not in self.odoms:
            return None

        res = max(1e-6, float(map_msg.info.resolution))
        w = int(map_msg.info.width)
        h = int(map_msg.info.height)
        data = map_msg.data

        start_w = self._robot_xy(ns)
        start = self._world_to_grid(map_msg, start_w[0], start_w[1])
        goal = self._world_to_grid(map_msg, final_goal[0], final_goal[1])
        if start is None or goal is None:
            return None

        near_search = max(4, int(math.ceil(0.8 / res)))
        start = self._find_nearest_free_cell(map_msg, start, near_search)
        goal = self._find_nearest_free_cell(map_msg, goal, near_search)
        if start is None or goal is None:
            return None

        steps = max(2, int(math.ceil(self.cfpa2_space_time_horizon_sec / self.cfpa2_space_time_dt_sec)))
        safety_cells = max(0, int(math.ceil(self.cfpa2_space_time_safety_radius_m / res)))
        blocked_by_t = self._predict_other_robot_blocks(
            map_msg=map_msg,
            ns=ns,
            planned_goals=planned_goals,
            steps=steps,
            dt_sec=self.cfpa2_space_time_dt_sec,
            safety_radius_cells=safety_cells,
        )

        margin_cells = max(
            4,
            int(
                math.ceil(
                    (self.cfpa2_space_time_window_margin_m + self.cfpa2_space_time_max_speed_mps * self.cfpa2_space_time_horizon_sec)
                    / res
                )
            ),
        )
        x_min = max(0, min(start[0], goal[0]) - margin_cells)
        x_max = min(w - 1, max(start[0], goal[0]) + margin_cells)
        y_min = max(0, min(start[1], goal[1]) - margin_cells)
        y_max = min(h - 1, max(start[1], goal[1]) + margin_cells)

        def in_window(x: int, y: int) -> bool:
            return x_min <= x <= x_max and y_min <= y <= y_max

        sx, sy = start
        gx, gy = goal
        if not in_window(sx, sy) or not in_window(gx, gy):
            return None

        neighbors = (
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
            (0, 0, 1.05),  # wait
        )

        start_key = (sx, sy, 0)
        open_heap: list[tuple[float, float, int, int, int]] = []
        heapq.heappush(open_heap, (abs(sx - gx) + abs(sy - gy), 0.0, sx, sy, 0))
        g_score: dict[tuple[int, int, int], float] = {start_key: 0.0}
        parent: dict[tuple[int, int, int], tuple[int, int, int]] = {}

        expansions = 0
        goal_key: Optional[tuple[int, int, int]] = None
        while open_heap and expansions < self.cfpa2_space_time_max_expansions:
            _f, cur_g, x, y, t = heapq.heappop(open_heap)
            key = (x, y, t)
            if cur_g > g_score.get(key, float("inf")) + 1e-9:
                continue
            expansions += 1

            if (x, y) == (gx, gy):
                goal_key = key
                break
            if t >= steps:
                continue

            nt = t + 1
            for dx, dy, move_cost in neighbors:
                nx = x + dx
                ny = y + dy
                if not in_window(nx, ny):
                    continue
                nidx = self._grid_index(nx, ny, w)
                if not self._is_free(data, nidx):
                    continue
                if (nx, ny) in blocked_by_t[nt]:
                    continue
                # Avoid swap-like crossing through a dynamic obstacle trajectory.
                if (nx, ny) in blocked_by_t[t] and (x, y) in blocked_by_t[nt]:
                    continue

                nkey = (nx, ny, nt)
                ng = cur_g + move_cost
                if ng >= g_score.get(nkey, float("inf")) - 1e-9:
                    continue
                g_score[nkey] = ng
                parent[nkey] = key
                h_manhattan = abs(nx - gx) + abs(ny - gy)
                heapq.heappush(open_heap, (ng + h_manhattan, ng, nx, ny, nt))

        if goal_key is None:
            return None

        path: list[tuple[int, int]] = []
        cur = goal_key
        while True:
            path.append((cur[0], cur[1]))
            if cur == start_key:
                break
            cur = parent[cur]
        path.reverse()
        return path

    def _cfpa2_space_time_waypoint(
        self,
        *,
        ns: str,
        map_msg: OccupancyGrid,
        final_goal: tuple[float, float],
        planned_goals: dict[str, tuple[float, float]],
    ) -> Optional[tuple[float, float]]:
        if not self.cfpa2_space_time_enabled or ns not in self.odoms:
            return None

        rx, ry = self._robot_xy(ns)
        if math.hypot(final_goal[0] - rx, final_goal[1] - ry) <= max(self.min_assign_distance, 0.20):
            return None

        path_cells = self._space_time_astar_cells(
            ns=ns,
            map_msg=map_msg,
            final_goal=final_goal,
            planned_goals=planned_goals,
        )
        if not path_cells or len(path_cells) < 2:
            return None

        res = float(map_msg.info.resolution)
        lookahead_m = max(res, self.cfpa2_space_time_waypoint_lookahead_m)
        traveled_m = 0.0
        chosen = path_cells[-1]
        prev = path_cells[0]
        for cell in path_cells[1:]:
            traveled_m += math.hypot(cell[0] - prev[0], cell[1] - prev[1]) * res
            prev = cell
            if traveled_m >= lookahead_m:
                chosen = cell
                break

        return self._grid_to_world(map_msg, chosen[0], chosen[1])

    def _cfpa2_best_available_goal(
        self,
        *,
        ns: str,
        now_ns: int,
        utilities: dict[tuple[float, float], float],
        exclude_goal: Optional[tuple[float, float]] = None,
        fallback_targets: Optional[list[tuple[float, float]]] = None,
    ) -> Optional[tuple[float, float]]:
        # TODO(maybe): Move fallback target prioritization into cfpa2_demo.core for reuse.
        excluded_key = self._goal_key(exclude_goal) if exclude_goal is not None else None

        for goal, _score in sorted(utilities.items(), key=lambda kv: kv[1], reverse=True):
            if excluded_key is not None and self._goal_key(goal) == excluded_key:
                continue
            if self._goal_too_close(ns, goal):
                continue
            if self._is_blacklisted(ns, goal, now_ns):
                continue
            return goal

        if fallback_targets:
            for goal in sorted(fallback_targets, key=lambda g: self._distance_robot_to_goal(ns, g)):
                if excluded_key is not None and self._goal_key(goal) == excluded_key:
                    continue
                if self._goal_too_close(ns, goal):
                    continue
                if self._is_blacklisted(ns, goal, now_ns):
                    continue
                return goal
        return None

    def _maybe_force_cfpa2_stuck_recovery(
        self,
        *,
        ns: str,
        now_ns: int,
        utilities: dict[tuple[float, float], float],
        fallback_targets: list[tuple[float, float]],
    ) -> Optional[tuple[float, float]]:
        if self.cfpa2_stuck_lock_sec <= 0.0 or self.cfpa2_stuck_blacklist_sec <= 0.0:
            return None
        if ns not in self.odoms:
            return None

        current_goal = self.last_goal.get(ns)
        if current_goal is None:
            return None
        if self._goal_too_close(ns, current_goal):
            return None

        lock_start_ns = self.last_goal_set_time_ns.get(ns, 0)
        if lock_start_ns <= 0:
            return None
        lock_age_sec = max(0.0, (now_ns - lock_start_ns) / 1e9)
        if lock_age_sec < self.cfpa2_stuck_lock_sec:
            return None

        lock_start_xy = self.goal_lock_start_xy.get(ns)
        if lock_start_xy is None:
            self.goal_lock_start_xy[ns] = self._robot_xy(ns)
            return None

        rx, ry = self._robot_xy(ns)
        moved_dist = math.hypot(rx - lock_start_xy[0], ry - lock_start_xy[1])
        if moved_dist >= self.cfpa2_stuck_min_motion_m:
            return None

        current_key = self._goal_key(current_goal)
        until_ns = now_ns + int(self.cfpa2_stuck_blacklist_sec * 1e9)
        self.goal_blacklist_until_ns[ns][current_key] = max(
            self.goal_blacklist_until_ns[ns].get(current_key, 0),
            until_ns,
        )

        alternative = self._cfpa2_best_available_goal(
            ns=ns,
            now_ns=now_ns,
            utilities=utilities,
            exclude_goal=current_goal,
            fallback_targets=fallback_targets,
        )

        last_event_ns = self.cfpa2_last_stuck_event_ns.get(ns, 0)
        if alternative is not None:
            if (now_ns - last_event_ns) > int(1e9):
                self.get_logger().warn(
                    f"{ns}: CFPA2 stuck-recovery triggered (lock={lock_age_sec:.1f}s, moved={moved_dist:.2f}m). "
                    f"Blacklisted ({current_goal[0]:.2f},{current_goal[1]:.2f}) for {self.cfpa2_stuck_blacklist_sec:.1f}s "
                    f"and switching to ({alternative[0]:.2f},{alternative[1]:.2f})."
                )
            self.cfpa2_last_stuck_event_ns[ns] = now_ns
            self._set_policy_reason(ns, "switch/cfpa2_stuck_recover")
            return alternative

        if (now_ns - last_event_ns) > int(2e9):
            self.get_logger().warn(
                f"{ns}: CFPA2 stuck-recovery blacklisted current goal but found no alternative frontier."
            )
            self.cfpa2_last_stuck_event_ns[ns] = now_ns
        self._set_policy_reason(ns, "hold/cfpa2_stuck_no_alternative")
        return None

    def _apply_cfpa2_proximity_stop(
        self,
        *,
        candidate_goals: dict[str, tuple[float, float]],
        assignment_scores: dict[str, float],
        now_ns: int,
    ) -> set[str]:
        if self.cfpa2_close_stop_radius_m <= 0.0 or len(self.namespaces) != 2:
            return set()

        ns_a, ns_b = self.namespaces[0], self.namespaces[1]
        if ns_a not in self.odoms or ns_b not in self.odoms:
            return set()

        ax, ay = self._robot_xy(ns_a)
        bx, by = self._robot_xy(ns_b)
        if math.hypot(ax - bx, ay - by) >= self.cfpa2_close_stop_radius_m:
            return set()

        p_a = self._cfpa2_momentum(ns_a)
        p_b = self._cfpa2_momentum(ns_b)
        eps = self.cfpa2_close_stop_speed_epsilon
        if p_a < (p_b - eps):
            stop_ns = ns_a
        elif p_b < (p_a - eps):
            stop_ns = ns_b
        else:
            # Deterministic tie-break keeps one robot moving.
            stop_ns = ns_b

        sx, sy = self._robot_xy(stop_ns)
        candidate_goals[stop_ns] = (sx, sy)
        assignment_scores[stop_ns] = -1e6
        self._set_policy_reason(stop_ns, "hold/cfpa2_close_low_momentum_stop")

        if (now_ns - self._cfpa2_last_close_stop_log_ns) > int(1e9):
            other_ns = ns_b if stop_ns == ns_a else ns_a
            self.get_logger().warn(
                f"CFPA2 close-range arbitration: d<{self.cfpa2_close_stop_radius_m:.2f}m, "
                f"{stop_ns} momentum={self._cfpa2_momentum(stop_ns):.3f} < "
                f"{other_ns} momentum={self._cfpa2_momentum(other_ns):.3f}; forcing {stop_ns} stop."
            )
            self._cfpa2_last_close_stop_log_ns = now_ns
        return {stop_ns}

    def _exploration_utility(
        self,
        *,
        ns: str,
        goal: tuple[float, float],
        map_msg: OccupancyGrid,
        dist_maps: dict[str, dict[int, int]],
        assigned_goals: dict[str, tuple[float, float]],
    ) -> tuple[float, Optional[float]]:
        self_dist = self._grid_path_cost_m(map_msg, dist_maps.get(ns, {}), goal)
        if self_dist is None or self_dist <= 0.0:
            return (-1e18, None)

        info_gain = self._frontier_information_gain(map_msg, goal)
        base_score = info_gain / max(self_dist, 0.1)

        overlap_penalty = 0.0
        for other in self.namespaces:
            if other == ns:
                continue
            other_dist = self._grid_path_cost_m(map_msg, dist_maps.get(other, {}), goal)
            if other_dist is None or other_dist <= 0.0:
                continue
            if other_dist < self_dist:
                overlap_penalty += self.overlap_weight / max(other_dist, 0.25)

            assigned = assigned_goals.get(other)
            if assigned is not None:
                d_assigned = math.hypot(goal[0] - assigned[0], goal[1] - assigned[1])
                if d_assigned < self.sensor_range:
                    overlap_penalty += self.overlap_weight / max(d_assigned, 0.25)

        return (base_score - overlap_penalty, self_dist)

    def _best_pursuit_target(
        self,
        *,
        ns: str,
        now_ns: int,
    ) -> tuple[Optional[tuple[float, float]], float]:
        if self.communication_timeout_sec <= 0.0:
            return (None, -1e18)

        local_xy = self._robot_xy(ns)
        best_target: Optional[tuple[float, float]] = None
        best_utility = -1e18

        for other in self.namespaces:
            if other == ns:
                continue
            last_rx_ns = self.odom_rx_time_ns.get(other, 0)
            if last_rx_ns <= 0:
                continue

            stale_sec = max(0.0, (now_ns - last_rx_ns) / 1e9)
            if stale_sec < self.communication_timeout_sec:
                continue
            if self.teammate_stale_ttl_sec > 0.0 and stale_sec > self.teammate_stale_ttl_sec:
                continue

            predicted = self._predict_teammate_xy(other, now_ns)
            meeting_dist = math.hypot(predicted[0] - local_xy[0], predicted[1] - local_xy[1])
            if meeting_dist < self.meeting_min_distance:
                continue

            expected_overlap_reduction = max(1.0, stale_sec)
            utility = self.pursuit_weight * expected_overlap_reduction / max(meeting_dist, 0.25)
            if utility > best_utility:
                best_utility = utility
                best_target = predicted

        return (best_target, best_utility)

    def _mui_cell_key(self, point: tuple[float, float, float]) -> tuple[int, int, int]:
        q = self.mui_cell_merge_resolution_m
        qz = max(0.05, q)
        return (
            int(round(point[0] / q)),
            int(round(point[1] / q)),
            int(round(point[2] / qz)),
        )

    def _collect_mui_exploring_cells(
        self,
    ) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]], dict[str, int]]:
        merged: dict[tuple[int, int, int], tuple[float, float, float]] = {}
        per_ns_frontiers: dict[str, int] = {}

        for ns in self.namespaces:
            status = self.grid_world_status.get(ns)
            if status is None:
                per_ns_frontiers[ns] = 0
                continue
            count = 0
            for pt in status.exploring_cell_positions:
                x = float(pt.x)
                y = float(pt.y)
                z = float(pt.z)
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue
                key = self._mui_cell_key((x, y, z))
                merged.setdefault(key, (x, y, z))
                count += 1
            per_ns_frontiers[ns] = count

        keys = sorted(merged.keys())
        if len(keys) > self.mui_max_exploring_cells:
            robot_xy = [self._robot_xy(ns) for ns in self.namespaces]

            def _score(k: tuple[int, int, int]) -> float:
                x, y, _ = merged[k]
                return min(math.hypot(x - rx, y - ry) for rx, ry in robot_xy)

            keys = sorted(keys, key=_score)[: self.mui_max_exploring_cells]

        cells = [merged[k] for k in keys]
        return (cells, keys, per_ns_frontiers)

    def _build_mui_distance_matrix(
        self,
        map_msg: OccupancyGrid,
        exploring_cells: list[tuple[float, float, float]],
        robot_positions: list[tuple[float, float, float]],
    ) -> list[list[int]]:
        locations = list(exploring_cells) + list(robot_positions)
        if not locations:
            return []

        map_res = max(1e-3, float(map_msg.info.resolution))
        unreachable_cm = int(round(self.mui_unreachable_penalty_m * 100.0))
        xy_locations = [(loc[0], loc[1]) for loc in locations]
        dist_maps = [self._distance_transform(map_msg, (x, y)) for x, y in xy_locations]
        map_w = int(map_msg.info.width)

        matrix: list[list[int]] = []
        for i, (fx, fy) in enumerate(xy_locations):
            dist_map = dist_maps[i]
            row: list[int] = []
            for j, (tx, ty) in enumerate(xy_locations):
                if i == j:
                    row.append(0)
                    continue
                target_grid = self._world_to_grid(map_msg, tx, ty)
                if target_grid is not None:
                    tidx = self._grid_index(target_grid[0], target_grid[1], map_w)
                    if tidx in dist_map:
                        row.append(max(1, int(round(float(dist_map[tidx]) * map_res * 100.0))))
                        continue
                euclid_cm = int(round(math.hypot(tx - fx, ty - fy) * 100.0))
                row.append(max(1, euclid_cm + unreachable_cm))
            matrix.append(row)
        return matrix

    def _should_resolve_mui(self, now_ns: int, cell_keys: list[tuple[int, int, int]]) -> bool:
        if self._mui_last_solve_ns <= 0:
            return True
        elapsed_ns = now_ns - self._mui_last_solve_ns
        # Only re-solve on timer expiry or when a robot has reached its goal
        timer_expired = elapsed_ns >= int(self.mui_resolve_period_sec * 1e9)
        robot_reached_goal = False
        for ns in self.namespaces:
            goal = self.last_goal.get(ns)
            if goal is None:
                robot_reached_goal = True
                break
            if self._distance_robot_to_goal(ns, goal) <= self.switch_min_dist:
                robot_reached_goal = True
                break
        return timer_expired or robot_reached_goal

    def _tick_impl_mui_tare(
        self,
        *,
        now_ns: int,
        planning_map: OccupancyGrid,
    ) -> bool:
        missing_status = [ns for ns in self.namespaces if ns not in self.grid_world_status]
        if missing_status:
            if now_ns - self._last_prereq_warn_ns > int(2e9):
                self.get_logger().warn(
                    f"Waiting for grid status topics from: {missing_status}; "
                    "no MUI-TARE goals will be published yet."
                )
                self._last_prereq_warn_ns = now_ns
            return True

        exploring_cells, cell_keys, per_ns_frontiers = self._collect_mui_exploring_cells()

        if not exploring_cells:
            return True

        should_resolve = self._should_resolve_mui(now_ns, cell_keys)
        if should_resolve:
            robot_positions = []
            for ns in self.namespaces:
                rx, ry = self._robot_xy(ns)
                robot_positions.append((rx, ry, 0.0))
            distance_matrix = self._build_mui_distance_matrix(planning_map, exploring_cells, robot_positions)
            routes = solve_mdvrp(
                exploring_cell_positions=exploring_cells,
                robot_positions=robot_positions,
                distance_matrix=distance_matrix,
                time_limit_sec=self.mui_mdvrp_time_limit_sec,
            )
            if not routes:
                if now_ns - self._last_prereq_warn_ns > int(2e9):
                    self.get_logger().warn(
                        "MUI-TARE MDVRP solve failed; keeping previous MUI assignments."
                    )
                    self._last_prereq_warn_ns = now_ns
            else:
                self._mui_last_solve_ns = now_ns
                self._mui_last_cell_keys = set(cell_keys)
                all_cell_indices = set(range(len(exploring_cells)))
                for idx, ns in enumerate(self.namespaces):
                    own_route = routes.get(idx, [])
                    self._mui_routes[ns] = own_route
                    self._mui_cover_by_others[ns] = set(int(i) for i in (all_cell_indices - set(own_route)))

        robot_xy = {ns: self._robot_xy(ns) for ns in self.namespaces}
        indexed_routes = {i: self._mui_routes.get(ns, []) for i, ns in enumerate(self.namespaces)}
        candidate_goals = select_first_route_goals(
            namespaces=self.namespaces,
            routes=indexed_routes,
            exploring_cells=exploring_cells,
            robot_xy=robot_xy,
            min_assign_distance=self.min_assign_distance,
        )

        per_ns_assigned: dict[str, tuple[float, float]] = {}
        for ns in self.namespaces:
            candidate = candidate_goals.get(ns)
            if candidate is None:
                held = self.last_goal.get(ns)
                if held is None:
                    self._set_policy_reason(ns, "hold/mui_no_candidate")
                    continue
                self._set_policy_reason(ns, "hold/mui_keep_previous")
                goal = held
            else:
                self._set_policy_reason(ns, "switch/mui_tare_mdvrp")
                goal = self._apply_switch_hysteresis(ns, candidate, 1.0)

            self._set_active_goal(ns, goal, now_ns)
            publish_map = self.maps.get(ns, planning_map)
            self._publish_goal(ns, publish_map, goal)
            per_ns_assigned[ns] = goal

        per_ns_reachable = {ns: len(self._mui_routes.get(ns, [])) for ns in self.namespaces}
        self._maybe_log_summary(
            targets_total=len(exploring_cells),
            per_ns_frontiers=per_ns_frontiers,
            per_ns_reachable=per_ns_reachable,
            per_ns_assigned=per_ns_assigned,
        )
        return True

    def _tick_impl(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        using_shared_map = self.use_shared_map and self.shared_map is not None
        target_map: OccupancyGrid
        if self.use_shared_map:
            if using_shared_map:
                target_map = self.shared_map  # type: ignore[assignment]
                self._warned_missing_shared_map = False
                self._shared_map_fallback_active = False
            else:
                waited_sec = (now_ns - self._start_ns) / 1e9
                if waited_sec >= self.shared_map_wait_sec:
                    # Fail-open: continue coordinated assignment on local map inputs
                    # until shared map becomes available.
                    fallback_map = self._build_fallback_map()
                    if fallback_map is None:
                        if now_ns - self._last_prereq_warn_ns > int(2e9):
                            self.get_logger().warn(
                                "Shared-map fallback active but local maps are still unavailable; "
                                "waiting for /<ns>/map inputs."
                            )
                            self._last_prereq_warn_ns = now_ns
                        return
                    target_map = fallback_map
                    if not self._shared_map_fallback_active:
                        self.get_logger().warn(
                            f"Shared map not available on {self.shared_map_topic} after {waited_sec:.1f}s; "
                            "falling back to per-robot maps."
                        )
                        self._shared_map_fallback_active = True
                else:
                    if not self._warned_missing_shared_map:
                        self.get_logger().warn(
                            f"Waiting for shared map on {self.shared_map_topic}; "
                            f"fallback in {max(0.0, self.shared_map_wait_sec - waited_sec):.1f}s."
                        )
                        self._warned_missing_shared_map = True
                    return
        else:
            if any(ns not in self.maps for ns in self.namespaces):
                if now_ns - self._last_prereq_warn_ns > int(2e9):
                    missing_maps = [ns for ns in self.namespaces if ns not in self.maps]
                    self.get_logger().warn(
                        f"Waiting for map topics from: {missing_maps}; no M-TARE goals will be published yet."
                    )
                    self._last_prereq_warn_ns = now_ns
                return
            fallback_map = self._build_fallback_map()
            if fallback_map is None:
                return
            target_map = fallback_map

        planning_map = target_map
        if using_shared_map:
            planning_map = self._build_shared_with_local_patches(target_map)

        self._publish_coordinator_map(planning_map)
        self._publish_robot_markers(planning_map)

        missing_odoms = [ns for ns in self.namespaces if ns not in self.odoms]
        if missing_odoms:
            if now_ns - self._last_prereq_warn_ns > int(2e9):
                self.get_logger().warn(
                    f"Waiting for odom/nav from: {missing_odoms}; no M-TARE goals will be published yet."
                )
                self._last_prereq_warn_ns = now_ns
            return

        for ns in self.namespaces:
            self._prune_blacklist(ns, now_ns)
            self._update_reached_goal_blacklist(ns, now_ns)

        per_ns_targets: dict[str, list[tuple[float, float]]] = {}
        for ns in self.namespaces:
            map_msg = self.maps.get(ns)
            per_ns_targets[ns] = self._extract_frontiers(map_msg) if map_msg is not None else []

        if using_shared_map:
            targets = self._extract_frontiers(planning_map)
        else:
            merge_res = max(0.1, float(planning_map.info.resolution) * 2.0)
            targets = self._merge_targets([per_ns_targets[ns] for ns in self.namespaces], merge_res)

        if self.algorithm_mode == "mui_tare":
            self._tick_impl_mui_tare(
                now_ns=now_ns,
                planning_map=planning_map,
            )
            return

        if not targets:
            self._log_no_goal_debug(
                now_ns=now_ns,
                reason="no_frontiers_after_extract",
                planning_map=planning_map,
                per_ns_targets=per_ns_targets,
            )
            return

        dist_maps = {}
        for ns in self.namespaces:
            od = self.odoms[ns]
            # When shared map is unavailable we fail-open to per-robot maps.
            cost_map = planning_map if using_shared_map else self.maps.get(ns)
            if cost_map is None:
                dist_maps[ns] = {}
                continue
            dist_maps[ns] = self._distance_transform(cost_map, (od.pose.pose.position.x, od.pose.pose.position.y))
            if self.algorithm_mode == "committed":
                self._update_progress_samples(ns, now_ns)

        local_nav_forced_switch_namespaces = self._consume_local_nav_stall_blacklists(now_ns)

        if self.algorithm_mode == "cfpa2" and len(self.namespaces) == 2:
            ns_a, ns_b = self.namespaces[0], self.namespaces[1]
            map_a = planning_map if using_shared_map else self.maps.get(ns_a)
            map_b = planning_map if using_shared_map else self.maps.get(ns_b)

            utilities_a: dict[tuple[float, float], float] = {}
            utilities_b: dict[tuple[float, float], float] = {}

            if map_a is not None:
                for goal in targets:
                    if self._goal_too_close(ns_a, goal):
                        continue
                    if self._is_blacklisted(ns_a, goal, now_ns):
                        continue
                    score = self._cfpa2_single_utility(
                        ns=ns_a,
                        goal=goal,
                        map_msg=map_a,
                        dist_map=dist_maps.get(ns_a, {}),
                    )
                    if score > -1e17:
                        utilities_a[goal] = score

            if map_b is not None:
                for goal in targets:
                    if self._goal_too_close(ns_b, goal):
                        continue
                    if self._is_blacklisted(ns_b, goal, now_ns):
                        continue
                    score = self._cfpa2_single_utility(
                        ns=ns_b,
                        goal=goal,
                        map_msg=map_b,
                        dist_map=dist_maps.get(ns_b, {}),
                    )
                    if score > -1e17:
                        utilities_b[goal] = score

            if not utilities_a and not utilities_b:
                self._log_no_goal_debug(
                    now_ns=now_ns,
                    reason="cfpa2_no_reachable_utilities",
                    planning_map=planning_map,
                    per_ns_targets=per_ns_targets,
                    dist_maps=dist_maps,
                    utilities_sizes={ns_a: len(utilities_a), ns_b: len(utilities_b)},
                )

            utilities_by_ns: dict[str, dict[tuple[float, float], float]] = {
                ns_a: utilities_a,
                ns_b: utilities_b,
            }

            candidate_goals: dict[str, tuple[float, float]] = {}
            assignment_scores: dict[str, float] = {}
            best_joint = -1e18
            best_pair: Optional[tuple[tuple[float, float], tuple[float, float], float, float]] = None

            for goal_a, score_a in utilities_a.items():
                for goal_b, score_b in utilities_b.items():
                    if goal_a == goal_b:
                        continue
                    overlap = self._cfpa2_overlap_penalty(goal_a, goal_b)
                    joint = score_a + score_b - (self.cfpa2_lambda_overlap * overlap)
                    if joint > best_joint:
                        best_joint = joint
                        best_pair = (goal_a, goal_b, score_a, score_b)

            if best_pair is not None:
                goal_a, goal_b, score_a, score_b = best_pair
                candidate_goals[ns_a] = goal_a
                candidate_goals[ns_b] = goal_b
                assignment_scores[ns_a] = score_a
                assignment_scores[ns_b] = score_b
                self._set_policy_reason(ns_a, "switch/cfpa2_joint")
                self._set_policy_reason(ns_b, "switch/cfpa2_joint")
            else:
                best_single_ns: Optional[str] = None
                best_single_goal: Optional[tuple[float, float]] = None
                best_single_score = -1e18
                if utilities_a:
                    goal_a, score_a = max(utilities_a.items(), key=lambda kv: kv[1])
                    if score_a > best_single_score:
                        best_single_ns = ns_a
                        best_single_goal = goal_a
                        best_single_score = score_a
                if utilities_b:
                    goal_b, score_b = max(utilities_b.items(), key=lambda kv: kv[1])
                    if score_b > best_single_score:
                        best_single_ns = ns_b
                        best_single_goal = goal_b
                        best_single_score = score_b
                if best_single_ns is not None and best_single_goal is not None:
                    candidate_goals[best_single_ns] = best_single_goal
                    assignment_scores[best_single_ns] = best_single_score
                    self._set_policy_reason(best_single_ns, "switch/cfpa2_fallback_single")

            forced_switch_namespaces: set[str] = set(local_nav_forced_switch_namespaces)
            for ns in self.namespaces:
                forced_goal = self._maybe_force_cfpa2_stuck_recovery(
                    ns=ns,
                    now_ns=now_ns,
                    utilities=utilities_by_ns.get(ns, {}),
                    fallback_targets=per_ns_targets.get(ns, []),
                )
                if forced_goal is None:
                    continue
                candidate_goals[ns] = forced_goal
                assignment_scores[ns] = utilities_by_ns.get(ns, {}).get(
                    forced_goal, assignment_scores.get(ns, 0.0)
                )
                forced_switch_namespaces.add(ns)

            forced_stop_namespaces = self._apply_cfpa2_proximity_stop(
                candidate_goals=candidate_goals,
                assignment_scores=assignment_scores,
                now_ns=now_ns,
            )

            per_ns_assigned: dict[str, tuple[float, float]] = {}
            for ns in self.namespaces:
                candidate = candidate_goals.get(ns)
                if candidate is None:
                    held = self.last_goal.get(ns)
                    if held is None:
                        self._set_policy_reason(ns, "hold/cfpa2_no_candidate")
                        continue
                    if self._is_blacklisted(ns, held, now_ns):
                        fallback = self._cfpa2_best_available_goal(
                            ns=ns,
                            now_ns=now_ns,
                            utilities=utilities_by_ns.get(ns, {}),
                            exclude_goal=held,
                            fallback_targets=per_ns_targets.get(ns, []),
                        )
                        if fallback is not None:
                            candidate = fallback
                            assignment_scores[ns] = utilities_by_ns.get(ns, {}).get(fallback, 0.0)
                            forced_switch_namespaces.add(ns)
                            self._set_policy_reason(ns, "switch/cfpa2_blacklist_fallback")
                        else:
                            # If current goal is blacklisted and no alternative exists, hold position.
                            candidate = self._robot_xy(ns)
                            forced_stop_namespaces.add(ns)
                            self._set_policy_reason(ns, "hold/cfpa2_blacklisted_stop")
                    else:
                        self._set_policy_reason(ns, "hold/cfpa2_keep_previous")
                        goal = held
                if candidate is not None:
                    if ns in forced_switch_namespaces or ns in forced_stop_namespaces:
                        goal = candidate
                    else:
                        goal = self._apply_switch_hysteresis(
                            ns,
                            candidate,
                            assignment_scores.get(ns, 0.0),
                        )

                self._set_active_goal(ns, goal, now_ns)
                publish_map = self.maps.get(ns, planning_map)
                publish_goal = goal
                if (
                    ns not in forced_stop_namespaces
                    and publish_map is not None
                    and self.cfpa2_space_time_enabled
                ):
                    st_waypoint = self._cfpa2_space_time_waypoint(
                        ns=ns,
                        map_msg=publish_map,
                        final_goal=goal,
                        planned_goals=candidate_goals,
                    )
                    if st_waypoint is not None:
                        publish_goal = st_waypoint
                self._publish_goal(ns, publish_map, publish_goal)
                per_ns_assigned[ns] = goal

            per_ns_reachable: dict[str, int] = {}
            for ns in self.namespaces:
                msg = planning_map if using_shared_map else self.maps.get(ns)
                if msg is None:
                    per_ns_reachable[ns] = 0
                    continue
                dist_map = dist_maps.get(ns, {})
                reachable = 0
                for goal in targets:
                    if self._goal_too_close(ns, goal):
                        continue
                    if self._is_blacklisted(ns, goal, now_ns):
                        continue
                    if self._goal_reachable(msg, dist_map, goal):
                        reachable += 1
                per_ns_reachable[ns] = reachable

            per_ns_frontiers = {ns: len(per_ns_targets.get(ns, [])) for ns in self.namespaces}
            self._maybe_log_summary(
                targets_total=len(targets),
                per_ns_frontiers=per_ns_frontiers,
                per_ns_reachable=per_ns_reachable,
                per_ns_assigned=per_ns_assigned,
            )
            if not per_ns_assigned:
                self._log_no_goal_debug(
                    now_ns=now_ns,
                    reason="cfpa2_no_assignment_published",
                    planning_map=planning_map,
                    per_ns_targets=per_ns_targets,
                    dist_maps=dist_maps,
                    utilities_sizes={ns_a: len(utilities_a), ns_b: len(utilities_b)},
                    candidate_goals=candidate_goals,
                    per_ns_assigned=per_ns_assigned,
                )
            return

        if self.algorithm_mode == "cfpa2" and not self._warned_cfpa2_two_robot_only:
            self.get_logger().warn(
                "algorithm_mode=cfpa2 requires exactly two namespaces; falling back to collaborative mode."
            )
            self._warned_cfpa2_two_robot_only = True

        if self.algorithm_mode == "mtare":
            per_ns_assigned: dict[str, tuple[float, float]] = {}
            assigned_goals: dict[str, tuple[float, float]] = {}

            for ns in self.namespaces:
                map_msg = planning_map if using_shared_map else self.maps.get(ns)
                if map_msg is None:
                    self._set_policy_reason(ns, "hold/no_local_map")
                    continue
                best_explore_goal: Optional[tuple[float, float]] = None
                best_explore_utility = -1e18
                for goal in targets:
                    if self._goal_too_close(ns, goal):
                        continue
                    if self._is_blacklisted(ns, goal, now_ns):
                        continue
                    utility, _ = self._exploration_utility(
                        ns=ns,
                        goal=goal,
                        map_msg=map_msg,
                        dist_maps=dist_maps,
                        assigned_goals=assigned_goals,
                    )
                    if utility > best_explore_utility:
                        best_explore_utility = utility
                        best_explore_goal = goal

                pursuit_goal, pursuit_utility = self._best_pursuit_target(ns=ns, now_ns=now_ns)

                selected_goal: Optional[tuple[float, float]] = None
                if (
                    pursuit_goal is not None
                    and pursuit_utility > (best_explore_utility + self.pursuit_switch_margin)
                ):
                    selected_goal = pursuit_goal
                    self._set_policy_reason(ns, "switch/mtare_pursuit")
                elif best_explore_goal is not None:
                    selected_goal = best_explore_goal
                    self._set_policy_reason(ns, "switch/mtare_explore")

                if selected_goal is None:
                    held = self.last_goal.get(ns)
                    if held is None:
                        self._set_policy_reason(ns, "hold/mtare_no_candidate")
                        continue
                    selected_goal = held
                else:
                    selected_goal = self._apply_switch_hysteresis(
                        ns,
                        selected_goal,
                        max(best_explore_utility, pursuit_utility),
                    )

                self._set_active_goal(ns, selected_goal, now_ns)
                publish_map = self.maps.get(ns, planning_map)
                self._publish_goal(ns, publish_map, selected_goal)
                per_ns_assigned[ns] = selected_goal
                assigned_goals[ns] = selected_goal

            per_ns_reachable: dict[str, int] = {}
            for ns in self.namespaces:
                msg = planning_map if using_shared_map else self.maps.get(ns)
                if msg is None:
                    per_ns_reachable[ns] = 0
                    continue
                dist_map = dist_maps.get(ns, {})
                reachable = 0
                for goal in targets:
                    if self._goal_too_close(ns, goal):
                        continue
                    if self._is_blacklisted(ns, goal, now_ns):
                        continue
                    if self._goal_reachable(msg, dist_map, goal):
                        reachable += 1
                per_ns_reachable[ns] = reachable

            per_ns_frontiers = {ns: len(per_ns_targets.get(ns, [])) for ns in self.namespaces}
            self._maybe_log_summary(
                targets_total=len(targets),
                per_ns_frontiers=per_ns_frontiers,
                per_ns_reachable=per_ns_reachable,
                per_ns_assigned=per_ns_assigned,
            )
            return

        utilities = [1.0 for _ in targets]
        unassigned = set(self.namespaces)
        assigned: dict[str, int] = {}
        assignment_scores: dict[str, float] = {}
        sigma = max(self.sensor_range * 0.5, 1e-3)

        while unassigned:
            best_pair = None
            best_score = -1e18

            for ns in list(unassigned):
                msg = planning_map if using_shared_map else self.maps.get(ns)
                if msg is None:
                    continue
                dist_map = dist_maps[ns]
                if not dist_map:
                    continue
                for ti, (wx, wy) in enumerate(targets):
                    if self._goal_too_close(ns, (wx, wy)):
                        continue
                    if self._is_blacklisted(ns, (wx, wy), now_ns):
                        continue
                    g = self._world_to_grid(msg, wx, wy)
                    if g is None:
                        continue
                    idx = self._grid_index(g[0], g[1], int(msg.info.width))
                    if idx not in dist_map:
                        continue
                    cost_m = float(dist_map[idx]) * msg.info.resolution
                    score = utilities[ti] - self.beta * cost_m
                    if score > best_score:
                        best_score = score
                        best_pair = (ns, ti, score)

            if best_pair is None:
                break

            ns, ti, score = best_pair
            assigned[ns] = ti
            assignment_scores[ns] = float(score)
            unassigned.remove(ns)

            tx, ty = targets[ti]
            for j, (wx, wy) in enumerate(targets):
                d = math.hypot(wx - tx, wy - ty)
                if d > self.sensor_range:
                    continue
                p = math.exp(-0.5 * (d / sigma) * (d / sigma))
                utilities[j] = max(0.0, utilities[j] - p)

        # Fail-open: if collaborative assignment cannot find a reachable shared target
        # for a robot, use that robot's nearest local frontier.
        candidate_goals: dict[str, tuple[float, float]] = {}
        for ns in list(unassigned):
            local_targets = [
                goal
                for goal in per_ns_targets.get(ns, [])
                if not self._goal_too_close(ns, goal)
                if not self._is_blacklisted(ns, goal, now_ns)
            ]
            if not local_targets:
                continue
            od = self.odoms[ns]
            rx = float(od.pose.pose.position.x)
            ry = float(od.pose.pose.position.y)
            nearest_idx = min(
                range(len(local_targets)),
                key=lambda i: math.hypot(local_targets[i][0] - rx, local_targets[i][1] - ry),
            )
            candidate_goals[ns] = local_targets[nearest_idx]
            assignment_scores.setdefault(ns, 1.0)
            unassigned.remove(ns)

        for ns, ti in assigned.items():
            candidate_goals[ns] = targets[ti]

        per_ns_assigned: dict[str, tuple[float, float]] = {}
        for ns in self.namespaces:
            candidate = candidate_goals.get(ns)
            if candidate is None:
                # No new assignment candidate, hold current goal if any.
                held = self.last_goal.get(ns)
                if held is None:
                    self._set_policy_reason(ns, "hold/no_candidate_no_previous_goal")
                    continue
                self._set_policy_reason(ns, "hold/no_candidate")
                goal = held
            else:
                msg_for_ns = planning_map if using_shared_map else self.maps.get(ns)
                if msg_for_ns is None:
                    held = self.last_goal.get(ns)
                    if held is None:
                        self._set_policy_reason(ns, "hold/no_local_map")
                        continue
                    self._set_policy_reason(ns, "hold/no_local_map")
                    goal = held
                    self._set_active_goal(ns, goal, now_ns)
                    publish_map = self.maps.get(ns, planning_map)
                    self._publish_goal(ns, publish_map, goal)
                    per_ns_assigned[ns] = goal
                    continue
                goal = self._apply_goal_policy(
                    ns=ns,
                    candidate_goal=candidate,
                    assignment_score=assignment_scores.get(ns, 0.0),
                    map_msg=msg_for_ns,
                    dist_map=dist_maps.get(ns, {}),
                    now_ns=now_ns,
                )

            self._set_active_goal(ns, goal, now_ns)
            publish_map = self.maps.get(ns, planning_map)
            self._publish_goal(ns, publish_map, goal)
            per_ns_assigned[ns] = goal

        per_ns_reachable: dict[str, int] = {}
        for ns in self.namespaces:
            msg = planning_map if using_shared_map else self.maps.get(ns)
            if msg is None:
                per_ns_reachable[ns] = 0
                continue
            dist_map = dist_maps.get(ns, {})
            if not dist_map:
                per_ns_reachable[ns] = 0
                continue
            reachable = 0
            for wx, wy in targets:
                if self._goal_too_close(ns, (wx, wy)):
                    continue
                if self._is_blacklisted(ns, (wx, wy), now_ns):
                    continue
                g = self._world_to_grid(msg, wx, wy)
                if g is None:
                    continue
                idx = self._grid_index(g[0], g[1], int(msg.info.width))
                if idx in dist_map:
                    reachable += 1
            per_ns_reachable[ns] = reachable

        per_ns_frontiers = {ns: len(per_ns_targets.get(ns, [])) for ns in self.namespaces}
        self._maybe_log_summary(
            targets_total=len(targets),
            per_ns_frontiers=per_ns_frontiers,
            per_ns_reachable=per_ns_reachable,
            per_ns_assigned=per_ns_assigned,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CFPA2Coordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
