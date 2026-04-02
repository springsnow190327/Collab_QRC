#!/usr/bin/env python3
"""Unified exploration experiment logger.

Writes a time-series CSV capturing standard multi-robot exploration metrics:
- Coverage percentage and area
- Per-robot trajectory length
- Velocity (instantaneous and average)
- Goal assignment / reached / collision-stop counters
- Frontier cluster count
- Exploration efficiency (m² explored per m traveled)
- Multi-robot overlap percentage

Output: {output_dir}/exploration_{experiment_name}_{timestamp}.csv

Parameters:
    namespaces          (list[str])  Robot namespaces          ["robot"]
    experiment_name     (str)        Tag for output filename   "run"
    log_rate            (float)      CSV write rate (Hz)       1.0
    output_dir          (str)        Output directory          "/tmp"
"""

from __future__ import annotations

import math
import os
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Int8, String
from visualization_msgs.msg import MarkerArray


class _RobotState:
    """Accumulates metrics for one robot."""

    __slots__ = (
        "ns",
        "x", "y", "yaw", "speed",
        "trajectory_m", "_prev_x", "_prev_y",
        "map_free", "map_occ", "map_total", "map_resolution",
        "goals_received", "goals_reached",
        "collision_stops", "_prev_stop",
        "frontier_clusters",
        "nav_mode",
        "explored_cells",
    )

    def __init__(self, ns: str) -> None:
        self.ns = ns
        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.yaw: Optional[float] = None
        self.speed: float = 0.0

        self.trajectory_m: float = 0.0
        self._prev_x: Optional[float] = None
        self._prev_y: Optional[float] = None

        self.map_free: int = 0
        self.map_occ: int = 0
        self.map_total: int = 0
        self.map_resolution: float = 0.05
        self.explored_cells: set[tuple[int, int]] = set()

        self.goals_received: int = 0
        self.goals_reached: int = 0

        self.collision_stops: int = 0
        self._prev_stop: int = 0

        self.frontier_clusters: int = 0
        self.nav_mode: str = "?"


class ExplorationMetricsLogger(Node):
    def __init__(self) -> None:
        super().__init__("exploration_metrics_logger")

        # Parameters
        self.declare_parameter("namespaces", ["robot"])
        self.declare_parameter("experiment_name", "run")
        self.declare_parameter("log_rate", 1.0)
        self.declare_parameter("output_dir", "/tmp")

        namespaces = self.get_parameter("namespaces").value
        experiment_name = str(self.get_parameter("experiment_name").value)
        log_rate = float(self.get_parameter("log_rate").value)
        output_dir = str(self.get_parameter("output_dir").value)

        os.makedirs(output_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(output_dir, f"exploration_{experiment_name}_{ts}.csv")

        self.robots: dict[str, _RobotState] = {}
        self.start_time: Optional[float] = None

        for ns in namespaces:
            rs = _RobotState(ns)
            self.robots[ns] = rs

            self.create_subscription(
                Odometry, f"/{ns}/odom/ground_truth",
                lambda msg, n=ns: self._odom_cb(msg, n), 10,
            )
            self.create_subscription(
                OccupancyGrid, f"/{ns}/map",
                lambda msg, n=ns: self._map_cb(msg, n), 1,
            )
            self.create_subscription(
                String, f"/{ns}/nav_status",
                lambda msg, n=ns: self._nav_status_cb(msg, n), 10,
            )
            self.create_subscription(
                Int8, f"/{ns}/stop",
                lambda msg, n=ns: self._stop_cb(msg, n), 10,
            )
            self.create_subscription(
                MarkerArray, f"/{ns}/frontier_markers",
                lambda msg, n=ns: self._frontier_cb(msg, n), 10,
            )

        # Build CSV header
        header_parts = ["t_wall", "t_sim"]
        for ns in namespaces:
            p = ns  # column prefix
            header_parts.extend([
                f"{p}_x", f"{p}_y", f"{p}_yaw_deg",
                f"{p}_velocity_mps", f"{p}_avg_velocity_mps",
                f"{p}_trajectory_m",
                f"{p}_coverage_pct", f"{p}_coverage_area_m2",
                f"{p}_goals_received", f"{p}_goals_reached",
                f"{p}_collision_stops",
                f"{p}_frontier_clusters",
                f"{p}_efficiency_m2_per_m",
                f"{p}_nav_mode",
            ])
        # Multi-robot overlap (only if > 1 robot)
        if len(namespaces) > 1:
            header_parts.append("overlap_pct")

        with open(self.csv_path, "w") as f:
            f.write(",".join(header_parts) + "\n")

        self.create_timer(1.0 / max(0.1, log_rate), self._log_row)

        self.get_logger().info(f"Exploration logger started → {self.csv_path}")
        self.get_logger().info(f"  namespaces={namespaces}  rate={log_rate}Hz")

    # ── Callbacks ─────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry, ns: str) -> None:
        rs = self.robots[ns]
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if rs._prev_x is not None:
            rs.trajectory_m += math.hypot(x - rs._prev_x, y - rs._prev_y)
        rs._prev_x = x
        rs._prev_y = y

        rs.x = x
        rs.y = y
        q = msg.pose.pose.orientation
        rs.yaw = math.degrees(math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        ))
        rs.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def _map_cb(self, msg: OccupancyGrid, ns: str) -> None:
        rs = self.robots[ns]
        rs.map_total = len(msg.data)
        rs.map_resolution = msg.info.resolution
        rs.map_free = 0
        rs.map_occ = 0
        w = msg.info.width
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        res = msg.info.resolution

        explored = set()
        for i, v in enumerate(msg.data):
            if v >= 0:  # known cell (free=0 or occupied=100)
                rs.map_free += 1 if v == 0 else 0
                rs.map_occ += 1 if v == 100 else 0
                gx = i % w
                gy = i // w
                # Store world-discretised cell for overlap computation
                wx = int((ox + (gx + 0.5) * res) / res)
                wy = int((oy + (gy + 0.5) * res) / res)
                explored.add((wx, wy))
        rs.explored_cells = explored

    def _nav_status_cb(self, msg: String, ns: str) -> None:
        import json
        rs = self.robots[ns]
        try:
            d = json.loads(msg.data)
            new_mode = d.get("mode", "?")
            # Detect goal-reached transitions
            if new_mode == "reached" and rs.nav_mode != "reached":
                rs.goals_reached += 1
            # Detect new goal assignments
            if new_mode in ("navigate", "steer") and rs.nav_mode in ("no_goal", "reached", "?"):
                rs.goals_received += 1
            rs.nav_mode = new_mode
        except Exception:
            pass

    def _stop_cb(self, msg: Int8, ns: str) -> None:
        rs = self.robots[ns]
        val = int(msg.data)
        if val == 1 and rs._prev_stop == 0:
            rs.collision_stops += 1
        rs._prev_stop = val

    def _frontier_cb(self, msg: MarkerArray, ns: str) -> None:
        self.robots[ns].frontier_clusters = len(msg.markers)

    # ── CSV logging ───────────────────────────────────────────────────

    def _log_row(self) -> None:
        now_wall = time.time()
        if self.start_time is None:
            self.start_time = now_wall
        t_wall = now_wall - self.start_time
        t_sim = self.get_clock().now().nanoseconds / 1e9

        parts: list[str] = [f"{t_wall:.2f}", f"{t_sim:.2f}"]

        for ns, rs in self.robots.items():
            explored = rs.map_free + rs.map_occ
            coverage_pct = (100.0 * explored / rs.map_total) if rs.map_total > 0 else 0.0
            coverage_area = explored * (rs.map_resolution ** 2)
            avg_vel = (rs.trajectory_m / t_wall) if t_wall > 1.0 else 0.0
            efficiency = (coverage_area / rs.trajectory_m) if rs.trajectory_m > 0.5 else 0.0

            parts.extend([
                f"{rs.x:.3f}" if rs.x is not None else "",
                f"{rs.y:.3f}" if rs.y is not None else "",
                f"{rs.yaw:.1f}" if rs.yaw is not None else "",
                f"{rs.speed:.3f}",
                f"{avg_vel:.3f}",
                f"{rs.trajectory_m:.3f}",
                f"{coverage_pct:.2f}",
                f"{coverage_area:.3f}",
                str(rs.goals_received),
                str(rs.goals_reached),
                str(rs.collision_stops),
                str(rs.frontier_clusters),
                f"{efficiency:.4f}",
                rs.nav_mode,
            ])

        # Multi-robot overlap
        if len(self.robots) > 1:
            all_sets = [rs.explored_cells for rs in self.robots.values() if rs.explored_cells]
            if len(all_sets) >= 2:
                union = set.union(*all_sets)
                intersection = set.intersection(*all_sets)
                overlap_pct = (100.0 * len(intersection) / len(union)) if union else 0.0
                parts.append(f"{overlap_pct:.2f}")
            else:
                parts.append("0.00")

        with open(self.csv_path, "a") as f:
            f.write(",".join(parts) + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationMetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"Final CSV at: {node.csv_path}")
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
