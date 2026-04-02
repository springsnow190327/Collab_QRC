#!/usr/bin/env python3
"""Simple occupancy grid mapper for Isaac exploration.

Single responsibility:
- consume LaserScan + Odometry
- publish OccupancyGrid

No frontier extraction or goal selection is done here.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


def _bresenham(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
    points: list[tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1

    if dx > dy:
        err = dx / 2.0
        while x != x1:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    points.append((x1, y1))
    return points


class SimpleScanMapper(Node):
    def __init__(self) -> None:
        super().__init__("simple_scan_mapper")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom/nav")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_frame", "world")
        self.declare_parameter("lidar_offset_x", 0.0)
        self.declare_parameter("lidar_offset_y", 0.0)

        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("width", 400)
        self.declare_parameter("height", 400)
        self.declare_parameter("origin_x", -20.0)
        self.declare_parameter("origin_y", -20.0)

        self.declare_parameter("max_range", 12.0)
        self.declare_parameter("max_clear_distance", 4.0)
        self.declare_parameter("clear_on_nohit", False)
        self.declare_parameter("update_rate", 4.0)
        self.declare_parameter("startup_delay", 4.0)
        # Set to 0 to disable timestamp strictness for simulator pipelines that
        # stamp scan/odom on slightly different clocks.
        self.declare_parameter("max_scan_odom_dt", 0.0)
        self.declare_parameter("odom_history_sec", 2.0)
        # Occupancy evidence integration (reduces wall flicker/starburst artifacts).
        self.declare_parameter("hit_increment", 3)
        self.declare_parameter("miss_decrement", 1)
        self.declare_parameter("score_min", -20)
        self.declare_parameter("score_max", 20)
        self.declare_parameter("occupied_score_threshold", 3)
        self.declare_parameter("free_score_threshold", -3)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.lidar_offset_x = float(self.get_parameter("lidar_offset_x").value)
        self.lidar_offset_y = float(self.get_parameter("lidar_offset_y").value)

        self.resolution = float(self.get_parameter("resolution").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.origin_x = float(self.get_parameter("origin_x").value)
        self.origin_y = float(self.get_parameter("origin_y").value)

        self.max_range = max(0.1, float(self.get_parameter("max_range").value))
        self.max_clear_distance = max(0.0, float(self.get_parameter("max_clear_distance").value))
        self.clear_on_nohit = bool(self.get_parameter("clear_on_nohit").value)
        self.update_rate = max(0.5, float(self.get_parameter("update_rate").value))
        self.startup_delay = max(0.0, float(self.get_parameter("startup_delay").value))
        self.max_scan_odom_dt = max(0.0, float(self.get_parameter("max_scan_odom_dt").value))
        self.odom_history_sec = max(0.5, float(self.get_parameter("odom_history_sec").value))
        self.hit_increment = max(1, int(self.get_parameter("hit_increment").value))
        self.miss_decrement = max(1, int(self.get_parameter("miss_decrement").value))
        self.score_min = int(self.get_parameter("score_min").value)
        self.score_max = int(self.get_parameter("score_max").value)
        if self.score_min >= self.score_max:
            self.score_min, self.score_max = -20, 20
        self.occupied_score_threshold = int(self.get_parameter("occupied_score_threshold").value)
        self.free_score_threshold = int(self.get_parameter("free_score_threshold").value)
        if self.free_score_threshold >= self.occupied_score_threshold:
            self.free_score_threshold, self.occupied_score_threshold = -3, 3

        n_cells = self.width * self.height
        self.grid = [-1] * n_cells
        self.scores = [0] * n_cells
        self.observed = [False] * n_cells
        self.last_scan: Optional[LaserScan] = None
        self.last_odom: Optional[Odometry] = None
        self.odom_hist: deque[tuple[int, Odometry]] = deque()
        self.start_time: Optional[Time] = None

        self._last_sync_warn_ns = 0
        self._last_summary_ns = 0
        self._last_match_dt = 0.0

        # Keep only newest samples to avoid callback backlog on heavy scenes.
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, scan_qos)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, odom_qos)

        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, 1)
        self.timer = self.create_timer(1.0 / self.update_rate, self._update)

        self.get_logger().info(
            "Simple scan mapper started | "
            f"scan={self.scan_topic} odom={self.odom_topic} map={self.map_topic} "
            f"size={self.width}x{self.height} res={self.resolution:.2f} "
            f"lidar_offset=({self.lidar_offset_x:.3f},{self.lidar_offset_y:.3f}) "
            f"score=[{self.score_min},{self.score_max}] hit={self.hit_increment} miss={self.miss_decrement} "
            f"odom_history={self.odom_history_sec:.1f}s"
        )

    def _scan_cb(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self.last_odom = msg
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if stamp_ns <= 0:
            # Fall back to local clock if stamp is unset.
            stamp_ns = self.get_clock().now().nanoseconds
        self.odom_hist.append((stamp_ns, msg))

        # Keep a small odom history window for scan timestamp matching.
        prune_before = stamp_ns - int(self.odom_history_sec * 1e9)
        while self.odom_hist and self.odom_hist[0][0] < prune_before:
            self.odom_hist.popleft()

    def _world_to_grid(self, x: float, y: float) -> Optional[tuple[int, int]]:
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        if gx < 0 or gy < 0 or gx >= self.width or gy >= self.height:
            return None
        return gx, gy

    def _apply_evidence(self, gx: int, gy: int, delta: int) -> None:
        idx = gy * self.width + gx
        self.observed[idx] = True
        score = self.scores[idx] + delta
        if score < self.score_min:
            score = self.score_min
        elif score > self.score_max:
            score = self.score_max
        self.scores[idx] = score

    @staticmethod
    def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny, cosy)

    def _publish_map(self, stamp) -> None:
        for i, score in enumerate(self.scores):
            if not self.observed[i]:
                self.grid[i] = -1
            elif score >= self.occupied_score_threshold:
                self.grid[i] = 100
            elif score <= self.free_score_threshold:
                self.grid[i] = 0
            else:
                # Keep uncertain cells unknown to avoid rapid free/occupied flicker.
                self.grid[i] = -1

        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid
        self.map_pub.publish(msg)

    def _select_odom_for_scan(self, scan: LaserScan) -> tuple[Optional[Odometry], float]:
        # Prefer odometry closest in time to each scan to reduce rotation smear.
        if not self.odom_hist:
            return self.last_odom, float("inf")

        scan_t = Time.from_msg(scan.header.stamp).nanoseconds
        if scan_t <= 0:
            return self.last_odom, 0.0

        best_msg: Optional[Odometry] = None
        best_dt = float("inf")
        for odom_t, odom_msg in self.odom_hist:
            dt = abs(scan_t - odom_t) / 1e9
            if dt < best_dt:
                best_dt = dt
                best_msg = odom_msg
        return best_msg, best_dt

    def _update(self) -> None:
        if self.last_scan is None or self.last_odom is None:
            return

        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        if (now - self.start_time).nanoseconds / 1e9 < self.startup_delay:
            return

        scan = self.last_scan
        odom, matched_dt = self._select_odom_for_scan(scan)
        if odom is None:
            return
        self._last_match_dt = matched_dt

        if self.max_scan_odom_dt > 0.0:
            if math.isfinite(matched_dt) and matched_dt > self.max_scan_odom_dt:
                if now.nanoseconds - self._last_sync_warn_ns > int(2e9):
                    self.get_logger().warn(
                        f"scan/odom desync: dt={matched_dt:.3f}s > {self.max_scan_odom_dt:.3f}s; skipping map update"
                    )
                    self._last_sync_warn_ns = now.nanoseconds
                return

        rx = float(odom.pose.pose.position.x)
        ry = float(odom.pose.pose.position.y)
        yaw = self._yaw_from_quat(
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        )
        # Map rays from physical lidar origin, not base center.
        sx = rx + (math.cos(yaw) * self.lidar_offset_x - math.sin(yaw) * self.lidar_offset_y)
        sy = ry + (math.sin(yaw) * self.lidar_offset_x + math.cos(yaw) * self.lidar_offset_y)

        origin_cell = self._world_to_grid(sx, sy)
        if origin_cell is None:
            return

        angle = float(scan.angle_min)
        inc = float(scan.angle_increment)
        for rng in scan.ranges:
            finite = math.isfinite(rng)
            if finite and rng < scan.range_min:
                angle += inc
                continue

            dist = min(float(rng), self.max_range) if finite else self.max_range
            has_hit = finite and (rng < self.max_range * 0.99)

            world_bearing = yaw + angle
            ex = sx + dist * math.cos(world_bearing)
            ey = sy + dist * math.sin(world_bearing)
            end_cell = self._world_to_grid(ex, ey)

            clear_dist = dist if self.max_clear_distance <= 0.0 else min(dist, self.max_clear_distance)
            cex = sx + clear_dist * math.cos(world_bearing)
            cey = sy + clear_dist * math.sin(world_bearing)
            clear_end_cell = self._world_to_grid(cex, cey)

            angle += inc
            if clear_end_cell is None:
                continue

            if has_hit or self.clear_on_nohit:
                # Ray carve free cells first, then set the endpoint occupancy.
                cells = _bresenham(origin_cell[0], origin_cell[1], clear_end_cell[0], clear_end_cell[1])
                for cx, cy in cells[:-1]:
                    self._apply_evidence(cx, cy, -self.miss_decrement)

            if has_hit and end_cell is not None:
                self._apply_evidence(end_cell[0], end_cell[1], self.hit_increment)
            elif finite and self.clear_on_nohit and end_cell is not None:
                self._apply_evidence(end_cell[0], end_cell[1], -self.miss_decrement)

        self._publish_map(scan.header.stamp)

        if self._last_summary_ns == 0 or (now.nanoseconds - self._last_summary_ns) > int(10e9):
            self._last_summary_ns = now.nanoseconds
            free_n = sum(1 for c in self.grid if c == 0)
            occ_n = sum(1 for c in self.grid if c == 100)
            self.get_logger().info(
                f"MAP step: free={free_n} occ={occ_n} unknown={len(self.grid) - free_n - occ_n} "
                f"matched_dt={self._last_match_dt:.3f}s"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimpleScanMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
