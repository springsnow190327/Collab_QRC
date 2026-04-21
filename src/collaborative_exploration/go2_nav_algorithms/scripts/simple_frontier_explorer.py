#!/usr/bin/env python3
"""Map-only frontier explorer for Isaac stacks.

Node responsibilities:
- read an existing OccupancyGrid + robot odometry
- detect free/unknown map boundaries (frontiers)
- cluster frontier cells and pick one goal
- publish goal + RViz markers

Algorithm swap guide:
- replace `_extract_frontier_cells`, `_cluster_frontiers`, and `_rank_candidates`
  to plug in a different exploration strategy without touching launch wiring.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class FrontierCandidate:
    """Candidate goal distilled from one frontier cluster."""

    world_x: float
    world_y: float
    distance_m: float
    size_cells: int


class SimpleFrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__("simple_frontier_explorer")

        # Topic contract used by single and dual Isaac launch files.
        self.declare_parameter("map_topic", "/map")
        # Optional Nav2 inflated occupancy source. When enabled, goals are chosen
        # from this safer map instead of raw SLAM cells near walls.
        self.declare_parameter("costmap_topic", "")
        self.declare_parameter("prefer_costmap", True)
        self.declare_parameter("costmap_stale_sec", 2.0)
        self.declare_parameter("odom_topic", "/odom/nav")
        self.declare_parameter("frontier_goal_topic", "/way_point")
        self.declare_parameter("frontier_marker_topic", "/frontier_goal_marker")
        self.declare_parameter("frontier_regions_topic", "/frontier_markers")
        self.declare_parameter("frontier_replan_topic", "/frontier_replan")
        self.declare_parameter("map_frame", "world")

        # Runtime pacing.
        self.declare_parameter("update_rate", 2.0)
        self.declare_parameter("startup_delay", 0.0)
        self.declare_parameter("max_map_odom_dt", 0.0)

        # Frontier extraction / goal selection knobs.
        self.declare_parameter("frontier_min_size", 6)
        self.declare_parameter("frontier_stride", 1)
        self.declare_parameter("goal_selection_mode", "nearest")  # nearest|farthest|largest
        self.declare_parameter("min_goal_distance", 0.8)
        self.declare_parameter("max_goal_distance", 0.0)
        self.declare_parameter("goal_min_separation", 1.0)
        self.declare_parameter("goal_reselect_distance", 0.9)
        # Keep target away from walls/obstacle cells.
        self.declare_parameter("goal_obstacle_clearance_m", 0.40)
        # Pull selected frontier goal back toward robot to avoid wall grazing.
        self.declare_parameter("goal_backoff_m", 0.35)

        # Occupancy semantics.
        self.declare_parameter("free_value", 0)
        self.declare_parameter("unknown_value", -1)
        self.declare_parameter("occupancy_block_threshold", 50)

        # RViz debugging.
        self.declare_parameter("publish_debug_markers", True)

        # Compatibility no-ops so legacy frontier yaml can be reused unchanged.
        self.declare_parameter("frontier_extraction_mode", "grid")
        self.declare_parameter("require_path_feasibility", False)
        self.declare_parameter("max_path_stretch", 2.5)
        self.declare_parameter("traversability_inflation_cells", 0)
        self.declare_parameter("denoise_isolated_obstacles", False)
        self.declare_parameter("denoise_occ_min_neighbors", 0)

        self.map_topic = str(self.get_parameter("map_topic").value)
        self.costmap_topic = str(self.get_parameter("costmap_topic").value).strip()
        self.prefer_costmap = bool(self.get_parameter("prefer_costmap").value)
        self.costmap_stale_sec = max(0.0, float(self.get_parameter("costmap_stale_sec").value))
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.frontier_goal_topic = str(self.get_parameter("frontier_goal_topic").value)
        self.frontier_marker_topic = str(self.get_parameter("frontier_marker_topic").value)
        self.frontier_regions_topic = str(self.get_parameter("frontier_regions_topic").value)
        self.frontier_replan_topic = str(self.get_parameter("frontier_replan_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)

        self.update_rate = max(0.2, float(self.get_parameter("update_rate").value))
        self.startup_delay = max(0.0, float(self.get_parameter("startup_delay").value))
        self.max_map_odom_dt = max(0.0, float(self.get_parameter("max_map_odom_dt").value))

        self.frontier_min_size = max(1, int(self.get_parameter("frontier_min_size").value))
        self.frontier_stride = max(1, int(self.get_parameter("frontier_stride").value))
        self.goal_selection_mode = str(self.get_parameter("goal_selection_mode").value).lower()
        self.min_goal_distance = max(0.0, float(self.get_parameter("min_goal_distance").value))
        self.max_goal_distance = max(0.0, float(self.get_parameter("max_goal_distance").value))
        self.goal_min_separation = max(0.0, float(self.get_parameter("goal_min_separation").value))
        self.goal_reselect_distance = max(0.0, float(self.get_parameter("goal_reselect_distance").value))
        self.goal_obstacle_clearance_m = max(0.0, float(self.get_parameter("goal_obstacle_clearance_m").value))
        self.goal_backoff_m = max(0.0, float(self.get_parameter("goal_backoff_m").value))

        self.free_value = int(self.get_parameter("free_value").value)
        self.unknown_value = int(self.get_parameter("unknown_value").value)
        self.occ_thresh = int(self.get_parameter("occupancy_block_threshold").value)

        self.publish_debug_markers = bool(self.get_parameter("publish_debug_markers").value)

        self.last_map: Optional[OccupancyGrid] = None
        self.last_costmap: Optional[OccupancyGrid] = None
        self.last_odom: Optional[Odometry] = None
        self.current_goal: Optional[tuple[float, float]] = None
        self.force_replan = False

        self._start_time: Optional[Time] = None
        self._active_map_source = "map"
        self._last_sync_warn_ns = 0
        self._last_costmap_warn_ns = 0
        self._last_summary_ns = 0
        self._last_goal_log: Optional[tuple[float, float]] = None

        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, 1)
        if self.costmap_topic:
            self.create_subscription(OccupancyGrid, self.costmap_topic, self._costmap_cb, 1)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_subscription(Empty, self.frontier_replan_topic, self._replan_cb, 10)

        self.goal_pub = self.create_publisher(PointStamped, self.frontier_goal_topic, 10)
        self.goal_marker_pub = self.create_publisher(Marker, self.frontier_marker_topic, 10)
        self.regions_pub = self.create_publisher(MarkerArray, self.frontier_regions_topic, 10)

        self.timer = self.create_timer(1.0 / self.update_rate, self._tick)

        self.get_logger().info(
            "Simple frontier explorer started | "
            f"map={self.map_topic} costmap={self.costmap_topic or 'disabled'} "
            f"prefer_costmap={self.prefer_costmap} "
            f"odom={self.odom_topic} goal={self.frontier_goal_topic} "
            f"mode={self.goal_selection_mode}"
        )

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.last_map = msg

    def _costmap_cb(self, msg: OccupancyGrid) -> None:
        self.last_costmap = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self.last_odom = msg

    def _replan_cb(self, _: Empty) -> None:
        self.force_replan = True

    @staticmethod
    def _idx(x: int, y: int, width: int) -> int:
        return y * width + x

    def _is_free(self, value: int) -> bool:
        # Primary mode: occupancy grids where free is exactly 0.
        if value == self.free_value:
            return True
        # Fallback for probability grids where low occupied values are still free-ish.
        if value >= 0 and value < self.occ_thresh and self.free_value == 0:
            return True
        return False

    def _is_unknown(self, value: int) -> bool:
        return value == self.unknown_value

    def _is_occupied(self, value: int) -> bool:
        return value >= self.occ_thresh

    @staticmethod
    def _grid_to_world(msg: OccupancyGrid, gx: float, gy: float) -> tuple[float, float]:
        return (
            msg.info.origin.position.x + (gx + 0.5) * msg.info.resolution,
            msg.info.origin.position.y + (gy + 0.5) * msg.info.resolution,
        )

    @staticmethod
    def _world_to_grid(msg: OccupancyGrid, wx: float, wy: float) -> Optional[tuple[int, int]]:
        gx = int((wx - msg.info.origin.position.x) / msg.info.resolution)
        gy = int((wy - msg.info.origin.position.y) / msg.info.resolution)
        if gx < 0 or gy < 0 or gx >= int(msg.info.width) or gy >= int(msg.info.height):
            return None
        return gx, gy

    def _has_obstacle_clearance(self, msg: OccupancyGrid, gx: int, gy: int, radius_cells: int) -> bool:
        if radius_cells <= 0:
            return True

        w = int(msg.info.width)
        h = int(msg.info.height)
        data = msg.data
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
                if self._is_occupied(data[self._idx(nx, ny, w)]):
                    return False
        return True

    @staticmethod
    def _backoff_goal_toward_robot(
        wx: float,
        wy: float,
        robot_x: float,
        robot_y: float,
        backoff_m: float,
    ) -> tuple[float, float]:
        if backoff_m <= 0.0:
            return wx, wy
        vx = robot_x - wx
        vy = robot_y - wy
        dist = math.hypot(vx, vy)
        if dist <= 1e-6:
            return wx, wy
        step = min(backoff_m, max(0.0, dist - 0.05))
        return wx + (vx / dist) * step, wy + (vy / dist) * step

    def _extract_frontier_cells(self, msg: OccupancyGrid) -> list[tuple[int, int]]:
        data = msg.data
        w = int(msg.info.width)
        h = int(msg.info.height)
        stride = self.frontier_stride
        out: list[tuple[int, int]] = []

        for gy in range(1, h - 1, stride):
            row = gy * w
            for gx in range(1, w - 1, stride):
                idx = row + gx
                if not self._is_free(data[idx]):
                    continue
                # Frontier definition: free cell touching at least one unknown neighbor.
                is_frontier = False
                for dx, dy in (
                    (1, 0),
                    (-1, 0),
                    (0, 1),
                    (0, -1),
                    (1, 1),
                    (-1, 1),
                    (1, -1),
                    (-1, -1),
                ):
                    nidx = (gy + dy) * w + (gx + dx)
                    if self._is_unknown(data[nidx]):
                        is_frontier = True
                        break
                if is_frontier:
                    out.append((gx, gy))
        return out

    def _cluster_frontiers(self, frontier_cells: list[tuple[int, int]]) -> list[list[tuple[int, int]]]:
        if not frontier_cells:
            return []

        frontier_set = set(frontier_cells)
        visited: set[tuple[int, int]] = set()
        clusters: list[list[tuple[int, int]]] = []

        for seed in frontier_cells:
            if seed in visited:
                continue
            q = deque([seed])
            visited.add(seed)
            cluster: list[tuple[int, int]] = []

            while q:
                cx, cy = q.popleft()
                cluster.append((cx, cy))
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        nb = (cx + dx, cy + dy)
                        if nb in frontier_set and nb not in visited:
                            visited.add(nb)
                            q.append(nb)

            if len(cluster) >= self.frontier_min_size:
                clusters.append(cluster)

        return clusters

    def _build_candidates(
        self,
        msg: OccupancyGrid,
        clusters: list[list[tuple[int, int]]],
        robot_x: float,
        robot_y: float,
    ) -> list[FrontierCandidate]:
        candidates: list[FrontierCandidate] = []
        clearance_cells = int(math.ceil(self.goal_obstacle_clearance_m / max(msg.info.resolution, 1e-6)))
        w = int(msg.info.width)
        data = msg.data

        for cluster in clusters:
            cx = sum(c[0] for c in cluster) / len(cluster)
            cy = sum(c[1] for c in cluster) / len(cluster)
            # Prefer a frontier cell near cluster center that has obstacle clearance.
            sorted_cells = sorted(cluster, key=lambda c: (c[0] - cx) * (c[0] - cx) + (c[1] - cy) * (c[1] - cy))

            chosen_world: Optional[tuple[float, float]] = None
            for gx, gy in sorted_cells:
                idx = self._idx(gx, gy, w)
                if not self._is_free(data[idx]):
                    continue
                if not self._has_obstacle_clearance(msg, gx, gy, clearance_cells):
                    continue
                wx, wy = self._grid_to_world(msg, gx, gy)
                bwx, bwy = self._backoff_goal_toward_robot(wx, wy, robot_x, robot_y, self.goal_backoff_m)
                backoff_cell = self._world_to_grid(msg, bwx, bwy)
                if backoff_cell is None:
                    continue
                bx, by = backoff_cell
                bidx = self._idx(bx, by, w)
                if not self._is_free(data[bidx]):
                    continue
                if not self._has_obstacle_clearance(msg, bx, by, clearance_cells):
                    continue
                chosen_world = (bwx, bwy)
                break

            if chosen_world is None:
                continue

            wx, wy = chosen_world
            dist = math.hypot(wx - robot_x, wy - robot_y)

            if dist < self.min_goal_distance:
                continue
            if self.max_goal_distance > 0.0 and dist > self.max_goal_distance:
                continue

            candidates.append(
                FrontierCandidate(
                    world_x=wx,
                    world_y=wy,
                    distance_m=dist,
                    size_cells=len(cluster),
                )
            )

        return candidates

    def _rank_candidates(self, candidates: list[FrontierCandidate]) -> list[FrontierCandidate]:
        if self.goal_selection_mode == "farthest":
            return sorted(candidates, key=lambda c: c.distance_m, reverse=True)
        if self.goal_selection_mode == "largest":
            return sorted(candidates, key=lambda c: (c.size_cells, -c.distance_m), reverse=True)
        # Default: nearest tends to reduce dead time and keeps dual robots moving.
        return sorted(candidates, key=lambda c: c.distance_m)

    def _publish_regions_markers(self, msg: OccupancyGrid, clusters: list[list[tuple[int, int]]]) -> None:
        if not self.publish_debug_markers:
            return

        frame_id = msg.header.frame_id or self.map_frame
        now = self.get_clock().now().to_msg()

        marker_array = MarkerArray()

        clear = Marker()
        clear.header.frame_id = frame_id
        clear.header.stamp = now
        clear.ns = "frontiers"
        clear.id = 0
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        # Cluster markers let us visually inspect frontier segmentation quality.
        for i, cluster in enumerate(clusters[:120], start=1):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = now
            marker.ns = "frontiers"
            marker.id = i
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = msg.info.resolution * 0.8
            marker.scale.y = msg.info.resolution * 0.8
            marker.color.a = 0.85
            marker.color.r = float((37 * i) % 255) / 255.0
            marker.color.g = float((97 * i) % 255) / 255.0
            marker.color.b = float((173 * i) % 255) / 255.0

            for gx, gy in cluster[:: max(1, self.frontier_stride)]:
                wx, wy = self._grid_to_world(msg, gx, gy)
                p = Point()
                p.x = float(wx)
                p.y = float(wy)
                p.z = 0.02
                marker.points.append(p)

            marker_array.markers.append(marker)

        self.regions_pub.publish(marker_array)

    def _publish_goal_marker(self, frame_id: str, goal_x: float, goal_y: float) -> None:
        if not self.publish_debug_markers:
            return

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = frame_id
        marker.ns = "frontier_goal"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = goal_x
        marker.pose.position.y = goal_y
        marker.pose.position.z = 0.08
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.28
        marker.scale.y = 0.28
        marker.scale.z = 0.28
        marker.color.a = 0.95
        marker.color.r = 1.0
        marker.color.g = 0.45
        marker.color.b = 0.05
        self.goal_marker_pub.publish(marker)

    def _publish_goal(self, frame_id: str, stamp, goal_x: float, goal_y: float) -> None:
        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.point.x = float(goal_x)
        msg.point.y = float(goal_y)
        msg.point.z = 0.0
        self.goal_pub.publish(msg)

        new_goal = (float(goal_x), float(goal_y))
        if self._last_goal_log is None or math.hypot(new_goal[0] - self._last_goal_log[0], new_goal[1] - self._last_goal_log[1]) > 0.05:
            self._last_goal_log = new_goal
            self.get_logger().info(f"Published frontier goal: ({goal_x:.2f}, {goal_y:.2f})")

    @staticmethod
    def _msg_age_sec(msg: OccupancyGrid, now_ns: int) -> float:
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if stamp_ns <= 0:
            return 0.0
        return max(0.0, (now_ns - stamp_ns) / 1e9)

    def _get_active_map(self, now_ns: int) -> Optional[OccupancyGrid]:
        map_msg = self.last_map
        costmap_msg = self.last_costmap

        # Prefer Nav2 inflated costmap when requested and fresh.
        if self.prefer_costmap and self.costmap_topic:
            if costmap_msg is not None:
                age = self._msg_age_sec(costmap_msg, now_ns)
                if self.costmap_stale_sec <= 0.0 or age <= self.costmap_stale_sec:
                    if self._active_map_source != "costmap":
                        self._active_map_source = "costmap"
                        self.get_logger().info(
                            f"Using inflated costmap source: {self.costmap_topic} (age={age:.2f}s)"
                        )
                    return costmap_msg

            if map_msg is not None:
                if now_ns - self._last_costmap_warn_ns > int(5e9):
                    self._last_costmap_warn_ns = now_ns
                    costmap_age = -1.0
                    if costmap_msg is not None:
                        costmap_age = self._msg_age_sec(costmap_msg, now_ns)
                    self.get_logger().warn(
                        f"Costmap {self.costmap_topic} unavailable/stale (age={costmap_age:.2f}s, "
                        f"limit={self.costmap_stale_sec:.2f}s); falling back to raw map {self.map_topic}."
                    )
                if self._active_map_source != "map":
                    self._active_map_source = "map"
                    self.get_logger().info(f"Using raw map fallback: {self.map_topic}")
                return map_msg
            return None

        if map_msg is not None:
            if self._active_map_source != "map":
                self._active_map_source = "map"
                self.get_logger().info(f"Using raw map source: {self.map_topic}")
            return map_msg

        if costmap_msg is not None:
            if self._active_map_source != "costmap":
                self._active_map_source = "costmap"
                self.get_logger().info(f"Using costmap source: {self.costmap_topic}")
            return costmap_msg

        return None

    def _tick(self) -> None:
        if self.last_odom is None:
            return

        now = self.get_clock().now()
        if self._start_time is None:
            self._start_time = now
        if (now - self._start_time).nanoseconds / 1e9 < self.startup_delay:
            return

        map_msg = self._get_active_map(now.nanoseconds)
        if map_msg is None:
            return
        odom_msg = self.last_odom

        if self.max_map_odom_dt > 0.0:
            map_t = Time.from_msg(map_msg.header.stamp).nanoseconds
            odom_t = Time.from_msg(odom_msg.header.stamp).nanoseconds
            if map_t > 0 and odom_t > 0:
                dt = abs(map_t - odom_t) / 1e9
                if dt > self.max_map_odom_dt:
                    if now.nanoseconds - self._last_sync_warn_ns > int(2e9):
                        self.get_logger().warn(
                            f"map/odom desync: source={self._active_map_source} dt={dt:.3f}s > "
                            f"{self.max_map_odom_dt:.3f}s (map_t={map_t / 1e9:.3f}, odom_t={odom_t / 1e9:.3f}); "
                            "skipping frontier update"
                        )
                        self._last_sync_warn_ns = now.nanoseconds
                    return

        robot_x = float(odom_msg.pose.pose.position.x)
        robot_y = float(odom_msg.pose.pose.position.y)
        frame_id = map_msg.header.frame_id or self.map_frame

        frontier_cells = self._extract_frontier_cells(map_msg)
        clusters = self._cluster_frontiers(frontier_cells)
        candidates = self._build_candidates(map_msg, clusters, robot_x, robot_y)

        if not candidates:
            self._publish_regions_markers(map_msg, [])
            return

        ranked = self._rank_candidates(candidates)

        # Keep the current goal while still far from it unless controller asked for replan.
        if self.current_goal is not None and not self.force_replan:
            dist_to_current = math.hypot(self.current_goal[0] - robot_x, self.current_goal[1] - robot_y)
            if dist_to_current > self.goal_reselect_distance:
                self._publish_goal(frame_id, map_msg.header.stamp, self.current_goal[0], self.current_goal[1])
                self._publish_goal_marker(frame_id, self.current_goal[0], self.current_goal[1])
                self._publish_regions_markers(map_msg, clusters)
                return

        chosen: Optional[FrontierCandidate] = None
        for cand in ranked:
            if self.current_goal is None:
                chosen = cand
                break
            if self.force_replan:
                chosen = cand
                break
            # Goal hysteresis: avoid tiny target hops that create cmd_vel oscillation.
            if math.hypot(cand.world_x - self.current_goal[0], cand.world_y - self.current_goal[1]) >= self.goal_min_separation:
                chosen = cand
                break

        if chosen is None:
            chosen = ranked[0]

        self.current_goal = (chosen.world_x, chosen.world_y)
        self.force_replan = False

        self._publish_goal(frame_id, map_msg.header.stamp, chosen.world_x, chosen.world_y)
        self._publish_goal_marker(frame_id, chosen.world_x, chosen.world_y)
        self._publish_regions_markers(map_msg, clusters)

        if self._last_summary_ns == 0 or (now.nanoseconds - self._last_summary_ns) > int(10e9):
            self._last_summary_ns = now.nanoseconds
            self.get_logger().info(
                f"FRONTIER step: cells={len(frontier_cells)} clusters={len(clusters)} candidates={len(candidates)} "
                f"goal=({chosen.world_x:.2f},{chosen.world_y:.2f})"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimpleFrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
