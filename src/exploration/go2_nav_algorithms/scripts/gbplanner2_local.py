#!/usr/bin/env python3
"""GBPlanner2-style exploration baseline (2D adaptation).

Local planner:  RRG sampling in free space + unknown-cell gain scoring.
Global planner: persistent graph of visited waypoints; reposition to
                best frontier when local exploration stalls.

Reference: Kulkarni et al., "Autonomous Teamed Exploration of Subterranean
Environments using Legged and Aerial Robots", ICRA 2022.
"""

from __future__ import annotations

import math
import random
import time
from collections import deque
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------------------------------------------------------------
# Occupancy grid helpers
# ---------------------------------------------------------------------------

FREE = 0
UNKNOWN = -1
OCC_THRESH = 50


def _grid_idx(gx: int, gy: int, w: int) -> int:
    return gy * w + gx


def _world_to_grid(msg: OccupancyGrid, wx: float, wy: float) -> Optional[tuple[int, int]]:
    gx = int((wx - msg.info.origin.position.x) / msg.info.resolution)
    gy = int((wy - msg.info.origin.position.y) / msg.info.resolution)
    if 0 <= gx < msg.info.width and 0 <= gy < msg.info.height:
        return (gx, gy)
    return None


def _grid_to_world(msg: OccupancyGrid, gx: int, gy: int) -> tuple[float, float]:
    return (
        msg.info.origin.position.x + (gx + 0.5) * msg.info.resolution,
        msg.info.origin.position.y + (gy + 0.5) * msg.info.resolution,
    )


def _is_free(data, idx: int) -> bool:
    return data[idx] == FREE


def _is_unknown(data, idx: int) -> bool:
    return data[idx] == UNKNOWN


def _is_occupied(data, idx: int) -> bool:
    v = data[idx]
    return v >= OCC_THRESH


def _bresenham_collision(
    data, w: int, h: int, gx0: int, gy0: int, gx1: int, gy1: int
) -> bool:
    """Return True if Bresenham line from (gx0,gy0) to (gx1,gy1) hits occupied."""
    dx = abs(gx1 - gx0)
    dy = abs(gy1 - gy0)
    sx = 1 if gx0 < gx1 else -1
    sy = 1 if gy0 < gy1 else -1
    err = dx - dy
    cx, cy = gx0, gy0
    while True:
        if cx < 0 or cy < 0 or cx >= w or cy >= h:
            return True
        if _is_occupied(data, _grid_idx(cx, cy, w)):
            return True
        if cx == gx1 and cy == gy1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            cx += sx
        if e2 < dx:
            err += dx
            cy += sy
    return False


# ---------------------------------------------------------------------------
# RRG (Rapidly-exploring Random Graph)
# ---------------------------------------------------------------------------

class RRGVertex:
    __slots__ = ("x", "y", "gain", "neighbors", "parent", "cost")

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.gain = 0.0
        self.neighbors: list[int] = []
        self.parent: int = -1
        self.cost: float = 0.0


def _build_rrg(
    map_msg: OccupancyGrid,
    robot_x: float,
    robot_y: float,
    local_bound_m: float,
    n_samples: int,
    connect_radius_m: float,
) -> list[RRGVertex]:
    """Build a local RRG in free space around the robot."""
    data = map_msg.data
    w = int(map_msg.info.width)
    h = int(map_msg.info.height)
    res = map_msg.info.resolution

    vertices = [RRGVertex(robot_x, robot_y)]

    for _ in range(n_samples):
        # Sample uniformly in local bounding box
        sx = robot_x + random.uniform(-local_bound_m, local_bound_m)
        sy = robot_y + random.uniform(-local_bound_m, local_bound_m)

        g = _world_to_grid(map_msg, sx, sy)
        if g is None:
            continue
        gx, gy = g
        idx = _grid_idx(gx, gy, w)
        if not _is_free(data, idx):
            continue

        # Find nearest existing vertex
        best_dist = float("inf")
        best_idx = -1
        for vi, v in enumerate(vertices):
            d = math.hypot(sx - v.x, sy - v.y)
            if d < best_dist:
                best_dist = d
                best_idx = vi

        if best_idx < 0 or best_dist > connect_radius_m * 2.0:
            continue

        # Check collision on edge
        v_near = vertices[best_idx]
        gn = _world_to_grid(map_msg, v_near.x, v_near.y)
        if gn is None:
            continue
        if _bresenham_collision(data, w, h, gn[0], gn[1], gx, gy):
            continue

        # Add vertex
        new_idx = len(vertices)
        new_v = RRGVertex(sx, sy)
        new_v.neighbors.append(best_idx)
        new_v.parent = best_idx
        new_v.cost = v_near.cost + best_dist
        vertices[best_idx].neighbors.append(new_idx)
        vertices.append(new_v)

        # Connect to other nearby vertices (RRG, not just RRT)
        for vi, v in enumerate(vertices[:-1]):
            if vi == best_idx:
                continue
            d = math.hypot(sx - v.x, sy - v.y)
            if d > connect_radius_m:
                continue
            gv = _world_to_grid(map_msg, v.x, v.y)
            if gv is None:
                continue
            if not _bresenham_collision(data, w, h, gv[0], gv[1], gx, gy):
                new_v.neighbors.append(vi)
                v.neighbors.append(new_idx)

    return vertices


def _evaluate_gain(
    map_msg: OccupancyGrid,
    vertices: list[RRGVertex],
    sensor_range_m: float,
) -> None:
    """Count unknown cells within sensor_range of each vertex (2D raycasting)."""
    data = map_msg.data
    w = int(map_msg.info.width)
    h = int(map_msg.info.height)
    res = map_msg.info.resolution
    range_cells = int(sensor_range_m / res)

    for v in vertices:
        g = _world_to_grid(map_msg, v.x, v.y)
        if g is None:
            v.gain = 0.0
            continue
        gx, gy = g
        count = 0
        # Count unknown cells in a square region (fast approximation)
        for dy in range(-range_cells, range_cells + 1):
            ny = gy + dy
            if ny < 0 or ny >= h:
                continue
            row = ny * w
            for dx in range(-range_cells, range_cells + 1):
                nx = gx + dx
                if nx < 0 or nx >= w:
                    continue
                if dx * dx + dy * dy > range_cells * range_cells:
                    continue
                if _is_unknown(data, row + nx):
                    count += 1
        v.gain = float(count)


def _best_gain_path(
    vertices: list[RRGVertex],
    gain_weight: float,
    cost_weight: float,
) -> Optional[int]:
    """Find vertex with best score = gain_weight * gain - cost_weight * cost."""
    if len(vertices) <= 1:
        return None

    best_score = -float("inf")
    best_idx = -1
    for vi in range(1, len(vertices)):  # skip root (robot position)
        v = vertices[vi]
        score = gain_weight * v.gain - cost_weight * v.cost
        if score > best_score:
            best_score = score
            best_idx = vi

    if best_idx < 0 or vertices[best_idx].gain <= 0.0:
        return None
    return best_idx


def _trace_path(vertices: list[RRGVertex], target_idx: int) -> list[int]:
    """Trace path from target back to root."""
    path = []
    idx = target_idx
    while idx >= 0:
        path.append(idx)
        idx = vertices[idx].parent
    path.reverse()
    return path


# ---------------------------------------------------------------------------
# Global graph for repositioning
# ---------------------------------------------------------------------------

class GlobalGraph:
    """Persistent graph of visited positions for long-range repositioning."""

    def __init__(self, merge_resolution: float = 0.5):
        self.positions: list[tuple[float, float]] = []
        self.edges: list[list[int]] = []
        self._merge_res = merge_resolution
        self._keys: set[tuple[int, int]] = set()

    def _key(self, x: float, y: float) -> tuple[int, int]:
        q = self._merge_res
        return (int(round(x / q)), int(round(y / q)))

    def add_position(self, x: float, y: float) -> int:
        key = self._key(x, y)
        if key in self._keys:
            # Find existing
            for i, (px, py) in enumerate(self.positions):
                if self._key(px, py) == key:
                    return i
            # Shouldn't happen, but add anyway
        self._keys.add(key)
        idx = len(self.positions)
        self.positions.append((x, y))
        self.edges.append([])
        return idx

    def add_edge(self, i: int, j: int) -> None:
        if j not in self.edges[i]:
            self.edges[i].append(j)
        if i not in self.edges[j]:
            self.edges[j].append(i)

    def add_path(self, waypoints: list[tuple[float, float]]) -> None:
        prev_idx = -1
        for wx, wy in waypoints:
            idx = self.add_position(wx, wy)
            if prev_idx >= 0 and prev_idx != idx:
                self.add_edge(prev_idx, idx)
            prev_idx = idx

    def nearest(self, x: float, y: float) -> Optional[int]:
        if not self.positions:
            return None
        best_d = float("inf")
        best_i = -1
        for i, (px, py) in enumerate(self.positions):
            d = math.hypot(px - x, py - y)
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def shortest_path_to(
        self, start_idx: int, target_x: float, target_y: float
    ) -> Optional[list[tuple[float, float]]]:
        """BFS to vertex nearest to target, return path as world coords."""
        if not self.positions:
            return None
        target_idx = self.nearest(target_x, target_y)
        if target_idx is None or target_idx == start_idx:
            return None

        # BFS
        visited = {start_idx}
        parent: dict[int, int] = {}
        queue = deque([start_idx])
        while queue:
            curr = queue.popleft()
            if curr == target_idx:
                break
            for nb in self.edges[curr]:
                if nb not in visited:
                    visited.add(nb)
                    parent[nb] = curr
                    queue.append(nb)

        if target_idx not in parent and target_idx != start_idx:
            return None

        # Trace
        path = []
        idx = target_idx
        while idx != start_idx:
            path.append(self.positions[idx])
            idx = parent.get(idx, start_idx)
        path.append(self.positions[start_idx])
        path.reverse()
        return path


# ---------------------------------------------------------------------------
# Frontier extraction (2D)
# ---------------------------------------------------------------------------

def _extract_frontiers(
    map_msg: OccupancyGrid, stride: int = 2, max_targets: int = 500
) -> list[tuple[float, float]]:
    w = int(map_msg.info.width)
    h = int(map_msg.info.height)
    data = map_msg.data
    out: list[tuple[float, float]] = []

    for gy in range(1, h - 1, stride):
        row = gy * w
        for gx in range(1, w - 1, stride):
            idx = row + gx
            if not _is_free(data, idx):
                continue
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nidx = (gy + dy) * w + (gx + dx)
                if _is_unknown(data, nidx):
                    out.append(_grid_to_world(map_msg, gx, gy))
                    if len(out) >= max_targets:
                        return out
                    break
    return out


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class GBPlanner2Local(Node):
    def __init__(self) -> None:
        super().__init__("gbplanner2_local")

        # Parameters
        self.declare_parameter("plan_rate_hz", 2.0)
        self.declare_parameter("local_bound_m", 5.0)
        self.declare_parameter("n_samples", 200)
        self.declare_parameter("connect_radius_m", 1.5)
        self.declare_parameter("sensor_range_m", 3.5)
        self.declare_parameter("min_gain", 5.0)
        self.declare_parameter("gain_weight", 1.0)
        self.declare_parameter("cost_weight", 0.3)
        self.declare_parameter("global_reposition_cooldown_s", 10.0)
        self.declare_parameter("goal_topic", "way_point_coord")
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("odom_topic", "odom/nav")
        self.declare_parameter("startup_delay_s", 12.0)
        self.declare_parameter("global_graph_merge_res", 0.5)
        self.declare_parameter("frontier_stride", 2)
        self.declare_parameter("local_stall_count", 3)

        self.plan_rate = max(0.5, self.get_parameter("plan_rate_hz").value)
        self.local_bound_m = max(1.0, self.get_parameter("local_bound_m").value)
        self.n_samples = max(20, int(self.get_parameter("n_samples").value))
        self.connect_radius_m = max(0.3, self.get_parameter("connect_radius_m").value)
        self.sensor_range_m = max(0.5, self.get_parameter("sensor_range_m").value)
        self.min_gain = max(1.0, self.get_parameter("min_gain").value)
        self.gain_weight = self.get_parameter("gain_weight").value
        self.cost_weight = max(0.0, self.get_parameter("cost_weight").value)
        self.global_cooldown_s = max(1.0, self.get_parameter("global_reposition_cooldown_s").value)
        self.startup_delay_s = max(0.0, self.get_parameter("startup_delay_s").value)
        self.global_merge_res = max(0.1, self.get_parameter("global_graph_merge_res").value)
        self.frontier_stride = max(1, int(self.get_parameter("frontier_stride").value))
        self.local_stall_count = max(1, int(self.get_parameter("local_stall_count").value))

        goal_topic = self.get_parameter("goal_topic").value
        map_topic = self.get_parameter("map_topic").value
        odom_topic = self.get_parameter("odom_topic").value

        # State
        self.map_msg: Optional[OccupancyGrid] = None
        self.odom_msg: Optional[Odometry] = None
        self.global_graph = GlobalGraph(merge_resolution=self.global_merge_res)
        self._last_global_reposition_ns = 0
        self._consecutive_local_stalls = 0
        self._start_ns = self.get_clock().now().nanoseconds
        self._last_goal: Optional[tuple[float, float]] = None
        self._last_rrg_vertices: list[RRGVertex] = []

        # Subscribers
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, map_qos)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

        # Publishers
        self.goal_pub = self.create_publisher(PointStamped, goal_topic, 10)
        self.rrg_marker_pub = self.create_publisher(MarkerArray, "gbplanner2/rrg_markers", 10)
        self.status_marker_pub = self.create_publisher(Marker, "gbplanner2/status", 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.plan_rate, self._plan_tick)

        self.get_logger().info(
            f"GBPlanner2 local started: bound={self.local_bound_m}m "
            f"samples={self.n_samples} connect_r={self.connect_radius_m}m "
            f"sensor={self.sensor_range_m}m min_gain={self.min_gain} "
            f"gain_w={self.gain_weight} cost_w={self.cost_weight} "
            f"global_cooldown={self.global_cooldown_s}s "
            f"stall_count={self.local_stall_count}"
        )

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom_msg = msg

    def _robot_xy(self) -> tuple[float, float]:
        od = self.odom_msg
        return (float(od.pose.pose.position.x), float(od.pose.pose.position.y))

    def _plan_tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds

        # Startup delay
        if (now_ns - self._start_ns) / 1e9 < self.startup_delay_s:
            return

        if self.map_msg is None or self.odom_msg is None:
            return

        rx, ry = self._robot_xy()

        # Record robot position in global graph
        self.global_graph.add_position(rx, ry)

        t0 = time.perf_counter()

        # --- Local planning: build RRG and evaluate gain ---
        vertices = _build_rrg(
            self.map_msg, rx, ry,
            self.local_bound_m, self.n_samples, self.connect_radius_m,
        )
        _evaluate_gain(self.map_msg, vertices, self.sensor_range_m)
        self._last_rrg_vertices = vertices

        best_idx = _best_gain_path(vertices, self.gain_weight, self.cost_weight)

        if best_idx is not None and vertices[best_idx].gain >= self.min_gain:
            # Local exploration: follow best-gain path
            self._consecutive_local_stalls = 0
            path_indices = _trace_path(vertices, best_idx)

            # Pick the second vertex in the path as immediate waypoint
            # (first is robot position)
            wp_idx = path_indices[1] if len(path_indices) > 1 else path_indices[0]
            goal = (vertices[wp_idx].x, vertices[wp_idx].y)

            # Record path in global graph
            path_xy = [(vertices[i].x, vertices[i].y) for i in path_indices]
            self.global_graph.add_path(path_xy)

            self._publish_goal(goal)
            self._last_goal = goal

            dt_ms = (time.perf_counter() - t0) * 1000.0
            self.get_logger().info(
                f"LOCAL: goal=({goal[0]:.2f},{goal[1]:.2f}) "
                f"gain={vertices[best_idx].gain:.0f} "
                f"cost={vertices[best_idx].cost:.2f} "
                f"rrg_vertices={len(vertices)} dt={dt_ms:.1f}ms"
            )
        else:
            # Local stall — try global repositioning
            self._consecutive_local_stalls += 1

            if self._consecutive_local_stalls < self.local_stall_count:
                # Not stalled enough yet, retry local
                return

            elapsed_since_global = (now_ns - self._last_global_reposition_ns) / 1e9
            if elapsed_since_global < self.global_cooldown_s:
                return

            frontiers = _extract_frontiers(
                self.map_msg, stride=self.frontier_stride
            )
            if not frontiers:
                self.get_logger().warn("GLOBAL: no frontiers found, exploration may be complete")
                return

            # Find nearest frontier that's far enough away
            best_frontier = None
            best_dist = float("inf")
            for fx, fy in frontiers:
                d = math.hypot(fx - rx, fy - ry)
                if d < 1.0:
                    continue  # too close, already near this frontier
                if d < best_dist:
                    best_dist = d
                    best_frontier = (fx, fy)

            if best_frontier is None:
                self.get_logger().warn("GLOBAL: no reachable frontiers beyond 1m")
                return

            self._last_global_reposition_ns = now_ns
            self._consecutive_local_stalls = 0
            self._publish_goal(best_frontier)
            self._last_goal = best_frontier

            dt_ms = (time.perf_counter() - t0) * 1000.0
            self.get_logger().info(
                f"GLOBAL REPOSITION: goal=({best_frontier[0]:.2f},{best_frontier[1]:.2f}) "
                f"dist={best_dist:.2f}m frontiers={len(frontiers)} dt={dt_ms:.1f}ms"
            )

        self._publish_rrg_markers(vertices)

    def _publish_goal(self, goal: tuple[float, float]) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.point.x = goal[0]
        msg.point.y = goal[1]
        msg.point.z = 0.0
        self.goal_pub.publish(msg)

    def _publish_rrg_markers(self, vertices: list[RRGVertex]) -> None:
        """Publish RRG as visualization markers for RViz."""
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Vertices as points
        pts_marker = Marker()
        pts_marker.header.stamp = stamp
        pts_marker.header.frame_id = "world"
        pts_marker.ns = "gbp2_rrg_vertices"
        pts_marker.id = 0
        pts_marker.type = Marker.POINTS
        pts_marker.action = Marker.ADD
        pts_marker.pose.orientation.w = 1.0
        pts_marker.scale.x = 0.06
        pts_marker.scale.y = 0.06

        # Edges as lines
        edge_marker = Marker()
        edge_marker.header.stamp = stamp
        edge_marker.header.frame_id = "world"
        edge_marker.ns = "gbp2_rrg_edges"
        edge_marker.id = 1
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.pose.orientation.w = 1.0
        edge_marker.scale.x = 0.02
        edge_marker.color.a = 0.4
        edge_marker.color.r = 0.3
        edge_marker.color.g = 0.7
        edge_marker.color.b = 1.0

        max_gain = max((v.gain for v in vertices), default=1.0) or 1.0
        for vi, v in enumerate(vertices):
            from geometry_msgs.msg import Point
            p = Point()
            p.x = v.x
            p.y = v.y
            p.z = 0.05

            # Color by gain: low=blue, high=red
            frac = min(1.0, v.gain / max_gain)
            from std_msgs.msg import ColorRGBA
            c = ColorRGBA()
            c.a = 0.8
            c.r = frac
            c.g = 0.2
            c.b = 1.0 - frac
            pts_marker.points.append(p)
            pts_marker.colors.append(c)

            # Edges
            for ni in v.neighbors:
                if ni > vi:  # avoid duplicate edges
                    p2 = Point()
                    p2.x = vertices[ni].x
                    p2.y = vertices[ni].y
                    p2.z = 0.05
                    edge_marker.points.append(p)
                    edge_marker.points.append(p2)

        markers.markers.append(pts_marker)
        markers.markers.append(edge_marker)
        self.rrg_marker_pub.publish(markers)


def main():
    rclpy.init()
    node = GBPlanner2Local()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
