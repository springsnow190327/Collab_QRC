#!/usr/bin/env python3
"""ROS2 MTARE common-executor fallback planner.

Upstream MTARE is ROS1/Noetic and can be run through ``mtare_external_cmd``.
When that stack is not available, this node provides a runnable autonomous
multi-robot exploration baseline under the same common-executor contract:
it reads the shared occupancy map, assigns frontier goals to robot_a/robot_b,
and publishes PointStamped waypoints for the existing Nav2 MPPI executor.

The node never emits scripted routes or hardcoded waypoint sequences; every
goal is recomputed from current map/frontier state and robot poses.
"""

from __future__ import annotations

import os
import sys
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mtare_common_executor_core import assign_frontiers, extract_frontier_clusters  # noqa: E402


class MTARECommonExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("mtare_common_executor")
        self.declare_parameter("namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("mobility_types", ["wheeled", "wheeled"])
        self.declare_parameter("shared_map_topic", "/merged_map")
        self.declare_parameter("waypoint_topic_suffix", "/mtare/way_point")
        self.declare_parameter("replan_period_sec", 4.0)
        self.declare_parameter("min_cluster_size", 8)
        self.declare_parameter("min_peer_goal_separation", 2.0)
        self.declare_parameter("min_goal_change_m", 0.6)
        self.declare_parameter("free_threshold", 20)

        self._namespaces = [
            str(ns).strip().strip("/")
            for ns in self.get_parameter("namespaces").value
            if str(ns).strip().strip("/")
        ]
        self._mobility_types = [str(v).strip() for v in self.get_parameter("mobility_types").value]
        self._waypoint_suffix = str(self.get_parameter("waypoint_topic_suffix").value)
        self._replan_period = float(self.get_parameter("replan_period_sec").value)
        self._min_cluster_size = int(self.get_parameter("min_cluster_size").value)
        self._min_peer_sep = float(self.get_parameter("min_peer_goal_separation").value)
        self._min_goal_change = float(self.get_parameter("min_goal_change_m").value)
        self._free_threshold = int(self.get_parameter("free_threshold").value)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._map: Optional[OccupancyGrid] = None
        self._poses: dict[str, tuple[float, float]] = {}
        self._last_goals: dict[str, tuple[float, float]] = {}
        self._pubs = {
            ns: self.create_publisher(PointStamped, f"/{ns}{self._waypoint_suffix}", qos)
            for ns in self._namespaces
        }

        shared_map_topic = str(self.get_parameter("shared_map_topic").value)
        self.create_subscription(OccupancyGrid, shared_map_topic, self._map_cb, 1)
        for ns in self._namespaces:
            self.create_subscription(
                Odometry,
                f"/{ns}/odom/nav",
                lambda msg, n=ns: self._odom_cb(msg, n),
                10,
            )
        self.create_timer(self._replan_period, self._tick)
        self.get_logger().info(
            f"MTARE common-executor fallback started: map={shared_map_topic}, "
            f"robots={self._namespaces}, mobility_types={self._mobility_types}, "
            f"output_suffix={self._waypoint_suffix}")

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _odom_cb(self, msg: Odometry, ns: str) -> None:
        p = msg.pose.pose.position
        self._poses[ns] = (float(p.x), float(p.y))

    def _tick(self) -> None:
        if self._map is None:
            self.get_logger().debug("waiting for shared map")
            return
        missing = [ns for ns in self._namespaces if ns not in self._poses]
        if missing:
            self.get_logger().debug(f"waiting for odom from {missing}")
            return

        info = self._map.info
        clusters = extract_frontier_clusters(
            data=self._map.data,
            width=info.width,
            height=info.height,
            resolution=info.resolution,
            origin_x=info.origin.position.x,
            origin_y=info.origin.position.y,
            min_cluster_size=self._min_cluster_size,
            free_threshold=self._free_threshold,
        )
        assignments = assign_frontiers(
            clusters=clusters,
            robot_positions={ns: self._poses[ns] for ns in self._namespaces},
            previous_goals=self._last_goals,
            min_peer_goal_separation=self._min_peer_sep,
        )
        if not assignments:
            self.get_logger().warn("no reachable frontier candidates from shared map")
            return

        for ns, xy in assignments.items():
            prev = self._last_goals.get(ns)
            if prev is not None:
                dx = xy[0] - prev[0]
                dy = xy[1] - prev[1]
                if (dx * dx + dy * dy) ** 0.5 < self._min_goal_change:
                    continue
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._map.header.frame_id or "map"
            msg.point.x = xy[0]
            msg.point.y = xy[1]
            msg.point.z = 0.0
            self._pubs[ns].publish(msg)
            self._last_goals[ns] = xy
            self.get_logger().info(
                f"{ns} frontier goal -> ({xy[0]:+.2f}, {xy[1]:+.2f}) "
                f"clusters={len(clusters)}")


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = MTARECommonExecutorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
