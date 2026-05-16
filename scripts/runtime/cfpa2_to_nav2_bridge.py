#!/usr/bin/env python3
"""cfpa2_to_nav2_bridge — translate CFPA2 way_point into Nav2 goal_pose.

CFPA2 publishes /<ns>/way_point as PointStamped (just an XY target with
no orientation). Nav2's bt_navigator subscribes to /<ns>/goal_pose as
PoseStamped (full pose with orientation). This bridge:

  - subscribes /<ns>/way_point (RELIABLE to match CFPA2's QoS)
  - synthesizes orientation = atan2(goal - robot_pose) so the planner
    has a sensible terminal heading
  - publishes /<ns>/goal_pose (RELIABLE to match Nav2's QoS) only
    when the goal *changed* — re-publishing identical goals at 2 Hz
    would force Nav2 to abort + replan every tick
  - converts repeated Nav2 BT planner failures into /<ns>/nav_status
    messages so CFPA2 can blacklist unreachable frontiers quickly

Run alongside:
  - the sim (any backend including 'none')
  - cfpa2_coordinator (with explore:=true)
  - nav2_robot_a.launch.py (the Nav2 stack)
"""
from __future__ import annotations

import json
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.msg import BehaviorTreeLog
from std_msgs.msg import String


def _split_ros_argv(argv):
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        return argv[:i], argv[i:]
    return argv, []


def _ns_topic(ns: str, topic: str) -> str:
    topic = str(topic)
    if topic.startswith("/"):
        return topic
    return f"/{ns}/{topic}" if ns else f"/{topic}"


class Cfpa2ToNav2Bridge(Node):
    def __init__(self) -> None:
        super().__init__("cfpa2_to_nav2_bridge")
        self.declare_parameter("namespace", "robot_a")
        self.declare_parameter("waypoint_topic", "way_point")
        self.declare_parameter("goal_pose_topic", "goal_pose")
        self.declare_parameter("odom_topic", "odom/nav")
        self.declare_parameter("nav_status_topic", "nav_status")
        self.declare_parameter("behavior_tree_log_topic", "behavior_tree_log")
        # Skip republishing if new goal is within this distance of last
        # published goal — CFPA2 republishes its current goal at 2 Hz to
        # keep the channel alive, we don't want Nav2 to restart every tick.
        self.declare_parameter("goal_change_min_m", 0.30)
        # Nav2 can keep the NavigateToPose action RUNNING while the BT's
        # planner child is repeatedly failing and running recoveries. Convert
        # those repeated planner failures into CFPA2's existing nav_status/v1
        # contract so frontier blacklisting happens at the task layer.
        self.declare_parameter("planner_failure_threshold", 3)
        self.declare_parameter(
            "planner_failure_node_names",
            ["ComputePathToPose", "ComputePathThroughPoses"],
        )

        ns = str(self.get_parameter("namespace").value)
        wp_topic = _ns_topic(ns, self.get_parameter("waypoint_topic").value)
        goal_topic = _ns_topic(ns, self.get_parameter("goal_pose_topic").value)
        odom_topic = _ns_topic(ns, self.get_parameter("odom_topic").value)
        nav_status_topic = _ns_topic(ns, self.get_parameter("nav_status_topic").value)
        bt_log_topic = _ns_topic(
            ns, self.get_parameter("behavior_tree_log_topic").value
        )
        self.goal_change_min_m = float(
            self.get_parameter("goal_change_min_m").value
        )
        self._planner_failure_threshold = max(
            1, int(self.get_parameter("planner_failure_threshold").value)
        )
        failure_nodes = self.get_parameter("planner_failure_node_names").value
        if isinstance(failure_nodes, str):
            failure_nodes = [failure_nodes]
        self._planner_failure_node_names = {str(v) for v in failure_nodes}

        # CFPA2's way_point_coord publishes RELIABLE; odom_relay also reliable.
        cfpa_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # Nav2's bt_navigator subscribes goal_pose with rclcpp::SystemDefaultsQoS,
        # which is RELIABLE/VOLATILE/KEEP_LAST(10). Earlier comment claiming
        # BEST_EFFORT was wrong — published goals were silently dropped on the
        # wire (publisher logs "forwarded goal" but bt_navigator never received
        # them). Confirmed via the runtime warning:
        #   "New publisher discovered on '/<ns>/goal_pose'... incompatible
        #    QoS. Last incompatible policy: RELIABILITY".
        nav2_goal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._last_pose_x: float | None = None
        self._last_pose_y: float | None = None
        self._last_goal_x: float | None = None
        self._last_goal_y: float | None = None
        self._goal_seq = 0
        self._active_goal: tuple[float, float] | None = None
        self._planner_failure_count = 0

        self.create_subscription(Odometry, odom_topic, self._on_odom, cfpa_qos)
        self.create_subscription(
            PointStamped, wp_topic, self._on_waypoint, cfpa_qos
        )
        self.create_subscription(
            BehaviorTreeLog, bt_log_topic, self._on_behavior_tree_log, 10
        )
        self._goal_pub = self.create_publisher(
            PoseStamped, goal_topic, nav2_goal_qos
        )
        self._nav_status_pub = self.create_publisher(String, nav_status_topic, 10)

        self.get_logger().info(
            f"bridge armed. {wp_topic} → {goal_topic}; pose from {odom_topic}; "
            f"nav_status on {nav_status_topic}; BT log from {bt_log_topic}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._last_pose_x = msg.pose.pose.position.x
        self._last_pose_y = msg.pose.pose.position.y

    def _emit_nav_status(
        self,
        state: str,
        reason: str,
        *,
        extra: dict | None = None,
    ) -> None:
        goal = getattr(self, "_active_goal", None)
        payload = {
            "schema": "nav_status/v1",
            "source": "cfpa2_to_nav2_bridge",
            "state": state,
            "reason": reason,
            "stamp_ns": int(self.get_clock().now().nanoseconds),
            "goal_seq": int(getattr(self, "_goal_seq", 0)),
        }
        if goal is not None:
            payload["goal"] = [float(goal[0]), float(goal[1])]
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._nav_status_pub.publish(msg)

    def _on_waypoint(self, msg: PointStamped) -> None:
        gx, gy = float(msg.point.x), float(msg.point.y)
        # Suppress duplicate / sub-threshold-change goals.
        if (
            self._last_goal_x is not None
            and math.hypot(gx - self._last_goal_x, gy - self._last_goal_y)
            < self.goal_change_min_m
        ):
            return

        # Synthesize orientation pointing from current robot pose to goal.
        # If we have no odom yet, default to facing +x (yaw=0).
        yaw = 0.0
        if self._last_pose_x is not None:
            dx = gx - self._last_pose_x
            dy = gy - self._last_pose_y
            if math.hypot(dx, dy) > 0.05:
                yaw = math.atan2(dy, dx)

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id or "map"
        out.pose.position.x = gx
        out.pose.position.y = gy
        out.pose.position.z = 0.0
        # quaternion from yaw alone (z-axis rotation).
        out.pose.orientation.z = math.sin(0.5 * yaw)
        out.pose.orientation.w = math.cos(0.5 * yaw)
        self._goal_pub.publish(out)

        self.get_logger().info(
            f"forwarded goal ({gx:+.2f}, {gy:+.2f}) yaw={math.degrees(yaw):+.1f}°"
        )
        self._last_goal_x = gx
        self._last_goal_y = gy
        self._goal_seq += 1
        self._active_goal = (gx, gy)
        self._planner_failure_count = 0
        self._emit_nav_status("navigating", "goal_forwarded")

    def _on_behavior_tree_log(self, msg: BehaviorTreeLog) -> None:
        if getattr(self, "_active_goal", None) is None:
            return

        failed_node_name: str | None = None
        for event in msg.event_log:
            node_name = str(event.node_name)
            if node_name not in self._planner_failure_node_names:
                continue

            current = str(event.current_status).upper()
            previous = str(event.previous_status).upper()
            if current == "SUCCESS":
                self._planner_failure_count = 0
                continue
            if previous == "RUNNING" and current == "FAILURE":
                failed_node_name = failed_node_name or node_name

        if failed_node_name is None:
            return

        self._planner_failure_count += 1
        if self._planner_failure_count < self._planner_failure_threshold:
            return

        # Keep publishing after the bridge-side threshold. CFPA2 applies its
        # own consecutive-message debounce keyed by goal_seq before it
        # blacklists, so a single terminal-looking message is intentionally
        # not enough for the task layer to act.
        self._emit_nav_status(
            "unreachable",
            "bt_compute_path_failure",
            extra={
                "bt_node": failed_node_name,
                "failure_count": int(self._planner_failure_count),
            },
        )


def main(argv=None) -> int:
    _, ros_argv = _split_ros_argv(argv if argv else sys.argv[1:])
    rclpy.init(args=([sys.argv[0]] + ros_argv) if ros_argv else None)
    try:
        node = Cfpa2ToNav2Bridge()
        rclpy.spin(node)
    finally:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
