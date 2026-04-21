#!/usr/bin/env python3
"""FAR planner → nav_status/v1 adapter.

FAR (CMU autonomy stack) doesn't speak the canonical `/nav_status` schema
directly. This node observes FAR's native outputs (/far_reach_goal_status,
/way_point, /goal_point, /far_planning_time) and emits the same
`nav_status/v1` JSON that reactive_nav + default_nav produce, so CFPA2 can
fast-blacklist unreachable goals regardless of which planner is running.

See docs/claude/nav_status_contract.md for the schema and states.

State inference (5 Hz timer):

  no /goal_point yet                        → state=idle,         reason=no_goal
  /far_reach_goal_status became True        → state=goal_reached, reason=reached
  /way_point arrived < way_point_timeout   → state=navigating,   reason=routing
  /goal_point held > unreachable_timeout
    with no /way_point in that window AND
    FAR heartbeat alive                     → state=unreachable,  reason=no_route
  no FAR heartbeat > far_heartbeat_timeout → state=failed,       reason=far_silent

`goal_seq` increments on every new /goal_point that moves more than 0.3 m
from the previous. That gives CFPA2 a stable handle to dedup fast-blacklist
decisions when it later re-picks the same coordinates.
"""
from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class FarStatusAdapter(Node):
    def __init__(self) -> None:
        super().__init__("far_status_adapter")

        self.declare_parameter("way_point_timeout_sec", 2.0)
        self.declare_parameter("unreachable_timeout_sec", 3.0)
        self.declare_parameter("far_heartbeat_timeout_sec", 5.0)
        self.declare_parameter("publish_rate_hz", 5.0)

        self.way_point_timeout = float(self.get_parameter("way_point_timeout_sec").value)
        self.unreachable_timeout = float(self.get_parameter("unreachable_timeout_sec").value)
        self.heartbeat_timeout = float(self.get_parameter("far_heartbeat_timeout_sec").value)
        rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))

        # Latest samples
        self.goal_xy: tuple[float, float] | None = None
        self.goal_seq: int = 0
        self.goal_rx_sec: float | None = None
        self.waypoint_rx_sec: float | None = None
        self.heartbeat_rx_sec: float | None = None
        self.reach_status: bool = False
        self.reach_rx_sec: float | None = None
        self.last_reach_transition_to_true_sec: float | None = None
        self.robot_xy: tuple[float, float] = (0.0, 0.0)

        # Subscribers — use absolute topics; launch maps into the namespace.
        self.create_subscription(PointStamped, "/goal_point", self._goal_cb, 10)
        self.create_subscription(PointStamped, "/way_point", self._wp_cb, 10)
        self.create_subscription(Bool, "/far_reach_goal_status", self._reach_cb, 10)
        self.create_subscription(Float32, "/far_planning_time", self._heartbeat_cb, 10)
        self.create_subscription(Odometry, "/odom/ground_truth", self._odom_cb, 10)

        self.status_pub = self.create_publisher(String, "/nav_status", 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f"FAR status adapter armed. Emitting nav_status/v1 at {rate_hz:.1f} Hz. "
            f"Unreachable after {self.unreachable_timeout:.1f}s without /way_point."
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    # ── callbacks ─────────────────────────────────────────────────────
    def _goal_cb(self, msg: PointStamped) -> None:
        x, y = float(msg.point.x), float(msg.point.y)
        if self.goal_xy is None or math.hypot(x - self.goal_xy[0], y - self.goal_xy[1]) > 0.3:
            self.goal_seq += 1
            self.last_reach_transition_to_true_sec = None   # new goal → reset reached latch
        self.goal_xy = (x, y)
        self.goal_rx_sec = self._now()

    def _wp_cb(self, _msg: PointStamped) -> None:
        self.waypoint_rx_sec = self._now()

    def _reach_cb(self, msg: Bool) -> None:
        now_sec = self._now()
        if bool(msg.data) and not self.reach_status:
            self.last_reach_transition_to_true_sec = now_sec
        self.reach_status = bool(msg.data)
        self.reach_rx_sec = now_sec

    def _heartbeat_cb(self, _msg: Float32) -> None:
        self.heartbeat_rx_sec = self._now()

    def _odom_cb(self, msg: Odometry) -> None:
        self.robot_xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))

    # ── state inference + publish ─────────────────────────────────────
    def _infer_state(self, now_sec: float) -> tuple[str, str]:
        if self.goal_xy is None or self.goal_rx_sec is None:
            return "idle", "no_goal"
        # FAR heartbeat check takes priority — if FAR is silent, everything
        # downstream is meaningless.
        if self.heartbeat_rx_sec is None:
            hb_age = None
        else:
            hb_age = now_sec - self.heartbeat_rx_sec
        if hb_age is None or hb_age > self.heartbeat_timeout:
            return "failed", "far_silent"
        # reach latch — once true, stay goal_reached until the next new goal
        if self.reach_status and self.last_reach_transition_to_true_sec is not None:
            return "goal_reached", "reached"
        # /way_point freshness — if FAR is still routing, we're navigating
        if self.waypoint_rx_sec is not None and (now_sec - self.waypoint_rx_sec) <= self.way_point_timeout:
            return "navigating", "routing"
        # goal held but no /way_point in the unreachable window → FAR's V-graph is disconnected
        goal_held_sec = now_sec - self.goal_rx_sec
        if goal_held_sec > self.unreachable_timeout:
            return "unreachable", "no_route"
        # New goal, FAR hasn't emitted a way_point yet — still give it time
        return "navigating", "warming_up"

    def _tick(self) -> None:
        now_sec = self._now()
        state, reason = self._infer_state(now_sec)

        payload: dict = {
            "schema": "nav_status/v1",
            "source": "far_adapter",
            "state": state,
            "goal_seq": self.goal_seq,
            "reason": reason,
            "stamp_sec": round(now_sec, 3),
        }
        if self.goal_xy is None:
            payload["goal"] = None
        else:
            payload["goal"] = [round(self.goal_xy[0], 3), round(self.goal_xy[1], 3)]
            payload["dist_goal_live"] = round(
                math.hypot(self.goal_xy[0] - self.robot_xy[0], self.goal_xy[1] - self.robot_xy[1]), 3
            )
        payload["pos"] = [round(self.robot_xy[0], 3), round(self.robot_xy[1], 3)]

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FarStatusAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
