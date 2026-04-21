#!/usr/bin/env python3
"""FAR Planner diagnostic monitor.

Subscribes to the key FAR planner + local_planner + pathFollower topics
and prints a compact 1 Hz snapshot of robot state, current goal, FAR's
intermediate waypoint, local path length, and pathFollower cmd_vel.

Usage:
    source install/setup.bash
    python3 scripts/far_monitor.py                     # default ns=robot
    python3 scripts/far_monitor.py --ros-args -p namespace:=robot

Prints go to stdout; paste the output when asking for param tweaks.
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class FarMonitor(Node):
    def __init__(self) -> None:
        super().__init__("far_monitor")
        self.ns = (
            self.declare_parameter("namespace", "robot")
            .get_parameter_value()
            .string_value.strip()
            .strip("/")
            or "robot"
        )

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        # State
        self.last_odom: Odometry | None = None
        self.last_goal: PointStamped | None = None
        self.last_waypoint: PointStamped | None = None
        self.last_local_path: Path | None = None
        self.last_viz_path: Marker | None = None
        self.last_cmd: TwistStamped | None = None
        self.last_reach_goal: bool | None = None
        self.last_planning_ms: float | None = None
        self.last_runtime_ms: float | None = None
        self.last_goal_rx_t: float = 0.0
        self.last_waypoint_rx_t: float = 0.0
        self.last_cmd_rx_t: float = 0.0
        self.last_path_rx_t: float = 0.0

        # Subscriptions
        self.create_subscription(
            Odometry, f"/{self.ns}/odom/nav", self._odom_cb, qos
        )
        self.create_subscription(
            PointStamped, f"/{self.ns}/way_point_coord", self._goal_cb, qos
        )
        self.create_subscription(
            PointStamped, f"/{self.ns}/way_point", self._waypoint_cb, qos
        )
        self.create_subscription(
            Path, f"/{self.ns}/local_path", self._local_path_cb, qos
        )
        self.create_subscription(
            Marker, "/viz_path_topic", self._viz_path_cb, qos
        )
        self.create_subscription(
            TwistStamped, f"/{self.ns}/cmd_vel_stamped", self._cmd_cb, qos
        )
        self.create_subscription(
            Bool, "/far_reach_goal_status", self._reach_cb, qos
        )
        self.create_subscription(
            Float32, f"/{self.ns}/far_planning_time", self._planning_cb, qos
        )
        self.create_subscription(
            Float32, f"/{self.ns}/far_runtime", self._runtime_cb, qos
        )

        self.create_timer(1.0, self._print_snapshot)

        self.get_logger().info(
            f"far_monitor: watching /{self.ns}/{{odom/nav, way_point_coord, "
            f"way_point, local_path, cmd_vel_stamped, far_*}} and /viz_path_topic"
        )

    # ── callbacks ──
    def _odom_cb(self, msg: Odometry) -> None:
        self.last_odom = msg

    def _goal_cb(self, msg: PointStamped) -> None:
        self.last_goal = msg
        self.last_goal_rx_t = time.time()

    def _waypoint_cb(self, msg: PointStamped) -> None:
        self.last_waypoint = msg
        self.last_waypoint_rx_t = time.time()

    def _local_path_cb(self, msg: Path) -> None:
        self.last_local_path = msg
        self.last_path_rx_t = time.time()

    def _viz_path_cb(self, msg: Marker) -> None:
        self.last_viz_path = msg

    def _cmd_cb(self, msg: TwistStamped) -> None:
        self.last_cmd = msg
        self.last_cmd_rx_t = time.time()

    def _reach_cb(self, msg: Bool) -> None:
        self.last_reach_goal = msg.data

    def _planning_cb(self, msg: Float32) -> None:
        self.last_planning_ms = msg.data * 1000.0

    def _runtime_cb(self, msg: Float32) -> None:
        self.last_runtime_ms = msg.data * 1000.0

    # ── printing ──
    def _print_snapshot(self) -> None:
        lines: list[str] = []
        now = time.time()

        # Robot pose
        if self.last_odom is not None:
            p = self.last_odom.pose.pose.position
            q = self.last_odom.pose.pose.orientation
            yaw_deg = math.degrees(_yaw_from_quat(q))
            v = self.last_odom.twist.twist.linear
            w = self.last_odom.twist.twist.angular
            lines.append(
                f"pose=({p.x:+.2f},{p.y:+.2f},{p.z:+.2f}) "
                f"yaw={yaw_deg:+6.1f}° vx={v.x:+.2f} wz={w.z:+.2f}"
            )
        else:
            lines.append("pose=<no odom/nav>")

        # User goal (RViz click, relayed to way_point_coord)
        if self.last_goal is not None:
            g = self.last_goal.point
            age = now - self.last_goal_rx_t
            lines.append(f"goal =({g.x:+.2f},{g.y:+.2f},{g.z:+.2f})  age={age:5.1f}s")
        else:
            lines.append("goal =<none yet — click 2D Goal Pose in RViz>")

        # FAR's intermediate waypoint
        if self.last_waypoint is not None:
            w = self.last_waypoint.point
            age = now - self.last_waypoint_rx_t
            lines.append(f"wpnt =({w.x:+.2f},{w.y:+.2f},{w.z:+.2f})  age={age:5.1f}s")
        else:
            lines.append("wpnt =<far_planner has not published /way_point>")

        # Goal reached?
        if self.last_reach_goal is not None:
            lines.append(
                f"reach_goal={self.last_reach_goal} "
                f"plan_ms={self.last_planning_ms or 0:6.1f} "
                f"run_ms={self.last_runtime_ms or 0:6.1f}"
            )

        # Local planner output
        if self.last_local_path is not None:
            n = len(self.last_local_path.poses)
            age = now - self.last_path_rx_t
            lines.append(f"local_path poses={n:3d}  age={age:5.1f}s")
        else:
            lines.append("local_path=<localPlanner not publishing>")

        # FAR global path marker
        if self.last_viz_path is not None:
            pts = len(self.last_viz_path.points)
            lines.append(f"viz_path marker pts={pts:3d}")
        else:
            lines.append("viz_path=<far has no global path yet>")

        # cmd_vel being sent to pathFollower→hybrid_router
        if self.last_cmd is not None:
            lx = self.last_cmd.twist.linear.x
            az = self.last_cmd.twist.angular.z
            age = now - self.last_cmd_rx_t
            lines.append(f"cmd_vel lx={lx:+.2f} az={az:+.2f}  age={age:5.1f}s")
        else:
            lines.append("cmd_vel=<pathFollower silent>")

        # Distances
        if self.last_odom is not None:
            p = self.last_odom.pose.pose.position
            if self.last_goal is not None:
                dx = self.last_goal.point.x - p.x
                dy = self.last_goal.point.y - p.y
                dist_goal = math.hypot(dx, dy)
                lines.append(f"dist_to_goal={dist_goal:.2f}m")
            if self.last_waypoint is not None:
                dx = self.last_waypoint.point.x - p.x
                dy = self.last_waypoint.point.y - p.y
                dist_wp = math.hypot(dx, dy)
                lines.append(f"dist_to_wpnt={dist_wp:.2f}m")

        banner = "═" * 64
        print(f"\n{banner}\n[far_monitor] t={time.strftime('%H:%M:%S')}")
        for ln in lines:
            print(f"  {ln}")


def main() -> None:
    rclpy.init()
    node = FarMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
