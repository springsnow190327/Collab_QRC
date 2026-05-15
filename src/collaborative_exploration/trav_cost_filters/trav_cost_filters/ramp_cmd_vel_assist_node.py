#!/usr/bin/env python3
"""Low-speed ramp ascent velocity assist for verified ramp goals."""

from __future__ import annotations

from dataclasses import dataclass
import math
from types import SimpleNamespace

try:
    import rclpy
    from geometry_msgs.msg import PointStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
except ModuleNotFoundError:  # pragma: no cover - lets pure helper tests run without ROS sourced.
    rclpy = None
    PointStamped = object
    Odometry = object

    class Node:  # type: ignore[no-redef]
        pass

    class Twist:  # type: ignore[no-redef]
        def __init__(self) -> None:
            self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)


@dataclass(frozen=True)
class RampAssistParams:
    min_x: float = 5.3
    max_x: float = 9.8
    max_abs_y: float = 0.9
    min_forward_error_m: float = 0.08
    max_goal_distance_m: float = 2.0
    min_vx_mps: float = 0.22
    max_vx_mps: float = 0.30
    forward_gain: float = 0.45
    yaw_gain: float = 1.2
    max_yaw_rate_rps: float = 0.45
    slow_yaw_error_rad: float = 0.45


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_ramp_assist_twist(
    *,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    goal_x: float,
    goal_y: float,
    params: RampAssistParams = RampAssistParams(),
) -> Twist | None:
    """Return a conservative forward command for an active ramp segment."""

    if not (
        params.min_x <= robot_x <= params.max_x
        and abs(robot_y) <= params.max_abs_y
    ):
        return None
    dx = goal_x - robot_x
    dy = goal_y - robot_y
    distance = math.hypot(dx, dy)
    if dx < params.min_forward_error_m or distance > params.max_goal_distance_m:
        return None

    desired_yaw = math.atan2(dy, dx)
    yaw_error = _wrap_angle(desired_yaw - robot_yaw)
    vx = max(
        params.min_vx_mps,
        min(params.max_vx_mps, params.forward_gain * dx),
    )
    if abs(yaw_error) > params.slow_yaw_error_rad:
        vx = min(vx, params.min_vx_mps)

    cmd = Twist()
    cmd.linear.x = vx
    cmd.angular.z = max(
        -params.max_yaw_rate_rps,
        min(params.max_yaw_rate_rps, params.yaw_gain * yaw_error),
    )
    return cmd


class RampCmdVelAssistNode(Node):
    def __init__(self) -> None:
        super().__init__("ramp_cmd_vel_assist")

        self.goal_topic = str(self.declare_parameter("goal_topic", "ramp_ascent_goal").value)
        self.odom_topic = str(self.declare_parameter("odom_topic", "odom/nav").value)
        self.cmd_vel_topic = str(self.declare_parameter("cmd_vel_topic", "cmd_vel").value)
        self.goal_stale_sec = max(
            0.1, float(self.declare_parameter("goal_stale_sec", 8.0).value)
        )
        self.params = RampAssistParams(
            min_x=float(self.declare_parameter("min_x", 5.3).value),
            max_x=float(self.declare_parameter("max_x", 9.8).value),
            max_abs_y=float(self.declare_parameter("max_abs_y", 0.9).value),
            min_forward_error_m=float(
                self.declare_parameter("min_forward_error_m", 0.08).value
            ),
            max_goal_distance_m=float(
                self.declare_parameter("max_goal_distance_m", 2.0).value
            ),
            min_vx_mps=float(self.declare_parameter("min_vx_mps", 0.22).value),
            max_vx_mps=float(self.declare_parameter("max_vx_mps", 0.30).value),
            forward_gain=float(self.declare_parameter("forward_gain", 0.45).value),
            yaw_gain=float(self.declare_parameter("yaw_gain", 1.2).value),
            max_yaw_rate_rps=float(
                self.declare_parameter("max_yaw_rate_rps", 0.45).value
            ),
            slow_yaw_error_rad=float(
                self.declare_parameter("slow_yaw_error_rad", 0.45).value
            ),
        )

        self.goal: tuple[float, float] | None = None
        self.goal_rx_ns = 0
        self.odom: Odometry | None = None
        self.active = False

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(PointStamped, self.goal_topic, self._on_goal, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.create_timer(0.05, self._on_timer)

        self.get_logger().info(
            f"ramp_cmd_vel_assist: goal={self.goal_topic} odom={self.odom_topic} "
            f"cmd_vel={self.cmd_vel_topic} vx=[{self.params.min_vx_mps:.2f},"
            f"{self.params.max_vx_mps:.2f}] corridor_x=[{self.params.min_x:.1f},"
            f"{self.params.max_x:.1f}]"
        )

    def _on_goal(self, msg: PointStamped) -> None:
        goal = (float(msg.point.x), float(msg.point.y))
        if math.isfinite(goal[0]) and math.isfinite(goal[1]):
            self.goal = goal
            self.goal_rx_ns = self.get_clock().now().nanoseconds

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _goal_fresh(self) -> bool:
        if self.goal is None or self.goal_rx_ns <= 0:
            return False
        return (
            self.get_clock().now().nanoseconds - self.goal_rx_ns
            <= int(self.goal_stale_sec * 1e9)
        )

    def _on_timer(self) -> None:
        if self.odom is None or self.goal is None or not self._goal_fresh():
            self.active = False
            return

        pose = self.odom.pose.pose
        cmd = compute_ramp_assist_twist(
            robot_x=float(pose.position.x),
            robot_y=float(pose.position.y),
            robot_yaw=_yaw_from_quat(pose.orientation),
            goal_x=self.goal[0],
            goal_y=self.goal[1],
            params=self.params,
        )
        if cmd is None:
            self.active = False
            return
        self.pub.publish(cmd)
        if not self.active:
            self.active = True
            self.get_logger().info(
                f"ramp assist active toward ({self.goal[0]:.2f},{self.goal[1]:.2f})"
            )


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is required to run ramp_cmd_vel_assist_node")
    rclpy.init(args=args)
    node = RampCmdVelAssistNode()
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
