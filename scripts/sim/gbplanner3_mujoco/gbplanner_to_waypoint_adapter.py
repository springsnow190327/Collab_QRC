#!/usr/bin/env python3
"""gbplanner_to_waypoint_adapter — bridge gbplanner3 trajectory output to
Collab_QRC's waypoint topic so CHAMP locomotion / FAR / Nav2 can execute.

  /command/trajectory (trajectory_msgs/MultiDOFJointTrajectory, from gbplanner
                       via ros1_bridge, world frame)
        │
        ▼
  this node
        │
        ├─► /<ns>/way_point_coord  (geometry_msgs/PointStamped, RELIABLE)
        │       → consumed by CFPA2-to-Nav2 bridge / CMU localPlanner
        │
        └─► /<ns>/goal_pose        (geometry_msgs/PoseStamped)
                → consumed by Nav2 NavigateToPose action server

Behavior:
  - On each /command/trajectory message, pick a lookahead point along the
    trajectory at parameter `lookahead_distance` (m) from the current robot
    pose (estimated from latest /robot/Odometry).
  - Publish the lookahead point at most every `republish_period` seconds to
    avoid spamming downstream (default 1 Hz).
  - Compute yaw from the next-next trajectory transform (so the robot walks
    forward along the path, not strafes).
"""

from __future__ import annotations
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory


class GBPlannerWaypointAdapter(Node):
    def __init__(self) -> None:
        super().__init__('gbplanner_waypoint_adapter')

        self.declare_parameter('robot_namespace', 'robot')
        self.declare_parameter('trajectory_topic', '/command/trajectory')
        self.declare_parameter('odometry_topic', '/robot/Odometry')
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('republish_period_sec', 1.0)
        self.declare_parameter('min_waypoint_separation', 0.5)
        self.declare_parameter('publish_goal_pose', True)
        self.declare_parameter('publish_way_point_coord', True)

        ns = self.get_parameter('robot_namespace').get_parameter_value().string_value
        traj_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        self._lookahead = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self._period = self.get_parameter('republish_period_sec').get_parameter_value().double_value
        self._min_sep = self.get_parameter('min_waypoint_separation').get_parameter_value().double_value
        self._pub_goal = self.get_parameter('publish_goal_pose').get_parameter_value().bool_value
        self._pub_wp = self.get_parameter('publish_way_point_coord').get_parameter_value().bool_value

        # CFPA2's convention: /<ns>/way_point_coord is RELIABLE + VOLATILE
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._wp_pub = self.create_publisher(
            PointStamped, f'/{ns}/way_point_coord', reliable_qos)
        self._goal_pub = self.create_publisher(
            PoseStamped, f'/{ns}/goal_pose', reliable_qos)

        self.create_subscription(MultiDOFJointTrajectory, traj_topic,
                                 self._on_trajectory, 5)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        self._last_traj: MultiDOFJointTrajectory | None = None
        self._last_pose: tuple[float, float, float] | None = None      # (x, y, z)
        self._last_pub_xy: tuple[float, float] | None = None

        self.create_timer(self._period, self._tick)

        self.get_logger().info(
            f'adapter up: subs={traj_topic},{odom_topic}; '
            f'pubs=/{ns}/way_point_coord,/{ns}/goal_pose; '
            f'lookahead={self._lookahead}m period={self._period}s')

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._last_pose = (p.x, p.y, p.z)

    def _on_trajectory(self, msg: MultiDOFJointTrajectory) -> None:
        if not msg.points:
            return
        self._last_traj = msg

    def _tick(self) -> None:
        if self._last_traj is None or not self._last_traj.points:
            return
        if self._last_pose is None:
            return

        rx, ry, _ = self._last_pose
        chosen: tuple[float, float, float, float] | None = None  # (x, y, z, yaw)
        accumulated = 0.0
        prev_xy: tuple[float, float] | None = (rx, ry)

        # Walk along trajectory, pick the first point >= lookahead_distance
        # along the path from current robot pose. Trajectory points have
        # `transforms[0]` (geometry_msgs/Transform).
        for i, pt in enumerate(self._last_traj.points):
            if not pt.transforms:
                continue
            tx = pt.transforms[0].translation.x
            ty = pt.transforms[0].translation.y
            tz = pt.transforms[0].translation.z
            if prev_xy is not None:
                seg = math.hypot(tx - prev_xy[0], ty - prev_xy[1])
                accumulated += seg
            prev_xy = (tx, ty)

            if accumulated >= self._lookahead:
                nxt = self._last_traj.points[i + 1] if i + 1 < len(self._last_traj.points) else None
                if nxt and nxt.transforms:
                    ny = math.atan2(
                        nxt.transforms[0].translation.y - ty,
                        nxt.transforms[0].translation.x - tx)
                else:
                    ny = math.atan2(ty - ry, tx - rx)
                chosen = (tx, ty, tz, ny)
                break

        # Fallback: use the FINAL trajectory point if we didn't hit lookahead
        if chosen is None:
            last_pt = self._last_traj.points[-1]
            if not last_pt.transforms:
                return
            tx = last_pt.transforms[0].translation.x
            ty = last_pt.transforms[0].translation.y
            tz = last_pt.transforms[0].translation.z
            ny = math.atan2(ty - ry, tx - rx)
            chosen = (tx, ty, tz, ny)

        tx, ty, tz, yaw = chosen

        # De-dup: don't republish if last published xy is within min_sep
        if self._last_pub_xy is not None:
            d = math.hypot(tx - self._last_pub_xy[0], ty - self._last_pub_xy[1])
            if d < self._min_sep:
                return
        self._last_pub_xy = (tx, ty)

        now = self.get_clock().now().to_msg()
        frame_id = self._last_traj.header.frame_id or 'world'

        if self._pub_wp:
            wp = PointStamped()
            wp.header.stamp = now
            wp.header.frame_id = frame_id
            wp.point.x = tx
            wp.point.y = ty
            wp.point.z = tz
            self._wp_pub.publish(wp)

        if self._pub_goal:
            gp = PoseStamped()
            gp.header.stamp = now
            gp.header.frame_id = frame_id
            gp.pose.position.x = tx
            gp.pose.position.y = ty
            gp.pose.position.z = tz
            half = 0.5 * yaw
            gp.pose.orientation.z = math.sin(half)
            gp.pose.orientation.w = math.cos(half)
            self._goal_pub.publish(gp)

        self.get_logger().info(
            f'waypoint → ({tx:.2f}, {ty:.2f}, {tz:.2f}) yaw={math.degrees(yaw):+.0f}° '
            f'[frame={frame_id}, robot@({rx:.2f},{ry:.2f})]')


def main(argv=None):
    rclpy.init(args=argv)
    node = GBPlannerWaypointAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
