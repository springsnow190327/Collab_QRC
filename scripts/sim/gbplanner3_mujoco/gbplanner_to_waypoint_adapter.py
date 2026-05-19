#!/usr/bin/env python3
"""gbplanner_to_waypoint_adapter — bridge GBPlanner trajectory output to
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
import json
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectory


class GBPlannerWaypointAdapter(Node):
    def __init__(self) -> None:
        super().__init__('gbplanner_waypoint_adapter')

        self.declare_parameter('robot_namespace', 'robot')
        self.declare_parameter('planner_label', 'GBPlanner')
        self.declare_parameter('trajectory_topic', '/command/trajectory')
        self.declare_parameter('path_topic', '')
        self.declare_parameter('odometry_topic', '/robot/Odometry')
        self.declare_parameter('nav_status_topic', 'nav_status')
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('republish_period_sec', 1.0)
        self.declare_parameter('min_waypoint_separation', 0.5)
        self.declare_parameter('min_robot_goal_distance', 0.75)
        self.declare_parameter('goal_reached_reselect_distance', 0.55)
        self.declare_parameter('blacklist_radius', 0.45)
        self.declare_parameter('prefer_trajectory_sec', 2.0)
        self.declare_parameter('max_source_age_sec', 30.0)
        self.declare_parameter('path_after_trajectory_grace_sec', 0.5)
        self.declare_parameter('publish_goal_pose', True)
        self.declare_parameter('publish_way_point_coord', True)

        ns = self.get_parameter('robot_namespace').get_parameter_value().string_value
        self._planner_label = (
            self.get_parameter('planner_label').get_parameter_value().string_value
            or 'GBPlanner'
        )
        traj_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        nav_status_topic = self.get_parameter('nav_status_topic').get_parameter_value().string_value
        self._lookahead = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self._period = self.get_parameter('republish_period_sec').get_parameter_value().double_value
        self._min_sep = self.get_parameter('min_waypoint_separation').get_parameter_value().double_value
        self._min_robot_goal_distance = self.get_parameter('min_robot_goal_distance').get_parameter_value().double_value
        self._goal_reached_reselect_distance = (
            self.get_parameter('goal_reached_reselect_distance').get_parameter_value().double_value)
        self._blacklist_radius = self.get_parameter('blacklist_radius').get_parameter_value().double_value
        self._prefer_trajectory_sec = self.get_parameter('prefer_trajectory_sec').get_parameter_value().double_value
        self._max_source_age_sec = self.get_parameter('max_source_age_sec').get_parameter_value().double_value
        self._path_after_trajectory_grace_sec = (
            self.get_parameter('path_after_trajectory_grace_sec').get_parameter_value().double_value)
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
        if path_topic:
            self.create_subscription(Path, path_topic, self._on_path, 5)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(
            String,
            nav_status_topic if nav_status_topic.startswith('/') else f'/{ns}/{nav_status_topic}',
            self._on_nav_status,
            10,
        )

        self._sources: dict[str, dict[str, object]] = {}
        self._last_pose: tuple[float, float, float] | None = None      # (x, y, z)
        self._last_pub_xy: tuple[float, float] | None = None
        self._last_pub_blacklisted = False
        self._last_pub_idx: int | None = None
        self._active_output_token: tuple[str, float] | None = None
        self._published_candidates: set[tuple[str, float, int]] = set()
        self._blacklisted_goals: list[tuple[float, float]] = []
        self._empty_log_deadline = 0.0
        self._near_goal_log_deadline = 0.0
        self._stale_log_deadline = 0.0
        self._source_log_deadline: dict[str, float] = {}

        self.create_timer(self._period, self._tick)

        self.get_logger().info(
            f'adapter up: subs={traj_topic},{path_topic or "<no_path>"},{odom_topic}; '
            f'pubs=/{ns}/way_point_coord,/{ns}/goal_pose; '
            f'lookahead={self._lookahead}m min_robot_goal={self._min_robot_goal_distance}m '
            f'goal_reached_reselect={self._goal_reached_reselect_distance}m '
            f'prefer_trajectory={self._prefer_trajectory_sec}s '
            f'max_source_age={self._max_source_age_sec}s period={self._period}s')

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._last_pose = (p.x, p.y, p.z)

    def _on_nav_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        state = str(payload.get('state', '')).lower()
        if state not in {'unreachable', 'failed'}:
            return
        goal = payload.get('goal')
        if isinstance(goal, list) and len(goal) >= 2:
            gx, gy = float(goal[0]), float(goal[1])
        elif self._last_pub_xy is not None:
            gx, gy = self._last_pub_xy
        else:
            return
        if self._is_blacklisted(gx, gy):
            return
        self._blacklisted_goals.append((gx, gy))
        if self._last_pub_xy is not None and math.hypot(
            gx - self._last_pub_xy[0], gy - self._last_pub_xy[1]
        ) <= self._blacklist_radius:
            self._last_pub_blacklisted = True
        self._last_pub_idx = None
        self.get_logger().warn(
            f'Nav2 reported {state} for planner waypoint ({gx:+.2f},{gy:+.2f}); '
            f'blacklisting this point and advancing within the {self._planner_label} trajectory')

    def _on_trajectory(self, msg: MultiDOFJointTrajectory) -> None:
        if not msg.points:
            self._log_empty_source_once('trajectory')
            return
        points: list[tuple[float, float, float]] = []
        for pt in msg.points:
            if not pt.transforms:
                continue
            tr = pt.transforms[0].translation
            points.append((tr.x, tr.y, tr.z))
        if not points:
            self._log_empty_source_once('trajectory-without-transforms')
            return
        self._store_points('trajectory', points, msg.header.frame_id or 'world')

    def _on_path(self, msg: Path) -> None:
        if not msg.poses:
            self._log_empty_source_once('path')
            return
        points = [
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            for pose in msg.poses
        ]
        self._store_points('path', points, msg.header.frame_id or 'world')

    def _log_empty_source_once(self, source: str) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now >= self._empty_log_deadline:
            self._empty_log_deadline = now + 10.0
            self.get_logger().debug(f'ignoring empty {self._planner_label} {source} output')

    def _store_points(
        self,
        source: str,
        points: list[tuple[float, float, float]],
        frame_id: str,
    ) -> None:
        stamp_sec = self.get_clock().now().nanoseconds * 1e-9
        self._sources[source] = {
            'points': points,
            'frame_id': frame_id,
            'stamp_sec': stamp_sec,
        }
        if self._last_pose is None:
            return
        rx, ry, _ = self._last_pose
        distances = [math.hypot(x - rx, y - ry) for x, y, _ in points]
        max_dist = max(distances) if distances else 0.0
        first_dist = distances[0] if distances else 0.0
        final_dist = distances[-1] if distances else 0.0
        if stamp_sec >= self._source_log_deadline.get(source, 0.0):
            self._source_log_deadline[source] = stamp_sec + 5.0
            self.get_logger().info(
                f'{self._planner_label} {source} update: points={len(points)} '
                f'first={first_dist:.2f}m final={final_dist:.2f}m '
                f'farthest={max_dist:.2f}m frame={frame_id}')

    def _active_source(self) -> tuple[str, str, float, list[tuple[float, float, float]]] | None:
        if not self._sources:
            return None
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        def is_fresh(src: dict[str, object]) -> bool:
            if self._max_source_age_sec <= 0.0:
                return True
            return now_sec - float(src['stamp_sec']) <= self._max_source_age_sec

        traj = self._sources.get('trajectory')
        path = self._sources.get('path')
        if traj is not None and is_fresh(traj):
            traj_age = now_sec - float(traj['stamp_sec'])
            if traj_age <= self._prefer_trajectory_sec or 'path' not in self._sources:
                return (
                    'trajectory',
                    str(traj['frame_id']),
                    float(traj['stamp_sec']),
                    list(traj['points']),
                )
        if path is not None and is_fresh(path):
            if traj is not None:
                path_stamp = float(path['stamp_sec'])
                traj_stamp = float(traj['stamp_sec'])
                # GBPlanner3 publishes Path and MultiDOF trajectory for the
                # same plan nearly simultaneously.  Do not let the path copy
                # re-trigger the common executor after the trajectory copy has
                # already been sent. Keep returning the trajectory so the
                # common executor can progressively walk through that planner
                # output while the upstream PCI waits for path-end progress.
                if path_stamp <= traj_stamp + self._path_after_trajectory_grace_sec:
                    return (
                        'trajectory',
                        str(traj['frame_id']),
                        float(traj['stamp_sec']),
                        list(traj['points']),
                    )
            return ('path', str(path['frame_id']), float(path['stamp_sec']), list(path['points']))
        if traj is not None and is_fresh(traj):
            return ('trajectory', str(traj['frame_id']), float(traj['stamp_sec']), list(traj['points']))
        if now_sec >= self._stale_log_deadline:
            self._stale_log_deadline = now_sec + 10.0
            self.get_logger().debug(
                f'latest {self._planner_label} output is stale; waiting for a new planner output')
        return None

    def _activate_output(self, source: str, stamp_sec: float) -> None:
        token = (source, stamp_sec)
        if token == self._active_output_token:
            return
        self._active_output_token = token
        self._last_pub_xy = None
        self._last_pub_blacklisted = False
        self._last_pub_idx = None
        self._published_candidates.clear()
        self._blacklisted_goals.clear()

    def _is_blacklisted(self, x: float, y: float) -> bool:
        return any(
            math.hypot(x - bx, y - by) <= self._blacklist_radius
            for bx, by in self._blacklisted_goals
        )

    def _select_goal(
        self,
        points: list[tuple[float, float, float]],
        rx: float,
        ry: float,
    ) -> tuple[float, float, float, float, float, float, bool, int] | None:
        """Select a Nav2-executable goal from GBPlanner output only."""
        if not points:
            return None

        def point_distance(index: int) -> float:
            x, y, _ = points[index]
            return math.hypot(x - rx, y - ry)

        nearest_idx = min(range(len(points)), key=point_distance)
        accumulated = 0.0
        prev_xy: tuple[float, float] | None = None
        selected_idx: int | None = None
        for i in range(nearest_idx, len(points)):
            tx, ty, _tz = points[i]
            if prev_xy is not None:
                accumulated += math.hypot(tx - prev_xy[0], ty - prev_xy[1])
            prev_xy = (tx, ty)
            if accumulated >= self._lookahead:
                selected_idx = i
                break

        if selected_idx is None:
            selected_idx = len(points) - 1

        chosen_idx: int | None = None
        fallback_used = False
        for idx in range(selected_idx, len(points)):
            x, y, _ = points[idx]
            if self._is_blacklisted(x, y):
                continue
            if point_distance(idx) >= self._min_robot_goal_distance:
                chosen_idx = idx
                break

        if chosen_idx is None:
            farthest_idx: int | None = None
            farthest_dist = -1.0
            for idx in range(nearest_idx, len(points)):
                x, y, _ = points[idx]
                if self._is_blacklisted(x, y):
                    continue
                dist = point_distance(idx)
                if dist > farthest_dist:
                    farthest_idx = idx
                    farthest_dist = dist
            if farthest_idx is not None and farthest_dist >= self._min_robot_goal_distance:
                chosen_idx = farthest_idx
                fallback_used = True

        if chosen_idx is None:
            chosen_idx = selected_idx
            x, y, _ = points[chosen_idx]
            if self._is_blacklisted(x, y):
                return None

        robot_to_goal = point_distance(chosen_idx)

        tx, ty, tz = points[chosen_idx]
        nxt = points[chosen_idx + 1] if chosen_idx + 1 < len(points) else None
        if nxt is not None:
            yaw = math.atan2(nxt[1] - ty, nxt[0] - tx)
        else:
            yaw = math.atan2(ty - ry, tx - rx)
        max_dist = max(point_distance(i) for i in range(len(points)))
        return tx, ty, tz, yaw, robot_to_goal, max_dist, fallback_used, chosen_idx

    def _tick(self) -> None:
        if self._last_pose is None:
            return

        active = self._active_source()
        if active is None:
            return
        source, frame_id, stamp_sec, points = active
        self._activate_output(source, stamp_sec)
        rx, ry, _ = self._last_pose
        if self._last_pub_xy is not None and not self._last_pub_blacklisted:
            last_dist = math.hypot(rx - self._last_pub_xy[0], ry - self._last_pub_xy[1])
            if last_dist > self._goal_reached_reselect_distance:
                return
        selected = self._select_goal(points, rx, ry)
        if selected is None:
            return
        tx, ty, tz, yaw, robot_to_goal, max_dist, fallback_used, chosen_idx = selected
        if robot_to_goal < self._min_robot_goal_distance:
            if robot_to_goal <= self._goal_reached_reselect_distance:
                return
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if now_sec >= self._near_goal_log_deadline:
                self._near_goal_log_deadline = now_sec + 5.0
                self.get_logger().warn(
                    f'ignoring {self._planner_label} {source} goal {robot_to_goal:.2f}m from robot; '
                    f'farthest planner point is {max_dist:.2f}m; '
                    f'need >= {self._min_robot_goal_distance:.2f}m for Nav2 common executor')
            return

        candidate_key = (source, stamp_sec, chosen_idx)
        if candidate_key in self._published_candidates:
            return

        # De-dup: don't republish if last published xy is within min_sep,
        # unless the previous point was blacklisted by Nav2 feedback.
        if self._last_pub_xy is not None:
            d = math.hypot(tx - self._last_pub_xy[0], ty - self._last_pub_xy[1])
            if d < self._min_sep:
                return
        self._last_pub_xy = (tx, ty)
        self._last_pub_blacklisted = False
        self._last_pub_idx = chosen_idx
        self._published_candidates.add(candidate_key)

        now = self.get_clock().now().to_msg()
        frame_id = frame_id or 'world'

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
            f'[frame={frame_id}, source={source}, idx={chosen_idx}/{len(points) - 1}, '
            f'robot@({rx:.2f},{ry:.2f})'
            f'{", farthest-fallback" if fallback_used else ""}]')


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
