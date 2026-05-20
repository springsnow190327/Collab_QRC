#!/usr/bin/env python3
"""Call a GBPlanner service periodically and publish its returned path.

The common-executor benchmark uses Nav2 MPPI to execute planner outputs.  GBPlanner2's
native PCI waits for its own path-following feedback, which is not available when
Nav2 is the executor.  This node keeps GBPlanner autonomous by asking the upstream
planner service for a path from the current odometry pose, then publishes that path
for the existing ROS2 waypoint adapter.  It does not create scripted waypoints.
"""

from __future__ import annotations

import argparse
import math
import sys
from typing import Iterable, List, Optional

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from planner_msgs.srv import planner_srv
from sensor_msgs.msg import PointCloud2


def _parse_bound_modes(raw: str) -> List[int]:
    modes: List[int] = []
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        modes.append(int(item))
    return modes or [0]


def _parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--planner-label", default="GBPlanner")
    parser.add_argument("--robot-label", default="robot")
    parser.add_argument("--odom-topic", default="/robot/gbplanner/odom_planning")
    parser.add_argument("--cloud-topic", default="/robot/lidar/points_downsampled")
    parser.add_argument("--planner-service", default="/gbplanner")
    parser.add_argument("--path-topic", default="/robot/gbplanner_path")
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--bound-modes", default="0,1,2,4")
    parser.add_argument("--input-timeout-sec", type=float, default=90.0)
    parser.add_argument("--service-timeout-sec", type=float, default=120.0)
    parser.add_argument("--warmup-sec", type=float, default=12.0)
    parser.add_argument("--period-sec", type=float, default=20.0)
    parser.add_argument("--min-path-points", type=int, default=2)
    parser.add_argument("--min-path-length-m", type=float, default=0.5)
    return parser.parse_args(argv)


def _path_length(path: Iterable) -> float:
    total = 0.0
    last = None
    for pose in path:
        p = pose.position
        if last is not None:
            total += math.sqrt(
                (p.x - last.x) * (p.x - last.x)
                + (p.y - last.y) * (p.y - last.y)
                + (p.z - last.z) * (p.z - last.z)
            )
        last = p
    return total


class ServicePathExecutor:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.bound_modes = _parse_bound_modes(args.bound_modes)
        self.odom: Optional[Odometry] = None
        self.seq = 0
        self.path_pub = rospy.Publisher(args.path_topic, Path, queue_size=2)
        self.odom_sub = rospy.Subscriber(args.odom_topic, Odometry, self._odom_cb, queue_size=10)
        self.plan = rospy.ServiceProxy(args.planner_service, planner_srv)

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def wait_until_ready(self) -> None:
        rospy.loginfo(
            "[%s] waiting for odom=%s cloud=%s",
            self.args.robot_label,
            self.args.odom_topic,
            self.args.cloud_topic,
        )
        self.odom = rospy.wait_for_message(
            self.args.odom_topic, Odometry, timeout=self.args.input_timeout_sec
        )
        rospy.wait_for_message(
            self.args.cloud_topic, PointCloud2, timeout=self.args.input_timeout_sec
        )
        if self.args.warmup_sec > 0.0:
            rospy.loginfo(
                "[%s] inputs alive; warming voxblox for %.1f s before service-path planning",
                self.args.robot_label,
                self.args.warmup_sec,
            )
            rospy.sleep(self.args.warmup_sec)
        rospy.loginfo("[%s] waiting for planner service %s", self.args.robot_label, self.args.planner_service)
        rospy.wait_for_service(self.args.planner_service, timeout=self.args.service_timeout_sec)

    def call_once(self) -> bool:
        if self.odom is None:
            return False
        root_pose = self.odom.pose.pose
        for bound_mode in self.bound_modes:
            req = planner_srv._request_class()
            req.header.stamp = rospy.Time.now()
            req.header.seq = self.seq
            req.header.frame_id = self.args.frame_id
            req.bound_mode = bound_mode
            req.root_pose = root_pose
            self.seq += 1
            try:
                res = self.plan(req)
            except rospy.ServiceException as exc:
                rospy.logwarn(
                    "[%s] %s planner service failed: %s",
                    self.args.robot_label,
                    self.args.planner_label,
                    exc,
                )
                return False

            path_len = _path_length(res.path)
            if len(res.path) < self.args.min_path_points or path_len < self.args.min_path_length_m:
                rospy.loginfo(
                    "[%s] %s bound_mode=%d returned path_len=%d length=%.2fm; trying next bound",
                    self.args.robot_label,
                    self.args.planner_label,
                    bound_mode,
                    len(res.path),
                    path_len,
                )
                continue

            msg = Path()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.args.frame_id
            for pose in res.path:
                ps = PoseStamped()
                ps.header = msg.header
                ps.pose = pose
                msg.poses.append(ps)
            self.path_pub.publish(msg)
            rospy.loginfo(
                "[%s] %s path published: points=%d length=%.2fm bound_mode=%d status=%d",
                self.args.robot_label,
                self.args.planner_label,
                len(res.path),
                path_len,
                bound_mode,
                res.status,
            )
            return True
        rospy.logwarn(
            "[%s] %s returned no executable path for bound_modes=%s",
            self.args.robot_label,
            self.args.planner_label,
            ",".join(str(m) for m in self.bound_modes),
        )
        return False

    def spin(self) -> None:
        period = max(1.0, float(self.args.period_sec))
        rate = rospy.Rate(1.0 / period)
        while not rospy.is_shutdown():
            self.call_once()
            rate.sleep()


def main(argv: Optional[List[str]] = None) -> int:
    args = _parse_args(argv or sys.argv[1:])
    rospy.init_node(f"{args.planner_label}_service_path_{args.robot_label}", anonymous=True)
    executor = ServicePathExecutor(args)
    try:
        executor.wait_until_ready()
        executor.spin()
    except rospy.ROSException as exc:
        rospy.logerr("[%s] %s service-path executor failed: %s", args.robot_label, args.planner_label, exc)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
