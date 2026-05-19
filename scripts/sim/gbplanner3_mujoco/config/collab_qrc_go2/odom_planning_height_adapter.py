#!/usr/bin/env python3
"""Normalize ROS 1 odometry height for GBPlanner3 common-executor mode.

GBPlanner3's high-level RRG is run as a planner only; the real robot dynamics
and collision checking stay in the ROS 2 Nav2 MPPI + safety stack.  The MuJoCo
robots have different body heights (Go2W vs Go2), so feeding raw base-link z
into the aerial/free-space GBPlanner3 model makes the same 2-D maze appear at
different vertical slices for each robot.  This adapter projects both robots
onto one SE(2) planning layer while preserving x/y/yaw and all velocity fields.
It does not publish goals, routes, or scripted behavior.
"""

from __future__ import annotations

import copy

import rospy
from nav_msgs.msg import Odometry


class OdomPlanningHeightAdapter:
    def __init__(self) -> None:
        self.source_topic = rospy.get_param("~source_topic", "/robot/odom/nav")
        self.dest_topic = rospy.get_param("~dest_topic", "/robot/gbplanner/odom")
        self.planning_z = float(rospy.get_param("~planning_z", 0.15))
        self.keep_covariance = bool(rospy.get_param("~keep_covariance", True))
        self._count = 0
        self._pub = rospy.Publisher(self.dest_topic, Odometry, queue_size=10)
        self._sub = rospy.Subscriber(self.source_topic, Odometry, self._callback, queue_size=20)
        rospy.loginfo(
            "odom_planning_height_adapter: %s -> %s z=%.3f keep_covariance=%s",
            self.source_topic,
            self.dest_topic,
            self.planning_z,
            self.keep_covariance,
        )

    def _callback(self, msg: Odometry) -> None:
        out = copy.deepcopy(msg)
        raw_z = out.pose.pose.position.z
        out.pose.pose.position.z = self.planning_z
        if not self.keep_covariance:
            out.pose.covariance = [0.0] * 36
        self._pub.publish(out)

        self._count += 1
        if self._count == 1 or self._count % 250 == 0:
            p = out.pose.pose.position
            rospy.loginfo(
                "odom_planning_height_adapter sample: raw_z=%.3f planning=(%.3f, %.3f, %.3f)",
                raw_z,
                p.x,
                p.y,
                p.z,
            )


def main() -> None:
    rospy.init_node("odom_planning_height_adapter", anonymous=False)
    OdomPlanningHeightAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
