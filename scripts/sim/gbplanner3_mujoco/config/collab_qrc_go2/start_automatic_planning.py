#!/usr/bin/env python3
"""Start GBPlanner3 autonomous planning after its ROS1 inputs are alive.

This only presses GBPlanner3's own ``automatic_planning`` service. It does not
publish routes, waypoints, or any scripted robot motion.
"""

import argparse
import sys
from typing import List, Optional

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger


def _parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-label", default="robot")
    parser.add_argument("--odom-topic", default="/rmf/odom")
    parser.add_argument("--cloud-topic", default="/rmf/lidar/points_downsampled")
    parser.add_argument(
        "--service",
        default="/planner_control_interface/std_srvs/automatic_planning",
    )
    parser.add_argument("--input-timeout-sec", type=float, default=90.0)
    parser.add_argument("--service-timeout-sec", type=float, default=120.0)
    parser.add_argument("--warmup-sec", type=float, default=12.0)
    parser.add_argument(
        "--repeat-sec",
        type=float,
        default=0.0,
        help=(
            "If >0, keep re-triggering GBPlanner3's own automatic_planning "
            "service at this cadence. This is used in common-executor mode, "
            "where PCI's native controller feedback is replaced by Nav2."
        ),
    )
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = _parse_args(argv or sys.argv[1:])
    rospy.init_node(f"gbplanner3_auto_start_{args.robot_label}", anonymous=True)

    rospy.loginfo(
        "[%s] waiting for odom=%s cloud=%s",
        args.robot_label,
        args.odom_topic,
        args.cloud_topic,
    )
    try:
        rospy.wait_for_message(args.odom_topic, Odometry, timeout=args.input_timeout_sec)
        rospy.wait_for_message(args.cloud_topic, PointCloud2, timeout=args.input_timeout_sec)
    except rospy.ROSException as exc:
        rospy.logerr("[%s] GBPlanner3 inputs not ready: %s", args.robot_label, exc)
        return 1

    if args.warmup_sec > 0.0:
        rospy.loginfo(
            "[%s] inputs alive; warming voxblox for %.1f s before planning",
            args.robot_label,
            args.warmup_sec,
        )
        rospy.sleep(args.warmup_sec)

    try:
        rospy.loginfo("[%s] waiting for service %s", args.robot_label, args.service)
        rospy.wait_for_service(args.service, timeout=args.service_timeout_sec)
        start = rospy.ServiceProxy(args.service, Trigger)
        response = start()
    except (rospy.ROSException, rospy.ServiceException) as exc:
        rospy.logerr("[%s] failed to start GBPlanner3: %s", args.robot_label, exc)
        return 1

    if response.success:
        rospy.loginfo("[%s] GBPlanner3 automatic planning started", args.robot_label)
        if args.repeat_sec > 0.0:
            while not rospy.is_shutdown():
                rospy.sleep(args.repeat_sec)
                try:
                    response = start()
                except (rospy.ROSException, rospy.ServiceException) as exc:
                    rospy.logwarn(
                        "[%s] periodic GBPlanner3 re-trigger failed: %s",
                        args.robot_label,
                        exc,
                    )
                    continue
                if response.success:
                    rospy.loginfo(
                        "[%s] periodic GBPlanner3 automatic_planning trigger sent",
                        args.robot_label,
                    )
                else:
                    rospy.logwarn(
                        "[%s] periodic GBPlanner3 trigger rejected: %s",
                        args.robot_label,
                        response.message,
                    )
        return 0

    rospy.logerr(
        "[%s] GBPlanner3 automatic planning rejected: %s",
        args.robot_label,
        response.message,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
