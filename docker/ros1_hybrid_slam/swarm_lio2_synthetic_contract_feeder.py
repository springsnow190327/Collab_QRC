#!/usr/bin/env python3
from __future__ import annotations

import math
import os
from typing import Iterable

import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def _namespaces() -> list[str]:
    raw = os.environ.get("ROBOT_NAMESPACES", "robot_a robot_b")
    return [item.strip().strip("/") for item in raw.split() if item.strip()]


def _base_frame(ns: str) -> str:
    return f"{ns}/base_link"


def _odom_frame(ns: str) -> str:
    return f"{ns}/odom"


def _map_frame(ns: str) -> str:
    return f"{ns}/map"


def _odom(ns: str, index: int, stamp: rospy.Time) -> Odometry:
    msg = Odometry()
    msg.header.stamp = stamp
    msg.header.frame_id = _odom_frame(ns)
    msg.child_frame_id = _base_frame(ns)
    msg.pose.pose.position.x = 0.05 * index
    msg.pose.pose.position.y = 0.25 if ns.endswith("b") else 0.0
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = 0.02
    return msg


def _cloud(frame_id: str, stamp: rospy.Time, points: Iterable[tuple[float, float, float]]) -> PointCloud2:
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return point_cloud2.create_cloud_xyz32(header, list(points))


def _relative(ns: str, stamp: rospy.Time) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = _map_frame(ns)
    msg.child_frame_id = _base_frame(ns)
    msg.transform.rotation.w = 1.0
    return msg


def main() -> None:
    rospy.init_node("swarm_lio2_synthetic_contract_feeder", anonymous=False)
    namespaces = _namespaces()
    rate_hz = float(os.environ.get("SWARM_LIO2_SYNTHETIC_RATE_HZ", "5.0"))
    pubs: dict[str, dict[str, rospy.Publisher]] = {}
    for ns in namespaces:
        prefix = f"/{ns}/swarm_lio2_raw"
        pubs[ns] = {
            "odom": rospy.Publisher(f"{prefix}/Odometry", Odometry, queue_size=10),
            "cloud_static": rospy.Publisher(f"{prefix}/cloud_static", PointCloud2, queue_size=5),
            "cloud_map": rospy.Publisher(f"{prefix}/cloud_map", PointCloud2, queue_size=5),
            "relative_transform": rospy.Publisher(
                f"{prefix}/relative_transform", TransformStamped, queue_size=5
            ),
        }
    rospy.logwarn(
        "SWARM_LIO2_FEED_SOURCE=synthetic_contract_test: publishing synthetic raw "
        "adapter inputs for bridge-contract validation only; this is not Swarm-LIO2 SLAM output."
    )
    rate = rospy.Rate(rate_hz)
    index = 0
    while not rospy.is_shutdown():
        stamp = rospy.Time.now()
        for robot_index, ns in enumerate(namespaces):
            offset = float(robot_index)
            pubs[ns]["odom"].publish(_odom(ns, index, stamp))
            pubs[ns]["cloud_static"].publish(
                _cloud(
                    _base_frame(ns),
                    stamp,
                    [
                        (0.0 + offset, 0.0, 0.0),
                        (0.4 + offset, 0.0, 0.0),
                        (0.0 + offset, 0.4, 0.0),
                    ],
                )
            )
            pubs[ns]["cloud_map"].publish(
                _cloud(
                    _map_frame(ns),
                    stamp,
                    [
                        (math.sin(index * 0.05) + offset, 0.0, 0.0),
                        (0.0 + offset, math.cos(index * 0.05), 0.0),
                    ],
                )
            )
            pubs[ns]["relative_transform"].publish(_relative(ns, stamp))
        index += 1
        rate.sleep()


if __name__ == "__main__":
    main()
