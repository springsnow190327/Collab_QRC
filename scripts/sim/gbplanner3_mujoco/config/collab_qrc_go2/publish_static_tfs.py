#!/usr/bin/env python3
"""Latched ROS 1 /tf_static publisher for Collab_QRC → GBPlanner3 bridge.

Why this exists: Humble Fast-LIO publishes its URDF + slam static TFs to
/robot/tf_static (TRANSIENT_LOCAL/latched). ros1_bridge's dynamic_bridge
does NOT preserve latching when forwarding to ROS 1, so by the time the
Noetic-side relay/subscribers wake up, the latched messages have already
been consumed and the chain world→map→odom→base_link→imu→body is broken
on the gbplanner side — every cloud lookup fails with "Could not find a
connection ... Tf has two or more unconnected trees", and voxblox refuses
to integrate any pointcloud.

ROS 1 `rosrun tf2_ros static_transform_publisher` in Noetic also turned
out to publish at ~9 Hz to /tf (not /tf_static), giving timing gaps too
big for voxblox's 50 ms waitForTransform window.

This script publishes a single latched TFMessage to /tf_static containing
every link we need on the Noetic side. Late subscribers get it for free
through latching, no rate tuning needed.

Values pulled from Humble Fast-LIO snapshot on 2026-05-13:
  map → odom (identity)
  base_link → imu  (-0.026, 0, 0.042)         no rotation
  imu → body       (identity)                 Fast-LIO body alias
  base_link → lidar          (0.161, 0, 0.123)  pitch +0.2269 rad (Mid-360 mount)
  base_link → livox_mid360   (0.161, 0, 0.123)  pitch +0.2269 rad
"""
import math
import rospy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


def make_tf(parent, child, x=0.0, y=0.0, z=0.0, pitch=0.0):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    # Only pitch rotation (rotation about Y axis) is used here.
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = math.sin(pitch * 0.5)
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = math.cos(pitch * 0.5)
    return t


def main():
    rospy.init_node("collab_qrc_static_tf_aliases", anonymous=False)
    pub = rospy.Publisher("/tf_static", TFMessage, queue_size=10, latch=True)

    msg = TFMessage()
    msg.transforms.append(make_tf("map",       "odom"))
    msg.transforms.append(make_tf("base_link", "imu",  x=-0.026, y=0.0, z=0.042))
    msg.transforms.append(make_tf("imu",       "body"))
    msg.transforms.append(make_tf("base_link", "lidar",         x=0.161, y=0.0, z=0.123, pitch=0.2269))
    msg.transforms.append(make_tf("base_link", "livox_mid360",  x=0.161, y=0.0, z=0.123, pitch=0.2269))

    # Give roscore + subscribers a moment to discover us before the latched send.
    rospy.sleep(0.5)
    pub.publish(msg)
    rospy.loginfo("collab_qrc_static_tf_aliases: published %d latched /tf_static transforms", len(msg.transforms))
    # Keep the node alive so the latched publisher stays connected for late subscribers.
    rospy.spin()


if __name__ == "__main__":
    main()
