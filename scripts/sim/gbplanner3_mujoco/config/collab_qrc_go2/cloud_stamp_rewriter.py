#!/usr/bin/env python3
"""cloud_stamp_rewriter — restamp Fast-LIO body cloud to current ROS time.

Why this exists: voxblox (used internally by gbplanner3 / OmniPlanner) calls
`tf_listener_.canTransform(to_frame, from_frame, cloud_stamp)` per cloud and
**drops the cloud silently if canTransform returns false** at that exact
timestamp (voxblox_ros/src/transformer.cc:103). When the cloud crosses
ros1_bridge from Humble → Noetic, the bridge adds variable latency on /tf vs
the cloud topic, so `cloud_stamp` regularly lands in a window where the TF
chain isn't fully populated yet on the Noetic side. Result on Collab_QRC:
voxblox queue grows to 10, "Input pointcloud queue getting too long!"
fires, every cloud gets dropped, TSDF stays empty, gbplanner spams
"No 'elevation' layer in map" at multi-kHz, no /command/trajectory is
ever produced.

This node:
  - subscribes to /robot/cloud_registered_body (bridged from Humble)
  - rewrites header.stamp to rospy.Time.now()
  - republishes to /rmf/lidar/points_downsampled (the topic voxblox + the
    elevation_mapping_lidar node already consume)

It REPLACES the existing topic_tools relay for this topic in
docker-compose.collab_qrc.yml. The relay for /robot/cloud_registered_body
must be removed when this node is enabled, otherwise both publish to the
same topic and voxblox sees an inconsistent stream.

Cost: loses temporal alignment between cloud and pose for moving robots.
Acceptable for our use case: robot is stationary at mission start, and
voxblox uses ros::Time(0) semantics via this rewrite (latest TF).
"""
import rospy
from sensor_msgs.msg import PointCloud2


def main() -> None:
    rospy.init_node("cloud_stamp_rewriter", anonymous=False)
    src_topic = rospy.get_param("~source_topic", "/robot/cloud_registered_body")
    dst_topic = rospy.get_param("~dest_topic",   "/rmf/lidar/points_downsampled")
    pub = rospy.Publisher(dst_topic, PointCloud2, queue_size=5)

    def cb(msg: PointCloud2) -> None:
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)

    rospy.Subscriber(src_topic, PointCloud2, cb, queue_size=1)
    rospy.loginfo(
        "cloud_stamp_rewriter: %s -> %s (header.stamp <- ros::Time::now())",
        src_topic,
        dst_topic,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
