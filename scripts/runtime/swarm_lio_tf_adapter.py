#!/usr/bin/env python3
"""swarm_lio_tf_adapter — bridge ROS2-side topics from the dockerized
Swarm-LIO2 stack into our nav-stack contract.

Two responsibilities (one node so the lifecycle stays atomic):

  1) Broadcast TF `quad{N}/world → quad{N}_aft_mapped` from the bridged
     `/robot_a/swarm_lio2_raw/Odometry`. Swarm-LIO2's native TF is
     published inside the docker (ROS1) and is NOT bridged by the default
     bridge.yaml; redoing it on the ROS2 side avoids a bridge.yaml change.
     We preserve the message's own `header.frame_id` and `child_frame_id`
     so downstream statics (`map → quad{N}/world`, `quad{N}_aft_mapped →
     base_link`) can wrap it without renaming inside this node.

  2) Republish `/robot_a/swarm_lio2_raw/cloud_static` with a corrected
     `header.frame_id`. Swarm-LIO2's `cloud_registered_body` topic carries
     body-frame points (laserMapping.cpp:805) but tags them with
     `frame_id = "<topic_name_prefix>world"` (laserMapping.cpp:809). That
     mismatch makes octomap apply the wrong transform: points are body-
     local but octomap treats them as world-frame, so sensor_origin gets
     pinned to (0, 0, 0) and the map degenerates the moment the robot
     moves. Rewriting frame_id to `quad{N}_aft_mapped` lets octomap look
     up `map → quad{N}_aft_mapped` through the static+dynamic chain and
     resolve sensor_origin correctly.

Default I/O matches the docker's hardcoded relays for drone_id=1 /
ROBOT_NAMESPACES=robot_a (see docker/ros1_hybrid_slam/ros1_hybrid_entrypoint.sh).
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster


class SwarmLioTfAdapter(Node):
    def __init__(self) -> None:
        super().__init__("swarm_lio_tf_adapter")

        # I/O parameters — defaults match docker/ros1_hybrid_slam/ros1_hybrid_entrypoint.sh
        # for drone_id=1 (single-robot path).
        self.declare_parameter("odom_input_topic", "/robot_a/swarm_lio2_raw/Odometry")
        self.declare_parameter("cloud_input_topic", "/robot_a/swarm_lio2_raw/cloud_static")
        self.declare_parameter("cloud_output_topic", "/cloud_registered_body")
        # Frame override for the republished cloud. Empty string means: leave
        # the input frame_id unchanged (debug only — octomap will misbehave).
        self.declare_parameter("cloud_output_frame_id", "quad1_aft_mapped")
        # Whether to broadcast TF from Odometry. Disable if /tf is bridged
        # natively from ROS1 (would double-publish quad1/world→quad1_aft_mapped).
        self.declare_parameter("publish_tf", True)

        odom_in = str(self.get_parameter("odom_input_topic").value)
        self.cloud_out_frame = str(self.get_parameter("cloud_output_frame_id").value).strip()
        self.publish_tf_flag = bool(self.get_parameter("publish_tf").value)
        cloud_in = str(self.get_parameter("cloud_input_topic").value)
        cloud_out = str(self.get_parameter("cloud_output_topic").value)

        # SensorData QoS for cloud (high-rate, drop-okay).
        cloud_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST)

        self._tf_br = TransformBroadcaster(self) if self.publish_tf_flag else None
        self.create_subscription(Odometry, odom_in, self._on_odom, 20)
        self.create_subscription(PointCloud2, cloud_in, self._on_cloud, cloud_qos)
        self._cloud_pub = self.create_publisher(PointCloud2, cloud_out, cloud_qos)

        self.get_logger().info(
            f"swarm_lio_tf_adapter: tf={self.publish_tf_flag} "
            f"odom_in={odom_in} cloud_in={cloud_in} → cloud_out={cloud_out} "
            f"(frame_id rewrite: '{self.cloud_out_frame}' or passthrough)"
        )
        self._got_odom = False
        self._got_cloud = False

    def _on_odom(self, msg: Odometry) -> None:
        if not self._got_odom:
            self.get_logger().info(
                f"first Odometry: {msg.header.frame_id} → {msg.child_frame_id}"
            )
            self._got_odom = True
        if self._tf_br is None:
            return
        # Preserve the swarm_lio frame names. Statics in the launch wrap this
        # dynamic with map → frame_id and child_frame_id → base_link.
        if not msg.header.frame_id or not msg.child_frame_id:
            return
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = msg.header.frame_id
        tf.child_frame_id = msg.child_frame_id
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self._tf_br.sendTransform(tf)

    def _on_cloud(self, msg: PointCloud2) -> None:
        if not self._got_cloud:
            self.get_logger().info(
                f"first cloud: input frame_id='{msg.header.frame_id}' "
                f"output frame_id='{self.cloud_out_frame or msg.header.frame_id}' "
                f"({len(msg.data)} bytes)"
            )
            self._got_cloud = True
        if self.cloud_out_frame:
            msg.header.frame_id = self.cloud_out_frame
        self._cloud_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SwarmLioTfAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
