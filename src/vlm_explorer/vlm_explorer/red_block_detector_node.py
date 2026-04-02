#!/usr/bin/env python3
"""Red block detector: HSV-based red blob detection from camera images.

Detects bright red objects in the camera feed and estimates their 2D world
position using the robot's odometry and a simple bearing projection.

Publishes detections as JSON on /vlm/artifact_detections and real-time
RViz MarkerArray on /vlm/artifact_markers.
"""

from __future__ import annotations

import json
import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RedBlockDetectorNode(Node):
    def __init__(self):
        super().__init__("red_block_detector")

        self.declare_parameter("robot_namespaces", ["robot"])
        self.declare_parameter("detections_topic", "/vlm/artifact_detections")
        self.declare_parameter("rate", 2.0)
        # Red in HSV wraps around 0/180, so we use two ranges:
        #   low band: 0-10,  high band: 170-180
        self.declare_parameter("hsv_h_low1", 0)
        self.declare_parameter("hsv_h_high1", 10)
        self.declare_parameter("hsv_h_low2", 170)
        self.declare_parameter("hsv_h_high2", 180)
        self.declare_parameter("hsv_s_low", 100)
        self.declare_parameter("hsv_v_low", 80)
        self.declare_parameter("min_blob_pixels", 50)
        self.declare_parameter("assumed_depth_m", 2.0)
        self.declare_parameter("camera_hfov_rad", 2.0944)
        self.declare_parameter("dedup_radius_m", 0.8)
        self.declare_parameter("marker_topic", "/vlm/artifact_markers")
        self.declare_parameter("marker_frame_id", "map")

        self._namespaces = list(self.get_parameter("robot_namespaces").value)
        self._rate = self.get_parameter("rate").value
        self._h_lo1 = self.get_parameter("hsv_h_low1").value
        self._h_hi1 = self.get_parameter("hsv_h_high1").value
        self._h_lo2 = self.get_parameter("hsv_h_low2").value
        self._h_hi2 = self.get_parameter("hsv_h_high2").value
        self._s_lo = self.get_parameter("hsv_s_low").value
        self._v_lo = self.get_parameter("hsv_v_low").value
        self._min_blob = self.get_parameter("min_blob_pixels").value
        self._depth = self.get_parameter("assumed_depth_m").value
        self._hfov = self.get_parameter("camera_hfov_rad").value
        self._dedup_r = self.get_parameter("dedup_radius_m").value
        self._marker_frame = str(self.get_parameter("marker_frame_id").value)

        self._det_pub = self.create_publisher(
            String, self.get_parameter("detections_topic").value, 10
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter("marker_topic").value, 10
        )

        self._images = {}
        self._odoms = {}
        for ns in self._namespaces:
            self.create_subscription(
                Image, f"/{ns}/front_camera/image_raw",
                lambda msg, _ns=ns: self._on_image(_ns, msg), 10,
            )
            self.create_subscription(
                Odometry, f"/{ns}/odom/nav",
                lambda msg, _ns=ns: self._on_odom(_ns, msg), 10,
            )

        self._registry: list[dict] = []
        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            f"RedBlockDetector: ns={self._namespaces} rate={self._rate}Hz"
        )

    def _on_image(self, ns: str, msg: Image):
        self._images[ns] = msg

    def _on_odom(self, ns: str, msg: Odometry):
        self._odoms[ns] = msg

    def _tick(self):
        new_dets = []
        for ns in self._namespaces:
            if ns not in self._images or ns not in self._odoms:
                continue
            det = self._detect_red(ns, self._images[ns], self._odoms[ns])
            if det is not None:
                new_dets.append(det)

        for d in new_dets:
            merged = False
            for existing in self._registry:
                dx = d["x"] - existing["x"]
                dy = d["y"] - existing["y"]
                if math.sqrt(dx * dx + dy * dy) < self._dedup_r:
                    if d["confidence"] > existing["confidence"]:
                        existing["x"] = d["x"]
                        existing["y"] = d["y"]
                        existing["confidence"] = d["confidence"]
                        existing["robot"] = d["robot"]
                    merged = True
                    break
            if not merged:
                d["id"] = f"red_{len(self._registry)}"
                self._registry.append(d)

        msg = String()
        msg.data = json.dumps(self._registry)
        self._det_pub.publish(msg)

        self._publish_markers()

    def _publish_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, det in enumerate(self._registry):
            # Cube marker at artifact position
            m = Marker()
            m.header.frame_id = self._marker_frame
            m.header.stamp = stamp
            m.ns = "artifact_cubes"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = det["x"]
            m.pose.position.y = det["y"]
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = 0.1
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 0.9
            ma.markers.append(m)

            # Text label hovering above
            t = Marker()
            t.header.frame_id = self._marker_frame
            t.header.stamp = stamp
            t.ns = "artifact_labels"
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = det["x"]
            t.pose.position.y = det["y"]
            t.pose.position.z = 0.25
            t.pose.orientation.w = 1.0
            t.scale.z = 0.12
            t.color.r = 1.0
            t.color.g = 1.0
            t.color.b = 1.0
            t.color.a = 1.0
            t.text = f"{det.get('label', 'red_block')} {det.get('confidence', 0.0):.2f}"
            ma.markers.append(t)

        self._marker_pub.publish(ma)

    def _detect_red(self, ns: str, img_msg: Image, odom: Odometry) -> Optional[dict]:
        """Detect largest red blob and estimate world position."""
        try:
            if img_msg.encoding in ("rgb8", "bgr8"):
                img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
                    (img_msg.height, img_msg.width, 3)
                )
                if img_msg.encoding == "bgr8":
                    img = img[:, :, ::-1]
            else:
                return None
        except (ValueError, IndexError):
            return None

        h_img, w_img = img.shape[:2]

        # RGB → HSV (manual, no OpenCV)
        r, g, b = img[:, :, 0].astype(float), img[:, :, 1].astype(float), img[:, :, 2].astype(float)
        mx = np.maximum(np.maximum(r, g), b)
        mn = np.minimum(np.minimum(r, g), b)
        diff = mx - mn

        hue = np.zeros_like(mx)
        mask_r = (mx == r) & (diff > 0)
        mask_g = (mx == g) & (diff > 0)
        mask_b = (mx == b) & (diff > 0)
        hue[mask_r] = (60 * ((g[mask_r] - b[mask_r]) / diff[mask_r]) + 360) % 360
        hue[mask_g] = (60 * ((b[mask_g] - r[mask_g]) / diff[mask_g]) + 120)
        hue[mask_b] = (60 * ((r[mask_b] - g[mask_b]) / diff[mask_b]) + 240)
        hue = (hue / 2).astype(np.uint8)  # 0-180

        sat = np.zeros_like(mx)
        sat[mx > 0] = (diff[mx > 0] / mx[mx > 0]) * 255
        sat = sat.astype(np.uint8)

        val = mx.astype(np.uint8)

        # Red wraps around hue 0/180 — match both bands
        red_mask = (
            (((hue >= self._h_lo1) & (hue <= self._h_hi1)) |
             ((hue >= self._h_lo2) & (hue <= self._h_hi2))) &
            (sat >= self._s_lo) & (val >= self._v_lo)
        )

        red_pixels = np.sum(red_mask)
        if red_pixels < self._min_blob:
            return None

        ys, xs = np.where(red_mask)
        cx = np.mean(xs)

        confidence = min(1.0, red_pixels / (self._min_blob * 5))

        bearing_offset = ((cx / w_img) - 0.5) * self._hfov
        robot_yaw = _yaw_from_quaternion(odom.pose.pose.orientation)
        world_bearing = robot_yaw + bearing_offset

        world_x = odom.pose.pose.position.x + self._depth * math.cos(world_bearing)
        world_y = odom.pose.pose.position.y + self._depth * math.sin(world_bearing)

        return {
            "x": round(world_x, 2),
            "y": round(world_y, 2),
            "confidence": round(confidence, 2),
            "robot": ns,
            "label": "red_block",
            "source": "red_hsv",
        }


def main(args=None):
    rclpy.init(args=args)
    node = RedBlockDetectorNode()
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
