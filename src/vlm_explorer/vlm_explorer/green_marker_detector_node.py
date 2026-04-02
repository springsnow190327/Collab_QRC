#!/usr/bin/env python3
"""Green marker detector: HSV-based green blob detection from camera images.

Detects bright green objects in the camera feed and estimates their 2D world
position using the robot's odometry and a simple bearing projection.

Publishes detections as JSON on /vlm/green_detections:
  [{"id": "green_0", "x": 2.5, "y": 3.1, "confidence": 0.85, "robot": "robot_a"}]
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


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GreenMarkerDetectorNode(Node):
    def __init__(self):
        super().__init__("green_marker_detector")

        self.declare_parameter("robot_namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("detections_topic", "/vlm/green_detections")
        self.declare_parameter("rate", 2.0)
        # HSV thresholds for green detection
        self.declare_parameter("hsv_h_low", 35)
        self.declare_parameter("hsv_h_high", 85)
        self.declare_parameter("hsv_s_low", 80)
        self.declare_parameter("hsv_v_low", 80)
        self.declare_parameter("min_blob_pixels", 200)
        # Assumed projection distance (meters) when depth is unknown
        self.declare_parameter("assumed_depth_m", 2.0)
        # Camera HFOV in radians (120 deg)
        self.declare_parameter("camera_hfov_rad", 2.0944)
        # Dedup radius: merge detections within this distance
        self.declare_parameter("dedup_radius_m", 0.8)

        self._namespaces = list(self.get_parameter("robot_namespaces").value)
        self._rate = self.get_parameter("rate").value
        self._h_lo = self.get_parameter("hsv_h_low").value
        self._h_hi = self.get_parameter("hsv_h_high").value
        self._s_lo = self.get_parameter("hsv_s_low").value
        self._v_lo = self.get_parameter("hsv_v_low").value
        self._min_blob = self.get_parameter("min_blob_pixels").value
        self._depth = self.get_parameter("assumed_depth_m").value
        self._hfov = self.get_parameter("camera_hfov_rad").value
        self._dedup_r = self.get_parameter("dedup_radius_m").value

        self._det_pub = self.create_publisher(
            String, self.get_parameter("detections_topic").value, 10
        )

        # Per-robot camera + odom subscriptions
        self._images = {}
        self._odoms = {}
        for ns in self._namespaces:
            cam_topic = f"/{ns}/front_camera/image_raw"
            odom_topic = f"/{ns}/odom/nav"
            self.create_subscription(
                Image, cam_topic, lambda msg, _ns=ns: self._on_image(_ns, msg), 10
            )
            self.create_subscription(
                Odometry, odom_topic, lambda msg, _ns=ns: self._on_odom(_ns, msg), 10
            )

        # Global detection registry (deduplicated)
        self._registry: list[dict] = []

        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            f"GreenMarkerDetector: ns={self._namespaces} rate={self._rate}Hz"
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
            det = self._detect_green(ns, self._images[ns], self._odoms[ns])
            if det is not None:
                new_dets.append(det)

        # Merge into global registry with dedup
        for d in new_dets:
            merged = False
            for existing in self._registry:
                dx = d["x"] - existing["x"]
                dy = d["y"] - existing["y"]
                if math.sqrt(dx * dx + dy * dy) < self._dedup_r:
                    # Update with higher confidence
                    if d["confidence"] > existing["confidence"]:
                        existing["x"] = d["x"]
                        existing["y"] = d["y"]
                        existing["confidence"] = d["confidence"]
                        existing["robot"] = d["robot"]
                    merged = True
                    break
            if not merged:
                d["id"] = f"green_{len(self._registry)}"
                self._registry.append(d)

        # Publish full registry
        msg = String()
        msg.data = json.dumps(self._registry)
        self._det_pub.publish(msg)

    def _detect_green(self, ns: str, img_msg: Image, odom: Odometry) -> Optional[dict]:
        """Detect largest green blob and estimate world position."""
        # Decode image
        try:
            if img_msg.encoding in ("rgb8", "bgr8"):
                img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
                    (img_msg.height, img_msg.width, 3)
                )
                if img_msg.encoding == "bgr8":
                    img = img[:, :, ::-1]  # BGR→RGB
            else:
                return None
        except (ValueError, IndexError):
            return None

        h_img, w_img = img.shape[:2]

        # RGB → HSV (manual, no OpenCV dependency)
        r, g, b = img[:, :, 0].astype(float), img[:, :, 1].astype(float), img[:, :, 2].astype(float)
        mx = np.maximum(np.maximum(r, g), b)
        mn = np.minimum(np.minimum(r, g), b)
        diff = mx - mn

        # Hue (0-180 range like OpenCV)
        hue = np.zeros_like(mx)
        mask_r = (mx == r) & (diff > 0)
        mask_g = (mx == g) & (diff > 0)
        mask_b = (mx == b) & (diff > 0)
        hue[mask_r] = (60 * ((g[mask_r] - b[mask_r]) / diff[mask_r]) + 360) % 360
        hue[mask_g] = (60 * ((b[mask_g] - r[mask_g]) / diff[mask_g]) + 120)
        hue[mask_b] = (60 * ((r[mask_b] - g[mask_b]) / diff[mask_b]) + 240)
        hue = (hue / 2).astype(np.uint8)  # Scale to 0-180

        sat = np.zeros_like(mx)
        sat[mx > 0] = (diff[mx > 0] / mx[mx > 0]) * 255
        sat = sat.astype(np.uint8)

        val = mx.astype(np.uint8)

        # Green mask
        green_mask = (
            (hue >= self._h_lo) & (hue <= self._h_hi) &
            (sat >= self._s_lo) & (val >= self._v_lo)
        )

        green_pixels = np.sum(green_mask)
        if green_pixels < self._min_blob:
            return None

        # Find centroid of green pixels
        ys, xs = np.where(green_mask)
        cx = np.mean(xs)
        cy = np.mean(ys)

        # Confidence based on blob size
        confidence = min(1.0, green_pixels / (self._min_blob * 5))

        # Estimate world position from bearing
        # Horizontal angle from center of image
        bearing_offset = ((cx / w_img) - 0.5) * self._hfov
        robot_yaw = _yaw_from_quaternion(odom.pose.pose.orientation)
        world_bearing = robot_yaw + bearing_offset

        # Crude depth estimate from blob size (larger blob = closer)
        # blob_fraction = green_pixels / (h_img * w_img)
        # Use assumed depth for now
        depth = self._depth

        world_x = odom.pose.pose.position.x + depth * math.cos(world_bearing)
        world_y = odom.pose.pose.position.y + depth * math.sin(world_bearing)

        return {
            "x": round(world_x, 2),
            "y": round(world_y, 2),
            "confidence": round(confidence, 2),
            "robot": ns,
        }


def main(args=None):
    rclpy.init(args=args)
    node = GreenMarkerDetectorNode()
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
