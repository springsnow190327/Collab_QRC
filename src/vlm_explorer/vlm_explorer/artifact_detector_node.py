#!/usr/bin/env python3
"""Low-cost artifact detector with placeholder and HSV modes.

Default mode is placeholder so VLM integration can be wired without adding
camera inference cost. A lightweight HSV green detector remains available for
Gazebo marker experiments.
"""

from __future__ import annotations

import json
import math
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ArtifactDetectorNode(Node):
    def __init__(self):
        super().__init__("artifact_detector")

        self.declare_parameter("robot_namespaces", ["robot"])
        self.declare_parameter("detections_topic", "/vlm/artifact_detections")
        self.declare_parameter("rate", 1.0)
        self.declare_parameter("mode", "placeholder")  # placeholder | green_hsv
        self.declare_parameter("placeholder_detections_json", "[]")
        self.declare_parameter("label", "artifact")
        self.declare_parameter("hsv_h_low", 35)
        self.declare_parameter("hsv_h_high", 85)
        self.declare_parameter("hsv_s_low", 80)
        self.declare_parameter("hsv_v_low", 80)
        self.declare_parameter("min_blob_pixels", 200)
        self.declare_parameter("assumed_depth_m", 2.0)
        self.declare_parameter("camera_hfov_rad", 2.0944)
        self.declare_parameter("dedup_radius_m", 0.8)

        self._namespaces = [str(x) for x in self.get_parameter("robot_namespaces").value]
        self._rate = max(0.2, float(self.get_parameter("rate").value))
        self._mode = str(self.get_parameter("mode").value).strip().lower() or "placeholder"
        self._placeholder_json = str(self.get_parameter("placeholder_detections_json").value)
        self._label = str(self.get_parameter("label").value).strip() or "artifact"
        self._h_lo = int(self.get_parameter("hsv_h_low").value)
        self._h_hi = int(self.get_parameter("hsv_h_high").value)
        self._s_lo = int(self.get_parameter("hsv_s_low").value)
        self._v_lo = int(self.get_parameter("hsv_v_low").value)
        self._min_blob = int(self.get_parameter("min_blob_pixels").value)
        self._depth = float(self.get_parameter("assumed_depth_m").value)
        self._hfov = float(self.get_parameter("camera_hfov_rad").value)
        self._dedup_r = float(self.get_parameter("dedup_radius_m").value)

        self._det_pub = self.create_publisher(String, self.get_parameter("detections_topic").value, 10)
        self._images = {}
        self._odoms = {}
        self._registry: list[dict] = []

        if self._mode == "green_hsv":
            for ns in self._namespaces:
                cam_topic = f"/{ns}/front_camera/image_raw"
                odom_topic = f"/{ns}/odom/nav"
                self.create_subscription(Image, cam_topic, lambda msg, _ns=ns: self._on_image(_ns, msg), 10)
                self.create_subscription(Odometry, odom_topic, lambda msg, _ns=ns: self._on_odom(_ns, msg), 10)

        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            f"ArtifactDetector: mode={self._mode} ns={self._namespaces} rate={self._rate:.1f}Hz"
        )

    def _on_image(self, ns: str, msg: Image):
        self._images[ns] = msg

    def _on_odom(self, ns: str, msg: Odometry):
        self._odoms[ns] = msg

    def _tick(self):
        if self._mode == "placeholder":
            self._publish(self._load_placeholder())
            return

        detections = []
        for ns in self._namespaces:
            if ns not in self._images or ns not in self._odoms:
                continue
            det = self._detect_green(ns, self._images[ns], self._odoms[ns])
            if det is not None:
                detections.append(det)
        self._merge_and_publish(detections)

    def _load_placeholder(self) -> list[dict]:
        try:
            detections = json.loads(self._placeholder_json)
            if isinstance(detections, list):
                return detections
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid placeholder_detections_json; publishing []")
        return []

    def _merge_and_publish(self, new_dets: list[dict]):
        for d in new_dets:
            merged = False
            for existing in self._registry:
                dx = float(d["x"]) - float(existing["x"])
                dy = float(d["y"]) - float(existing["y"])
                if math.hypot(dx, dy) < self._dedup_r:
                    if float(d["confidence"]) > float(existing.get("confidence", 0.0)):
                        existing.update(d)
                    merged = True
                    break
            if not merged:
                d["id"] = f"artifact_{len(self._registry)}"
                self._registry.append(d)
        self._publish(self._registry)

    def _publish(self, detections: list[dict]):
        msg = String()
        msg.data = json.dumps(detections)
        self._det_pub.publish(msg)

    def _detect_green(self, ns: str, img_msg: Image, odom: Odometry) -> Optional[dict]:
        try:
            if img_msg.encoding not in ("rgb8", "bgr8"):
                return None
            img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape((img_msg.height, img_msg.width, 3))
            if img_msg.encoding == "bgr8":
                img = img[:, :, ::-1]
        except (ValueError, IndexError):
            return None

        h_img, w_img = img.shape[:2]
        r = img[:, :, 0].astype(float)
        g = img[:, :, 1].astype(float)
        b = img[:, :, 2].astype(float)
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
        hue = (hue / 2).astype(np.uint8)

        sat = np.zeros_like(mx)
        sat[mx > 0] = (diff[mx > 0] / mx[mx > 0]) * 255
        sat = sat.astype(np.uint8)
        val = mx.astype(np.uint8)

        green_mask = (
            (hue >= self._h_lo)
            & (hue <= self._h_hi)
            & (sat >= self._s_lo)
            & (val >= self._v_lo)
        )
        green_pixels = int(np.sum(green_mask))
        if green_pixels < self._min_blob:
            return None

        ys, xs = np.where(green_mask)
        cx = float(np.mean(xs))
        confidence = min(1.0, green_pixels / float(max(1, self._min_blob * 5)))
        bearing_offset = ((cx / max(1, w_img)) - 0.5) * self._hfov
        robot_yaw = _yaw_from_quaternion(odom.pose.pose.orientation)
        world_bearing = robot_yaw + bearing_offset
        world_x = odom.pose.pose.position.x + self._depth * math.cos(world_bearing)
        world_y = odom.pose.pose.position.y + self._depth * math.sin(world_bearing)
        return {
            "label": self._label,
            "x": round(world_x, 2),
            "y": round(world_y, 2),
            "confidence": round(confidence, 2),
            "robot": ns,
            "source": self._mode,
        }


def main(args=None):
    rclpy.init(args=args)
    node = ArtifactDetectorNode()
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
