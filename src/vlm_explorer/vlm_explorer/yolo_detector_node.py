#!/usr/bin/env python3
"""YOLO object detector: runs YOLOv8 on camera images at configurable rate.

Projects detected bounding-box centres to 2D world coordinates via
odometry bearing + assumed depth.  Publishes deduplicated detections
on /vlm/artifact_detections for the VLM coordinator.

Loads the model on a background thread so the node starts instantly.
"""

from __future__ import annotations

import json
import math
import threading
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


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        self.declare_parameter("robot_namespaces", ["robot"])
        self.declare_parameter("detections_topic", "/vlm/artifact_detections")
        self.declare_parameter("rate", 2.0)
        self.declare_parameter("model_id", "yolov8n.pt")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("assumed_depth_m", 2.0)
        self.declare_parameter("camera_hfov_rad", 2.0944)
        self.declare_parameter("dedup_radius_m", 0.8)
        # Comma-separated COCO class names to ignore (e.g. "person,car")
        self.declare_parameter("ignore_classes", "")

        self._namespaces = list(self.get_parameter("robot_namespaces").value)
        self._rate = float(self.get_parameter("rate").value)
        self._model_id = str(self.get_parameter("model_id").value)
        self._device = str(self.get_parameter("device").value)
        self._conf_thresh = float(self.get_parameter("confidence_threshold").value)
        self._depth = float(self.get_parameter("assumed_depth_m").value)
        self._hfov = float(self.get_parameter("camera_hfov_rad").value)
        self._dedup_r = float(self.get_parameter("dedup_radius_m").value)

        ignore_raw = str(self.get_parameter("ignore_classes").value).strip()
        self._ignore_classes = set(
            c.strip().lower() for c in ignore_raw.split(",") if c.strip()
        )

        self._det_pub = self.create_publisher(
            String, self.get_parameter("detections_topic").value, 10
        )

        self._images: dict[str, Image] = {}
        self._odoms: dict[str, Odometry] = {}
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
        self._model = None
        self._model_ready = False

        # Load model in background so node doesn't block
        self._load_thread = threading.Thread(target=self._load_model, daemon=True)
        self._load_thread.start()

        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            f"YoloDetector: ns={self._namespaces} rate={self._rate}Hz "
            f"model={self._model_id} device={self._device}"
        )

    def _load_model(self):
        try:
            from ultralytics import YOLO
            self._model = YOLO(self._model_id)
            self._model_ready = True
            self.get_logger().info(f"YOLO model loaded: {self._model_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")

    def _on_image(self, ns: str, msg: Image):
        self._images[ns] = msg

    def _on_odom(self, ns: str, msg: Odometry):
        self._odoms[ns] = msg

    def _tick(self):
        if not self._model_ready:
            return

        new_dets: list[dict] = []
        for ns in self._namespaces:
            if ns not in self._images or ns not in self._odoms:
                continue
            dets = self._detect(ns, self._images[ns], self._odoms[ns])
            new_dets.extend(dets)

        for d in new_dets:
            merged = False
            for existing in self._registry:
                if existing["label"] != d["label"]:
                    continue
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
                d["id"] = f"yolo_{len(self._registry)}"
                self._registry.append(d)

        msg = String()
        msg.data = json.dumps(self._registry)
        self._det_pub.publish(msg)

    def _detect(
        self, ns: str, img_msg: Image, odom: Odometry
    ) -> list[dict]:
        try:
            if img_msg.encoding in ("rgb8", "bgr8"):
                img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
                    (img_msg.height, img_msg.width, 3)
                )
                if img_msg.encoding == "rgb8":
                    img = img[:, :, ::-1]  # YOLO expects BGR
            else:
                return []
        except (ValueError, IndexError):
            return []

        h_img, w_img = img.shape[:2]
        results = self._model(img, verbose=False, conf=self._conf_thresh)

        if not results or len(results[0].boxes) == 0:
            return []

        robot_yaw = _yaw_from_quaternion(odom.pose.pose.orientation)
        rx = odom.pose.pose.position.x
        ry = odom.pose.pose.position.y

        dets = []
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            label = self._model.names.get(cls_id, f"class_{cls_id}").lower()
            if label in self._ignore_classes:
                continue

            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx = (x1 + x2) / 2.0

            bearing_offset = ((cx / w_img) - 0.5) * self._hfov
            world_bearing = robot_yaw + bearing_offset
            world_x = rx + self._depth * math.cos(world_bearing)
            world_y = ry + self._depth * math.sin(world_bearing)

            dets.append({
                "x": round(world_x, 2),
                "y": round(world_y, 2),
                "confidence": round(conf, 2),
                "robot": ns,
                "label": label,
                "source": "yolo",
            })

        return dets


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
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
