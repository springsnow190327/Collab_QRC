#!/usr/bin/env python3
"""Map renderer: fused OccupancyGrid → annotated image for VLM consumption.

Subscribes to:
  - Fused OccupancyGrid
  - Per-robot Odometry (pose)
  - Skeleton image (from skeleton_extractor)
  - Generic artifact detections (placeholder now, camera-based later)

Publishes:
  - sensor_msgs/Image: annotated map image on /vlm/rendered_map
  - std_msgs/String: JSON scene description on /vlm/scene_json
"""

from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MapRendererNode(Node):
    def __init__(self):
        super().__init__("map_renderer")

        self.declare_parameter("map_topic", "/world/map")
        self.declare_parameter("robot_namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("skeleton_image_topic", "/vlm/skeleton_image")
        self.declare_parameter("artifact_detections_topic", "/vlm/artifact_detections")
        self.declare_parameter("rendered_map_topic", "/vlm/rendered_map")
        self.declare_parameter("scene_json_topic", "/vlm/scene_json")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("rate", 1.0)

        self._map_topic = self.get_parameter("map_topic").value
        self._namespaces = list(self.get_parameter("robot_namespaces").value)
        self._frame_id = self.get_parameter("frame_id").value
        rate = self.get_parameter("rate").value

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._map_sub = self.create_subscription(
            OccupancyGrid, self._map_topic, self._on_map, map_qos
        )
        self._skeleton_sub = self.create_subscription(
            Image,
            self.get_parameter("skeleton_image_topic").value,
            self._on_skeleton,
            10,
        )
        self._green_sub = self.create_subscription(
            String,
            self.get_parameter("artifact_detections_topic").value,
            self._on_artifact_detections,
            10,
        )

        self._odom_subs = {}
        self._odoms = {}
        for ns in self._namespaces:
            topic = f"/{ns}/odom/nav"
            self._odom_subs[ns] = self.create_subscription(
                Odometry, topic, lambda msg, _ns=ns: self._on_odom(_ns, msg), 10
            )

        self._rendered_pub = self.create_publisher(
            Image,
            self.get_parameter("rendered_map_topic").value,
            10,
        )
        self._json_pub = self.create_publisher(
            String,
            self.get_parameter("scene_json_topic").value,
            10,
        )

        self._latest_map = None
        self._skeleton_img = None
        self._artifact_detections = []
        self._timer = self.create_timer(1.0 / rate, self._tick)

        # Robot colors: robot_a=green, robot_b=blue
        self._robot_colors = {
            self._namespaces[0]: (0, 200, 0) if len(self._namespaces) > 0 else (0, 200, 0),
        }
        if len(self._namespaces) > 1:
            self._robot_colors[self._namespaces[1]] = (0, 100, 255)

        self.get_logger().info(
            f"MapRenderer: map={self._map_topic} ns={self._namespaces}"
        )

    def _on_map(self, msg: OccupancyGrid):
        self._latest_map = msg

    def _on_odom(self, ns: str, msg: Odometry):
        self._odoms[ns] = msg

    def _on_skeleton(self, msg: Image):
        self._skeleton_img = msg

    def _on_artifact_detections(self, msg: String):
        try:
            self._artifact_detections = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self._artifact_detections = []

    def _tick(self):
        if self._latest_map is None:
            return

        grid_msg = self._latest_map
        w = grid_msg.info.width
        h = grid_msg.info.height
        res = grid_msg.info.resolution
        ox = grid_msg.info.origin.position.x
        oy = grid_msg.info.origin.position.y

        grid = np.array(grid_msg.data, dtype=np.int8).reshape((h, w))

        # Build RGB image: unknown=gray, free=white, occupied=black
        img = np.full((h, w, 3), 128, dtype=np.uint8)  # gray = unknown
        img[grid == 0] = [240, 240, 240]  # free = near-white
        img[(grid > 0) & (grid < 50)] = [220, 220, 220]  # low-confidence free
        img[grid >= 50] = [30, 30, 30]  # occupied = dark
        img[grid == -1] = [128, 128, 128]  # unknown

        # Overlay skeleton (yellow)
        if self._skeleton_img is not None:
            try:
                skel = np.frombuffer(self._skeleton_img.data, dtype=np.uint8).reshape((
                    self._skeleton_img.height, self._skeleton_img.width
                ))
                if skel.shape == (h, w):
                    mask = skel > 0
                    img[mask] = [0, 230, 230]  # yellow (BGR→RGB: cyan in image, yellow in RViz convention)
            except (ValueError, IndexError):
                pass

        # Overlay artifact detections (bright green circles for now)
        for det in self._artifact_detections:
            gx = int((det.get("x", 0) - ox) / res)
            gy = int((det.get("y", 0) - oy) / res)
            r = max(3, int(0.3 / res))
            self._draw_circle(img, gx, gy, r, (0, 255, 0))

        # Overlay robot positions
        scene_robots = {}
        for ns in self._namespaces:
            if ns not in self._odoms:
                continue
            odom = self._odoms[ns]
            rx = odom.pose.pose.position.x
            ry = odom.pose.pose.position.y
            yaw = _yaw_from_quaternion(odom.pose.pose.orientation)

            gx = int((rx - ox) / res)
            gy = int((ry - oy) / res)
            color = self._robot_colors.get(ns, (255, 0, 0))

            # Draw robot as filled circle with heading line
            self._draw_circle(img, gx, gy, max(4, int(0.3 / res)), color, filled=True)
            # Heading line
            hlen = int(0.5 / res)
            hx = gx + int(hlen * math.cos(yaw))
            hy = gy + int(hlen * math.sin(yaw))
            self._draw_line(img, gx, gy, hx, hy, color)

            scene_robots[ns] = {
                "position": [round(rx, 2), round(ry, 2)],
                "heading": round(yaw, 3),
            }

        # Flip vertically so Y-up in world = top of image
        img = np.flipud(img)

        # Publish rendered image (RGB8)
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = self._frame_id
        img_msg.height = h
        img_msg.width = w
        img_msg.encoding = "rgb8"
        img_msg.is_bigendian = False
        img_msg.step = w * 3
        img_msg.data = img.tobytes()
        self._rendered_pub.publish(img_msg)

        # Publish scene JSON
        scene = {
            "robot_states": scene_robots,
            "artifact_detections": self._artifact_detections,
            "map_info": {
                "width": w,
                "height": h,
                "resolution": res,
                "origin": [round(ox, 2), round(oy, 2)],
                "known_cells": int(np.count_nonzero(grid != -1)),
                "free_cells": int(np.count_nonzero((grid >= 0) & (grid < 50))),
            },
        }
        json_msg = String()
        json_msg.data = json.dumps(scene)
        self._json_pub.publish(json_msg)

    @staticmethod
    def _draw_circle(img, cx, cy, r, color, filled=False):
        h, w = img.shape[:2]
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if filled:
                    if dx * dx + dy * dy <= r * r:
                        y, x = cy + dy, cx + dx
                        if 0 <= y < h and 0 <= x < w:
                            img[y, x] = color
                else:
                    dist = abs(math.sqrt(dx * dx + dy * dy) - r)
                    if dist < 1.0:
                        y, x = cy + dy, cx + dx
                        if 0 <= y < h and 0 <= x < w:
                            img[y, x] = color

    @staticmethod
    def _draw_line(img, x0, y0, x1, y1, color):
        h, w = img.shape[:2]
        steps = max(abs(x1 - x0), abs(y1 - y0), 1)
        for i in range(steps + 1):
            t = i / steps
            x = int(x0 + t * (x1 - x0))
            y = int(y0 + t * (y1 - y0))
            if 0 <= y < h and 0 <= x < w:
                img[y, x] = color


def main(args=None):
    rclpy.init(args=args)
    node = MapRendererNode()
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
