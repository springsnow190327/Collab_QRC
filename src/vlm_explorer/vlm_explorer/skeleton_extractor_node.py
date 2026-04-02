#!/usr/bin/env python3
"""Skeleton extractor: Zhang-Suen thinning on fused OccupancyGrid free-space.

Publishes a topological skeleton as a MarkerArray (line strips) for RViz
and as a flattened binary image (sensor_msgs/Image) for the VLM map renderer.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray


def _zhang_suen_thinning(binary: np.ndarray) -> np.ndarray:
    """Zhang-Suen thinning on a binary image (1 = foreground, 0 = background).

    Returns a thinned binary image (skeleton).
    """
    img = binary.copy().astype(np.uint8)
    rows, cols = img.shape

    def _neighbours(r, c):
        """Return 8-connected neighbours in clockwise order starting from top."""
        return [
            img[r - 1, c], img[r - 1, c + 1], img[r, c + 1], img[r + 1, c + 1],
            img[r + 1, c], img[r + 1, c - 1], img[r, c - 1], img[r - 1, c - 1],
        ]

    def _transitions(neighbours):
        n = neighbours + [neighbours[0]]
        return sum(1 for i in range(len(n) - 1) if n[i] == 0 and n[i + 1] == 1)

    changed = True
    while changed:
        changed = False
        # Sub-iteration 1
        to_remove = []
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if img[r, c] != 1:
                    continue
                nb = _neighbours(r, c)
                s = sum(nb)
                if s < 2 or s > 6:
                    continue
                if _transitions(nb) != 1:
                    continue
                # p2 * p4 * p6
                if nb[0] * nb[2] * nb[4] != 0:
                    continue
                # p4 * p6 * p8
                if nb[2] * nb[4] * nb[6] != 0:
                    continue
                to_remove.append((r, c))
        for r, c in to_remove:
            img[r, c] = 0
            changed = True

        # Sub-iteration 2
        to_remove = []
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if img[r, c] != 1:
                    continue
                nb = _neighbours(r, c)
                s = sum(nb)
                if s < 2 or s > 6:
                    continue
                if _transitions(nb) != 1:
                    continue
                # p2 * p4 * p8
                if nb[0] * nb[2] * nb[6] != 0:
                    continue
                # p2 * p6 * p8
                if nb[0] * nb[4] * nb[6] != 0:
                    continue
                to_remove.append((r, c))
        for r, c in to_remove:
            img[r, c] = 0
            changed = True

    return img


class SkeletonExtractorNode(Node):
    def __init__(self):
        super().__init__("skeleton_extractor")

        self.declare_parameter("map_topic", "/world/map")
        self.declare_parameter("skeleton_marker_topic", "/vlm/skeleton_markers")
        self.declare_parameter("skeleton_image_topic", "/vlm/skeleton_image")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("rate", 1.0)
        self.declare_parameter("free_threshold", 50)
        self.declare_parameter("downsample", 2)

        self._map_topic = self.get_parameter("map_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        self._rate = self.get_parameter("rate").value
        self._free_thresh = self.get_parameter("free_threshold").value
        self._downsample = max(1, self.get_parameter("downsample").value)

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._map_sub = self.create_subscription(
            OccupancyGrid, self._map_topic, self._on_map, map_qos
        )
        self._marker_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter("skeleton_marker_topic").value,
            10,
        )
        self._image_pub = self.create_publisher(
            Image,
            self.get_parameter("skeleton_image_topic").value,
            10,
        )

        self._latest_map = None
        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            f"SkeletonExtractor: map={self._map_topic} rate={self._rate}Hz"
        )

    def _on_map(self, msg: OccupancyGrid):
        self._latest_map = msg

    def _tick(self):
        if self._latest_map is None:
            return

        msg = self._latest_map
        w, h = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        grid = np.array(msg.data, dtype=np.int8).reshape((h, w))

        # Free-space binary mask: free cells (0 <= val < free_thresh) = 1
        free = np.zeros_like(grid, dtype=np.uint8)
        free[(grid >= 0) & (grid < self._free_thresh)] = 1

        # Downsample for performance
        ds = self._downsample
        if ds > 1:
            free_ds = free[::ds, ::ds]
        else:
            free_ds = free

        # Zhang-Suen thinning
        skeleton = _zhang_suen_thinning(free_ds)

        # Publish skeleton as MarkerArray (yellow line strips)
        self._publish_markers(skeleton, ds, res, ox, oy, h, w, msg.header)

        # Publish skeleton as mono8 image for map renderer
        self._publish_image(skeleton, ds, h, w, msg.header)

    def _publish_markers(self, skeleton, ds, res, ox, oy, h, w, header):
        ma = MarkerArray()

        # Delete old markers
        del_marker = Marker()
        del_marker.header.frame_id = self._frame_id
        del_marker.header.stamp = header.stamp
        del_marker.action = Marker.DELETEALL
        ma.markers.append(del_marker)

        # Collect skeleton points
        ys, xs = np.where(skeleton == 1)
        if len(xs) == 0:
            self._marker_pub.publish(ma)
            return

        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = header.stamp
        marker.ns = "skeleton"
        marker.id = 1
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = res * ds * 0.8
        marker.scale.y = res * ds * 0.8
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.7
        marker.pose.orientation.w = 1.0

        from geometry_msgs.msg import Point
        for xi, yi in zip(xs, ys):
            pt = Point()
            pt.x = ox + (xi * ds + 0.5) * res
            pt.y = oy + (yi * ds + 0.5) * res
            pt.z = 0.05
            marker.points.append(pt)

        ma.markers.append(marker)
        self._marker_pub.publish(ma)

    def _publish_image(self, skeleton, ds, h, w, header):
        # Upscale skeleton back to original grid size
        full_h, full_w = h, w
        if ds > 1:
            full_skel = np.zeros((full_h, full_w), dtype=np.uint8)
            sh, sw = skeleton.shape
            for y in range(sh):
                for x in range(sw):
                    if skeleton[y, x]:
                        fy, fx = y * ds, x * ds
                        if fy < full_h and fx < full_w:
                            full_skel[fy, fx] = 255
        else:
            full_skel = (skeleton * 255).astype(np.uint8)

        img_msg = Image()
        img_msg.header = header
        img_msg.header.frame_id = self._frame_id
        img_msg.height = full_h
        img_msg.width = full_w
        img_msg.encoding = "mono8"
        img_msg.is_bigendian = False
        img_msg.step = full_w
        img_msg.data = full_skel.tobytes()
        self._image_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SkeletonExtractorNode()
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
