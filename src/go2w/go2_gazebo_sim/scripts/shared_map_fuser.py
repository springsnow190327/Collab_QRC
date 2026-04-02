#!/usr/bin/env python3
"""Fuse per-robot occupancy grids into a shared global map topic.

This node is intended for dual-robot Gazebo runs where each robot publishes
its own local occupancy grid (e.g. /robot_a/map, /robot_b/map), but the
coordinator expects a shared global map topic (e.g. /disco_slam/global_map).
"""

from __future__ import annotations

from copy import deepcopy
from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _grid_index(x: int, y: int, width: int) -> int:
    return (y * width) + x


def _world_to_grid(msg: OccupancyGrid, wx: float, wy: float) -> Optional[tuple[int, int]]:
    res = float(msg.info.resolution)
    if res <= 1e-9:
        return None
    gx = int((wx - msg.info.origin.position.x) / res)
    gy = int((wy - msg.info.origin.position.y) / res)
    if gx < 0 or gy < 0 or gx >= int(msg.info.width) or gy >= int(msg.info.height):
        return None
    return (gx, gy)


def _grid_to_world(msg: OccupancyGrid, gx: int, gy: int) -> tuple[float, float]:
    return (
        msg.info.origin.position.x + (gx + 0.5) * msg.info.resolution,
        msg.info.origin.position.y + (gy + 0.5) * msg.info.resolution,
    )


def _merge_cell_value(dst_value: int, src_value: int, *, unknown_value: int, free_value: int, occ_threshold: int) -> int:
    if src_value == unknown_value:
        return dst_value

    src_occ = src_value >= occ_threshold
    dst_occ = dst_value >= occ_threshold

    # Occupied evidence dominates free/unknown.
    if src_occ:
        return src_value
    if dst_occ:
        return dst_value

    # Prefer known over unknown.
    if dst_value == unknown_value:
        return src_value

    # Preserve explicit free when available.
    if src_value == free_value:
        return free_value
    if dst_value == free_value:
        return free_value

    return src_value


class SharedMapFuser(Node):
    def __init__(self) -> None:
        super().__init__("shared_map_fuser")

        self.declare_parameter("map_a_topic", "/robot_a/map")
        self.declare_parameter("map_b_topic", "/robot_b/map")
        self.declare_parameter("output_topic", "/disco_slam/global_map")
        self.declare_parameter("publish_rate", 2.0)
        self.declare_parameter("frame_id", "")
        self.declare_parameter("unknown_value", -1)
        self.declare_parameter("free_value", 0)
        self.declare_parameter("occupancy_block_threshold", 65)

        self.map_a_topic = str(self.get_parameter("map_a_topic").value)
        self.map_b_topic = str(self.get_parameter("map_b_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.publish_rate = max(0.2, float(self.get_parameter("publish_rate").value))
        self.frame_id = str(self.get_parameter("frame_id").value).strip()
        self.unknown_value = int(self.get_parameter("unknown_value").value)
        self.free_value = int(self.get_parameter("free_value").value)
        self.occ_threshold = int(self.get_parameter("occupancy_block_threshold").value)

        self._map_a: Optional[OccupancyGrid] = None
        self._map_b: Optional[OccupancyGrid] = None
        self._published_once = False

        # Upstream map publishers (simple_scan_mapper_cpp) use VOLATILE durability.
        # Subscriptions must match that to receive data.
        map_sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Keep fused output latched so late-joining consumers can get latest map.
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(OccupancyGrid, self.map_a_topic, self._map_a_cb, map_sub_qos)
        self.create_subscription(OccupancyGrid, self.map_b_topic, self._map_b_cb, map_sub_qos)
        self.pub = self.create_publisher(OccupancyGrid, self.output_topic, output_qos)

        self.timer = self.create_timer(1.0 / self.publish_rate, self._tick)
        self.get_logger().info(
            f"SharedMapFuser started: {self.map_a_topic}, {self.map_b_topic} -> {self.output_topic}"
        )

    def _map_a_cb(self, msg: OccupancyGrid) -> None:
        self._map_a = msg

    def _map_b_cb(self, msg: OccupancyGrid) -> None:
        self._map_b = msg

    def _copy_map(self, src: OccupancyGrid) -> OccupancyGrid:
        out = OccupancyGrid()
        out.header = src.header
        out.info = deepcopy(src.info)
        out.data = list(src.data)
        return out

    def _overlay(self, dst: OccupancyGrid, src: OccupancyGrid) -> None:
        sw = int(src.info.width)
        sh = int(src.info.height)
        if sw <= 0 or sh <= 0:
            return

        dw = int(dst.info.width)
        ddata = list(dst.data)
        sdata = src.data

        for gy in range(sh):
            srow = gy * sw
            for gx in range(sw):
                sidx = srow + gx
                sval = int(sdata[sidx])
                if sval == self.unknown_value:
                    continue

                wx, wy = _grid_to_world(src, gx, gy)
                dg = _world_to_grid(dst, wx, wy)
                if dg is None:
                    continue

                didx = _grid_index(dg[0], dg[1], dw)
                ddata[didx] = _merge_cell_value(
                    int(ddata[didx]),
                    sval,
                    unknown_value=self.unknown_value,
                    free_value=self.free_value,
                    occ_threshold=self.occ_threshold,
                )

        dst.data = ddata

    def _tick(self) -> None:
        base = self._map_a if self._map_a is not None else self._map_b
        if base is None:
            return

        merged = self._copy_map(base)
        for src in (self._map_a, self._map_b):
            if src is None or src is base:
                continue
            self._overlay(merged, src)

        merged.header.stamp = self.get_clock().now().to_msg()
        if self.frame_id:
            merged.header.frame_id = self.frame_id

        self.pub.publish(merged)

        if not self._published_once:
            self._published_once = True
            self.get_logger().info(
                f"Publishing fused shared map on {self.output_topic} "
                f"(frame={merged.header.frame_id}, size={merged.info.width}x{merged.info.height}, "
                f"res={merged.info.resolution:.3f})"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SharedMapFuser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
