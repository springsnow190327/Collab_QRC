#!/usr/bin/env python3
"""map_augmenter — fold swarm-shared knowledge back into THIS robot's
own local OccupancyGrid.

Architecture (matches what real-robot multi-agent deployment looks like):

    [own LiDAR]  ──>  octomap_server  ──>  /{ns}/map_raw   (purely local)
                                                ↓
    [swarm  data]  ──>  /merged_map  ──>  map_augmenter (per ns)
                                                ↓
                                           /{ns}/map      (augmented)
                                                ↓
                                  astar_nav, safety monitor, rviz, …

Cell-merge rule: local cell wins when KNOWN; the merged map only fills
positions where local is unknown. Local sensor values are always trusted
over swarm contributions; swarm contributions only ever populate the
gaps. Output preserves the local map's geometry exactly (same origin,
resolution, width, height) so downstream planners see a drop-in
replacement for the raw octomap topic.

Why this rather than astar_nav subscribing to /merged_map directly:

    The "let merged-map update my map" framing is what real robots need.
    On-robot, each platform maintains its own occupancy representation;
    contributions from peers arrive as messages over a wireless link and
    get reconciled into the local map. There is no centralised planner
    state. multirobot_map_merge in sim is just one source of those
    contributions; on a real swarm it could be replaced by peer-to-peer
    octomap-diff messages or a shared-key-frame matcher without changing
    this node or the planner.

Frame alignment: this node assumes both maps live in the same world
frame (in MuJoCo sim with GT-bootstrapped init poses, both /merged_map
and /{ns}/map_raw use the same global "map" frame). On real robots the
caller is responsible for ensuring the swarm-merge upstream has aligned
the maps before publishing /merged_map. If frames differ, this node
will produce a corrupted output — a deliberate failure rather than
silent miscompose.
"""
from __future__ import annotations
import array
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)


class MapAugmenter(Node):
    def __init__(self) -> None:
        super().__init__("map_augmenter")
        # Topic names are relative — launch puts the node in /{ns}, so
        # `map_raw` resolves to /{ns}/map_raw and `map` to /{ns}/map.
        # `merged_map_topic` is absolute by default so all robots share
        # one swarm view.
        self.declare_parameter("local_map_topic", "map_raw")
        self.declare_parameter("merged_map_topic", "/merged_map")
        self.declare_parameter("augmented_map_topic", "map")
        # Republish at the local map's update rate by default. We
        # passthrough on every local message; the timer is just a
        # heartbeat for the case where local stops updating but merged
        # keeps flowing (so the planner still gets the latest swarm
        # view through us).
        self.declare_parameter("heartbeat_rate_hz", 1.0)

        local_topic = self.get_parameter("local_map_topic").value
        merged_topic = self.get_parameter("merged_map_topic").value
        out_topic = self.get_parameter("augmented_map_topic").value
        hb_rate = max(0.1, float(self.get_parameter("heartbeat_rate_hz").value))

        # Octomap and multirobot_map_merge both publish RELIABLE +
        # TRANSIENT_LOCAL — match or DDS silently drops messages.
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )

        self._local_msg: OccupancyGrid | None = None
        self._merged_msg: OccupancyGrid | None = None
        self._merge_count = 0
        self._republish_count = 0

        self.create_subscription(OccupancyGrid, local_topic,
                                 self._on_local, map_qos)
        self.create_subscription(OccupancyGrid, merged_topic,
                                 self._on_merged, map_qos)
        self._pub = self.create_publisher(OccupancyGrid, out_topic, map_qos)
        self.create_timer(1.0 / hb_rate, self._heartbeat)
        self.get_logger().info(
            f"map_augmenter started: local={local_topic} merged={merged_topic} "
            f"→ {out_topic} (heartbeat={hb_rate:.1f} Hz)"
        )

    def _on_local(self, msg: OccupancyGrid) -> None:
        self._local_msg = msg
        self._publish_augmented()

    def _on_merged(self, msg: OccupancyGrid) -> None:
        self._merged_msg = msg
        # Don't republish on every merged update — wait for the next
        # local update or the heartbeat to drive cadence. Otherwise
        # downstream consumers see staircased timestamps.

    def _heartbeat(self) -> None:
        # Re-emit the last good augmented map even if no new local
        # arrived — gives downstream a fresh stamp every 1 s.
        if self._local_msg is not None:
            self._publish_augmented()

    def _publish_augmented(self) -> None:
        local = self._local_msg
        if local is None:
            return
        merged = self._merged_msg
        if merged is None:
            # No swarm data yet — pass local through unchanged so the
            # planner doesn't have to wait for swarm bootstrap.
            out = OccupancyGrid()
            out.header = local.header
            out.info = local.info
            out.data = local.data
            self._pub.publish(out)
            self._republish_count += 1
            return

        # Cell-by-cell merge. Output keeps local's geometry. For each
        # local cell index i with world coords (wx, wy), we compute the
        # corresponding merged cell index. Where local is unknown AND
        # merged is in-bounds AND merged is known, the merged value
        # populates the output.
        lw = int(local.info.width)
        lh = int(local.info.height)
        lres = float(local.info.resolution)
        lox = float(local.info.origin.position.x)
        loy = float(local.info.origin.position.y)

        mw = int(merged.info.width)
        mh = int(merged.info.height)
        mres = float(merged.info.resolution)
        mox = float(merged.info.origin.position.x)
        moy = float(merged.info.origin.position.y)

        if lw < 1 or lh < 1 or mw < 1 or mh < 1 or lres <= 0 or mres <= 0:
            return

        local_arr = np.frombuffer(bytes(local.data), dtype=np.int8)
        merged_arr = np.frombuffer(bytes(merged.data), dtype=np.int8)
        if local_arr.size != lw * lh or merged_arr.size != mw * mh:
            self.get_logger().warn(
                "Map size mismatch: local has %d cells (expected %d), "
                "merged has %d cells (expected %d). Passing local through."
                % (local_arr.size, lw * lh, merged_arr.size, mw * mh)
            )
            self._pub.publish(local)
            return

        # Vectorised world-coord → merged-cell-index mapping.
        gy_arr, gx_arr = np.divmod(np.arange(lw * lh, dtype=np.int64), lw)
        wx = lox + (gx_arr.astype(np.float64) + 0.5) * lres
        wy = loy + (gy_arr.astype(np.float64) + 0.5) * lres
        mgx = np.floor((wx - mox) / mres).astype(np.int64)
        mgy = np.floor((wy - moy) / mres).astype(np.int64)
        in_bounds = (mgx >= 0) & (mgx < mw) & (mgy >= 0) & (mgy < mh)

        # Output starts as a copy of local; only overwrite where local
        # is unknown (-1) AND merged provides a known value at the
        # same world position.
        out_arr = local_arr.copy()
        unknown_local = (local_arr < 0)
        # Safe lookup: clamp out-of-bounds to 0, mask result with in_bounds.
        midx = np.where(in_bounds, mgy * mw + mgx, 0)
        merged_value = merged_arr[midx]
        merged_known = (merged_value >= 0) & in_bounds
        replace_mask = unknown_local & merged_known
        if replace_mask.any():
            out_arr[replace_mask] = merged_value[replace_mask]
            self._merge_count += 1

        out = OccupancyGrid()
        out.header = local.header  # Keep local frame_id + stamp
        out.info = local.info
        # rclpy's OccupancyGrid.data setter asserts the value is a sequence
        # of int8s. The fast path is `array.array('b', ...)` (signed char,
        # zero-copy from a contiguous int8 numpy buffer); plain bytes
        # objects fail the assertion (each "int in [-128, 127]" check),
        # and `.tolist()` is correct but does a per-cell Python conversion
        # that costs ~40 ms on a 487×327 grid at 1 Hz.
        out.data = array.array("b", out_arr.astype(np.int8, copy=False).tobytes())
        self._pub.publish(out)
        self._republish_count += 1

        if self._republish_count % 10 == 0:
            n_total = lw * lh
            n_unknown_local = int(unknown_local.sum())
            n_filled = int(replace_mask.sum())
            self.get_logger().info(
                "augment: local has %d unknown cells, "
                "%d filled from merged (%.1f%% of unknown filled)"
                % (n_unknown_local, n_filled,
                   100.0 * n_filled / max(1, n_unknown_local))
            )


def main(argv=None) -> int:
    rclpy.init(args=argv if argv is not None else sys.argv)
    node = MapAugmenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
