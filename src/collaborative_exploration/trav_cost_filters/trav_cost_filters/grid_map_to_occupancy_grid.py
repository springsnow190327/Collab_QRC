#!/usr/bin/env python3
"""Convert traversability layer of a GridMap to OccupancyGrid for Nav2.

Subscribes to a filtered GridMap topic (expects a 'trav_eth' layer
in [0, 1], where 1 = fully traversable, 0 = blocked).
Layer name is 'trav_eth' (not 'traversability') to avoid aliasing with the
pre-existing all-NaN traversability layer from elevation_mapping_cupy.

Publishes OccupancyGrid in Nav2 cost convention:
  0   = free  (trav >= free_threshold)
  100 = lethal (trav < lethal_threshold)
  <linear interpolation in between>

This gives Nav2 a traversability-weighted costmap layer (via the StaticLayer
or a custom costmap plugin that reads OccupancyGrid).
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid


class GridMapToOccupancyGrid(Node):
    def __init__(self) -> None:
        super().__init__("grid_map_to_occupancy_grid")

        self.input_topic = self.declare_parameter(
            "input_topic", "elevation_map_filtered"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "traversability_grid"
        ).value
        self.traversability_layer = self.declare_parameter(
            "traversability_layer", "trav_eth"
        ).value
        # trav >= free_threshold → cost 0 (free)
        self.free_threshold = self.declare_parameter("free_threshold", 0.7).value
        # trav < lethal_threshold → cost 100 (lethal)
        self.lethal_threshold = self.declare_parameter("lethal_threshold", 0.3).value

        # Publisher uses TRANSIENT_LOCAL so late subscribers (diag tools, RViz,
        # CFPA2 BFS) receive the last grid immediately on connection.
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = self.create_publisher(OccupancyGrid, self.output_topic, pub_qos)
        self.sub = self.create_subscription(
            GridMap, self.input_topic, self._on_map, sub_qos
        )

        self.get_logger().info(
            f"grid_map_to_occupancy_grid: {self.input_topic} → {self.output_topic} "
            f"layer={self.traversability_layer} "
            f"free>={self.free_threshold} lethal<{self.lethal_threshold}"
        )

    def _on_map(self, msg: GridMap) -> None:
        if self.traversability_layer not in msg.layers:
            self.get_logger().warn_throttle(
                self.get_clock(), 5000,
                f"layer '{self.traversability_layer}' not in GridMap; "
                f"available: {list(msg.layers)}"
            )
            return

        layer_idx = list(msg.layers).index(self.traversability_layer)
        info = msg.info

        rows = info.length_y / info.resolution
        cols = info.length_x / info.resolution
        n_cells = int(round(rows)) * int(round(cols))

        data_float = np.array(msg.data[layer_idx].data, dtype=np.float32)

        if data_float.size != n_cells:
            self.get_logger().warn_throttle(
                self.get_clock(), 5000,
                f"size mismatch: data={data_float.size} expected={n_cells}"
            )
            return

        # Reshape: grid_map stores data column-major (x varies first for each column).
        # OccupancyGrid is row-major (x = column, y = row in grid_map convention).
        # grid_map cell (row i, col j) → index i*cols + j in OccupancyGrid.
        n_rows = int(round(rows))
        n_cols = int(round(cols))

        trav = data_float.reshape(n_cols, n_rows)  # (cols, rows) column-major
        trav = trav.T  # → (rows, cols) row-major for OccupancyGrid

        # Convert traversability [0,1] to occupancy [0,100].
        # NaN (unknown cells) → -1.
        cost = np.full(trav.shape, -1, dtype=np.int8)
        valid = np.isfinite(trav)

        t = trav[valid]
        c = np.where(
            t >= self.free_threshold,
            0,
            np.where(
                t < self.lethal_threshold,
                100,
                np.int8(
                    np.round(
                        (self.free_threshold - t)
                        / (self.free_threshold - self.lethal_threshold)
                        * 100
                    )
                ),
            ),
        )
        cost[valid] = c.astype(np.int8)

        occ = OccupancyGrid()
        occ.header.stamp = msg.header.stamp
        occ.header.frame_id = msg.header.frame_id
        occ.info.resolution = info.resolution
        occ.info.width = n_cols
        occ.info.height = n_rows
        # grid_map origin is centre; OccupancyGrid origin is bottom-left corner.
        occ.info.origin.position.x = (
            info.pose.position.x - info.length_x / 2.0
        )
        occ.info.origin.position.y = (
            info.pose.position.y - info.length_y / 2.0
        )
        occ.info.origin.position.z = info.pose.position.z
        occ.info.origin.orientation = info.pose.orientation
        occ.data = cost.flatten().tolist()

        self.pub.publish(occ)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GridMapToOccupancyGrid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
