#!/usr/bin/env python3
"""ROS 1 traversability filter chain + OccupancyGrid converter.

Subscribes to /robot/elevation_map_raw (grid_map_msgs/GridMap) published by
elevation_mapping_cupy.  Runs the full analytical filter chain in numpy,
fuses the CNN traversability layer with the analytical ramp_safe mask, and
publishes /robot/traversability_grid (nav_msgs/OccupancyGrid) for bridging
to the laptop Nav2 stack via ros1_bridge.

Filter chain implemented here matches grid_map_filters.yaml exactly:
  elevation → normals → slope → roughness → step_height → step_residual
  → trav_eth (analytical)  +  traversability (CNN from elevation_mapping_cupy)
  → ramp_safe (trapezoidal slope × step_margin)
  → trav_fused = max(CNN, ramp_safe) → OccupancyGrid

GridMap data layout (empirically verified against upstream elevation_mapping_cupy):
  Float32MultiArray with dim[0]="column_index" (y-axis, col=0=max-y) and
  dim[1]="row_index" (x-axis, row=0=max-x).  Raw flat array is stored
  column-major (data.T.flatten()).
  Decode: reshape(N_y, N_x)[::-1, ::-1] → world array [row=y_min→max, col=x_min→max]
  matching OccupancyGrid convention origin=(min_x, min_y).

Node runs under ns=robot, so topics resolve as:
  Sub:  /robot/elevation_map_raw
  Pub:  /robot/traversability_grid
"""

import math
import numpy as np
from scipy.ndimage import maximum_filter, minimum_filter, uniform_filter

import rospy
from nav_msgs.msg import OccupancyGrid
from grid_map_msgs.msg import GridMap


# ---------------------------------------------------------------------------
# GridMap decode
# ---------------------------------------------------------------------------

def decode_layer(gm: GridMap, name: str):
    """Decode a named GridMap layer into a world-indexed (row=y↑, col=x→) array.

    Returns None if the layer is absent or malformed.
    """
    if name not in gm.layers:
        return None
    idx = gm.layers.index(name)
    ma = gm.data[idx]
    if len(ma.layout.dim) < 2:
        return None
    # dim[0] = column_index (y-axis, outer), dim[1] = row_index (x-axis, inner)
    N_y = ma.layout.dim[0].size
    N_x = ma.layout.dim[1].size
    arr = np.array(ma.data, dtype=np.float32).reshape(N_y, N_x)
    # Both axes stored max→min; flip to world convention (min→max for both).
    return arr[::-1, ::-1].copy()


# ---------------------------------------------------------------------------
# Analytical filter chain (mirrors grid_map_filters.yaml)
# ---------------------------------------------------------------------------

def _safe_fill(arr, valid):
    """Replace NaN with array median for window operations."""
    if not np.any(valid):
        return np.zeros_like(arr)
    return np.where(valid, arr, float(np.nanmedian(arr)))


def run_filter_chain(elevation, traversability_cnn, resolution):
    """Run the full trav_cost_filters chain in numpy.

    Parameters
    ----------
    elevation          : (H, W) float32, NaN = unknown/no-data
    traversability_cnn : (H, W) float32 in [0,1], CNN output from elevation_mapping_cupy
    resolution         : float, metres per cell

    Returns
    -------
    dict with keys: slope, step_residual, step_height, trav_eth, ramp_safe, trav_fused
    """
    valid = np.isfinite(elevation)
    elev_f = _safe_fill(elevation, valid)

    # ---- Surface normals via elevation gradient ----
    # np.gradient axis=0 = row = y direction; axis=1 = col = x direction.
    dz_dy = np.gradient(elev_f, resolution, axis=0)
    dz_dx = np.gradient(elev_f, resolution, axis=1)
    dz_dy[~valid] = np.nan
    dz_dx[~valid] = np.nan
    # Normal z-component: nz = 1 / sqrt(dx² + dy² + 1)
    nz = 1.0 / np.sqrt(dz_dx ** 2 + dz_dy ** 2 + 1.0)
    slope = np.arccos(np.clip(nz, 0.0, 1.0))  # radians
    slope[~valid] = np.nan

    # ---- Slope cost: linear 0→1 for 0→30° ----
    slope_cost = np.clip(slope / 0.5236, 0.0, None)

    # ---- Roughness: RMS of elevation deviation in 5-cell window ----
    mean5 = uniform_filter(elev_f, size=5)
    sq5 = uniform_filter((elev_f - mean5) ** 2, size=5)
    roughness = np.sqrt(np.maximum(sq5, 0.0))
    roughness[~valid] = np.nan
    roughness_cost = np.clip(roughness / 0.05, 0.0, None)

    # ---- Step height: max−min in 3-cell window ----
    e_for_max = np.where(valid, elevation, -1e6)
    e_for_min = np.where(valid, elevation, +1e6)
    step_height = np.where(valid, maximum_filter(e_for_max, size=3) - minimum_filter(e_for_min, size=3), np.nan)
    step_height = np.clip(step_height, 0.0, None)

    # ---- Step residual: slope-compensated step (removes expected ramp rise) ----
    step_residual = np.maximum(step_height - np.tan(slope) * 0.30, 0.0)
    step_cost = np.clip(step_residual / 0.20, 0.0, None)

    # ---- Wall seed + 7-cell dilation (catches wall tops beyond 1-cell rim) ----
    wall_seed = np.maximum(step_height - 0.25, 0.0)
    wall_dilated = maximum_filter(np.where(np.isfinite(wall_seed), wall_seed, 0.0), size=7)
    wall_cost = np.clip(wall_dilated / 0.50, 0.0, None)

    # ---- trav_eth: multiplicative analytical traversability ----
    trav_eth = np.clip(
        (1.0 - slope_cost) * (1.0 - roughness_cost) * (1.0 - step_cost) * (1.0 - wall_cost),
        0.0, 1.0,
    )
    trav_eth[~valid] = np.nan

    # ---- ramp_safe: trapezoidal slope window [8°,12°..24°,30°] × step_margin ----
    # Trapezoidal: zero below 8°, ramp up 8°→12°, flat 12°→24°, ramp down 24°→30°.
    slope_floor = np.clip((slope - math.radians(8)) / math.radians(4), 0.0, 1.0)
    slope_ceil = np.clip((math.radians(30) - slope) / math.radians(6), 0.0, 1.0)
    step_margin = np.clip((0.06 - step_residual) / 0.06, 0.0, 1.0)
    ramp_safe = np.clip(slope_floor * slope_ceil * step_margin, 0.0, 1.0)
    ramp_safe[~valid] = np.nan

    # ---- trav_fused = max(CNN traversability, ramp_safe) ----
    cnn = np.where(np.isfinite(traversability_cnn), traversability_cnn, 0.0)
    rs = np.where(np.isfinite(ramp_safe), ramp_safe, 0.0)
    trav_fused = np.clip(np.maximum(cnn, rs), 0.0, 1.0)
    trav_fused[~valid] = np.nan

    return {
        "slope": slope,
        "step_height": step_height,
        "step_residual": step_residual,
        "trav_eth": trav_eth,
        "ramp_safe": ramp_safe,
        "trav_fused": trav_fused,
    }


# ---------------------------------------------------------------------------
# Traversability → OccupancyGrid cost
# ---------------------------------------------------------------------------

def trav_to_occ(trav, free_thresh, lethal_thresh):
    """Convert traversability [0,1] to OccupancyGrid int8 [0,100], −1=unknown."""
    cost = np.full(trav.shape, -1, dtype=np.int8)
    valid = np.isfinite(trav)
    if not np.any(valid):
        return cost
    t = np.clip(trav[valid], 0.0, 1.0)
    c = np.empty(t.shape, dtype=np.int16)
    free = t >= free_thresh
    lethal = t < lethal_thresh
    mid = ~(free | lethal)
    c[free] = 0
    c[lethal] = 100
    if np.any(mid):
        raw = (free_thresh - t[mid]) / (free_thresh - lethal_thresh) * 100.0
        c[mid] = np.clip(np.rint(raw), 1, 99).astype(np.int16)
    cost[valid] = c.astype(np.int8)
    return cost


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class TravFilterOccGrid:
    """elevation_map_raw (GridMap) → filter chain → traversability_grid (OccupancyGrid)."""

    def __init__(self):
        rospy.init_node("trav_filter_occ_grid", anonymous=False)

        self.free_thresh = float(rospy.get_param("~free_threshold", 0.60))
        self.lethal_thresh = float(rospy.get_param("~lethal_threshold", 0.30))
        # fixed_grid_cells: side length of the world-fixed output grid.
        # At 0.10 m resolution: 1000 cells = 100 m square.
        self.fixed_grid_cells = int(rospy.get_param("~fixed_grid_cells", 1000))
        # Radius around the robot to stamp unknown cells free (Mid-360 blind disk).
        self.blind_radius_m = float(rospy.get_param("~blind_disk_radius_m", 3.0))

        # World-fixed accumulation buffer (locked on first message).
        self._fixed = None
        self._fixed_ox = None
        self._fixed_oy = None
        self._fixed_res = None

        self._sub = rospy.Subscriber(
            "elevation_map_raw", GridMap, self._cb, queue_size=1,
        )
        self._pub = rospy.Publisher(
            "traversability_grid", OccupancyGrid, queue_size=1,
        )
        rospy.loginfo("[trav_filter_occ_grid] ready — waiting for elevation_map_raw")

    # ------------------------------------------------------------------
    # Main callback
    # ------------------------------------------------------------------

    def _cb(self, msg):
        elevation = decode_layer(msg, "elevation")
        if elevation is None:
            rospy.logwarn_throttle(5.0, "[trav_filter_occ_grid] elevation layer missing")
            return

        cnn = decode_layer(msg, "traversability")
        if cnn is None:
            cnn = np.full_like(elevation, np.nan)

        res = msg.info.resolution
        cx = msg.info.pose.position.x
        cy = msg.info.pose.position.y
        H, W = elevation.shape
        # Bottom-left (min_x, min_y) origin of the rolling GridMap window.
        ox = cx - W * res / 2.0
        oy = cy - H * res / 2.0

        # Filter chain.
        layers = run_filter_chain(elevation, cnn, res)
        trav = layers["trav_fused"]

        # Stamp Mid-360 blind disk (no ground returns within ~3 m) as free.
        if self.blind_radius_m > 0.0:
            # Robot centre is the GridMap centre in world coords.
            rc = int(round((cx - ox) / res - 0.5))  # robot col in rolling grid
            rr = int(round((cy - oy) / res - 0.5))  # robot row in rolling grid
            pr = int(math.ceil(self.blind_radius_m / res))
            r0, r1 = max(0, rr - pr), min(H, rr + pr + 1)
            c0, c1 = max(0, rc - pr), min(W, rc + pr + 1)
            yi = np.arange(r0, r1)[:, None]
            xi = np.arange(c0, c1)[None, :]
            in_disk = np.hypot((xi - rc) * res, (yi - rr) * res) <= self.blind_radius_m
            patch = trav[r0:r1, c0:c1]
            patch[in_disk & ~np.isfinite(patch)] = 1.0

        # Convert to occupancy cost.
        occ = trav_to_occ(trav, self.free_thresh, self.lethal_thresh)

        # Initialise world-fixed buffer on first message.
        if self._fixed is None:
            N = self.fixed_grid_cells
            self._fixed = np.full((N, N), -1, dtype=np.int8)
            self._fixed_ox = cx - N * res / 2.0
            self._fixed_oy = cy - N * res / 2.0
            self._fixed_res = res
            rospy.loginfo(
                f"[trav_filter_occ_grid] fixed grid origin "
                f"({self._fixed_ox:.1f}, {self._fixed_oy:.1f}), "
                f"{N}×{N} cells @ {res} m/cell"
            )

        self._project(occ, ox, oy, res)
        self._publish(msg)

    # ------------------------------------------------------------------
    # Project rolling occupancy into the world-fixed buffer
    # ------------------------------------------------------------------

    def _project(self, rolling, ox, oy, res):
        fx, fy = self._fixed_ox, self._fixed_oy
        N = self.fixed_grid_cells
        H, W = rolling.shape

        # World x/y centre of each rolling-grid cell.
        xs = ox + (np.arange(W, dtype=np.float64) + 0.5) * res
        ys = oy + (np.arange(H, dtype=np.float64) + 0.5) * res

        # Target column/row in the fixed grid.
        dc = np.floor((xs - fx) / res).astype(np.int64)
        dr = np.floor((ys - fy) / res).astype(np.int64)

        in_c = (dc >= 0) & (dc < N)
        in_r = (dr >= 0) & (dr < N)
        if not (np.any(in_c) and np.any(in_r)):
            return

        sc = np.nonzero(in_c)[0]
        sr = np.nonzero(in_r)[0]
        vals = rolling[np.ix_(sr, sc)]
        yy, xx = np.meshgrid(dr[sr], dc[sc], indexing="ij")

        observed = vals >= 0
        if np.any(observed):
            self._fixed[yy[observed], xx[observed]] = vals[observed]

    # ------------------------------------------------------------------
    # Publish world-fixed OccupancyGrid
    # ------------------------------------------------------------------

    def _publish(self, ref_msg):
        N = self.fixed_grid_cells
        msg = OccupancyGrid()
        msg.header.stamp = ref_msg.info.header.stamp
        msg.header.frame_id = ref_msg.info.header.frame_id
        msg.info.resolution = self._fixed_res
        msg.info.width = N
        msg.info.height = N
        msg.info.origin.position.x = self._fixed_ox
        msg.info.origin.position.y = self._fixed_oy
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self._fixed.flatten(order="C").tolist()
        self._pub.publish(msg)


def main():
    node = TravFilterOccGrid()
    rospy.spin()


if __name__ == "__main__":
    main()
