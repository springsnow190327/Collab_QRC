#!/usr/bin/env python3
"""Auto-collect (heightmap_patch, GT_label) training rows during a benchmark trial.

Sibling of session_reporter.py — designed to run as an ExecuteProcess inside
the same launch, terminated on SIGTERM by the launch's session-duration timer.

For each `/<ns>/elevation_map_raw` GridMap published by elevation_mapping_cupy,
sample N random patches from the live elevation layer, look up the
ground-truth label for each patch center from the per-scene static label map
produced by `scripts/training/mujoco_static_label_map.py`, and accumulate
rows in memory. On SIGTERM (or natural end), flush everything to a
compressed .npz compatible with `scripts/training/build_trav_dataset.py`.

The collector deliberately captures the *noisy* runtime elevation map (with
Mid-360 Risley sampling + walking-pitch oscillation + Fast-LIO pose drift) so
the CNN trained on this corpus learns to denoise end-to-end.

Output schema (matches build_trav_dataset.py / train_trav_filter.py input):
    patches    : (N, 7, 7) float32   — elevation [m], NaN where unmapped
    labels     : (N,)      float32   — 1.0 if free, 0.0 if lethal/inflated
    classes    : (N,)      int8      — 0=FREE  1=LETHAL  2=INFLATED
    world_xy   : (N, 2)    float32   — patch center in world frame
    t          : (N,)      float32   — sim time at capture (seconds)
    cell_size_m: float32             — patch cell spacing (default 0.10)
    scene      : str                 — MJCF stem (e.g. "demo3_mixed")
    trial_id   : str                 — caller-provided string for joining

CLI:
    python3 scripts/bench/trav_corpus_collector.py \\
        --namespace robot \\
        --scene demo3_mixed \\
        --static-label src/go2w/go2_gazebo_sim/mujoco/demo3_mixed_gtlabel.npy \\
        --output /tmp/trav_corpus/demo3_mixed_trial001.npz \\
        --patches-per-frame 50

Termination: SIGTERM / SIGINT triggers a clean flush + exit 0.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from grid_map_msgs.msg import GridMap


# Label codes — must match mujoco_static_label_map.py.
FREE = 0
LETHAL = 1
INFLATED = 2
UNKNOWN = -1


def grid_map_layer_to_world_array(data: np.ndarray, *, height: int, width: int) -> np.ndarray:
    """Local copy of trav_cost_filters.occupancy_conversion helper.

    GridMap is column-major from max to min along both axes; flip to standard
    OccupancyGrid layout where arr[r, c] is at (origin_x + c*res, origin_y + r*res).
    """
    expected = int(height) * int(width)
    arr = np.asarray(data, dtype=np.float32)
    if arr.size != expected:
        raise ValueError(f"GridMap layer has {arr.size} values, expected {expected}")
    return arr.reshape(int(height), int(width))[::-1, ::-1]


class TravCorpusCollector(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("trav_corpus_collector")
        self.ns = args.namespace.strip("/")
        self.scene = args.scene
        self.trial_id = args.trial_id
        self.output_path = Path(args.output)
        self.patches_per_frame = int(args.patches_per_frame)
        self.patch_size = int(args.patch_size)
        self.min_touched_frac = float(args.min_touched_frac)
        self.max_patches = int(args.max_patches) if args.max_patches > 0 else 10_000_000
        self.elevation_layer = args.elevation_layer
        self.rng = np.random.default_rng(args.seed)

        if self.patch_size % 2 == 0:
            raise ValueError(f"patch_size must be odd, got {self.patch_size}")
        self.half = self.patch_size // 2

        self._load_static_label(args.static_label)

        # In-memory accumulators (lists; converted to np arrays on flush).
        self._patches: list[np.ndarray] = []
        self._classes: list[int] = []
        self._world_xy: list[tuple[float, float]] = []
        self._t: list[float] = []

        self._n_frames = 0
        self._n_skipped_bounds = 0
        self._n_skipped_sparse = 0
        self._n_skipped_unknown = 0

        topic = f"/{self.ns}/elevation_map_raw"
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._sub = self.create_subscription(GridMap, topic, self._on_map, qos)
        self._t0 = time.monotonic()
        self.get_logger().info(
            f"collector ready ns={self.ns} scene={self.scene} "
            f"patches_per_frame={self.patches_per_frame} "
            f"label_map={args.static_label} subscribing {topic}"
        )

    # ----------------------------------------------------------------
    def _load_static_label(self, label_path: str) -> None:
        p = Path(label_path)
        if not p.exists():
            raise FileNotFoundError(f"static label not found: {p}")
        json_path = p.with_suffix(".json")
        if not json_path.exists():
            raise FileNotFoundError(f"static label sidecar not found: {json_path}")
        self._label_grid = np.load(p)  # (H, W) int8
        self._label_meta = json.loads(json_path.read_text())
        self._label_res = float(self._label_meta["resolution_m"])
        self._label_ox = float(self._label_meta["origin_xy"][0])
        self._label_oy = float(self._label_meta["origin_xy"][1])
        H, W = self._label_grid.shape
        self._label_W = W
        self._label_H = H

    def _world_to_label(self, wx: float, wy: float) -> int:
        """Return label code at world (wx, wy), or UNKNOWN if out of bounds."""
        ix = int((wx - self._label_ox) / self._label_res)
        iy = int((wy - self._label_oy) / self._label_res)
        if 0 <= ix < self._label_W and 0 <= iy < self._label_H:
            return int(self._label_grid[iy, ix])
        return UNKNOWN

    # ----------------------------------------------------------------
    def _on_map(self, msg: GridMap) -> None:
        if self.elevation_layer not in msg.layers:
            self.get_logger().warn(
                f"layer '{self.elevation_layer}' not in GridMap; available: {list(msg.layers)}",
                throttle_duration_sec=10.0,
            )
            return

        if len(self._patches) >= self.max_patches:
            return

        info = msg.info
        res = float(info.resolution)
        n_x = int(round(info.length_x / res))
        n_y = int(round(info.length_y / res))
        layer_idx = list(msg.layers).index(self.elevation_layer)
        data_float = np.array(msg.data[layer_idx].data, dtype=np.float32)
        if data_float.size != n_x * n_y:
            return
        try:
            elev = grid_map_layer_to_world_array(data_float, height=n_y, width=n_x)
        except ValueError:
            return

        # GridMap origin: info.pose.position is the CENTER → bottom-left corner.
        cx = float(info.pose.position.x)
        cy = float(info.pose.position.y)
        ox = cx - info.length_x / 2.0
        oy = cy - info.length_y / 2.0

        H, W = elev.shape
        if H < self.patch_size or W < self.patch_size:
            return

        # Sample random patch centers (in pixel coords, excluding the
        # half-window border so the patch fits inside the map).
        n_try = max(self.patches_per_frame * 3, 30)
        cs = self.rng.integers(self.half, W - self.half, size=n_try)
        rs = self.rng.integers(self.half, H - self.half, size=n_try)

        t_now = float(self.get_clock().now().nanoseconds) / 1e9 - (self._t0 if False else 0.0)
        # use sim time so it lines up with bag/launch wall-clock if needed.

        kept = 0
        for r, c in zip(rs, cs):
            if kept >= self.patches_per_frame:
                break
            patch = elev[r - self.half:r + self.half + 1,
                         c - self.half:c + self.half + 1]
            if patch.shape != (self.patch_size, self.patch_size):
                continue
            valid = np.isfinite(patch)
            frac = float(valid.mean())
            if frac < self.min_touched_frac:
                self._n_skipped_sparse += 1
                continue
            wx = ox + (c + 0.5) * res
            wy = oy + (r + 0.5) * res
            label = self._world_to_label(wx, wy)
            if label == UNKNOWN:
                self._n_skipped_bounds += 1
                continue
            # Skip the inflated class for now? Keep it — annotate as class 2.
            # CNN training collapses 2→0 (lethal) via labels float, but raw
            # class is preserved in `classes` for downstream analysis.

            self._patches.append(patch.astype(np.float32, copy=True))
            self._classes.append(int(label))
            self._world_xy.append((float(wx), float(wy)))
            self._t.append(float(t_now))
            kept += 1
            if len(self._patches) >= self.max_patches:
                break

        self._n_frames += 1
        if self._n_frames % 25 == 0:
            self.get_logger().info(
                f"[corpus] frame={self._n_frames} kept_total={len(self._patches)} "
                f"skip_bounds={self._n_skipped_bounds} "
                f"skip_sparse={self._n_skipped_sparse}"
            )

    # ----------------------------------------------------------------
    def flush(self) -> None:
        if not self._patches:
            self.get_logger().warn("no patches collected; not writing output")
            return
        patches = np.stack(self._patches, axis=0).astype(np.float32)
        classes = np.array(self._classes, dtype=np.int8)
        world_xy = np.array(self._world_xy, dtype=np.float32)
        t = np.array(self._t, dtype=np.float32)
        # labels: 1.0 if FREE, 0.0 if LETHAL or INFLATED.
        labels = (classes == FREE).astype(np.float32)

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            self.output_path,
            patches=patches,
            labels=labels,
            classes=classes,
            world_xy=world_xy,
            t=t,
            cell_size_m=np.float32(self._label_res),
            scene=np.array(self.scene),
            trial_id=np.array(self.trial_id),
            origin_xy=np.array(
                [self._label_ox, self._label_oy], dtype=np.float32),
        )
        n_free = int(np.sum(classes == FREE))
        n_lethal = int(np.sum(classes == LETHAL))
        n_inflated = int(np.sum(classes == INFLATED))
        self.get_logger().info(
            f"flushed {patches.shape[0]} patches to {self.output_path}\n"
            f"  free={n_free}  lethal={n_lethal}  inflated={n_inflated}\n"
            f"  frames={self._n_frames}  skip_bounds={self._n_skipped_bounds}  "
            f"skip_sparse={self._n_skipped_sparse}"
        )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--namespace", default="robot", help="Robot namespace (default 'robot')")
    ap.add_argument("--scene", required=True, help="Scene MJCF stem, e.g. 'demo3_mixed'")
    ap.add_argument("--static-label", required=True, help="Path to <scene>_gtlabel.npy")
    ap.add_argument("--output", required=True, help="Output .npz path")
    ap.add_argument("--trial-id", default="t000", help="Trial identifier (string)")
    ap.add_argument("--patches-per-frame", type=int, default=50,
                    help="Max patches sampled per GridMap (default 50)")
    ap.add_argument("--patch-size", type=int, default=7,
                    help="Patch side length in cells (default 7; must be odd)")
    ap.add_argument("--min-touched-frac", type=float, default=0.6,
                    help="Reject patches with < this fraction valid cells (default 0.6)")
    ap.add_argument("--max-patches", type=int, default=200_000,
                    help="Stop after this many patches in one trial (default 200k)")
    ap.add_argument("--elevation-layer", default="elevation",
                    help="GridMap layer name to sample from (default 'elevation')")
    ap.add_argument("--seed", type=int, default=0, help="RNG seed (default 0)")
    args = ap.parse_args()

    rclpy.init()
    node = TravCorpusCollector(args)

    # Clean flush on SIGTERM / SIGINT (sent by launch on session-duration timeout).
    interrupted = {"flag": False}

    def _on_sig(signum, _frame):
        interrupted["flag"] = True

    signal.signal(signal.SIGTERM, _on_sig)
    signal.signal(signal.SIGINT, _on_sig)

    try:
        while rclpy.ok() and not interrupted["flag"]:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        try:
            node.flush()
        finally:
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
