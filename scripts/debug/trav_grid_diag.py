#!/usr/bin/env python3
"""Traversability-grid diagnostic for nvblox_frontend / CFPA2 3D exploration.

Run this against a LIVE session (mapper already publishing /<ns>/traversability_grid).
Single command → prints class counts, samples cells at named world coords (ramp
endpoints, outer walls, robot, custom probes), measures leak past the outer-wall
box, and optionally dumps the grid as a PNG for visual inspection.

Usage
-----
    python3 scripts/debug/trav_grid_diag.py                       # default probes
    python3 scripts/debug/trav_grid_diag.py --ns robot
    python3 scripts/debug/trav_grid_diag.py --probe 9.5,0,ramp_pre10 10,0,ramp_head
    python3 scripts/debug/trav_grid_diag.py --png /tmp/trav.png   # also write image
    python3 scripts/debug/trav_grid_diag.py --wall-box 0,16,-8,8  # leak-box overrides

The script subscribes once, prints in <3 s, and exits. Designed to be cheap to
invoke from claude tool calls during debugging — one Bash call, fully self
contained output.

Why a dedicated script: prior debug sessions spawned a fresh inspect.py per
question and burned tokens parsing inline output. Funnel all of it through here.
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from nav_msgs.msg import OccupancyGrid, Odometry


def _parse_probe(s: str) -> Tuple[float, float, str]:
    parts = s.split(",")
    if len(parts) < 2:
        raise argparse.ArgumentTypeError(f"--probe '{s}' needs at least x,y")
    x = float(parts[0])
    y = float(parts[1])
    name = parts[2] if len(parts) >= 3 else f"({x:+.2f},{y:+.2f})"
    return (x, y, name)


def _parse_wall_box(s: str) -> Tuple[float, float, float, float]:
    parts = [float(v) for v in s.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("--wall-box wants x_lo,x_hi,y_lo,y_hi")
    return tuple(parts)


# demo_ramp probe set — geometric landmarks of the active 3D-frontier scene.
DEFAULT_PROBES: List[Tuple[float, float, str]] = [
    (2.0, 0.0, "robot_spawn"),
    (6.0, 0.0, "ramp_tail_x=6"),
    (6.2, 0.0, "ramp_tail+0.2"),
    (8.0, 0.0, "ramp_mid_x=8"),
    (9.5, 0.0, "ramp_head-0.5"),
    (9.8, 0.0, "ramp_head-0.2"),
    (10.0, 0.0, "ramp_head_x=10"),
    (10.2, 0.0, "ramp_head+0.2"),
    (11.0, 0.0, "platform_x=11"),
    (6.0, 1.0, "ramp_+y_edge_tail"),
    (8.0, 1.0, "ramp_+y_edge_mid"),
    (10.0, 1.0, "ramp_+y_edge_head"),
    (8.0, 8.5, "past_N_wall"),
    (8.0, -8.5, "past_S_wall"),
    (-1.0, 0.0, "past_W_wall"),
    (17.0, 0.0, "past_E_wall"),
]


class TravGridDiag(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("trav_grid_diag")
        self.args = args
        topic = f"/{args.ns}/traversability_grid" if args.ns else args.topic
        odom_topic = f"/{args.ns}/odom/nav" if args.ns else args.odom_topic
        # nvblox_frontend publishes the grid with TRANSIENT_LOCAL durability so
        # late subscribers get the last sample. Match it or we miss the latched
        # message and see nothing for the full timeout.
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(OccupancyGrid, topic, self._cb_grid, latched_qos)
        self.create_subscription(Odometry, odom_topic, self._cb_odom, 10)
        self.odom: Optional[Tuple[float, float]] = None
        self.done = False
        self.deadline = time.monotonic() + args.timeout
        self.create_timer(0.5, self._tick)

    def _cb_odom(self, m: Odometry) -> None:
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y)

    def _tick(self) -> None:
        if self.done:
            return
        if time.monotonic() > self.deadline:
            print(f"TIMEOUT after {self.args.timeout:.0f}s — no grid (odom={self.odom})", file=sys.stderr)
            rclpy.shutdown()

    def _cb_grid(self, m: OccupancyGrid) -> None:
        if self.done:
            return
        if self.odom is None and not self.args.no_wait_odom:
            return
        self.done = True
        nx, ny = m.info.width, m.info.height
        ox, oy, vs = (
            m.info.origin.position.x,
            m.info.origin.position.y,
            m.info.resolution,
        )
        d = np.array(m.data, dtype=np.int8).reshape(ny, nx)
        rx, ry = self.odom if self.odom else (float("nan"), float("nan"))

        # ---- Summary ----
        n_unk = int((d == -1).sum())
        n_free = int((d == 0).sum())
        n_occ = int((d == 100).sum())
        total = nx * ny
        print(f"grid: {nx}x{ny}  vs={vs:.3f}m  origin=({ox:.2f},{oy:.2f})  extent={nx*vs:.1f}x{ny*vs:.1f}m")
        print(f"robot=({rx:.2f},{ry:.2f})")
        print(f"class counts: UNK={n_unk} ({100*n_unk/total:.1f}%)  FREE={n_free} ({100*n_free/total:.1f}%)  OCC={n_occ} ({100*n_occ/total:.1f}%)")

        # ---- Point probes ----
        labels = {None: "OUT-OF-GRID", -1: "UNK", 0: "FREE", 100: "OCC"}

        def at(x: float, y: float) -> Optional[int]:
            i = int((x - ox) / vs)
            j = int((y - oy) / vs)
            if 0 <= i < nx and 0 <= j < ny:
                return int(d[j, i])
            return None

        probes = list(DEFAULT_PROBES)
        if self.args.probe:
            probes = list(self.args.probe)
        if self.args.add_probe:
            probes.extend(self.args.add_probe)
        if probes:
            print("\nPoint probes (world x,y → trav class):")
            for x, y, name in probes:
                v = at(x, y)
                print(f"  ({x:+6.2f},{y:+6.2f}) [{name}]: {labels.get(v, str(v))}")

        # ---- Outer-wall-box leak measurement ----
        if self.args.wall_box:
            wx_lo, wx_hi, wy_lo, wy_hi = self.args.wall_box
            iy, ix = np.indices(d.shape)
            wx = ix * vs + ox
            wy = iy * vs + oy
            inside = (wx >= wx_lo) & (wx <= wx_hi) & (wy >= wy_lo) & (wy <= wy_hi)
            free_outside = int(((d == 0) & ~inside).sum())
            occ_outside = int(((d == 100) & ~inside).sum())
            print(
                f"\nFREE cells OUTSIDE wall box "
                f"x∈[{wx_lo:.1f},{wx_hi:.1f}] y∈[{wy_lo:.1f},{wy_hi:.1f}]: "
                f"{free_outside} cells = {free_outside * vs * vs:.2f} m²  "
                f"(OCC outside={occ_outside})"
            )

        # ---- Per-wall coverage (sanity for wall-leak debugging) ----
        if self.args.wall_bands:
            print("\nWall-band coverage (OCC/total of each named axis-aligned band):")
            iy, ix = np.indices(d.shape)
            wx = ix * vs + ox
            wy = iy * vs + oy
            for desc, wxl, wxh, wyl, wyh in self.args.wall_bands:
                band = (wx >= wxl) & (wx <= wxh) & (wy >= wyl) & (wy <= wyh)
                tot = int(band.sum())
                if tot == 0:
                    print(f"  {desc}: EMPTY BAND")
                    continue
                noc = int(((d == 100) & band).sum())
                nfr = int(((d == 0) & band).sum())
                nun = int(((d == -1) & band).sum())
                print(
                    f"  {desc}: OCC={noc}/{tot} ({100*noc/tot:.0f}%)  "
                    f"FREE={nfr}  UNK={nun}"
                )

        # ---- Distance-from-robot leak histogram ----
        if self.args.robot_radial:
            iy, ix = np.indices(d.shape)
            wx = ix * vs + ox
            wy = iy * vs + oy
            r2 = (wx - rx) ** 2 + (wy - ry) ** 2
            print("\nFREE cells by radial distance from robot:")
            for r_hi in [1, 2, 3, 5, 10, 20]:
                inside = r2 <= r_hi * r_hi
                nfr = int(((d == 0) & inside).sum())
                print(f"  r ≤ {r_hi:>2.0f} m: FREE={nfr} ({nfr * vs * vs:6.2f} m²)")

        # ---- Optional PNG dump ----
        if self.args.png:
            try:
                from PIL import Image
                # UNK=128 gray, FREE=255 white, OCC=0 black
                img = np.full(d.shape, 128, dtype=np.uint8)
                img[d == 0] = 255
                img[d == 100] = 0
                # Mark robot with a 3x3 red dot
                Image.fromarray(img, mode="L").save(self.args.png)
                print(f"\nPNG saved: {self.args.png} ({nx}x{ny}, white=FREE black=OCC gray=UNK)")
            except Exception as e:  # noqa: BLE001
                print(f"PNG dump failed: {e}", file=sys.stderr)

        # ---- Optional NPZ dump ----
        if self.args.npz:
            np.savez(self.args.npz, d=d, nx=nx, ny=ny, ox=ox, oy=oy, vs=vs, rx=rx, ry=ry)
            print(f"NPZ saved: {self.args.npz}")

        rclpy.shutdown()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ns", default="robot", help="robot namespace (default: robot)")
    ap.add_argument("--topic", default=None, help="explicit grid topic (overrides --ns)")
    ap.add_argument("--odom-topic", default=None, help="explicit odom topic (overrides --ns)")
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--no-wait-odom", action="store_true",
                    help="don't wait for /odom — useful if SLAM is down")
    ap.add_argument("--probe", action="append", type=_parse_probe,
                    help="replace default probe list (repeat per probe: x,y[,name])")
    ap.add_argument("--add-probe", action="append", type=_parse_probe,
                    help="add probes on top of default list")
    ap.add_argument("--wall-box", type=_parse_wall_box,
                    default=(0.0, 16.0, -8.0, 8.0),
                    help="leak-box x_lo,x_hi,y_lo,y_hi (default demo_ramp 0..16 × -8..8)")
    ap.add_argument("--no-wall-box", dest="wall_box", action="store_const", const=None)
    ap.add_argument("--wall-bands", action="store_true",
                    help="print per-wall band coverage for demo_ramp outer walls")
    ap.add_argument("--robot-radial", action="store_true",
                    help="print FREE-cell counts by radial distance from robot")
    ap.add_argument("--png", default=None, help="dump grid as PNG to this path")
    ap.add_argument("--npz", default=None, help="dump grid as NPZ for offline analysis")
    args = ap.parse_args()

    # demo_ramp wall bands wired in here (so the user doesn't have to repeat them)
    if args.wall_bands:
        args.wall_bands = [
            ("north_wall y∈[8.0,8.3]", 0, 16, 8.0, 8.3),
            ("south_wall y∈[-8.3,-8.0]", 0, 16, -8.3, -8.0),
            ("west_wall  x∈[-0.3,0.05]", -0.3, 0.05, -8, 8),
            ("east_wall  x∈[15.95,16.3]", 15.95, 16.3, -8, 8),
        ]

    rclpy.init()
    node = TravGridDiag(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
