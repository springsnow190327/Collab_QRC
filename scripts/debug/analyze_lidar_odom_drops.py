#!/usr/bin/env python3
"""Diagnose Fast-LIO frame drops in a recorded Livox dataset.

Usage:
    ./scripts/debug/analyze_lidar_odom_drops.py [BAG_DIR] [--gap_ms 100] [--top 20]

Reads a rosbag2 (sqlite3 or mcap) directory and:
  1) Pulls every /livox/lidar timestamp (the "scan-arrived" event)
  2) Pulls every /Odometry timestamp     (the "Fast-LIO finished" event)
  3) For each lidar timestamp, finds the closest *later* odometry timestamp
     and treats lidar→odom delta as that scan's processing latency.
  4) Flags lidar timestamps with NO odometry within `gap_ms` as DROPPED
     (Fast-LIO never finished processing that scan before the next one
     arrived and overwrote it).
  5) Prints summary stats + the worst N drops with their wall-clock time
     so you can correlate to the drift trajectory in RViz.

Picks the latest livox_dataset_* bag if no path given.
"""
from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import rclpy.serialization  # noqa: F401  (registers serializers)
from nav_msgs.msg import Odometry
from livox_ros_driver2.msg import CustomMsg

import rosbag2_py


def _stamp_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


@dataclass
class BagSummary:
    livox_t: list[float]
    odom_t: list[float]
    duration_s: float


def read_bag(bag_dir: str) -> BagSummary:
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if "/livox/lidar" not in type_map:
        sys.exit(f"ERROR: /livox/lidar not in bag (topics: {list(type_map)[:8]} ...)")
    if "/Odometry" not in type_map:
        sys.exit(f"ERROR: /Odometry not in bag")

    livox_t: list[float] = []
    odom_t: list[float] = []
    t_first = None
    t_last = None

    while reader.has_next():
        topic, raw, recv_t = reader.read_next()
        recv_s = recv_t * 1e-9
        if t_first is None:
            t_first = recv_s
        t_last = recv_s

        if topic == "/livox/lidar":
            msg = rclpy.serialization.deserialize_message(raw, CustomMsg)
            livox_t.append(_stamp_sec(msg.header.stamp))
        elif topic == "/Odometry":
            msg = rclpy.serialization.deserialize_message(raw, Odometry)
            odom_t.append(_stamp_sec(msg.header.stamp))

    return BagSummary(
        livox_t=sorted(livox_t),
        odom_t=sorted(odom_t),
        duration_s=(t_last or 0) - (t_first or 0),
    )


def analyze(s: BagSummary, gap_ms: float, top_n: int) -> None:
    if not s.livox_t or not s.odom_t:
        sys.exit("ERROR: empty livox or odom timestamp list")

    gap_s = gap_ms / 1000.0

    # Per-scan latency: for each lidar, find smallest odom_t >= lidar_t.
    import bisect
    scan_latencies: list[tuple[float, float]] = []  # (lidar_t, latency_or_NaN)
    drops: list[float] = []  # lidar_t without odom within gap_s
    for lt in s.livox_t:
        idx = bisect.bisect_left(s.odom_t, lt)
        if idx >= len(s.odom_t):
            scan_latencies.append((lt, float("nan")))
            drops.append(lt)
            continue
        latency = s.odom_t[idx] - lt
        scan_latencies.append((lt, latency))
        if latency > gap_s:
            drops.append(lt)

    finished = [l for _, l in scan_latencies if l == l]  # exclude NaN
    finished.sort()

    n_lidar = len(s.livox_t)
    n_odom = len(s.odom_t)
    n_drop = len(drops)
    drop_rate = n_drop / n_lidar if n_lidar else 0

    # Per-scan inter-arrival: should be ~100 ms for 10 Hz lidar
    lidar_dts = [s.livox_t[i+1] - s.livox_t[i] for i in range(len(s.livox_t)-1)]
    odom_dts  = [s.odom_t[i+1]  - s.odom_t[i]  for i in range(len(s.odom_t)-1)]

    def stats(xs):
        if not xs: return (0, 0, 0, 0)
        xs = sorted(xs)
        n = len(xs)
        return (
            xs[n // 2],
            xs[int(n * 0.95)],
            xs[int(n * 0.99)],
            xs[-1],
        )

    print(f"=== Bag: duration={s.duration_s:.1f}s  livox={n_lidar}  odom={n_odom} ===")
    print(f"  /livox/lidar   rate: {n_lidar / s.duration_s:6.2f} Hz")
    print(f"  /Odometry      rate: {n_odom / s.duration_s:6.2f} Hz")
    print(f"  ratio odom/lidar  : {n_odom / n_lidar:.3f}  (1.0 = no drops)")
    print()
    print(f"=== Inter-arrival gaps (ms) ===")
    p50, p95, p99, pmax = stats(lidar_dts)
    print(f"  /livox/lidar  p50={p50*1000:6.1f}  p95={p95*1000:6.1f}  p99={p99*1000:6.1f}  max={pmax*1000:7.1f}")
    p50, p95, p99, pmax = stats(odom_dts)
    print(f"  /Odometry     p50={p50*1000:6.1f}  p95={p95*1000:6.1f}  p99={p99*1000:6.1f}  max={pmax*1000:7.1f}")
    print()
    print(f"=== Per-scan latency (lidar.stamp → next odom.stamp, ms) ===")
    p50, p95, p99, pmax = stats(finished)
    print(f"  finished       p50={p50*1000:6.1f}  p95={p95*1000:6.1f}  p99={p99*1000:6.1f}  max={pmax*1000:7.1f}")
    print(f"  threshold      gap_ms={gap_ms}  → {n_drop} drops / {n_lidar} scans = {drop_rate*100:.1f}%")
    print()

    if not drops:
        print("No drops above threshold — Fast-LIO kept up. Drift is not from frame drops.")
        return

    print(f"=== Worst {top_n} drops (lidar timestamps with latency > {gap_ms}ms) ===")
    print(f"  bag t  | wall t (rel s) | latency (ms) | bin")
    drop_lat = [(lt, lat) for lt, lat in scan_latencies if lat != lat or lat > gap_s]
    drop_lat.sort(key=lambda x: (-(x[1] if x[1] == x[1] else 1e9)))
    t0 = s.livox_t[0]
    for lt, lat in drop_lat[:top_n]:
        rel = lt - t0
        if lat != lat:
            print(f"  {rel:6.2f}s | NO matching odom (end of bag)")
        else:
            bar = "#" * min(40, int(lat * 1000 / 10))
            print(f"  {rel:6.2f}s |  ms={lat*1000:7.1f}  {bar}")

    # Bin drops over the timeline so you can see if drops cluster
    print()
    print(f"=== Drop density over time (10s bins) ===")
    if drops:
        n_bins = max(1, int(s.duration_s / 10) + 1)
        bins = [0] * n_bins
        for lt in drops:
            i = min(n_bins - 1, int((lt - t0) / 10))
            bins[i] += 1
        for i, c in enumerate(bins):
            bar = "#" * min(40, c)
            print(f"  t=[{i*10:3d}, {(i+1)*10:3d})s  drops={c:4d}  {bar}")


def find_default_bag() -> str | None:
    repo_root = Path(__file__).resolve().parent.parent.parent
    bags = sorted((repo_root / "bags").glob("livox_dataset_*"), reverse=True)
    return str(bags[0]) if bags else None


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("bag", nargs="?", default=None,
                   help="rosbag2 directory (defaults to latest bags/livox_dataset_*)")
    p.add_argument("--gap_ms", type=float, default=100.0,
                   help="latency above this counts as a drop (default 100 = the lidar period)")
    p.add_argument("--top", type=int, default=20, help="how many worst drops to print")
    p.add_argument("--mask", type=str, default="",
                   help="semicolon-sep [start,end] windows (relative seconds) to mask out, "
                        "e.g. '0,2;29,34' to ignore Fast-LIO init AND the 30s LiDAR dropout")
    args = p.parse_args()

    bag = args.bag or find_default_bag()
    if not bag or not Path(bag).is_dir():
        sys.exit("ERROR: no bag found / specify path explicitly")

    summary = read_bag(bag)
    if args.mask:
        windows = []
        for piece in args.mask.split(";"):
            try:
                lo, hi = (float(x) for x in piece.split(","))
            except ValueError:
                sys.exit(f"ERROR: --mask piece '{piece}' must be 'start,end'")
            windows.append((lo, hi))
        t0 = summary.livox_t[0]
        def keep(t: float) -> bool:
            rel = t - t0
            return not any(lo <= rel <= hi for lo, hi in windows)
        before_l = len(summary.livox_t); before_o = len(summary.odom_t)
        summary.livox_t = [t for t in summary.livox_t if keep(t)]
        summary.odom_t  = [t for t in summary.odom_t  if keep(t)]
        print(f"[mask] excluding {windows}s — livox {before_l}→{len(summary.livox_t)},"
              f" odom {before_o}→{len(summary.odom_t)}\n")
    analyze(summary, args.gap_ms, args.top)


if __name__ == "__main__":
    main()
