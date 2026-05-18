#!/usr/bin/env python3
"""
prep_erasor_input.py — convert SC-A-LOAM output → ERASOR your_own_env format.

ERASOR expects:
  <data_dir>/
    dense_global_map.pcd    ← copy of aft_pgo_map.pcd
    poses_lidar2body.csv    ← KITTI poses → CSV with header + tx,ty,tz,qx,qy,qz,qw
    pcds/                   ← keyframe PCDs renamed to 6-digit
      000000.pcd
      000001.pcd
      ...

Usage:
  python3 prep_erasor_input.py <sc_pgo_dir> <output_data_dir>
"""
import argparse
import shutil
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sc_pgo_dir", help="SC-A-LOAM output dir (has Scans/ + optimized_poses.txt)")
    ap.add_argument("out_dir",    help="ERASOR data_dir to populate")
    ap.add_argument("--map", default=None,
                    help="path to dense_global_map.pcd (default: <sc_pgo_dir>/../aft_pgo_map.pcd)")
    args = ap.parse_args()

    src   = Path(args.sc_pgo_dir)
    dst   = Path(args.out_dir)
    poses_path = src / "optimized_poses.txt"
    scans_dir  = src / "Scans"

    if not poses_path.exists():
        sys.exit(f"ERROR: {poses_path} not found")
    if not scans_dir.exists():
        sys.exit(f"ERROR: {scans_dir} not found")

    dst.mkdir(parents=True, exist_ok=True)
    (dst / "pcds").mkdir(exist_ok=True)

    # 1. Copy / link keyframe pcds
    scans = sorted(scans_dir.glob("*.pcd"), key=lambda p: int(p.stem))
    print(f"  copying {len(scans)} keyframe pcds → {dst}/pcds/")
    for i, src_pcd in enumerate(scans):
        dst_pcd = dst / "pcds" / f"{i:06d}.pcd"
        if dst_pcd.exists():
            dst_pcd.unlink()
        # Symlink (cheap) — adjust to copy if needed:
        dst_pcd.symlink_to(src_pcd.resolve())

    # 2. Convert KITTI poses → ERASOR CSV
    out_csv = dst / "poses_lidar2body.csv"
    with open(poses_path) as f, open(out_csv, "w") as w:
        # ERASOR parses pose[2..8] as tx,ty,tz,qx,qy,qz,qw. The first 2 columns
        # are some kind of index/timestamp it ignores. Write 9 cols + 1 header.
        w.write("# idx,timestamp,tx,ty,tz,qx,qy,qz,qw\n")
        for i, line in enumerate(f):
            vals = line.split()
            if len(vals) != 12:
                continue
            T = np.array(list(map(float, vals))).reshape(3, 4)
            t  = T[:3, 3]
            Rm = T[:3, :3]
            q  = R.from_matrix(Rm).as_quat()  # x, y, z, w
            w.write(f"{i},{i*0.1:.6f},"
                    f"{t[0]:.6f},{t[1]:.6f},{t[2]:.6f},"
                    f"{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f}\n")
    print(f"  wrote {out_csv}  ({len(scans)} poses)")

    # 3. Copy / link the dense global map
    if args.map:
        map_src = Path(args.map)
    else:
        map_src = src.parent / "aft_pgo_map.pcd"
    if not map_src.exists():
        sys.exit(f"ERROR: dense_global_map source {map_src} not found")
    map_dst = dst / "dense_global_map.pcd"
    if map_dst.exists():
        map_dst.unlink()
    map_dst.symlink_to(map_src.resolve())
    print(f"  linked dense_global_map.pcd → {map_src}")

    print(f"\n✓ ERASOR data dir ready: {dst}")
    print(f"   Run inside container:")
    print(f"   roslaunch /host_scripts/run_erasor_mid360.launch")


if __name__ == "__main__":
    main()
