#!/usr/bin/env python3
"""
build_corrected_map.py — combine SC-A-LOAM keyframe PCDs with optimized poses
                         into a single loop-closure-corrected accumulated map.

SC-A-LOAM outputs:
  Scans/0.pcd, 1.pcd, ... (each in body frame at time of keyframe)
  optimized_poses.txt     (KITTI format: one 3x4 transformation per line)

Usage:
  python3 build_corrected_map.py <sc_pgo_dir> [--out <output.pcd>] [--voxel 0.05]

The pose at row i is the world-frame transform T_world<-body for scan i.
Final map = union of (T_i @ scan_i) for all keyframes.
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d


def load_kitti_poses(path: Path) -> list:
    """Each line: 12 floats = 3x4 matrix in row-major order."""
    poses = []
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        vals = list(map(float, line.split()))
        if len(vals) != 12:
            continue
        T = np.eye(4)
        T[:3, :] = np.array(vals).reshape(3, 4)
        poses.append(T)
    return poses


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sc_pgo_dir", help="directory with Scans/ + optimized_poses.txt")
    ap.add_argument("--out", default=None, help="output PCD path")
    ap.add_argument("--voxel", type=float, default=0.0,
                    help="voxel size for final downsampling (0 = no downsampling)")
    args = ap.parse_args()

    d = Path(args.sc_pgo_dir)
    poses_path = d / "optimized_poses.txt"
    scans_dir  = d / "Scans"
    if not poses_path.exists():
        sys.exit(f"ERROR: {poses_path} not found")
    if not scans_dir.exists():
        sys.exit(f"ERROR: {scans_dir} not found")

    poses = load_kitti_poses(poses_path)
    scans = sorted(scans_dir.glob("*.pcd"), key=lambda p: int(p.stem))
    print(f"  poses: {len(poses)}, scans: {len(scans)}")

    n = min(len(poses), len(scans))
    if n == 0:
        sys.exit("ERROR: no poses or scans")

    merged = o3d.geometry.PointCloud()
    for i in range(n):
        pcd = o3d.io.read_point_cloud(str(scans[i]))
        pts = np.asarray(pcd.points)
        if pts.shape[0] == 0:
            continue
        T = poses[i]
        pts_world = (T[:3, :3] @ pts.T).T + T[:3, 3]
        # Append
        merged_pts = np.vstack([np.asarray(merged.points), pts_world]) \
                        if len(merged.points) > 0 else pts_world
        merged.points = o3d.utility.Vector3dVector(merged_pts)
        if (i + 1) % 50 == 0:
            print(f"  [{i+1}/{n}]  total points: {len(merged.points):,}")

    print(f"\n  total before voxel: {len(merged.points):,}")
    if args.voxel > 0:
        merged = merged.voxel_down_sample(voxel_size=args.voxel)
        print(f"  after voxel({args.voxel}m): {len(merged.points):,}")

    out = Path(args.out) if args.out else d / "aft_pgo_map_built.pcd"
    o3d.io.write_point_cloud(str(out), merged)
    print(f"\n✓ wrote {out} ({out.stat().st_size // (1024*1024)} MB)")


if __name__ == "__main__":
    main()
