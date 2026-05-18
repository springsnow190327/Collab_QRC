#!/usr/bin/env python3
"""
temporal_filter.py — voxel-hit-count temporal consistency filter for accumulated
LiDAR maps. Removes dynamic objects (pedestrian trails) and glass-reflection
outliers based on the principle:

    static structure = observed in many frames; dynamic = observed in few

Inputs:
    Scans/000NNN.pcd      per-frame body-frame point clouds (from SC-A-LOAM)
    odom_poses.txt        KITTI-format 3x4 poses (one per scan), in body frame
                          (use the FAST-LIO odometry version, NOT optimized_poses
                          which may have been corrupted by glass-driven false LC)

Algorithm:
    For each scan k:
        Transform its points to world frame via pose[k]
        Voxelize at <voxel_size> and record which voxels were hit
    For each voxel: count distinct scan ids that hit it (hit_count)
    Keep voxels with hit_count >= min_hits

Result: a "static map" where every retained point was consistently observed
across many frames as the sensor moved around.

Usage:
    python3 temporal_filter.py <sc_pgo_dir>
        [--voxel 0.05]             voxel size for hit-counting
        [--min-hits 15]            minimum frames to consider voxel static
        [--out static_map.pcd]
        [--use-optimized]          use optimized_poses.txt instead of odom
"""
import argparse
import sys
from pathlib import Path
from collections import defaultdict

import numpy as np
import open3d as o3d


def load_kitti_poses(path: Path) -> list:
    poses = []
    for line in path.read_text().splitlines():
        v = line.split()
        if len(v) != 12:
            continue
        T = np.eye(4)
        T[:3, :] = np.array(list(map(float, v))).reshape(3, 4)
        poses.append(T)
    return poses


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sc_pgo_dir")
    ap.add_argument("--voxel",     type=float, default=0.05,
                    help="voxel size for hit-counting (m)")
    ap.add_argument("--min-hits",  type=int,   default=15,
                    help="keep voxels hit in >= N distinct frames")
    ap.add_argument("--out",       default=None)
    ap.add_argument("--use-optimized", action="store_true",
                    help="use optimized_poses.txt (with LC) instead of odom_poses.txt")
    args = ap.parse_args()

    d = Path(args.sc_pgo_dir)
    poses_file = "optimized_poses.txt" if args.use_optimized else "odom_poses.txt"
    poses = load_kitti_poses(d / poses_file)
    scans = sorted((d / "Scans").glob("*.pcd"), key=lambda p: int(p.stem))
    n = min(len(poses), len(scans))
    print(f"  using poses={poses_file}  ({len(poses)} entries)")
    print(f"  scans: {len(scans)}; processing min={n}")
    print(f"  voxel={args.voxel}m  min_hits={args.min_hits}")

    # voxel_idx (i,j,k) → set(frame_id)
    voxel_hits = defaultdict(set)
    inv_voxel = 1.0 / args.voxel

    for k in range(n):
        pc = o3d.io.read_point_cloud(str(scans[k]))
        pts = np.asarray(pc.points)
        if pts.shape[0] == 0:
            continue
        # transform body → world
        T = poses[k]
        pts_w = (T[:3, :3] @ pts.T).T + T[:3, 3]
        # voxelize
        idx = np.floor(pts_w * inv_voxel).astype(np.int64)
        # use np.unique to dedup within this frame, then bulk-update
        idx_tup = list(map(tuple, np.unique(idx, axis=0)))
        for t in idx_tup:
            voxel_hits[t].add(k)

        if (k + 1) % 50 == 0:
            print(f"    [{k+1}/{n}] frames done, voxels seen so far: {len(voxel_hits):,}")

    print(f"\n  total unique voxels: {len(voxel_hits):,}")
    # Stats on hit distribution
    hits = np.array([len(s) for s in voxel_hits.values()])
    print(f"  hit-count percentiles: "
          f"p50={np.percentile(hits,50):.0f}  p75={np.percentile(hits,75):.0f}  "
          f"p90={np.percentile(hits,90):.0f}  p99={np.percentile(hits,99):.0f}  "
          f"max={hits.max()}")
    print(f"  voxels with >= {args.min_hits} hits: "
          f"{(hits >= args.min_hits).sum():,} "
          f"({100*(hits >= args.min_hits).mean():.1f}%)")

    # Reconstruct static-map points: take voxel center for each surviving voxel
    static_centers = np.array(
        [list(idx) for idx, s in voxel_hits.items() if len(s) >= args.min_hits],
        dtype=np.float64,
    )
    static_pts = (static_centers + 0.5) * args.voxel
    print(f"  static map: {len(static_pts):,} voxel centers")

    out = Path(args.out) if args.out else d.parent / f"static_temporal_filter.pcd"
    out_pcd = o3d.geometry.PointCloud()
    out_pcd.points = o3d.utility.Vector3dVector(static_pts)
    o3d.io.write_point_cloud(str(out), out_pcd)
    print(f"\n✓ wrote {out} ({out.stat().st_size // (1024*1024)} MB)")


if __name__ == "__main__":
    main()
