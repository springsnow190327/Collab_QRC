#!/usr/bin/env python3
"""
density_filter.py — voxel-point-density temporal consistency filter.

Premise: when accumulating ALL FAST-LIO scans (no keyframe sub-sampling), a
voxel containing a static surface accumulates points from every frame that
passed near it (~50-200 points), while a voxel containing a transient point
(pedestrian trail, glass reflection) gets points from only a few frames.

So we voxelize the accumulated map and threshold by points-per-voxel.

Inputs:
    accumulated.pcd      e.g. src/vendor/fast_lio/PCD/scans.pcd (25M+ points)

Algorithm:
    1. voxelize at <voxel_size>
    2. for each voxel: count how many raw points fell in it
    3. keep only voxels with point_count >= <min_pts>
    4. write the surviving raw points (or voxel centers) to output

Usage:
    python3 density_filter.py <input.pcd>
        [--voxel 0.05]       voxel size for density estimation
        [--min-pts 30]       minimum points per voxel to keep
        [--out <path>]
        [--mode keep_points|voxel_centers]
"""
import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("--voxel",     type=float, default=0.05)
    ap.add_argument("--min-pts",   type=int,   default=30)
    ap.add_argument("--out",       default=None)
    ap.add_argument("--mode", choices=["keep_points", "voxel_centers"],
                    default="keep_points")
    args = ap.parse_args()

    in_path = Path(args.input)
    print(f"  loading {in_path}...")
    pcd = o3d.io.read_point_cloud(str(in_path))
    pts = np.asarray(pcd.points)
    print(f"  loaded {len(pts):,} points")

    inv_v = 1.0 / args.voxel
    idx = np.floor(pts * inv_v).astype(np.int64)

    # encode (i,j,k) as single int64 for fast hashing via unique
    OFFSET = 1 << 20  # supports voxel indices in [-2^20, 2^20] = -52km..52km at 5cm
    flat = (idx[:, 0] + OFFSET) * (4 * OFFSET) ** 2 \
         + (idx[:, 1] + OFFSET) * (4 * OFFSET) \
         + (idx[:, 2] + OFFSET)
    print("  hashing voxels...")
    uniq, inv, counts = np.unique(flat, return_inverse=True, return_counts=True)
    print(f"  unique voxels: {len(uniq):,}")
    print(f"  per-voxel-count percentiles: "
          f"p50={np.percentile(counts,50):.0f}  "
          f"p75={np.percentile(counts,75):.0f}  "
          f"p90={np.percentile(counts,90):.0f}  "
          f"p99={np.percentile(counts,99):.0f}  "
          f"max={counts.max()}")

    keep_voxel_mask = counts >= args.min_pts
    keep_voxel_indices = np.where(keep_voxel_mask)[0]
    print(f"  voxels kept (>= {args.min_pts} pts): "
          f"{len(keep_voxel_indices):,} ({100*keep_voxel_mask.mean():.1f}%)")

    if args.mode == "keep_points":
        keep_pt_mask = keep_voxel_mask[inv]
        out_pts = pts[keep_pt_mask]
        print(f"  output points: {len(out_pts):,} "
              f"(from {len(pts):,}, kept {100*keep_pt_mask.mean():.1f}%)")
        out_pcd = o3d.geometry.PointCloud()
        out_pcd.points = o3d.utility.Vector3dVector(out_pts)
    else:
        # voxel centers
        kept_uniq = uniq[keep_voxel_indices]
        # decode
        k = (kept_uniq % (4 * OFFSET)) - OFFSET
        j = ((kept_uniq // (4 * OFFSET)) % (4 * OFFSET)) - OFFSET
        i = (kept_uniq // ((4 * OFFSET) ** 2)) - OFFSET
        centers = np.stack([i, j, k], axis=1).astype(np.float64)
        centers = (centers + 0.5) * args.voxel
        print(f"  output voxel centers: {len(centers):,}")
        out_pcd = o3d.geometry.PointCloud()
        out_pcd.points = o3d.utility.Vector3dVector(centers)

    out = Path(args.out) if args.out else in_path.with_name(
        in_path.stem + "_density_filtered.pcd")
    o3d.io.write_point_cloud(str(out), out_pcd)
    print(f"\n✓ wrote {out} ({out.stat().st_size // (1024*1024)} MB)")


if __name__ == "__main__":
    main()
