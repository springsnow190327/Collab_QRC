#!/usr/bin/env python3
"""pcd_to_voxel_mesh.py — direct voxel-hull mesh reconstruction.

Why this exists: Poisson reconstruction smooths over holes and hallucinates
surfaces where the cloud was sparse → mesh ends up "off" relative to the
real geometry. For Livox-density indoor scans the cloud is already dense
enough that we don't need surface-fitting; we just need to turn each
occupied voxel into a small cube and connect them.

This is conceptually the same as marching cubes on a binary occupancy grid:
- bin points into a voxel grid (resolution VOXEL m, default 0.05 m / 5 cm)
- emit one axis-aligned cube per occupied voxel
- merge into a single triangle soup

Result is faithful (every cube corresponds to a real LiDAR return), no
surface hallucination, and — critically for MuJoCo — each cube is its own
convex hull, so collision works without splitting into 22 tiles.

Usage:
    python3 pcd_to_voxel_mesh.py input.pcd out.obj --voxel 0.05 --min-pts 4

Filters applied in order:
  1. RANSAC ground alignment (rotate so floor → z=0)
  2. Voxel downsample at `--voxel` resolution
  3. Per-voxel point-count gate (`--min-pts`) — drops sparse outliers
  4. Optional clustering cull (`--min-cluster-pts`) — drops floating
     islands smaller than threshold
  5. Voxel → cube mesh
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d


def align_ground(pcd: o3d.geometry.PointCloud, bottom_pct: float = 0.05):
    """RANSAC plane fit on bottom-bottom_pct points; rotate to z=0.

    Tight (5%) z-band avoids picking up walls. The original 2026-05-16
    pipeline used the same approach (see CLAUDE.md "Active state 2026-05-16"
    item 4 for the 30% vs 5% comparison: 30% gave 2.5° residual tilt because
    walls leaked into the plane fit; 5% gives 0.19°).
    """
    pts = np.asarray(pcd.points)
    if len(pts) < 200:
        return pcd, np.eye(3), 0.0

    z_thresh = np.quantile(pts[:, 2], bottom_pct)
    ground_pts = pts[pts[:, 2] <= z_thresh]
    if len(ground_pts) < 100:
        print(f"  ground-align: only {len(ground_pts)} pts in bottom {bottom_pct*100:.0f}% — skipping")
        return pcd, np.eye(3), 0.0

    ground_pcd = o3d.geometry.PointCloud()
    ground_pcd.points = o3d.utility.Vector3dVector(ground_pts)
    plane_model, inliers = ground_pcd.segment_plane(
        distance_threshold=0.03, ransac_n=3, num_iterations=1000)
    a, b, c, d = plane_model
    normal = np.array([a, b, c])
    normal /= np.linalg.norm(normal)
    if normal[2] < 0:
        normal = -normal
        d = -d

    z_axis = np.array([0.0, 0.0, 1.0])
    cos_t = float(np.clip(normal @ z_axis, -1, 1))
    angle_rad = float(np.arccos(cos_t))
    if angle_rad < 1e-4:
        rot = np.eye(3)
    else:
        axis = np.cross(normal, z_axis)
        axis /= np.linalg.norm(axis)
        K = np.array([[0, -axis[2], axis[1]],
                      [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        rot = np.eye(3) + np.sin(angle_rad) * K + (1 - cos_t) * (K @ K)

    # Apply rotation, then translate so plane → z=0.
    pts_aligned = pts @ rot.T
    z_shift = -np.median(pts_aligned[inliers, 2]) if len(inliers) else 0.0
    pts_aligned[:, 2] += z_shift

    out = o3d.geometry.PointCloud()
    out.points = o3d.utility.Vector3dVector(pts_aligned)
    if pcd.has_colors():
        out.colors = pcd.colors
    return out, rot, np.degrees(angle_rad)


def voxel_density_filter(
    pcd: o3d.geometry.PointCloud,
    voxel: float,
    min_pts: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Bin to voxel grid; return (occupied_centers, per_voxel_counts)."""
    pts = np.asarray(pcd.points)
    # Quantize to voxel indices, then collapse duplicates.
    idx = np.floor(pts / voxel).astype(np.int64)
    keys = idx[:, 0].astype(np.int64) * 1_000_000_000 \
         + idx[:, 1].astype(np.int64) * 1_000_000 \
         + idx[:, 2].astype(np.int64)
    uniq, inv, counts = np.unique(keys, return_inverse=True, return_counts=True)
    keep = counts >= min_pts
    print(f"  voxel-density: {len(uniq):,} occupied voxels, "
          f"{keep.sum():,} kept (>={min_pts} pts), "
          f"{(~keep).sum():,} dropped as sparse")
    # Compute center for each kept voxel.
    occ_idx = idx[np.isin(inv, np.where(keep)[0])]
    occ_idx_uniq = np.unique(occ_idx, axis=0)
    centers = (occ_idx_uniq + 0.5) * voxel
    return centers, counts[keep]


def cluster_cull(
    centers: np.ndarray,
    voxel: float,
    min_cluster_pts: int,
) -> np.ndarray:
    """Drop disconnected voxel islands smaller than min_cluster_pts.

    Build a temporary point cloud at voxel centers, run DBSCAN with eps
    = sqrt(3)*voxel (next-cell diagonal), keep clusters above threshold.
    """
    if min_cluster_pts <= 1:
        return centers
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(centers)
    eps = voxel * np.sqrt(3) * 1.01  # slightly above diagonal
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=1))
    if labels.max() < 0:
        return centers
    bincount = np.bincount(labels[labels >= 0])
    keep_clusters = np.where(bincount >= min_cluster_pts)[0]
    keep_mask = np.isin(labels, keep_clusters)
    print(f"  cluster-cull: {labels.max()+1} components, "
          f"{len(keep_clusters)} kept (>={min_cluster_pts} voxels), "
          f"{labels.max()+1-len(keep_clusters)} dropped as islands")
    return centers[keep_mask]


def voxels_to_mesh(centers: np.ndarray, voxel: float) -> o3d.geometry.TriangleMesh:
    """Emit one axis-aligned cube per voxel center.

    Vectorized — builds (8N, 3) verts and (12N, 3) tris in numpy, then
    hands to Open3D. ~100k voxels → 800k verts in a second.
    """
    n = len(centers)
    h = voxel / 2.0
    # Base cube: 8 verts (corners) + 12 tris (2 per face).
    base_v = np.array([
        [-1, -1, -1], [+1, -1, -1], [+1, +1, -1], [-1, +1, -1],
        [-1, -1, +1], [+1, -1, +1], [+1, +1, +1], [-1, +1, +1],
    ], dtype=np.float32) * h
    base_t = np.array([
        [0, 2, 1], [0, 3, 2],   # -z
        [4, 5, 6], [4, 6, 7],   # +z
        [0, 1, 5], [0, 5, 4],   # -y
        [2, 3, 7], [2, 7, 6],   # +y
        [1, 2, 6], [1, 6, 5],   # +x
        [0, 4, 7], [0, 7, 3],   # -x
    ], dtype=np.int64)

    verts = (centers[:, None, :] + base_v[None, :, :]).reshape(-1, 3)
    offsets = (np.arange(n, dtype=np.int64) * 8)[:, None, None]
    tris = (base_t[None, :, :] + offsets).reshape(-1, 3)

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(verts)
    mesh.triangles = o3d.utility.Vector3iVector(tris)
    mesh.compute_vertex_normals()
    return mesh


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="input .pcd or .ply")
    ap.add_argument("output", help="output .obj")
    ap.add_argument("--voxel", type=float, default=0.05,
                    help="voxel size in meters (default 0.05 = 5 cm)")
    ap.add_argument("--min-pts", type=int, default=4,
                    help="min LiDAR points per voxel to count as occupied "
                         "(default 4 — kills sparse outliers / glass reflections)")
    ap.add_argument("--min-cluster-pts", type=int, default=500,
                    help="drop disconnected voxel islands smaller than this "
                         "(default 500 voxels ≈ 0.5 m³ at 5 cm voxels)")
    ap.add_argument("--skip-ground-align", action="store_true",
                    help="skip RANSAC ground alignment (use if cloud is "
                         "already in a frame where z=up and floor ≈ 0)")
    args = ap.parse_args()

    in_path = Path(args.input)
    out_path = Path(args.output)
    if not in_path.exists():
        print(f"ERROR: input not found: {in_path}", file=sys.stderr)
        sys.exit(1)

    print(f"loading {in_path}...")
    pcd = o3d.io.read_point_cloud(str(in_path))
    print(f"  {len(pcd.points):,} points, bbox extent "
          f"{(np.asarray(pcd.points).max(0) - np.asarray(pcd.points).min(0))}")

    if not args.skip_ground_align:
        print("aligning ground...")
        pcd, _, tilt = align_ground(pcd)
        print(f"  rotated {tilt:.2f}°")

    print(f"binning at {args.voxel:.3f} m, min_pts={args.min_pts}...")
    centers, _ = voxel_density_filter(pcd, args.voxel, args.min_pts)

    if args.min_cluster_pts > 1:
        print(f"culling islands smaller than {args.min_cluster_pts} voxels...")
        centers = cluster_cull(centers, args.voxel, args.min_cluster_pts)

    print(f"emitting {len(centers):,} cubes → "
          f"{len(centers)*8:,} verts / {len(centers)*12:,} tris...")
    mesh = voxels_to_mesh(centers, args.voxel)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_triangle_mesh(str(out_path), mesh,
                               write_triangle_uvs=False,
                               write_vertex_normals=False)
    print(f"✓ wrote {out_path} ({out_path.stat().st_size / 1e6:.1f} MB)")
    print()
    print("MuJoCo collision note: each cube is its own convex hull, so")
    print("this mesh works as a single <geom type=\"mesh\"> WITH collision")
    print("(contype=1 conaffinity=1) — no tile splitting needed.")


if __name__ == "__main__":
    main()
