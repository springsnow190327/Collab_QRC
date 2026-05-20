#!/usr/bin/env python3
"""polyfit_lite.py — PolyFit-style plane primitive extraction from a point
cloud, output as MuJoCo <geom type="box"/> primitives.

Real PolyFit (Nan & Wonka 2017) extracts plane primitives via RANSAC then
runs an ILP to pick the optimal subset that forms a watertight polyhedral
surface. We skip the ILP step and instead emit each detected plane as a
thin oriented AABB-along-plane-normal — equivalent for our purposes:
collision in MuJoCo per-box is exact, no convex hull problem, ~100 boxes
covers an 80×32m building (vs millions of triangles).

Pipeline:
  1. Voxel downsample for speed (per-cell normal estimation)
  2. Repeat:
     - RANSAC fit a plane to remaining points
     - Stop if inliers < min_inliers
     - Compute 2D bbox of inliers in plane frame
     - Emit oriented box (plane bbox × `box_thickness`)
     - Remove inliers from pool
  3. Output JSON manifest + MJCF snippet

Each plane becomes:
  <geom type="box" pos="..." size="hx hy hz/2" euler="..."/>
where (hx, hy) match the plane's 2D extent and hz = box_thickness.

Usage:
    python3 polyfit_lite.py points.pcd out/  --voxel 0.05 --min-inliers 500
"""
import argparse
import json
import math
import sys
import types
from pathlib import Path

import numpy as np

# open3d.__init__ eagerly imports open3d.ml, which pulls in a newer sklearn
# whose array_api_compat fails against the system numpy 1.x (`module 'numpy'
# has no attribute '_CopyMode'`). We don't use open3d.ml — stub it out before
# importing open3d so the core geometry/RANSAC/DBSCAN API loads cleanly.
sys.modules.setdefault("open3d.ml", types.ModuleType("open3d.ml"))
import open3d as o3d


def fit_plane_local_frame(inlier_pts: np.ndarray, normal: np.ndarray):
    """Return (center, R_world2plane, 2D extent in plane) for a plane.

    Build a right-handed local frame whose +z is the plane normal. Find
    an in-plane axis that aligns with the dominant variance direction of
    the inliers (PCA). Then compute the 2D bbox in this frame.
    """
    # Make sure normal is unit length and pointing consistently up-ish.
    normal = normal / np.linalg.norm(normal)
    # PCA on inliers to find dominant in-plane direction.
    center = inlier_pts.mean(axis=0)
    centered = inlier_pts - center
    # Remove the normal component so we get PCA in the plane.
    centered_in_plane = centered - np.outer(centered @ normal, normal)
    # PCA via SVD.
    U, S, Vt = np.linalg.svd(centered_in_plane, full_matrices=False)
    # First in-plane axis = first right singular vector (largest variance).
    # If S[0] is on the normal direction it'll be tiny; defensive:
    in_plane_axis = Vt[0]
    # Ensure axis is perpendicular to normal (defensive in case of noise).
    in_plane_axis = in_plane_axis - normal * (in_plane_axis @ normal)
    if np.linalg.norm(in_plane_axis) < 1e-6:
        in_plane_axis = np.array([1.0, 0, 0])
        in_plane_axis = in_plane_axis - normal * (in_plane_axis @ normal)
    in_plane_axis /= np.linalg.norm(in_plane_axis)
    # Second in-plane axis = normal × first (right-handed).
    second_axis = np.cross(normal, in_plane_axis)
    # Rotation R: columns = [in_plane_axis, second_axis, normal]
    R = np.column_stack([in_plane_axis, second_axis, normal])
    # Project all inliers into plane frame to get 2D extents.
    proj = centered @ R  # (N, 3) → (x, y, z); z ≈ 0
    bbox_min_xy = proj[:, :2].min(0)
    bbox_max_xy = proj[:, :2].max(0)
    ext_xy = bbox_max_xy - bbox_min_xy
    center_offset_xy = (bbox_min_xy + bbox_max_xy) / 2.0
    # Adjust center to be the bbox centre (not the points' centroid).
    center_world = center + R[:, 0] * center_offset_xy[0] + R[:, 1] * center_offset_xy[1]
    return center_world, R, ext_xy


def R_to_quat(R: np.ndarray) -> tuple:
    """Convert 3×3 rotation matrix to MuJoCo (w, x, y, z) quaternion."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (w, x, y, z)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="input .pcd / .ply / .obj")
    ap.add_argument("out_dir", help="output directory")
    ap.add_argument("--voxel", type=float, default=0.05,
                    help="downsample voxel size (m)")
    ap.add_argument("--ransac-dist", type=float, default=0.05,
                    help="RANSAC inlier distance threshold (m)")
    ap.add_argument("--ransac-iter", type=int, default=2000)
    ap.add_argument("--min-inliers", type=int, default=500,
                    help="stop when largest plane has fewer inliers than this")
    ap.add_argument("--max-planes", type=int, default=300,
                    help="hard cap on number of planes extracted")
    ap.add_argument("--box-thickness", type=float, default=0.05,
                    help="z-thickness of each plane's box geom in meters")
    ap.add_argument("--min-extent", type=float, default=0.30,
                    help="discard plane patches smaller than this (m) in "
                         "either x or y of plane frame")
    ap.add_argument("--name", default="ops2_polyfit",
                    help="prefix for emitted geom names")
    # 2026-05-20: inlier clustering. RANSAC inliers for one plane are often
    # scattered across the building (e.g. the left + right corridor walls are
    # COPLANAR, so one RANSAC plane catches both). Taking the AABB over all of
    # them produces a single 47m box that bridges the corridor gap and blocks
    # the path. DBSCAN-clustering the inliers in the plane frame and emitting
    # one tight box per cluster preserves the gap.
    ap.add_argument("--cluster-eps", type=float, default=0.6,
                    help="DBSCAN epsilon (m) for splitting coplanar inliers "
                         "into separate wall segments. 0 disables clustering.")
    ap.add_argument("--cluster-min-pts", type=int, default=30,
                    help="DBSCAN min points per cluster")
    # Clamp wall vertical extent so misfit planes don't make 10m-tall slabs.
    ap.add_argument("--max-wall-height", type=float, default=4.0,
                    help="clamp a wall box's vertical (world-z) extent to this "
                         "many meters, anchored at the cluster's min-z. 0 = off.")
    args = ap.parse_args()

    in_path = Path(args.input)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Load points
    print(f"loading {in_path}...")
    if in_path.suffix.lower() in (".obj", ".ply"):
        mesh = o3d.io.read_triangle_mesh(str(in_path))
        pts = np.asarray(mesh.vertices)
    else:
        pcd = o3d.io.read_point_cloud(str(in_path))
        pts = np.asarray(pcd.points)
    print(f"  {len(pts):,} points")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd_ds = pcd.voxel_down_sample(args.voxel)
    pts_ds = np.asarray(pcd_ds.points)
    print(f"  after voxel ds {args.voxel}m: {len(pts_ds):,} points")

    planes = []
    remaining = pts_ds.copy()

    for k in range(args.max_planes):
        if len(remaining) < args.min_inliers:
            print(f"\nremaining {len(remaining):,} < min_inliers {args.min_inliers}; stop")
            break
        pcd_rem = o3d.geometry.PointCloud()
        pcd_rem.points = o3d.utility.Vector3dVector(remaining)
        plane, inliers = pcd_rem.segment_plane(
            distance_threshold=args.ransac_dist,
            ransac_n=3,
            num_iterations=args.ransac_iter,
        )
        a, b, c, d = plane
        normal = np.array([a, b, c]); normal /= np.linalg.norm(normal)
        inlier_pts = remaining[inliers]
        if len(inlier_pts) < args.min_inliers:
            print(f"  plane[{k}]: only {len(inlier_pts)} inliers (<{args.min_inliers}); stop")
            break
        # Split coplanar inliers into spatially-separate clusters so a single
        # RANSAC plane that caught both sides of a corridor becomes 2 tight
        # boxes (gap preserved) instead of one corridor-spanning slab.
        if args.cluster_eps > 0.0:
            cl_pcd = o3d.geometry.PointCloud()
            cl_pcd.points = o3d.utility.Vector3dVector(inlier_pts)
            labels = np.asarray(cl_pcd.cluster_dbscan(
                eps=args.cluster_eps, min_points=args.cluster_min_pts))
            unique = [lbl for lbl in set(labels.tolist()) if lbl >= 0]
            clusters = [inlier_pts[labels == lbl] for lbl in unique]
            if not clusters:  # all noise → fall back to whole-plane box
                clusters = [inlier_pts]
        else:
            clusters = [inlier_pts]

        n_emitted = 0
        for cl_pts in clusters:
            if len(cl_pts) < args.cluster_min_pts:
                continue
            center_world, R, ext_xy = fit_plane_local_frame(cl_pts, normal)
            if ext_xy[0] < args.min_extent or ext_xy[1] < args.min_extent:
                continue

            sx = float(ext_xy[0] / 2.0)
            sy = float(ext_xy[1] / 2.0)
            # Clamp wall vertical extent: find which in-plane axis is most
            # vertical (largest |world-z| component of its column in R) and
            # cap that half-size, re-anchoring the box center at the cluster's
            # min-z + half the clamped height so it grows UP from the floor.
            if args.max_wall_height > 0.0:
                col_x_z = abs(R[2, 0])  # world-z component of local-x axis
                col_y_z = abs(R[2, 1])  # world-z component of local-y axis
                half_h = args.max_wall_height / 2.0
                if col_x_z > col_y_z and (2 * sx) > args.max_wall_height:
                    z_min = float(cl_pts[:, 2].min())
                    sx = half_h
                    center_world = center_world.copy()
                    center_world[2] = z_min + half_h
                elif col_y_z >= col_x_z and (2 * sy) > args.max_wall_height:
                    z_min = float(cl_pts[:, 2].min())
                    sy = half_h
                    center_world = center_world.copy()
                    center_world[2] = z_min + half_h

            quat = R_to_quat(R)
            planes.append({
                "id": len(planes),
                "n_inliers": int(len(cl_pts)),
                "normal": normal.tolist(),
                "center": center_world.tolist(),
                "size_xyz": [sx, sy, float(args.box_thickness / 2.0)],
                "quat_wxyz": list(quat),
            })
            n_emitted += 1
        print(f"  plane[{k}]: {len(inlier_pts):,} inliers → {len(clusters)} "
              f"cluster(s) → {n_emitted} box(es)  "
              f"normal=({normal[0]:+.2f},{normal[1]:+.2f},{normal[2]:+.2f})")
        # Remove inliers
        mask = np.ones(len(remaining), dtype=bool)
        mask[inliers] = False
        remaining = remaining[mask]

    print(f"\nkept {len(planes)} planes out of {args.max_planes} max")

    # Emit MJCF snippet
    geom_lines = []
    for p in planes:
        cx, cy, cz = p["center"]
        sx, sy, sz = p["size_xyz"]
        qw, qx, qy, qz = p["quat_wxyz"]
        geom_lines.append(
            f'    <geom name="{args.name}_p{p["id"]:03d}" type="box" '
            f'pos="{cx:.4f} {cy:.4f} {cz:.4f}" '
            f'size="{sx:.4f} {sy:.4f} {sz:.4f}" '
            f'quat="{qw:.6f} {qx:.6f} {qy:.6f} {qz:.6f}" '
            f'rgba="0.65 0.6 0.55 0.5" contype="1" conaffinity="1" '
            f'condim="3" friction="0.8 0.02 0.01"/>'
        )
    snippet = "\n".join(geom_lines)

    (out_dir / f"{args.name}_planes.json").write_text(json.dumps(planes, indent=2))
    (out_dir / f"{args.name}_geoms.xml").write_text(snippet + "\n")
    print(f"\n✓ wrote {out_dir / f'{args.name}_planes.json'}  ({len(planes)} planes)")
    print(f"✓ wrote {out_dir / f'{args.name}_geoms.xml'}  ({len(geom_lines)} geom lines)")
    print()
    print("Paste the geoms.xml content inside <worldbody> of your MJCF.")


if __name__ == "__main__":
    main()
