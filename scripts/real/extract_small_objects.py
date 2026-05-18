#!/usr/bin/env python3
"""extract_small_objects.py — DBSCAN clusters of leftover points → AABB boxes.

Companion to polyfit_lite.py. After plane extraction strips out walls/
floors/ramps, the residual point cloud contains small non-planar
objects: bike racks, handrails, furniture, signs. Each isolated cluster
becomes a tight axis-aligned bounding box, emitted as a MuJoCo box geom.

For thin / curved objects an AABB is a tight wrapper because they don't
span much in any axis. A bike rack (3cm tube × 1m long × 1m tall) →
box of size approx (0.05, 0.50, 0.50) — collidable, cheap.
"""
import argparse, json
from pathlib import Path
import numpy as np
import open3d as o3d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="point cloud .pcd / .ply (or .obj)")
    ap.add_argument("out_dir")
    ap.add_argument("--voxel", type=float, default=0.03,
                    help="downsample voxel size (default 0.03)")
    ap.add_argument("--ransac-dist", type=float, default=0.05)
    ap.add_argument("--strip-floor-z-max", type=float, default=0.10,
                    help="discard points with z below this (floor)")
    ap.add_argument("--strip-min-inliers", type=int, default=500,
                    help="iteratively strip planes with >= this many "
                         "inliers (default 500)")
    ap.add_argument("--strip-max-iter", type=int, default=120)
    ap.add_argument("--dbscan-eps", type=float, default=0.20)
    ap.add_argument("--dbscan-min-pts", type=int, default=20)
    ap.add_argument("--min-cluster-pts", type=int, default=30,
                    help="discard clusters smaller than this")
    ap.add_argument("--max-clusters", type=int, default=2000)
    ap.add_argument("--max-box-extent", type=float, default=2.0,
                    help="discard clusters larger than this in ANY axis "
                         "(default 2 m — bigger than this is probably a "
                         "missed plane fragment, not a small object)")
    ap.add_argument("--min-box-extent", type=float, default=0.05,
                    help="pad each AABB to at least this size in each axis")
    ap.add_argument("--name", default="ops2_small")
    args = ap.parse_args()

    in_path = Path(args.input)
    out_dir = Path(args.out_dir); out_dir.mkdir(parents=True, exist_ok=True)

    # Load
    if in_path.suffix.lower() in (".obj", ".ply"):
        mesh = o3d.io.read_triangle_mesh(str(in_path))
        pts = np.asarray(mesh.vertices)
    else:
        pcd_in = o3d.io.read_point_cloud(str(in_path))
        pts = np.asarray(pcd_in.points)
    print(f"loaded {len(pts):,} pts")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd_ds = pcd.voxel_down_sample(args.voxel)
    pts_ds = np.asarray(pcd_ds.points)
    # Strip ground
    pts_ds = pts_ds[pts_ds[:, 2] > args.strip_floor_z_max]
    print(f"after voxel ds {args.voxel}m + floor strip: {len(pts_ds):,} pts")

    # Iteratively strip the LARGE planes (these become walls — handled by polyfit_lite)
    remaining = pts_ds.copy()
    for it in range(args.strip_max_iter):
        if len(remaining) < args.strip_min_inliers:
            break
        rp = o3d.geometry.PointCloud()
        rp.points = o3d.utility.Vector3dVector(remaining)
        plane, inl = rp.segment_plane(
            distance_threshold=args.ransac_dist, ransac_n=3,
            num_iterations=2000)
        if len(inl) < args.strip_min_inliers:
            break
        mask = np.ones(len(remaining), dtype=bool); mask[inl] = False
        remaining = remaining[mask]
    print(f"after stripping large planes ({it+1} iter): {len(remaining):,} pts (small-object candidates)")

    if len(remaining) < args.dbscan_min_pts:
        print("not enough residual pts; nothing to cluster")
        return

    # DBSCAN on residual
    rp = o3d.geometry.PointCloud()
    rp.points = o3d.utility.Vector3dVector(remaining)
    labels = np.asarray(rp.cluster_dbscan(
        eps=args.dbscan_eps, min_points=args.dbscan_min_pts))
    n_clusters = int(labels.max() + 1) if labels.size and labels.max() >= 0 else 0
    print(f"DBSCAN: {n_clusters} clusters, "
          f"{int((labels < 0).sum()):,} noise pts")

    if n_clusters == 0:
        print("no clusters; aborting")
        return

    counts = np.bincount(labels[labels >= 0])
    # Sort clusters by size descending
    cluster_ids = np.argsort(-counts)
    boxes = []
    for cid in cluster_ids:
        n = int(counts[cid])
        if n < args.min_cluster_pts:
            continue
        pts_c = remaining[labels == cid]
        bmin = pts_c.min(0); bmax = pts_c.max(0)
        ext = bmax - bmin
        if ext.max() > args.max_box_extent:
            continue  # too big to be a "small object"; probably plane leftover
        # Pad to min extent so very thin objects still get a usable AABB
        for ax in range(3):
            if ext[ax] < args.min_box_extent:
                pad = (args.min_box_extent - ext[ax]) / 2.0
                bmin[ax] -= pad; bmax[ax] += pad
                ext[ax] = args.min_box_extent
        cx, cy, cz = (bmin + bmax) / 2.0
        hx, hy, hz = ext / 2.0
        boxes.append({
            "id": len(boxes),
            "n_pts": n,
            "center": [float(cx), float(cy), float(cz)],
            "size": [float(hx), float(hy), float(hz)],
        })
        if len(boxes) >= args.max_clusters:
            break
    print(f"kept {len(boxes)} small-object boxes")

    # Emit MJCF snippet
    lines = []
    for b in boxes:
        cx,cy,cz = b['center']; hx,hy,hz = b['size']
        lines.append(
            f'    <geom name="{args.name}_b{b["id"]:04d}" type="box" '
            f'pos="{cx:.4f} {cy:.4f} {cz:.4f}" '
            f'size="{hx:.4f} {hy:.4f} {hz:.4f}" '
            f'rgba="0.5 0.55 0.45 0.55" contype="1" conaffinity="1" '
            f'condim="3" friction="0.8 0.02 0.01"/>'
        )
    snippet = "\n".join(lines) + "\n"
    (out_dir / f"{args.name}_boxes.json").write_text(json.dumps(boxes, indent=2))
    (out_dir / f"{args.name}_geoms.xml").write_text(snippet)
    print(f"✓ wrote {out_dir / f'{args.name}_boxes.json'}")
    print(f"✓ wrote {out_dir / f'{args.name}_geoms.xml'}  ({len(lines)} boxes)")


if __name__ == "__main__":
    main()
