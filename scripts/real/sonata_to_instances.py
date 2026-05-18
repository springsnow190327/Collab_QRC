#!/usr/bin/env python3
"""sonata_to_instances.py — Sonata semantic labels → per-instance MJCF.

Pipeline (after sonata_inference.py produces labels.npy):
  1. Load the *labelled sample PCD* + per-point labels
  2. KNN-propagate labels from sample → all mesh vertices in the full OBJ
  3. For every "object" class (everything except wall + floor):
       a. Extract vertices of that class
       b. DBSCAN spatial cluster (one cluster = one physical instance)
       c. For each cluster:
           - If cluster small / thin → emit AABB box (cheap, exact for thin)
           - Else → CoACD convex decomposition → emit <mesh> + collision compound
  4. Write per-instance OBJ files + a single MJCF snippet

Why: Sonata gives per-point semantic labels but not instances. ScanNet-20 has
no "bike rack" class — bike racks all get labelled "otherfurniture" + maybe
"chair"/"table". DBSCAN within each semantic class recovers the instances.

CoACD vs AABB trade-off:
  - thin tubes (bike rack, handrail): AABB is already tight, CoACD overkill
  - chunky objects (table, cabinet): CoACD wins on collision realism
  - rule of thumb: AABB if max_extent < 1.5m AND volume < 1m³, else CoACD

Usage:
    python3 sonata_to_instances.py \\
        --mesh   bags/meshes/ops2_cuda/scans_v4_aligned.obj \\
        --sample /tmp/ops2_v4_sample.ply \\
        --labels bags/meshes/ops2_cuda/sonata/labels.npy \\
        --out    bags/meshes/ops2_cuda/sonata/instances
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import open3d as o3d
from sklearn.neighbors import KDTree

# Same labels as sonata_inference.py
CLASS_LABELS_20 = (
    "wall", "floor", "cabinet", "bed", "chair", "sofa", "table", "door",
    "window", "bookshelf", "picture", "counter", "desk", "curtain",
    "refrigerator", "shower_curtain", "toilet", "sink", "bathtub",
    "otherfurniture",
)
# Classes we treat as "non-collidable static surface" (handled separately
# by polyfit_lite). Everything else becomes a separate collidable instance.
STATIC_SURFACE_CLASSES = {0, 1}  # wall, floor


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mesh", required=True,
                    help="full mesh OBJ/PLY whose vertices we want to label")
    ap.add_argument("--coord", required=True,
                    help="coord.npy from sonata_inference (post-subsample)")
    ap.add_argument("--labels", required=True,
                    help="labels.npy from sonata_inference")
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--dbscan-eps", type=float, default=0.25)
    ap.add_argument("--dbscan-min-pts", type=int, default=30)
    ap.add_argument("--min-cluster-verts", type=int, default=80,
                    help="reject smaller clusters")
    ap.add_argument("--max-instances-per-class", type=int, default=200)
    ap.add_argument("--coacd-threshold", type=float, default=0.08,
                    help="CoACD concavity threshold (lower = more parts)")
    ap.add_argument("--max-aabb-extent", type=float, default=1.5,
                    help="if max extent < this AND volume < 1m^3, emit AABB "
                         "instead of CoACD (saves convex decomp work for "
                         "thin tubes / handrails / racks)")
    ap.add_argument("--max-aabb-volume", type=float, default=1.0)
    ap.add_argument("--robot-z-max", type=float, default=1.5,
                    help="discard instances whose BOTTOM is above this z. "
                         "Sonata trained on ScanNet (indoor, ~3m ceilings) "
                         "mislabels building rooftops/ceilings/awnings as "
                         "'table' or 'bed'; filter to robot-reachable z.")
    ap.add_argument("--name", default="ops2_inst")
    args = ap.parse_args()

    out_dir = Path(args.out); out_dir.mkdir(parents=True, exist_ok=True)
    meshes_dir = out_dir / "meshes"; meshes_dir.mkdir(exist_ok=True)

    # 1. Load full mesh
    print(f"[{time.strftime('%H:%M:%S')}] loading mesh {args.mesh}")
    mesh = o3d.io.read_triangle_mesh(args.mesh)
    verts = np.asarray(mesh.vertices)
    print(f"  {len(verts):,} verts, {len(mesh.triangles):,} tris")

    # 2. Load labelled sample coords + per-point labels (must be aligned 1:1)
    sample_pts = np.load(args.coord)
    sample_lbl = np.load(args.labels).astype(np.int32)
    assert len(sample_pts) == len(sample_lbl), \
        f"coord/labels length mismatch {len(sample_pts)} vs {len(sample_lbl)}"
    print(f"  labelled sample: {len(sample_pts):,} pts")

    # 3. KNN propagate: each mesh vert gets the modal label of its 5 nearest
    #    sample points. Robust to a few mislabels in dense clusters.
    print(f"[{time.strftime('%H:%M:%S')}] KNN-propagating labels (k=5)")
    tree = KDTree(sample_pts)
    _, idx = tree.query(verts, k=5)
    nbr_lbl = sample_lbl[idx]  # (N_mesh, 5)
    # Modal label per row
    vert_lbl = np.zeros(len(verts), dtype=np.int8)
    # Vectorised mode: count each of 20 classes per row, pick argmax
    for c in range(20):
        cnt = (nbr_lbl == c).sum(axis=1)
        # `gt` is the count of the currently-winning class
        if c == 0:
            best_c = np.full(len(verts), c, dtype=np.int8)
            best_n = cnt
        else:
            replace = cnt > best_n
            best_c[replace] = c
            best_n[replace] = cnt[replace]
    vert_lbl = best_c
    print(f"  mesh-vert distribution: " + ", ".join(
        f"{CLASS_LABELS_20[c]}:{int((vert_lbl == c).sum()):,}"
        for c in range(20) if (vert_lbl == c).sum() > 0))

    # Save mesh vert labels for downstream
    np.save(out_dir / "mesh_vert_labels.npy", vert_lbl)

    # 4. Per-class DBSCAN → per-cluster mesh export
    tris = np.asarray(mesh.triangles)
    geom_lines = []
    asset_lines = []
    summary = {"classes": {}, "instances": []}

    for cls_id, cls_name in enumerate(CLASS_LABELS_20):
        if cls_id in STATIC_SURFACE_CLASSES:
            continue
        sel = np.where(vert_lbl == cls_id)[0]
        if len(sel) < args.min_cluster_verts:
            continue
        print(f"[{time.strftime('%H:%M:%S')}] class={cls_name} "
              f"verts={len(sel):,} → DBSCAN")

        # DBSCAN via open3d (it has built-in)
        pcd_c = o3d.geometry.PointCloud()
        pcd_c.points = o3d.utility.Vector3dVector(verts[sel])
        labels_c = np.asarray(pcd_c.cluster_dbscan(
            eps=args.dbscan_eps,
            min_points=args.dbscan_min_pts,
            print_progress=False))
        n_clusters = int(labels_c.max() + 1) if labels_c.size and labels_c.max() >= 0 else 0
        print(f"    {n_clusters} clusters")

        if n_clusters == 0:
            continue
        cluster_counts = np.bincount(labels_c[labels_c >= 0])
        cluster_order = np.argsort(-cluster_counts)
        kept = 0
        for ci in cluster_order:
            if kept >= args.max_instances_per_class:
                break
            n_verts_in_cluster = int(cluster_counts[ci])
            if n_verts_in_cluster < args.min_cluster_verts:
                break  # sorted desc, rest are smaller
            vert_mask_global = np.zeros(len(verts), dtype=bool)
            vert_mask_global[sel[labels_c == ci]] = True
            # Extract sub-mesh: keep triangles whose ALL 3 verts are in cluster.
            tri_mask = vert_mask_global[tris].all(axis=1)
            sub_tris = tris[tri_mask]
            if len(sub_tris) < 10:
                # Pure-point cluster (no surface): build hull from points.
                pts_c = verts[sel[labels_c == ci]]
                hull = build_point_hull(pts_c)
                sub_verts = np.asarray(hull.vertices)
                sub_tris_local = np.asarray(hull.triangles)
            else:
                # Compact triangles to local indices.
                used = np.unique(sub_tris.flatten())
                local_idx = -np.ones(len(verts), dtype=np.int64)
                local_idx[used] = np.arange(len(used))
                sub_verts = verts[used]
                sub_tris_local = local_idx[sub_tris]

            inst_id = len(summary["instances"])
            inst_name = f"{args.name}_{cls_name}_{inst_id:04d}"
            mesh_path = meshes_dir / f"{inst_name}.obj"
            sub_mesh = o3d.geometry.TriangleMesh()
            sub_mesh.vertices = o3d.utility.Vector3dVector(sub_verts)
            sub_mesh.triangles = o3d.utility.Vector3iVector(sub_tris_local)
            sub_mesh.compute_vertex_normals()
            o3d.io.write_triangle_mesh(str(mesh_path), sub_mesh)

            bmin, bmax = sub_verts.min(0), sub_verts.max(0)
            ext = bmax - bmin
            vol = float(ext[0] * ext[1] * ext[2])
            # Drop instances whose entire body is above robot-reachable z.
            if bmin[2] > args.robot_z_max:
                print(f"    [skip] {cls_name} cluster {ci}: "
                      f"z=[{bmin[2]:.2f},{bmax[2]:.2f}] above {args.robot_z_max}m")
                continue
            # AABB shortcut: small thin/medium objects
            if (ext.max() < args.max_aabb_extent and vol < args.max_aabb_volume):
                # Emit AABB box
                cx, cy, cz = (bmin + bmax) / 2.0
                hx, hy, hz = (ext / 2.0).clip(min=0.025)  # 5cm min thickness
                geom_lines.append(
                    f'    <geom name="{inst_name}" type="box" '
                    f'pos="{cx:.4f} {cy:.4f} {cz:.4f}" '
                    f'size="{hx:.4f} {hy:.4f} {hz:.4f}" '
                    f'rgba="0.5 0.6 0.4 0.6" contype="1" conaffinity="1" '
                    f'condim="3" friction="0.8 0.02 0.01"/>'
                )
                method = "aabb"
            else:
                # CoACD decomposition
                method = "coacd"
                try:
                    import coacd
                    coacd_mesh = coacd.Mesh(sub_verts.astype(np.float64),
                                            sub_tris_local.astype(np.int32))
                    parts = coacd.run_coacd(coacd_mesh,
                                            threshold=args.coacd_threshold,
                                            max_convex_hull=-1,
                                            preprocess_mode="auto")
                    if len(parts) == 0:
                        raise RuntimeError("coacd returned 0 parts")
                    # Emit one <mesh> asset + one geom per convex part
                    for pi, (pv, pt) in enumerate(parts):
                        part_name = f"{inst_name}_p{pi:03d}"
                        part_path = meshes_dir / f"{part_name}.obj"
                        pm = o3d.geometry.TriangleMesh()
                        pm.vertices = o3d.utility.Vector3dVector(pv)
                        pm.triangles = o3d.utility.Vector3iVector(pt.astype(np.int32))
                        o3d.io.write_triangle_mesh(str(part_path), pm)
                        asset_lines.append(
                            f'    <mesh name="{part_name}" file="meshes/{part_name}.obj"/>'
                        )
                        geom_lines.append(
                            f'    <geom name="{part_name}_g" type="mesh" '
                            f'mesh="{part_name}" '
                            f'rgba="0.5 0.6 0.4 0.6" contype="1" conaffinity="1" '
                            f'condim="3" friction="0.8 0.02 0.01"/>'
                        )
                except Exception as e:
                    print(f"      coacd failed ({e}) — fall back to AABB")
                    method = "aabb_fallback"
                    cx, cy, cz = (bmin + bmax) / 2.0
                    hx, hy, hz = (ext / 2.0).clip(min=0.025)
                    geom_lines.append(
                        f'    <geom name="{inst_name}" type="box" '
                        f'pos="{cx:.4f} {cy:.4f} {cz:.4f}" '
                        f'size="{hx:.4f} {hy:.4f} {hz:.4f}" '
                        f'rgba="0.5 0.6 0.4 0.6" contype="1" conaffinity="1" '
                        f'condim="3" friction="0.8 0.02 0.01"/>'
                    )

            summary["instances"].append({
                "id": inst_id, "name": inst_name, "class": cls_name,
                "n_verts": n_verts_in_cluster,
                "extent": ext.tolist(),
                "method": method,
            })
            kept += 1
        summary["classes"][cls_name] = kept
        print(f"    kept {kept} instances")

    # 5. Write MJCF snippet
    asset_block = "\n".join(asset_lines) if asset_lines else ""
    geom_block = "\n".join(geom_lines)
    (out_dir / f"{args.name}_assets.xml").write_text(asset_block + "\n")
    (out_dir / f"{args.name}_geoms.xml").write_text(geom_block + "\n")
    (out_dir / f"{args.name}_summary.json").write_text(json.dumps(summary, indent=2))
    print(f"\n✓ {len(summary['instances'])} instances total")
    print(f"  classes: {summary['classes']}")
    print(f"✓ wrote {out_dir}/{args.name}_assets.xml")
    print(f"✓ wrote {out_dir}/{args.name}_geoms.xml")
    print(f"✓ wrote {out_dir}/{args.name}_summary.json")


def build_point_hull(pts):
    """For a cluster with too few triangles, build a coarse convex hull."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    try:
        hull, _ = pcd.compute_convex_hull()
        return hull
    except Exception:
        # Fall back to AABB-corners mesh
        bmin, bmax = pts.min(0), pts.max(0)
        corners = np.array([[bmin[0], bmin[1], bmin[2]],
                            [bmax[0], bmin[1], bmin[2]],
                            [bmax[0], bmax[1], bmin[2]],
                            [bmin[0], bmax[1], bmin[2]],
                            [bmin[0], bmin[1], bmax[2]],
                            [bmax[0], bmin[1], bmax[2]],
                            [bmax[0], bmax[1], bmax[2]],
                            [bmin[0], bmax[1], bmax[2]]])
        tris = np.array([[0,1,2],[0,2,3],[4,6,5],[4,7,6],
                         [0,4,5],[0,5,1],[1,5,6],[1,6,2],
                         [2,6,7],[2,7,3],[3,7,4],[3,4,0]])
        m = o3d.geometry.TriangleMesh()
        m.vertices = o3d.utility.Vector3dVector(corners)
        m.triangles = o3d.utility.Vector3iVector(tris)
        return m


if __name__ == "__main__":
    main()
