#!/usr/bin/env python3
"""
decompose_mesh.py — V-HACD-style convex decomposition via coacd.

For MuJoCo, each <geom type="mesh"> collides as its convex hull. A single SLAM
mesh's hull engulfs the whole scene, so the robot penetrates floor/walls.
Splitting into MANY convex pieces via collision-aware decomposition produces
hulls that hug local geometry — robot interacts with each piece independently.

Tunable: `threshold` (0..1, smaller = more pieces, finer collision).
For our 80x32 m indoor scene, threshold 0.05 typically gives 50-200 pieces.

Usage:
    python3 decompose_mesh.py <input.obj> <out_dir> [--threshold 0.05]
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
import trimesh
import coacd
import open3d as o3d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("out_dir")
    ap.add_argument("--threshold", type=float, default=0.05,
                    help="coacd concavity threshold (lower=more pieces, default 0.05)")
    ap.add_argument("--max-convex-hull", type=int, default=-1,
                    help="cap on number of convex pieces (-1 = unlimited)")
    args = ap.parse_args()

    in_path = Path(args.input)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"loading {in_path}...")
    mesh = trimesh.load(str(in_path), force="mesh")
    print(f"  {len(mesh.vertices):,} verts / {len(mesh.faces):,} faces")
    print(f"  bbox: {mesh.bounds[1] - mesh.bounds[0]}")

    print(f"\nrunning coacd (threshold={args.threshold})...")
    coacd_mesh = coacd.Mesh(mesh.vertices, mesh.faces)
    parts = coacd.run_coacd(coacd_mesh,
                            threshold=args.threshold,
                            max_convex_hull=args.max_convex_hull,
                            preprocess_mode="auto")
    print(f"  → {len(parts)} convex pieces")

    manifest = []
    total_tris = 0
    for k, (verts, faces) in enumerate(parts):
        name = f"hull_{k:03d}"
        fpath = out_dir / f"{name}.obj"
        sub = o3d.geometry.TriangleMesh()
        sub.vertices = o3d.utility.Vector3dVector(np.asarray(verts))
        sub.triangles = o3d.utility.Vector3iVector(np.asarray(faces))
        sub.compute_vertex_normals()
        o3d.io.write_triangle_mesh(str(fpath), sub, write_triangle_uvs=False)
        total_tris += len(faces)
        bbox_min = np.min(verts, axis=0)
        bbox_max = np.max(verts, axis=0)
        manifest.append({
            "name": name,
            "file": fpath.name,
            "n_verts": len(verts),
            "n_tris": len(faces),
            "bbox_min": bbox_min.tolist(),
            "bbox_max": bbox_max.tolist(),
        })

    manifest_path = out_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2))
    print(f"\n✓ wrote {len(parts)} hulls + manifest to {out_dir}")
    print(f"  total triangles: {total_tris:,} (orig {len(mesh.faces):,})")
    n_tris = [m["n_tris"] for m in manifest]
    print(f"  tris/hull: min={min(n_tris)}, max={max(n_tris)}, median={sorted(n_tris)[len(n_tris)//2]}")


if __name__ == "__main__":
    main()
