#!/usr/bin/env python3
"""
tile_mesh.py — split a large reconstructed mesh into XY-grid tiles so MuJoCo's
                convex-hull collision becomes LOCAL geometry instead of one giant
                hull encompassing the entire scene.

Why: MuJoCo treats <geom type="mesh"> as its convex hull for collision. For a
80m × 32m × 10m SLAM scene that's a single huge hull — the robot ends up
penetrating arbitrary surfaces. By splitting the mesh into e.g. 8×4 = 32 XY
tiles each ~10m × 8m, the hull of each tile approximates the local geometry
well enough that the robot can stand on it, collide with walls within it,
etc. The hull is still convex-per-tile so concave features (alcoves, doorways)
are smoothed over, but at tile size 10m × 8m the convex approximation is
much closer to truth than the whole-scene hull.

Usage:
    python3 tile_mesh.py <input.obj> <out_dir> [--tiles NX NY]

Outputs:
    <out_dir>/tile_NN.obj          (NX*NY files)
    <out_dir>/tiles_manifest.json  (index for MJCF generation)
"""
import argparse
import json
from pathlib import Path

import numpy as np
import open3d as o3d


def split_mesh_xy(mesh: o3d.geometry.TriangleMesh, nx: int, ny: int):
    """Split mesh by which XY tile each triangle's CENTROID falls in."""
    V = np.asarray(mesh.vertices)
    T = np.asarray(mesh.triangles)
    centroids = V[T].mean(axis=1)   # (N, 3) triangle centroids

    x_min, x_max = V[:, 0].min(), V[:, 0].max()
    y_min, y_max = V[:, 1].min(), V[:, 1].max()
    dx = (x_max - x_min) / nx
    dy = (y_max - y_min) / ny

    tiles = []
    for i in range(nx):
        for j in range(ny):
            x_lo, x_hi = x_min + i * dx, x_min + (i + 1) * dx
            y_lo, y_hi = y_min + j * dy, y_min + (j + 1) * dy
            mask = ((centroids[:, 0] >= x_lo) & (centroids[:, 0] < x_hi) &
                    (centroids[:, 1] >= y_lo) & (centroids[:, 1] < y_hi))
            if mask.sum() == 0:
                continue
            tile_tri = T[mask]
            # Remap vertex indices: keep only used vertices, renumber.
            used = np.unique(tile_tri)
            old_to_new = -np.ones(len(V), dtype=np.int64)
            old_to_new[used] = np.arange(len(used))
            new_tri = old_to_new[tile_tri]
            new_V = V[used]
            sub = o3d.geometry.TriangleMesh()
            sub.vertices = o3d.utility.Vector3dVector(new_V)
            sub.triangles = o3d.utility.Vector3iVector(new_tri)
            sub.compute_vertex_normals()
            tiles.append({
                "ix": i, "iy": j,
                "x_lo": float(x_lo), "x_hi": float(x_hi),
                "y_lo": float(y_lo), "y_hi": float(y_hi),
                "mesh": sub,
                "n_tri": len(new_tri),
            })
    return tiles, (x_min, x_max, y_min, y_max)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("out_dir")
    ap.add_argument("--tiles", nargs=2, type=int, default=[8, 4],
                    help="NX NY: number of tiles along x and y (default 8 4)")
    args = ap.parse_args()

    in_path = Path(args.input)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"loading {in_path}...")
    mesh = o3d.io.read_triangle_mesh(str(in_path))
    print(f"  {len(mesh.vertices):,} verts / {len(mesh.triangles):,} tris")
    print(f"splitting into {args.tiles[0]}x{args.tiles[1]} tiles...")

    tiles, bbox = split_mesh_xy(mesh, args.tiles[0], args.tiles[1])
    print(f"  bbox xy: [{bbox[0]:.1f},{bbox[1]:.1f}] x [{bbox[2]:.1f},{bbox[3]:.1f}]")
    print(f"  {len(tiles)} non-empty tiles produced")

    manifest = []
    for k, t in enumerate(tiles):
        name = f"tile_{k:02d}"
        fpath = out_dir / f"{name}.obj"
        o3d.io.write_triangle_mesh(str(fpath), t["mesh"], write_triangle_uvs=False)
        manifest.append({
            "name": name,
            "file": fpath.name,
            "ix": t["ix"], "iy": t["iy"],
            "x_lo": t["x_lo"], "x_hi": t["x_hi"],
            "y_lo": t["y_lo"], "y_hi": t["y_hi"],
            "n_tri": t["n_tri"],
        })

    manifest_path = out_dir / "tiles_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2))
    print(f"\n✓ wrote {len(manifest)} tiles + manifest to {out_dir}")
    print(f"  tile triangle counts: min={min(m['n_tri'] for m in manifest)}, "
          f"max={max(m['n_tri'] for m in manifest)}, "
          f"total={sum(m['n_tri'] for m in manifest)}")


if __name__ == "__main__":
    main()
