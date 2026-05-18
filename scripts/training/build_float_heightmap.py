#!/usr/bin/env python3
"""build_float_heightmap.py — top-down z(x,y) from mesh, float32 npz.

Companion to mesh_to_hfield.py (which writes uint8 PNG for MuJoCo
<hfield>). For CNN training we need the actual height in meters per
cell, not 0-255 normalised. We also persist the mesh→cell mask so the
labeler can show "no mesh here" cells differently.

Output (saved to --out npz):
    heights  : (H, W) float32   — meters, NaN where no mesh
    touched  : (H, W) bool      — True where at least one mesh tri projects
    cell_size_m : float
    origin_xy   : (2,) float32  — world (x, y) at cell (0, 0)
    H, W : int
"""
import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mesh", help="input mesh OBJ/PLY")
    ap.add_argument("--out", required=True, help="output npz")
    ap.add_argument("--cell-size", type=float, default=0.10)
    ap.add_argument("--zmax", type=float, default=5.0)
    ap.add_argument("--zmin", type=float, default=-0.5)
    args = ap.parse_args()

    print(f"loading mesh {args.mesh}...")
    m = o3d.io.read_triangle_mesh(args.mesh)
    verts = np.asarray(m.vertices, dtype=np.float32)
    tris = np.asarray(m.triangles, dtype=np.int32)
    print(f"  {len(verts):,} verts, {len(tris):,} tris")

    # Grid extent
    bmin = verts[:, :2].min(0)
    bmax = verts[:, :2].max(0)
    cs = args.cell_size
    nx = int(np.ceil((bmax[0] - bmin[0]) / cs)) + 1
    ny = int(np.ceil((bmax[1] - bmin[1]) / cs)) + 1
    print(f"  grid: {ny} × {nx}  cell {cs}m  origin {bmin}")

    # Per-vertex contribution (faster than per-triangle for our use):
    # rasterise each vert's z into the cell it falls into; keep max
    heights = np.full((ny, nx), -1e9, dtype=np.float32)
    ix = ((verts[:, 0] - bmin[0]) / cs).astype(np.int64).clip(0, nx - 1)
    iy = ((verts[:, 1] - bmin[1]) / cs).astype(np.int64).clip(0, ny - 1)
    # mask out-of-z verts (e.g. floating outliers)
    valid = (verts[:, 2] > args.zmin) & (verts[:, 2] < args.zmax)
    ix, iy, vz = ix[valid], iy[valid], verts[valid, 2]
    # np.maximum.at handles the per-cell max
    np.maximum.at(heights, (iy, ix), vz)
    touched = heights > -1e8
    # Fill untouched with 0 (ground plane assumed)
    heights[~touched] = 0.0
    print(f"  touched: {int(touched.sum()):,} cells  "
          f"({100 * touched.sum() / touched.size:.1f}%)")

    np.savez_compressed(
        args.out,
        heights=heights,
        touched=touched,
        cell_size_m=np.float32(cs),
        origin_xy=bmin.astype(np.float32),
        H=ny, W=nx,
    )
    print(f"✓ wrote {args.out}  shape ({ny}, {nx})")
    print(f"  z range  {heights[touched].min():.2f}  →  {heights[touched].max():.2f} m")


if __name__ == "__main__":
    main()
