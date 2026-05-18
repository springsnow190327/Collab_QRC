#!/usr/bin/env python3
"""Generate a MuJoCo heightfield (hfield) from a mesh's top-down profile.

For each XY cell in a grid, ray-cast downward from above and record the
highest z hit by the mesh. The resulting (H, W) elevation grid is saved
as a PNG (8-bit grayscale, normalised) plus a sidecar JSON with the
metadata MuJoCo needs to convert PNG luminance back to world heights.

Why heightfield: MuJoCo collides <geom type="hfield"> via the actual
elevation profile, NOT a convex hull. Walls become vertical extrusions
the robot literally bounces off; corridors between walls stay free.

Limitation: heightfield is 2.5D — only ONE elevation per XY cell. Mesh
ceilings / overhangs are flattened to their bottom surface. For ops2's
indoor layout (floor → ceilings >2m above) this is fine.

Usage:
    python3 mesh_to_hfield.py obstacles.obj out_dir \
        --resolution 0.10 --max-height 4.0
"""
import argparse
import json
from pathlib import Path

import numpy as np
import open3d as o3d
from PIL import Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mesh", help="input .obj (ideally obstacles-only)")
    ap.add_argument("out_dir", help="output directory for png + meta")
    ap.add_argument("--resolution", type=float, default=0.10,
                    help="hfield cell size in meters (default 0.10)")
    ap.add_argument("--max-height", type=float, default=4.0,
                    help="max height the hfield encodes (default 4 m, "
                         "anything taller saturates)")
    ap.add_argument("--margin", type=float, default=1.0,
                    help="extra padding around mesh bbox (default 1 m)")
    ap.add_argument("--name", default="ops2_hfield",
                    help="MuJoCo hfield asset name (default ops2_hfield)")
    args = ap.parse_args()

    mesh = o3d.io.read_triangle_mesh(args.mesh)
    vs = np.asarray(mesh.vertices)
    fs = np.asarray(mesh.triangles)
    print(f"mesh: {len(vs):,} v / {len(fs):,} f  "
          f"bbox z=[{vs[:,2].min():.2f}, {vs[:,2].max():.2f}]")

    # Grid bbox
    bmin = vs[:, :2].min(0) - args.margin
    bmax = vs[:, :2].max(0) + args.margin
    extent = bmax - bmin
    nrow = int(np.ceil(extent[1] / args.resolution))  # y axis = rows
    ncol = int(np.ceil(extent[0] / args.resolution))  # x axis = cols
    print(f"grid: {nrow} rows × {ncol} cols  ({nrow*ncol:,} cells, "
          f"world bbox=[{bmin}, {bmax}])")

    # For each face, rasterise its bounding triangle into the XY grid and
    # update z_max at touched cells. Fast vectorised version.
    z_grid = np.full((nrow, ncol), -1e9, dtype=np.float32)

    tri_pts = vs[fs]  # (F, 3, 3)
    # Bound each triangle's XY footprint to grid cells
    tri_xy_min = tri_pts[:, :, :2].min(axis=1)
    tri_xy_max = tri_pts[:, :, :2].max(axis=1)
    col_min = np.clip(((tri_xy_min[:, 0] - bmin[0]) / args.resolution).astype(np.int32), 0, ncol-1)
    col_max = np.clip(((tri_xy_max[:, 0] - bmin[0]) / args.resolution).astype(np.int32), 0, ncol-1)
    row_min = np.clip(((tri_xy_min[:, 1] - bmin[1]) / args.resolution).astype(np.int32), 0, nrow-1)
    row_max = np.clip(((tri_xy_max[:, 1] - bmin[1]) / args.resolution).astype(np.int32), 0, nrow-1)

    # For each triangle: for each cell in its bbox, set z_grid = max(z_grid, z_max(tri))
    tri_z_max = tri_pts[:, :, 2].max(axis=1)
    print(f"rasterising {len(fs):,} triangles into grid...")
    # Heavy loop. Even at 1.4M tris and a 10-cell average bbox, ~14M cell
    # updates. ~3-5 s on CPU. Could vectorise with np.add.at but the
    # per-tri inner loop is harder to express vectorised cleanly.
    for i in range(len(fs)):
        r0, r1 = row_min[i], row_max[i] + 1
        c0, c1 = col_min[i], col_max[i] + 1
        z = tri_z_max[i]
        if z > z_grid[r0:r1, c0:c1].min():
            sub = z_grid[r0:r1, c0:c1]
            np.maximum(sub, z, out=sub)
    print(f"  done")

    # Cells never touched stay at -1e9; map to 0 (floor level).
    touched = z_grid > -1e8
    z_grid = np.where(touched, z_grid, 0.0)
    z_grid = np.clip(z_grid, 0.0, args.max_height)

    # Normalise to [0, 1] for PNG; MuJoCo expects 0=zfloor, 1=zsize_z.
    norm = z_grid / args.max_height
    img = (norm * 255).astype(np.uint8)
    # PIL uses (row=y) ascending top-down; MuJoCo's hfield row 0 is
    # AT -y_extent. Flip vertical so image top = +y.
    img_flipped = np.flipud(img)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    png_path = out_dir / f"{args.name}.png"
    Image.fromarray(img_flipped, mode="L").save(png_path)

    # MuJoCo <hfield> attributes:
    #   size = "radius_x radius_y zsize_z zbase"
    #   nrow / ncol / size in xml; PNG luminance ∈ [0, 1] is multiplied
    #   by zsize_z + zbase added.
    rx = extent[0] / 2.0
    ry = extent[1] / 2.0
    # Grid center in world coords; <geom> pos must equal this to align.
    center_x = (bmin[0] + bmax[0]) / 2.0
    center_y = (bmin[1] + bmax[1]) / 2.0
    meta = {
        "name": args.name,
        "png": str(png_path.resolve()),
        "nrow": nrow,
        "ncol": ncol,
        "resolution": args.resolution,
        "max_height_m": args.max_height,
        "world_bbox_min": list(map(float, bmin)),
        "world_bbox_max": list(map(float, bmax)),
        "radius_x_m": float(rx),
        "radius_y_m": float(ry),
        "geom_center": [float(center_x), float(center_y), 0.0],
        "n_cells_touched": int(touched.sum()),
        "mjcf_snippet": (
            f'<asset>\n'
            f'  <hfield name="{args.name}" file="{png_path.resolve()}" '
            f'size="{rx:.3f} {ry:.3f} {args.max_height:.3f} 0.05" '
            f'nrow="{nrow}" ncol="{ncol}"/>\n'
            f'</asset>\n'
            f'<worldbody>\n'
            f'  <geom name="{args.name}_collision" type="hfield" '
            f'hfield="{args.name}" pos="{center_x:.3f} {center_y:.3f} 0" '
            f'rgba="0.6 0.6 0.65 0.4" contype="1" conaffinity="1"/>\n'
            f'</worldbody>\n'
        ),
    }
    meta_path = out_dir / f"{args.name}.json"
    meta_path.write_text(json.dumps(meta, indent=2))

    print(f"✓ wrote {png_path} ({png_path.stat().st_size/1e3:.1f} KB)")
    print(f"✓ wrote {meta_path}")
    print(f"  touched cells: {int(touched.sum()):,} / {nrow*ncol:,} "
          f"({100*touched.sum()/(nrow*ncol):.1f}%)")
    print(f"  z range in grid: [{z_grid.min():.2f}, {z_grid.max():.2f}]")
    print()
    print("MJCF snippet to paste:")
    print(meta["mjcf_snippet"])


if __name__ == "__main__":
    main()
