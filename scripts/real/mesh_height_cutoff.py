#!/usr/bin/env python3
"""mesh_height_cutoff.py — interactive 2D top-down max-z cutoff for a mesh.

Slide a single slider from min-z (bottom of mesh) to max-z. The top-down
view shows the mesh rasterised by per-cell max-z, BUT only counting
vertices/tris with z below the cutoff. Slide down → roof / overhead
disappears; slide up → everything visible.

When you press S (or click SAVE), all triangles whose CENTROID z exceeds
the cutoff get dropped; the result is saved to --out. Mesh topology
otherwise unchanged.

Usage:
    python3 mesh_height_cutoff.py \\
        --in  bags/meshes/ops2_cuda/scans_v4_tiled.obj \\
        --out bags/meshes/ops2_cuda/scans_v4_cut.obj \\
        --cell-size 0.10
"""
import argparse
from pathlib import Path

import numpy as np
import open3d as o3d
import matplotlib
matplotlib.use("TkAgg", force=True)
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--cell-size", type=float, default=0.10,
                    help="rasterisation cell size for top-down preview (m)")
    args = ap.parse_args()

    print(f"loading {args.inp}")
    mesh = o3d.io.read_triangle_mesh(args.inp)
    verts = np.asarray(mesh.vertices, dtype=np.float32)
    tris = np.asarray(mesh.triangles, dtype=np.int32)
    print(f"  {len(verts):,} v, {len(tris):,} t")

    z_min = float(verts[:, 2].min())
    z_max = float(verts[:, 2].max())
    print(f"  z range: [{z_min:.3f}, {z_max:.3f}] m")

    # Build a rasterisation index: for each vert, (iy, ix) on the grid
    cs = args.cell_size
    x0 = float(verts[:, 0].min()); y0 = float(verts[:, 1].min())
    x1 = float(verts[:, 0].max()); y1 = float(verts[:, 1].max())
    nx = int(np.ceil((x1 - x0) / cs)) + 1
    ny = int(np.ceil((y1 - y0) / cs)) + 1
    ix = ((verts[:, 0] - x0) / cs).astype(np.int64).clip(0, nx - 1)
    iy = ((verts[:, 1] - y0) / cs).astype(np.int64).clip(0, ny - 1)
    print(f"  raster grid {ny}×{nx} @ {cs}m")

    # Per-tri centroid z (we use this for the cutoff decision)
    tri_centroid_z = verts[tris, 2].mean(axis=1)

    def rasterise_below(cutoff):
        """Per-cell max-z of verts with z <= cutoff. Untouched cells = NaN."""
        mask = verts[:, 2] <= cutoff
        grid = np.full((ny, nx), -np.inf, dtype=np.float32)
        if mask.any():
            np.maximum.at(grid, (iy[mask], ix[mask]), verts[mask, 2])
        out = grid.copy()
        out[grid == -np.inf] = np.nan
        return out

    # Initial: cutoff at the top (show everything)
    initial_cutoff = z_max
    raster = rasterise_below(initial_cutoff)

    # Build figure
    fig, ax = plt.subplots(figsize=(18, 9))
    plt.subplots_adjust(bottom=0.15)
    extent = (x0, x1, y0, y1)
    img = ax.imshow(raster, origin="lower", extent=extent, cmap="viridis",
                    vmin=z_min, vmax=z_max, interpolation="nearest")
    cbar = fig.colorbar(img, ax=ax, label="max-z below cutoff [m]")
    ax.set_aspect("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")

    title_t = ax.set_title(_title(initial_cutoff, z_min, z_max,
                                   tri_centroid_z, len(tris)))

    # Slider
    ax_sl = fig.add_axes([0.15, 0.05, 0.65, 0.03])
    slider = Slider(ax_sl, "max-z cutoff [m]", z_min, z_max,
                    valinit=initial_cutoff, valstep=0.05)

    def on_slider(val):
        img.set_data(rasterise_below(val))
        title_t.set_text(_title(val, z_min, z_max, tri_centroid_z, len(tris)))
        fig.canvas.draw_idle()
    slider.on_changed(on_slider)

    # Save button
    ax_sv = fig.add_axes([0.85, 0.03, 0.10, 0.06])
    btn = Button(ax_sv, "SAVE", color="lightgreen")

    def on_save(_evt):
        cutoff = slider.val
        keep = tri_centroid_z <= cutoff
        kept_tris = tris[keep]
        n_drop = int((~keep).sum())
        print(f"\nsaving with cutoff z={cutoff:.3f}m")
        print(f"  dropped {n_drop:,} tris  ({100*n_drop/len(tris):.1f}%)  "
              f"→ {len(kept_tris):,} remain")
        new_mesh = o3d.geometry.TriangleMesh()
        new_mesh.vertices = mesh.vertices
        new_mesh.triangles = o3d.utility.Vector3iVector(kept_tris)
        new_mesh.compute_vertex_normals()
        out_path = Path(args.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        o3d.io.write_triangle_mesh(str(out_path), new_mesh)
        print(f"✓ wrote {out_path}")
    btn.on_clicked(on_save)

    # Key 's' also saves
    def on_key(ev):
        if ev.key == "s":
            on_save(None)
    fig.canvas.mpl_connect("key_press_event", on_key)

    print()
    print("=== controls ===")
    print("  drag slider     : preview top-down with z below cutoff")
    print("  SAVE button / s : drop tris with centroid above cutoff → write --out")
    print("  q               : quit (no save unless you pressed s first)")
    plt.show()


def _title(cutoff, z_min, z_max, tri_z, n_total):
    n_drop = int((tri_z > cutoff).sum())
    return (f"cutoff z = {cutoff:.2f} m  "
            f"(z range [{z_min:.2f}, {z_max:.2f}])  "
            f"would drop {n_drop:,} / {n_total:,} tris "
            f"({100*n_drop/n_total:.1f}%)")


if __name__ == "__main__":
    main()
