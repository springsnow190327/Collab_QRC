#!/usr/bin/env python3
"""trav_labeler_3d.py — Open3D 3D heightmap viewer + label editor.

Renders the heightmap as a point cloud (one point per touched cell at
its world (x, y, z)) coloured by label. Supports:

    LMB drag    : rotate
    Shift+LMB   : pick a vertex (it'll get a marker dot)
    MMB drag    : pan
    Wheel       : zoom
    q           : close viewer → CLI prompts what to do with picked cells

Workflow:
    1. Inspect labels visually in 3D
    2. Shift+click any cells you want to re-label
    3. Press 'q' to close window
    4. CLI asks: apply (l)ethal / (f)ree / (c)lear / (n)othing
    5. Saves + re-opens viewer for next round (or exit with 'x')

Usage:
    python3 trav_labeler_3d.py \\
        --labels bags/meshes/ops2_cuda/hfield/ops2_v4_auto_labels.npz \\
        --out   bags/meshes/ops2_cuda/hfield/ops2_v4_auto_labels.npz
"""
import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


# Same conventions as trav_labeler.py
UNLBL = -1
FREE = 0
LETHAL = 1


def make_cloud(heights, touched, labels, cell, origin, z_lift=0.01):
    """Build (points, idx_grid_yx) so we can map picked points back to cells."""
    H, W = heights.shape
    ys, xs = np.where(touched)
    z = heights[ys, xs]
    wx = origin[0] + (xs + 0.5) * cell
    wy = origin[1] + (ys + 0.5) * cell
    pts = np.stack([wx, wy, z + z_lift], axis=1).astype(np.float64)
    grid_yx = np.stack([ys, xs], axis=1)  # row -> (iy, ix)
    return pts, grid_yx


def label_colors(labels_at_pts):
    """Map label values per-point to RGB."""
    col = np.full((len(labels_at_pts), 3), 0.55, dtype=np.float64)  # gray
    col[labels_at_pts == LETHAL] = [0.85, 0.10, 0.10]   # red
    col[labels_at_pts == FREE]   = [0.10, 0.75, 0.10]   # green
    return col


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--labels", required=True,
                    help="npz from auto_label_heightmap.py or trav_labeler.py")
    ap.add_argument("--out", default=None,
                    help="output .npz (defaults to overwriting --labels)")
    ap.add_argument("--point-size", type=float, default=4.0)
    ap.add_argument("--height-exaggerate", type=float, default=1.0,
                    help="multiply z by this for better visual relief")
    args = ap.parse_args()

    out_path = Path(args.out) if args.out else Path(args.labels)

    d = np.load(args.labels, allow_pickle=False)
    heights = d["heights"]
    touched = d["touched"]
    labels = d["labels"].astype(np.int8)
    cell = float(d["cell_size_m"])
    origin = d["origin_xy"].astype(np.float32)

    pts, grid_yx = make_cloud(
        heights * args.height_exaggerate, touched, labels, cell, origin)
    print(f"loaded {len(pts):,} touched cells "
          f"(lethal={int((labels == LETHAL).sum()):,}, "
          f"free={int((labels == FREE).sum()):,}, "
          f"unlabelled={int((labels == UNLBL).sum() - (~touched).sum()):,})")

    while True:
        labels_at_pts = labels[grid_yx[:, 0], grid_yx[:, 1]]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pcd.colors = o3d.utility.Vector3dVector(label_colors(labels_at_pts))

        print()
        print("=== 3D viewer (Open3D VisualizerWithEditing) ===")
        print("  K          : enter SELECTION mode   ⚠ (REQUIRED before picking)")
        print("  Shift+LMB  : pick a cell (yellow marker appears)")
        print("  Shift+RMB  : un-pick last cell")
        print("  F          : back to free-view (rotate/pan/zoom)")
        print("  LMB drag   : rotate     MMB drag: pan     Wheel: zoom")
        print("  Q          : close window  → CLI prompt for action")
        print()
        print("  Typical flow:  F → navigate → K → shift+click cells → Q")

        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="trav_labeler_3d", width=1600, height=900)
        vis.add_geometry(pcd)
        opt = vis.get_render_option()
        opt.point_size = args.point_size
        opt.background_color = np.array([0.10, 0.10, 0.12])

        # Live cursor: poll picked list each frame; new picks → bright yellow
        # sphere added at that point. Persists across the whole pick session.
        sphere_radius = max(0.10, cell * 1.5)
        live_marker_state = {"last_n": 0, "spheres": []}

        def _live_marker_cb(_vis):
            picks = _vis.get_picked_points()
            n = len(picks)
            if n > live_marker_state["last_n"]:
                for idx in picks[live_marker_state["last_n"]:]:
                    s = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_radius)
                    s.translate(pts[idx])
                    s.paint_uniform_color([1.0, 1.0, 0.0])
                    s.compute_vertex_normals()
                    try:
                        _vis.add_geometry(s, reset_bounding_box=False)
                    except TypeError:
                        # older Open3D signature
                        _vis.add_geometry(s)
                    live_marker_state["spheres"].append(s)
                live_marker_state["last_n"] = n
            elif n < live_marker_state["last_n"]:
                # User cleared picks (rare); drop all markers
                for s in live_marker_state["spheres"]:
                    try:
                        _vis.remove_geometry(s, reset_bounding_box=False)
                    except TypeError:
                        _vis.remove_geometry(s)
                live_marker_state["spheres"].clear()
                live_marker_state["last_n"] = 0
            return False

        try:
            vis.register_animation_callback(_live_marker_cb)
        except Exception as e:
            print(f"  (animation callback unavailable: {e}; "
                  f"markers will only show in confirmation window)")

        vis.run()
        picked = vis.get_picked_points()
        vis.destroy_window()
        print(f"\npicked {len(picked)} cells")

        if len(picked) == 0:
            choice = input("nothing picked. (s)ave / (q)uit / (c)ontinue ? ").strip().lower()
            if choice == "s":
                save(out_path, labels, heights, touched, cell, origin)
            elif choice == "q":
                save(out_path, labels, heights, touched, cell, origin)
                break
            continue

        # Show picked cell coords for trust
        for idx in picked[:8]:
            yy, xx = grid_yx[idx]
            wx = origin[0] + (xx + 0.5) * cell
            wy = origin[1] + (yy + 0.5) * cell
            print(f"  picked cell ({yy},{xx})  world=({wx:.2f}, {wy:.2f})  "
                  f"z={float(heights[yy, xx]):.2f}m  "
                  f"label={int(labels[yy, xx])}")
        if len(picked) > 8:
            print(f"  ... +{len(picked) - 8} more")

        print("apply to picked cells:")
        print("  (l) LETHAL    (f) FREE    (c) clear (unlabel)")
        print("  (b) brush radius -> apply label to picked + neighbours")
        print("  (n) nothing — back to viewer")
        print("  (x) save & exit")
        choice = input("> ").strip().lower()

        if choice in ("l", "f", "c"):
            tgt = {"l": LETHAL, "f": FREE, "c": UNLBL}[choice]
            for idx in picked:
                yy, xx = grid_yx[idx]
                labels[yy, xx] = tgt
            print(f"applied {len(picked)} cells -> {tgt}")
            save(out_path, labels, heights, touched, cell, origin)
        elif choice == "b":
            try:
                r = int(input("brush radius (cells) > ").strip())
            except ValueError:
                r = 2
            tgt_s = input("label (l/f/c) > ").strip().lower()
            tgt = {"l": LETHAL, "f": FREE, "c": UNLBL}.get(tgt_s, None)
            if tgt is None:
                print("bad label key, skipping")
                continue
            count = 0
            for idx in picked:
                yy, xx = grid_yx[idx]
                y0 = max(0, yy - r); y1 = min(labels.shape[0], yy + r + 1)
                x0 = max(0, xx - r); x1 = min(labels.shape[1], xx + r + 1)
                Y, X = np.mgrid[y0:y1, x0:x1]
                d2 = (Y - yy) ** 2 + (X - xx) ** 2
                m = (d2 <= r * r) & touched[y0:y1, x0:x1]
                labels[y0:y1, x0:x1] = np.where(m, tgt, labels[y0:y1, x0:x1])
                count += int(m.sum())
            print(f"applied brush r={r} on {len(picked)} centers → {count} cells -> {tgt}")
            save(out_path, labels, heights, touched, cell, origin)
        elif choice == "x":
            save(out_path, labels, heights, touched, cell, origin)
            break
        else:
            print("no-op")


def save(out_path, labels, heights, touched, cell, origin):
    n_lethal = int((labels == LETHAL).sum())
    n_free = int((labels == FREE).sum())
    np.savez_compressed(
        out_path,
        labels=labels.astype(np.int8),
        heights=heights,
        touched=touched,
        cell_size_m=np.float32(cell),
        origin_xy=origin,
    )
    print(f"✓ saved {out_path}  lethal={n_lethal:,}  free={n_free:,}")


if __name__ == "__main__":
    main()
