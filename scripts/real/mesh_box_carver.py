#!/usr/bin/env python3
"""mesh_box_carver.py — Open3D 3D viewer for batch-deleting mesh tris.

Workflow:
  1. Mesh shown in 3D + verts as a point cloud overlay (so you can
     shift+click them).
  2. Press K to enter pick mode; shift+click 2+ corner points of a
     region you want to cut.
  3. Press Q to close window → bright-yellow spheres at picked points
     in a confirmation window so you can see what you grabbed.
  4. Close confirmation → CLI prompts:
       - (d)elete tris in AABB of picks (any z padding)
       - (D)elete tris in TIGHT AABB (no padding)
       - (n)othing → back to viewer to add more picks
       - (u)ndo last delete
       - (s)ave snapshot
       - (x)save & exit
  5. Mesh is updated in place; next viewer shows the new mesh.

Usage:
    python3 mesh_box_carver.py \\
        --in  bags/meshes/ops2_cuda/scans_v4_tiled.obj \\
        --out bags/meshes/ops2_cuda/scans_v4_carved.obj
"""
import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--point-size", type=float, default=3.0)
    ap.add_argument("--vert-stride", type=int, default=1,
                    help="show every Nth vert as a pickable point "
                         "(higher = sparser, faster, less precise)")
    ap.add_argument("--default-z-pad-m", type=float, default=0.20,
                    help="padding around picked z range when deleting")
    args = ap.parse_args()

    inp_path = Path(args.inp)
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"loading {inp_path}")
    mesh = o3d.io.read_triangle_mesh(str(inp_path))
    mesh.compute_vertex_normals()
    print(f"  {len(mesh.vertices):,} v, {len(mesh.triangles):,} t")

    undo_stack = []

    while True:
        verts = np.asarray(mesh.vertices, dtype=np.float64)
        tris = np.asarray(mesh.triangles, dtype=np.int32)
        if args.vert_stride > 1:
            pickable_verts = verts[::args.vert_stride]
            pickable_idx = np.arange(0, len(verts), args.vert_stride)
        else:
            pickable_verts = verts
            pickable_idx = np.arange(len(verts))

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pickable_verts)
        # Color by z for visual distinction
        z = pickable_verts[:, 2]
        zn = (z - z.min()) / max(1e-6, z.max() - z.min())
        col = np.zeros((len(pickable_verts), 3))
        col[:, 0] = zn          # R = high z
        col[:, 2] = 1 - zn      # B = low z
        col[:, 1] = 0.3
        pcd.colors = o3d.utility.Vector3dVector(col)

        print()
        print(f"=== viewer ===  mesh: {len(mesh.vertices):,} v, {len(mesh.triangles):,} t")
        print("  K               : enter SELECTION mode (required)")
        print("  Shift + LMB     : pick a corner point  (≥2 needed for box)")
        print("  Shift + RMB     : un-pick last")
        print("  F               : back to freeview (rotate/pan/zoom)")
        print("  Q               : close → confirm picks → CLI prompt")

        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="mesh_box_carver", width=1600, height=900)
        vis.add_geometry(mesh)
        vis.add_geometry(pcd)
        opt = vis.get_render_option()
        opt.point_size = args.point_size
        opt.background_color = np.array([0.10, 0.10, 0.12])
        opt.mesh_show_wireframe = False
        opt.mesh_show_back_face = True
        vis.run()
        picked = vis.get_picked_points()
        vis.destroy_window()

        if len(picked) == 0:
            print("\nno picks. (s)ave / (x)save+exit / (c)ontinue ?")
            ch = input("> ").strip().lower()
            if ch == "s":
                o3d.io.write_triangle_mesh(str(out_path), mesh)
                print(f"✓ saved {out_path}")
            elif ch == "x":
                o3d.io.write_triangle_mesh(str(out_path), mesh)
                print(f"✓ saved + exit {out_path}")
                break
            continue

        # Confirmation viewer
        picked_xyz = pickable_verts[picked]
        print(f"\npicked {len(picked)} points")
        print(f"  XYZ AABB: x=[{picked_xyz[:,0].min():.2f},{picked_xyz[:,0].max():.2f}] "
              f"y=[{picked_xyz[:,1].min():.2f},{picked_xyz[:,1].max():.2f}] "
              f"z=[{picked_xyz[:,2].min():.2f},{picked_xyz[:,2].max():.2f}]")

        # Show AABB + yellow spheres in a confirmation window
        bmin = picked_xyz.min(0).astype(np.float64)
        bmax = picked_xyz.max(0).astype(np.float64)
        # Apply default z padding
        bmin[2] -= args.default_z_pad_m
        bmax[2] += args.default_z_pad_m
        aabb_geom = o3d.geometry.AxisAlignedBoundingBox(bmin, bmax)
        aabb_geom.color = (1, 1, 0)
        print(f"  proposed delete AABB (with z pad ±{args.default_z_pad_m}m): "
              f"x=[{bmin[0]:.2f},{bmax[0]:.2f}] "
              f"y=[{bmin[1]:.2f},{bmax[1]:.2f}] "
              f"z=[{bmin[2]:.2f},{bmax[2]:.2f}]")

        # Compute affected tris
        centroid = verts[tris].mean(axis=1)
        in_box = (
            (centroid[:, 0] >= bmin[0]) & (centroid[:, 0] <= bmax[0]) &
            (centroid[:, 1] >= bmin[1]) & (centroid[:, 1] <= bmax[1]) &
            (centroid[:, 2] >= bmin[2]) & (centroid[:, 2] <= bmax[2])
        )
        n_to_delete = int(in_box.sum())
        print(f"  → would delete {n_to_delete:,} tris "
              f"({100*n_to_delete/len(tris):.1f}%)")

        # Confirmation viewer
        vis2 = o3d.visualization.Visualizer()
        vis2.create_window(window_name="confirm AABB", width=1600, height=900)
        # Highlight the tris that would be deleted (red overlay)
        if n_to_delete > 0:
            del_mesh = o3d.geometry.TriangleMesh()
            del_mesh.vertices = mesh.vertices
            del_mesh.triangles = o3d.utility.Vector3iVector(tris[in_box])
            del_mesh.paint_uniform_color([1.0, 0.2, 0.2])
            del_mesh.compute_vertex_normals()
            vis2.add_geometry(del_mesh)
        vis2.add_geometry(mesh)
        vis2.add_geometry(aabb_geom)
        opt2 = vis2.get_render_option()
        opt2.background_color = np.array([0.10, 0.10, 0.12])
        opt2.mesh_show_back_face = True
        vis2.run()
        vis2.destroy_window()

        print("apply:")
        print("  (d) delete with z-pad             (D) delete with TIGHT z (no pad)")
        print("  (n) nothing → back to viewer       (u) undo last delete")
        print("  (s) save snapshot                  (x) save & exit")
        ch = input("> ").strip()

        if ch == "d":
            undo_stack.append(np.asarray(mesh.triangles))
            kept = tris[~in_box]
            mesh.triangles = o3d.utility.Vector3iVector(kept)
            mesh.compute_vertex_normals()
            print(f"  deleted {n_to_delete:,} tris ({len(tris)-len(kept)} actual)")
        elif ch == "D":
            # Recompute with tight z
            tight_zmin = picked_xyz[:, 2].min()
            tight_zmax = picked_xyz[:, 2].max()
            in_box_t = (
                (centroid[:, 0] >= bmin[0]) & (centroid[:, 0] <= bmax[0]) &
                (centroid[:, 1] >= bmin[1]) & (centroid[:, 1] <= bmax[1]) &
                (centroid[:, 2] >= tight_zmin) & (centroid[:, 2] <= tight_zmax)
            )
            undo_stack.append(np.asarray(mesh.triangles))
            mesh.triangles = o3d.utility.Vector3iVector(tris[~in_box_t])
            mesh.compute_vertex_normals()
            print(f"  deleted {int(in_box_t.sum()):,} tris (tight z)")
        elif ch == "u":
            if undo_stack:
                mesh.triangles = o3d.utility.Vector3iVector(undo_stack.pop())
                mesh.compute_vertex_normals()
                print("  ↩ undid last delete")
        elif ch == "s":
            o3d.io.write_triangle_mesh(str(out_path), mesh)
            print(f"  ✓ saved {out_path}")
        elif ch == "x":
            o3d.io.write_triangle_mesh(str(out_path), mesh)
            print(f"  ✓ saved + exit {out_path}")
            break


if __name__ == "__main__":
    main()
