#!/usr/bin/env python3
"""Split aligned mesh into floor (discarded) and obstacle parts.

Why: MuJoCo's per-mesh convex hull collision engulfs floor verts that
have small bumps + height variation, so the robot ends up trapped in
the hull slab. Real-world fix: floor is a perfect MuJoCo plane (already
in the scene), and only the non-floor geometry (walls, furniture,
handrails, bike racks) becomes collidable. This script extracts the
non-floor faces and writes them as a separate OBJ.

Inputs:
  --floor-z-max    : verts with z <= this are floor (default 0.20m)
  --floor-shrink-z : how much to also discard "near-floor" faces above
                     to avoid carving slivers into the floor (default 0.10m)
"""
import argparse
import numpy as np
import open3d as o3d
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("obstacles_out")
    ap.add_argument("--floor-z-max", type=float, default=0.20,
                    help="discard faces whose centroid z <= this (default 0.20m)")
    ap.add_argument("--floor-out", default="",
                    help="optional: also write the floor part for inspection")
    args = ap.parse_args()

    mesh = o3d.io.read_triangle_mesh(args.input)
    vs = np.asarray(mesh.vertices)
    fs = np.asarray(mesh.triangles)
    print(f"in: {len(vs):,} v / {len(fs):,} f  z range [{vs[:,2].min():.2f}, {vs[:,2].max():.2f}]")

    centroid_z = vs[fs].mean(axis=1)[:, 2]
    is_floor = centroid_z <= args.floor_z_max
    print(f"floor faces (centroid z <= {args.floor_z_max}): {int(is_floor.sum()):,}")
    print(f"obstacle faces: {int((~is_floor).sum()):,}")

    obs = o3d.geometry.TriangleMesh(mesh)
    obs.remove_triangles_by_mask(is_floor)
    obs.remove_unreferenced_vertices()
    obs.compute_vertex_normals()
    print(f"obstacles out: {len(obs.vertices):,} v / {len(obs.triangles):,} f")
    Path(args.obstacles_out).parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_triangle_mesh(args.obstacles_out, obs,
                                write_triangle_uvs=False, write_vertex_normals=False)
    print(f"✓ wrote {args.obstacles_out}")

    if args.floor_out:
        floor = o3d.geometry.TriangleMesh(mesh)
        floor.remove_triangles_by_mask(~is_floor)
        floor.remove_unreferenced_vertices()
        o3d.io.write_triangle_mesh(args.floor_out, floor,
                                    write_triangle_uvs=False, write_vertex_normals=False)
        print(f"✓ wrote {args.floor_out} (floor; not needed in sim)")


if __name__ == "__main__":
    main()
