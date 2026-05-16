#!/usr/bin/env python3
"""
pcd_to_mesh.py — Convert a Fast-LIO accumulated PCD to a triangle mesh for
                  NuRec's mesh_to_usd_collider.py and/or MuJoCo direct import.

Pipeline:
  1. Load PCD (xyz or xyzrgb)
  2. Voxel downsample  (reduces noise, speeds up Poisson)
  3. Statistical outlier removal
  4. Normal estimation  (oriented toward sensor origin = camera_init)
  5. Poisson surface reconstruction
  6. Trim low-density triangles  (crop faces at forest edges)
  7. Save .ply (for NuRec) and .obj (for MuJoCo <mesh file=...>)

Usage:
  python3 pcd_to_mesh.py <input.pcd> [options]

Options:
  --out-dir DIR          Output directory (default: same dir as input)
  --voxel FLOAT          Voxel size for downsampling in metres (default: 0.05)
  --depth INT            Poisson octree depth — higher = more detail (default: 10)
  --density-pct FLOAT    Remove faces below this density percentile (default: 5.0)
  --no-obj               Skip OBJ export (MuJoCo)
  --no-ply               Skip PLY export (NuRec)
  --nurec-collider       Also run mesh_to_usd_collider.py if available

Example:
  python3 scripts/real/pcd_to_mesh.py src/vendor/fast_lio/PCD/scans.pcd \\
      --out-dir /tmp/slam_mesh --depth 10 --voxel 0.05

Coordinate frame:
  Fast-LIO publishes in the 'camera_init' frame (z-up, x-forward).
  MuJoCo expects z-up — no rotation needed.
  NuRec's mesh_to_usd_collider.py also defaults to z-up.
"""

import argparse
import sys
import os
from pathlib import Path

import numpy as np
import open3d as o3d


def load_pcd(path: str) -> o3d.geometry.PointCloud:
    pcd = o3d.io.read_point_cloud(path)
    if len(pcd.points) == 0:
        sys.exit(f"ERROR: {path} is empty or unreadable")
    print(f"  loaded  {len(pcd.points):,} points  from {path}")
    return pcd


def preprocess(pcd: o3d.geometry.PointCloud, voxel: float) -> o3d.geometry.PointCloud:
    pcd = pcd.voxel_down_sample(voxel_size=voxel)
    print(f"  voxel({voxel}m) → {len(pcd.points):,} points")

    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    print(f"  outlier removal → {len(pcd.points):,} points")
    return pcd


def estimate_normals(pcd: o3d.geometry.PointCloud, voxel: float) -> o3d.geometry.PointCloud:
    radius = voxel * 6
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30)
    )
    # Orient normals consistently (toward positive z — sensor is roughly above ground)
    pcd.orient_normals_to_align_with_direction(orientation_reference=[0, 0, 1])
    print(f"  normals estimated  (search_radius={radius:.3f}m)")
    return pcd


def poisson_reconstruct(
    pcd: o3d.geometry.PointCloud, depth: int, density_pct: float
) -> o3d.geometry.TriangleMesh:
    print(f"  Poisson reconstruction  depth={depth} ...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth, width=0, scale=1.1, linear_fit=False
    )
    densities = np.asarray(densities)
    threshold = np.percentile(densities, density_pct)
    keep = densities > threshold
    mesh.remove_vertices_by_mask(~keep)
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    print(f"  mesh  {len(mesh.vertices):,} verts  {len(mesh.triangles):,} faces"
          f"  (removed bottom {density_pct:.0f}% density)")
    return mesh


def ball_pivot_reconstruct(
    pcd: o3d.geometry.PointCloud, voxel: float
) -> o3d.geometry.TriangleMesh:
    """Ball Pivoting Algorithm — only connects existing points, doesn't hallucinate
    surfaces between gaps. Right choice for dense LiDAR clouds where Poisson
    would over-smooth or fill cavities. Radii are derived from voxel size."""
    # Multiple radii catch features at different scales (small first for detail,
    # larger to bridge expected scan-line gaps).
    radii = [voxel * 1.5, voxel * 3.0, voxel * 6.0, voxel * 12.0]
    print(f"  Ball Pivoting  radii={[f'{r:.3f}' for r in radii]} ...")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector(radii)
    )
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    print(f"  mesh  {len(mesh.vertices):,} verts  {len(mesh.triangles):,} faces")
    return mesh


def save_outputs(
    mesh: o3d.geometry.TriangleMesh,
    out_dir: Path,
    stem: str,
    write_ply: bool,
    write_obj: bool,
) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = {}

    if write_ply:
        p = out_dir / f"{stem}.ply"
        o3d.io.write_triangle_mesh(str(p), mesh)
        print(f"  → PLY  {p}  ({p.stat().st_size // 1024} KB)")
        paths["ply"] = p

    if write_obj:
        p = out_dir / f"{stem}.obj"
        o3d.io.write_triangle_mesh(str(p), mesh, write_triangle_uvs=False)
        print(f"  → OBJ  {p}  ({p.stat().st_size // 1024} KB)")
        paths["obj"] = p

    return paths


def run_nurec_collider(ply_path: Path, out_dir: Path):
    nurec_script = Path.home() / "Research/3D_Reconstruction/nurec_pipeline/scripts/mesh_to_usd_collider.py"
    if not nurec_script.exists():
        print(f"  WARN: NuRec collider script not found at {nurec_script}, skipping")
        return
    usd_path = out_dir / f"{ply_path.stem}_collision.usd"
    cmd = (f"python3 {nurec_script} {ply_path} {usd_path} "
           f"--up-axis Z --approx convexDecomposition")
    print(f"  running NuRec collider: {cmd}")
    ret = os.system(cmd)
    if ret == 0:
        print(f"  → USD  {usd_path}")
    else:
        print(f"  WARN: mesh_to_usd_collider.py exited {ret}")


def print_mjcf_snippet(obj_path: Path):
    rel = obj_path.name
    print(f"""
── MuJoCo MJCF snippet ──────────────────────────────────────────
  Copy {obj_path} into your scene's mesh directory, then add:

  <asset>
    <mesh name="slam_scene" file="{rel}" scale="1 1 1"/>
  </asset>
  <worldbody>
    <geom type="mesh" mesh="slam_scene" contype="1" conaffinity="1"
          rgba="0.7 0.7 0.7 1" pos="0 0 0"/>
  </worldbody>
─────────────────────────────────────────────────────────────────
""")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help="Path to input .pcd file")
    ap.add_argument("--out-dir", default=None,
                    help="Output directory (default: <input_dir>/<stem>_mesh/)")
    ap.add_argument("--voxel", type=float, default=0.05,
                    help="Voxel downsampling size in metres (default 0.05)")
    ap.add_argument("--method", choices=["ball_pivot", "poisson"], default="ball_pivot",
                    help="Reconstruction method: 'ball_pivot' (faithful, default for dense"
                         " Livox) or 'poisson' (smooth, can hallucinate)")
    ap.add_argument("--depth", type=int, default=8,
                    help="Poisson octree depth (default 8, was 10; smaller=more conservative)")
    ap.add_argument("--density-pct", type=float, default=5.0,
                    help="Remove faces below this density percentile (poisson only, default 5.0)")
    ap.add_argument("--no-obj", action="store_true", help="Skip OBJ export")
    ap.add_argument("--no-ply", action="store_true", help="Skip PLY export")
    ap.add_argument("--nurec-collider", action="store_true",
                    help="Run NuRec mesh_to_usd_collider.py on the PLY output")
    args = ap.parse_args()

    in_path = Path(args.input).expanduser().resolve()
    if not in_path.exists():
        sys.exit(f"ERROR: {in_path} not found")

    stem = in_path.stem
    out_dir = Path(args.out_dir).expanduser().resolve() if args.out_dir \
              else in_path.parent / f"{stem}_mesh"

    print(f"\n{'='*60}")
    print(f"  PCD → Mesh  {in_path.name}")
    print(f"  out_dir     {out_dir}")
    print(f"  voxel={args.voxel}m  depth={args.depth}  density_pct={args.density_pct}%")
    print(f"{'='*60}\n")

    pcd  = load_pcd(str(in_path))
    pcd  = preprocess(pcd, args.voxel)
    pcd  = estimate_normals(pcd, args.voxel)
    if args.method == "ball_pivot":
        mesh = ball_pivot_reconstruct(pcd, args.voxel)
    else:
        mesh = poisson_reconstruct(pcd, args.depth, args.density_pct)

    paths = save_outputs(
        mesh, out_dir, stem,
        write_ply=not args.no_ply,
        write_obj=not args.no_obj,
    )

    if args.nurec_collider and "ply" in paths:
        run_nurec_collider(paths["ply"], out_dir)

    if "obj" in paths:
        print_mjcf_snippet(paths["obj"])

    print("done.\n")


if __name__ == "__main__":
    main()
