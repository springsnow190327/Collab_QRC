#!/usr/bin/env python3
"""pcd_to_mesh_cuda.py — GPU-accelerated faithful mesh reconstruction.

Pipeline (all GPU except marching cubes):
  1. Load PCD  (host)
  2. Voxelise + sparse occupancy field on GPU                    [cupy]
  3. 3D Gaussian smoothing of occupancy → density field          [cupy]
  4. Marching cubes at iso-threshold → triangle mesh             [skimage]
  5. RANSAC floor align + connected-component cull               [open3d]

Why this is "faithful, no blob":
  - Voxel occupancy is binary: a voxel is only marked if a real point fell
    in it. No Poisson-style implicit-field extrapolation into empty regions.
  - The Gaussian smooth has tight finite support (3-sigma radius). It can
    blur a surface OUTWARD by σ cells but cannot create geometry far from
    observed points.
  - Marching cubes at iso = 0.5 walks the boundary between voxels touched
    and untouched by the smoothing kernel. No blob can form in any region
    further than ~3σ from a real point.

5090 (sm_120) compatibility: cupy works directly, no torch involvement.

Usage:
    python3 pcd_to_mesh_cuda.py input.pcd out.obj \
        --voxel 0.05 --sigma 1.5 --iso 0.4 \
        --min-pts-per-voxel 3 \
        --cluster-min-tri 2000
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np
import open3d as o3d


def vlog(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def load_pcd(path: Path) -> np.ndarray:
    vlog(f"loading {path}...")
    pcd = o3d.io.read_point_cloud(str(path))
    pts = np.asarray(pcd.points, dtype=np.float32)
    vlog(f"  {len(pts):,} points  bbox extent={pts.max(0) - pts.min(0)}")
    return pts


def voxel_connected_cluster_filter(
    occ: np.ndarray,
    keep_pct: float = 0.001,
    min_voxels: int = 50,
) -> np.ndarray:
    """DBSCAN on the SPARSE occupied voxel centres (sklearn, ball-tree).

    Dense scipy.ndimage.label requires N×4 bytes for the label array; for
    3.5 B cells that's 14 GB → OOM after the count buffer is already
    holding 14 GB. The voxel grid is 99.9% empty, so we operate on the
    occupied indices directly: 3.8 M (z,y,x) triples → ~50 MB.

    eps = sqrt(3) ≈ 1.733 in voxel units → 26-connectivity equivalent.
    """
    from sklearn.cluster import DBSCAN

    occ_idx = np.argwhere(occ > 0).astype(np.float32)  # (N, 3) z,y,x
    n_occ = len(occ_idx)
    vlog(f"  sparse DBSCAN on {n_occ:,} occupied voxels "
         f"(keep_pct={keep_pct:.3f}, min_voxels={min_voxels})...")
    if n_occ == 0:
        return occ
    # eps = sqrt(3) covers 26-neighbour distance in voxel units.
    db = DBSCAN(eps=1.733, min_samples=1, algorithm="ball_tree", n_jobs=-1)
    labels = db.fit_predict(occ_idx)
    n_clusters = int(labels.max() + 1)
    counts = np.bincount(labels[labels >= 0])
    biggest = int(counts.max()) if counts.size else 0
    threshold = max(min_voxels, int(keep_pct * biggest))
    keep_cluster_ids = np.where(counts >= threshold)[0]
    keep_mask = np.isin(labels, keep_cluster_ids)
    n_kept_clusters = int(len(keep_cluster_ids))
    n_keep_vox = int(keep_mask.sum())
    n_drop_vox = n_occ - n_keep_vox
    vlog(f"  clusters: {n_clusters} total, {n_kept_clusters} kept "
         f"(≥{threshold} voxels each); voxels kept: {n_keep_vox:,}/{n_occ:,} "
         f"({100*n_keep_vox/n_occ:.1f}%), dropped {n_drop_vox:,}")

    # Rebuild sparse occupancy with only kept voxels.
    out = np.zeros_like(occ)
    kept_idx = occ_idx[keep_mask].astype(np.int64)
    out[kept_idx[:, 0], kept_idx[:, 1], kept_idx[:, 2]] = 1
    return out


def cpu_bin_occupancy(
    pts: np.ndarray,
    voxel: float,
    min_pts_per_voxel: int,
    margin_cells: int = 3,
) -> tuple[np.ndarray, np.ndarray]:
    """Bin points into a dense voxel grid on CPU (numpy bincount).

    Returns (occupancy_uint8, origin). The binning is fast enough on CPU
    (~1 s for 25M points) that we skip GPU to avoid the cupy/nvrtc CCCL
    JIT path that breaks on Blackwell sm_120. The expensive step
    (3D Gaussian smoothing) still runs on GPU.
    """
    bbox_min = pts.min(0) - voxel * margin_cells
    bbox_max = pts.max(0) + voxel * margin_cells
    grid_shape = np.ceil((bbox_max - bbox_min) / voxel).astype(np.int64)
    nx, ny, nz = int(grid_shape[0]), int(grid_shape[1]), int(grid_shape[2])
    n_cells = nx * ny * nz
    bytes_count = n_cells * 4
    vlog(f"  voxel {voxel:.3f} m  →  grid {nx}×{ny}×{nz} = {n_cells:,} cells "
         f"({bytes_count/1e9:.2f} GB count buffer)")
    if bytes_count > 32_000_000_000:
        raise SystemExit(
            f"ERROR: count buffer {bytes_count/1e9:.1f} GB exceeds 32 GB cap. "
            "Use a coarser --voxel or tighter bbox.")
    if bytes_count > 12_000_000_000:
        vlog(f"  WARNING: large count buffer {bytes_count/1e9:.1f} GB — "
             "make sure host RAM has headroom")

    ijk = np.floor((pts - bbox_min) / voxel).astype(np.int64)
    inside = (
        (ijk[:, 0] >= 0) & (ijk[:, 0] < nx)
        & (ijk[:, 1] >= 0) & (ijk[:, 1] < ny)
        & (ijk[:, 2] >= 0) & (ijk[:, 2] < nz)
    )
    ijk = ijk[inside]
    flat = (ijk[:, 2] * (ny * nx) + ijk[:, 1] * nx + ijk[:, 0]).astype(np.int64)
    vlog(f"  counting per-voxel ({len(flat):,} valid points)...")
    counts = np.bincount(flat, minlength=n_cells)
    occ = (counts >= min_pts_per_voxel).astype(np.uint8).reshape(nz, ny, nx)
    n_occ = int(occ.sum())
    vlog(f"  occupied voxels: {n_occ:,}  ({100.0*n_occ/n_cells:.2f}%)")
    return occ, bbox_min.astype(np.float32)


def smooth_occupancy(occ_cpu: np.ndarray, sigma: float) -> np.ndarray:
    """3D Gaussian blur of binary occupancy → smooth density [0, 1].

    cupy 14 + Blackwell sm_120: cupyx.scipy.ndimage.gaussian_filter hits an
    nvrtc CCCL FP8 JIT bug and won't compile. Fall back to scipy on CPU —
    separable 3 × 1D pass, ~15-25 s for 200M voxels with σ≈1.5.
    """
    from scipy.ndimage import gaussian_filter

    vlog(f"  CPU Gaussian smooth σ={sigma:.2f} cells (separable 3×1D)...")
    density = gaussian_filter(
        occ_cpu.astype(np.float32), sigma=sigma, mode="constant", cval=0.0
    )
    peak = float(density.max())
    if peak > 1e-6:
        density /= peak
    vlog(f"  density peak after norm: {float(density.max()):.3f}  "
         f"mean (occupied): {float(density[density > 0.1].mean()):.3f}")
    return density


def marching_cubes_cpu(
    density: np.ndarray,
    voxel: float,
    origin: np.ndarray,
    iso: float,
) -> o3d.geometry.TriangleMesh:
    """Extract iso-surface at `iso` value of density. CPU (skimage)."""
    from skimage.measure import marching_cubes

    vlog(f"  marching cubes  iso={iso}  (skimage)...")
    verts, faces, _, _ = marching_cubes(
        density, level=iso, spacing=(voxel, voxel, voxel),
    )
    # skimage returns verts indexed (z, y, x). Swap to (x, y, z) world.
    verts_xyz = verts[:, [2, 1, 0]].astype(np.float32)
    verts_xyz += origin
    # Triangle winding swap to keep mesh outward.
    faces = faces[:, [0, 2, 1]].astype(np.int64)
    vlog(f"  raw mesh: {len(verts_xyz):,} verts  {len(faces):,} tris")
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(verts_xyz.astype(np.float64))
    mesh.triangles = o3d.utility.Vector3iVector(faces)
    mesh.compute_vertex_normals()
    return mesh


def align_ground(
    mesh: o3d.geometry.TriangleMesh, bottom_pct: float = 0.05
) -> o3d.geometry.TriangleMesh:
    """Rotate so the dominant horizontal plane is z=0 (RANSAC on bottom 5%)."""
    vs = np.asarray(mesh.vertices)
    if len(vs) < 200:
        return mesh
    z_thresh = np.quantile(vs[:, 2], bottom_pct)
    floor_vs = vs[vs[:, 2] <= z_thresh]
    if len(floor_vs) < 100:
        vlog("  ground-align skipped (too few low verts)")
        return mesh
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(floor_vs)
    plane, inliers = pcd.segment_plane(
        distance_threshold=0.03, ransac_n=3, num_iterations=1000)
    a, b, c, d = plane
    normal = np.array([a, b, c]); normal /= np.linalg.norm(normal)
    if normal[2] < 0:
        normal = -normal
        d = -d
    z_axis = np.array([0.0, 0.0, 1.0])
    cos_t = float(np.clip(normal @ z_axis, -1, 1))
    angle = float(np.arccos(cos_t))
    if angle < 1e-4:
        rot = np.eye(3)
    else:
        axis = np.cross(normal, z_axis); axis /= np.linalg.norm(axis)
        K = np.array([[0, -axis[2], axis[1]],
                      [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        rot = np.eye(3) + np.sin(angle) * K + (1 - cos_t) * (K @ K)
    vs2 = (vs @ rot.T).astype(np.float64)
    vs2[:, 2] -= float(np.median(vs2[inliers, 2])) if len(inliers) else 0.0
    mesh.vertices = o3d.utility.Vector3dVector(vs2)
    mesh.compute_vertex_normals()
    vlog(f"  rotated {np.degrees(angle):.2f}° to z-up")
    return mesh


def cluster_cull(
    mesh: o3d.geometry.TriangleMesh, min_tri: int
) -> o3d.geometry.TriangleMesh:
    """Drop disconnected components below `min_tri` triangles."""
    if min_tri <= 0:
        return mesh
    vlog(f"  cluster cull (keep components ≥ {min_tri} tris)...")
    tri_labels, n_tri_per_cluster, _ = mesh.cluster_connected_triangles()
    tri_labels = np.asarray(tri_labels)
    n_tri = np.asarray(n_tri_per_cluster)
    drop = n_tri[tri_labels] < min_tri
    mesh.remove_triangles_by_mask(drop)
    mesh.remove_unreferenced_vertices()
    vlog(f"  kept {len(mesh.triangles):,} tris  "
         f"({(~drop).sum()} of {len(drop)} tris)")
    return mesh


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="input .pcd / .ply")
    ap.add_argument("output", help="output .obj")
    ap.add_argument("--voxel", type=float, default=0.05,
                    help="voxel size in meters (default 0.05). 0.02 gives "
                         "fine detail but uses ~3 GB on 80×32×10m scene.")
    ap.add_argument("--sigma", type=float, default=1.5,
                    help="Gaussian smoothing σ in voxel cells (default 1.5). "
                         "Higher = smoother surface, slightly more inward "
                         "rounding of corners.")
    ap.add_argument("--iso", type=float, default=0.4,
                    help="marching-cubes iso threshold on smoothed density "
                         "(default 0.4). Lower = thicker shell, higher = "
                         "tighter to true surface.")
    ap.add_argument("--min-pts-per-voxel", type=int, default=3,
                    help="voxel marked occupied iff at least N points fall "
                         "in it (default 3). Drops sparse outliers.")
    ap.add_argument("--cluster-min-tri", type=int, default=2000,
                    help="drop disconnected mesh components smaller than "
                         "this (default 2000 triangles).")
    ap.add_argument("--skip-ground-align", action="store_true")
    ap.add_argument("--target-tri", type=int, default=0,
                    help="optional quadric decimation to this triangle "
                         "count (default 0 = no decimation)")
    ap.add_argument("--voxel-cluster-filter", action="store_true",
                    help="3D connected-components filter on the occupancy "
                         "grid (post-voxelisation). Drops floating noise + "
                         "blobs that are disconnected from the main scene. "
                         "Equivalent to DBSCAN on points but ~1000× faster.")
    ap.add_argument("--cluster-keep-pct", type=float, default=0.001,
                    help="connected-component filter keeps clusters whose "
                         "voxel count ≥ pct of largest (default 0.001 = 0.1%%). "
                         "Lower = keeps tiny objects (bike racks); higher = "
                         "keeps only main scene.")
    ap.add_argument("--cluster-min-voxels", type=int, default=50,
                    help="hard floor on cluster size in voxels (default 50). "
                         "Always-drop tiny components even if they pass "
                         "keep-pct threshold.")
    args = ap.parse_args()

    in_path = Path(args.input)
    out_path = Path(args.output)
    if not in_path.exists():
        print(f"ERROR: {in_path} not found", file=sys.stderr)
        sys.exit(1)

    t0 = time.time()
    pts = load_pcd(in_path)

    vlog("STEP 1/4  CPU voxelisation (numpy bincount)")
    occ_cpu, origin = cpu_bin_occupancy(pts, args.voxel, args.min_pts_per_voxel)
    if args.voxel_cluster_filter:
        vlog("STEP 1.5  3D connected-components filter")
        occ_cpu = voxel_connected_cluster_filter(
            occ_cpu, args.cluster_keep_pct, args.cluster_min_voxels,
        )
    vlog("STEP 2/4  CPU smoothing (scipy gaussian_filter)")
    density = smooth_occupancy(occ_cpu, args.sigma)
    del occ_cpu
    vlog("STEP 3/4  marching cubes")
    mesh = marching_cubes_cpu(density, args.voxel, origin, args.iso)
    del density

    vlog("STEP 4/4  post-processing")
    if not args.skip_ground_align:
        mesh = align_ground(mesh)
    mesh = cluster_cull(mesh, args.cluster_min_tri)
    if args.target_tri > 0 and len(mesh.triangles) > args.target_tri:
        vlog(f"  quadric decimation → {args.target_tri:,} tris...")
        mesh = mesh.simplify_quadric_decimation(args.target_tri)
        mesh.remove_unreferenced_vertices()

    out_path.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_triangle_mesh(
        str(out_path), mesh,
        write_triangle_uvs=False, write_vertex_normals=False,
    )
    sz = out_path.stat().st_size / 1e6
    elapsed = time.time() - t0
    vlog(f"✓ wrote {out_path}  ({sz:.1f} MB)  in {elapsed:.1f}s")
    vlog(f"  final: {len(mesh.vertices):,} verts  {len(mesh.triangles):,} tris")


if __name__ == "__main__":
    main()
