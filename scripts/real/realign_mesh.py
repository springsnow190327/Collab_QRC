#!/usr/bin/env python3
"""Iterative RANSAC ground alignment for a mesh.

Loops: pick tight floor band → RANSAC → rotate so plane normal = +Z →
shift so plane d = 0. Repeats until residual tilt < `--tilt-deg` or
`--max-iter`. Single-pass RANSAC inside pcd_to_mesh_cuda misses by 3-5°
because the bottom-5% band includes ramp / dropdown verts that drag the
fit; tighter bands per iteration converge cleanly.
"""
import argparse, sys
from pathlib import Path
import numpy as np
import open3d as o3d


def fit_floor(vs: np.ndarray, bottom_pct: float,
              z_band_center: float | None = None,
              z_band_half: float = 0.10,
              dist_thr: float = 0.02):
    """Fit a plane to either bottom_pct of verts (first pass) OR a tight
    z-band around a known floor estimate (refinement pass)."""
    if z_band_center is not None:
        mask = np.abs(vs[:, 2] - z_band_center) <= z_band_half
        band = vs[mask]
    else:
        thr = np.quantile(vs[:, 2], bottom_pct)
        band = vs[vs[:, 2] <= thr]
    if len(band) < 200:
        return None
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(band)
    plane, inl = pcd.segment_plane(
        distance_threshold=dist_thr, ransac_n=3, num_iterations=2000,
    )
    a, b, c, d = plane
    normal = np.array([a, b, c]); normal /= np.linalg.norm(normal)
    if normal[2] < 0:
        normal = -normal; d = -d
    return normal, d, inl, band


def rotate_to_z(vs: np.ndarray, normal: np.ndarray):
    z = np.array([0.0, 0.0, 1.0])
    cos_t = float(np.clip(normal @ z, -1, 1))
    angle = float(np.arccos(cos_t))
    if angle < 1e-5:
        return vs, np.eye(3), 0.0
    axis = np.cross(normal, z); axis /= np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    R = np.eye(3) + np.sin(angle) * K + (1 - cos_t) * (K @ K)
    return vs @ R.T, R, np.degrees(angle)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help=".obj / .ply")
    ap.add_argument("output", help=".obj / .ply")
    ap.add_argument("--bottom-pct", type=float, default=0.02,
                    help="band of bottom verts used as floor seed (default 0.02)")
    ap.add_argument("--max-iter", type=int, default=5)
    ap.add_argument("--tilt-deg", type=float, default=0.10,
                    help="converge when residual tilt < this (deg)")
    args = ap.parse_args()

    in_path = Path(args.input)
    out_path = Path(args.output)
    if not in_path.exists():
        print(f"ERROR: {in_path} not found"); sys.exit(1)

    mesh = o3d.io.read_triangle_mesh(str(in_path))
    vs = np.asarray(mesh.vertices).copy()
    print(f"loaded {len(vs):,} verts, bbox z=[{vs[:,2].min():.3f}, {vs[:,2].max():.3f}]")

    R_total = np.eye(3)
    z_shift_total = 0.0
    locked_floor_z = None  # set after iter 0, used to track the SAME floor level
    for it in range(args.max_iter):
        if it == 0:
            fit = fit_floor(vs, args.bottom_pct)
        else:
            # Refinement: re-fit using the SAME floor band (don't jump between levels).
            fit = fit_floor(vs, args.bottom_pct,
                            z_band_center=0.0, z_band_half=0.10, dist_thr=0.015)
        if fit is None:
            print(f"  iter {it}: too few floor verts; stop"); break
        normal, d, inl, band = fit
        tilt = np.degrees(np.arccos(np.clip(normal[2], -1, 1)))
        plane_z = -d / normal[2] if abs(normal[2]) > 1e-6 else 0.0
        print(f"  iter {it}: tilt={tilt:.3f}°  floor_z={plane_z:+.3f}  "
              f"band={len(band):,}  inliers={len(inl)}")
        if tilt < args.tilt_deg and abs(plane_z) < 0.01:
            print(f"  converged"); break
        vs, R, _ = rotate_to_z(vs, normal)
        R_total = R @ R_total
        # Shift so the fitted plane passes through z=0 IN THE NEW FRAME.
        # The plane d (after rotation) is -plane_z; shifting by -plane_z aligns
        # the floor to z=0. Using median of inliers is fragile when inliers
        # straddle two levels.
        # After rotation, the plane normal becomes (0, 0, 1) and the plane
        # equation in new frame is z = plane_z (orig). So we shift by -plane_z.
        # But we already mutated vs by rotation; the plane in new frame is
        # at z = plane_z if rotation was perfect. Use inlier median as fallback.
        med_z = float(np.median(vs[inl, 2]))
        vs[:, 2] -= med_z
        z_shift_total += -med_z

    print(f"\nfinal: total_rotation matrix = (concatenated)")
    print(f"       total z_shift = {z_shift_total:+.3f}")
    mesh.vertices = o3d.utility.Vector3dVector(vs)
    mesh.compute_vertex_normals()

    out_path.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_triangle_mesh(str(out_path), mesh,
                               write_triangle_uvs=False,
                               write_vertex_normals=False)
    # Final report
    fit2 = fit_floor(vs, args.bottom_pct)
    if fit2:
        nrm, d, inl, _ = fit2
        tilt = np.degrees(np.arccos(np.clip(nrm[2], -1, 1)))
        plane_z = -d / nrm[2] if abs(nrm[2]) > 1e-6 else 0.0
        print(f"final tilt={tilt:.3f}°  floor_z={plane_z:+.3f}")
    print(f"✓ wrote {out_path} ({out_path.stat().st_size/1e6:.1f} MB)")


if __name__ == "__main__":
    main()
