"""
3D frontier extraction on a sparse occupancy voxel grid.

Algorithm:
  1. Border margin: zero out frontier voxels within `border_margin_cells`
     of the grid edge (they own outside-grid unknowns → inflated volume).
  2. Frontier mask = FREE ∩ dilate6(UNKNOWN)
  3. 26-connectivity CCL on the mask
  4. Pre-filter: drop clusters with fewer than `min_frontier_voxels` voxels
     (single stray points near a wall face get huge Voronoi volumes otherwise).
  5. Voronoi partition: each UNKNOWN voxel -> nearest frontier cluster.
     Two modes:
       geodesic=False (default, fast): Euclidean via distance_transform_edt.
         Propagates through occupied voxels → wall-crossing artifact for
         narrow-corridor scenes.
       geodesic=True (accurate, ~4× slower): scikit-image watershed on the
         ~(unknown | frontier_mask) region, seeded by cluster labels.
         Propagates only through free/unknown voxels → no wall crossing.
  6. Per-cluster unknown volume = vs³ * count of unknowns owned
  7. Per-cluster frontier surface area = vs² * count of UNKNOWN-facing faces
     (discrete digital-topology formula, Klette & Rosenfeld 2004)
  8. Volume filter: keep clusters with unknown_volume_m3 > threshold

References:
  - Cieslewski et al. 2017 (ICRA) — voxel-CCL for 3D frontier extraction
  - Dai et al. 2020 (IROS) — Voronoi-volume formula on TSDF
  - Klette & Rosenfeld 2004 — digital surface-area formula

Pure-function module: no ROS dependencies. CFPA2 imports this for the 3D
candidate path; a standalone test node lives in
frontier_3d_test_node.py for visual validation.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy import ndimage


@dataclass
class Frontier3DCluster:
    """One 3D frontier cluster after volume filtering."""
    id: int                              # 1-based label from CCL
    centroid_world: tuple[float, float, float]
    unknown_volume_m3: float             # IG contribution
    frontier_voxel_count: int            # cluster size
    frontier_area_m2: float              # discrete surface area
    aabb_voxel: tuple[tuple[int, int, int], tuple[int, int, int]]  # (min, max) in voxel coords


def extract_3d_frontiers(
    voxel_data: np.ndarray,
    voxel_size_m: float,
    origin_xyz: tuple[float, float, float],
    min_unknown_volume_m3: float = 1.0,
    min_frontier_voxels: int = 50,
    border_margin_cells: int = 3,
    geodesic_voronoi: bool = False,
    free_value: int = 0,
    unknown_value: int = -1,
    z_band_min_m: float = -0.2,
    z_band_max_m: float = 1.5,
    robot_xy: Optional[tuple[float, float]] = None,
) -> list[Frontier3DCluster]:
    """
    Args:
        voxel_data: int8 ndarray, shape (nz, ny, nx), values in {unknown=-1, free=0, occ=100}.
            Row-major order matching nvblox_frontend_msgs/VoxelGrid3D::data.
        voxel_size_m: edge length of a voxel in metres.
        origin_xyz: world-frame coords of the corner of voxel (0,0,0).
        min_unknown_volume_m3: cluster passes filter iff its owned-unknown volume
            exceeds this (default 1.0 m³, see interpretation B in nvblox_3d_frontier.md).
        min_frontier_voxels: drop clusters with fewer than this many frontier voxels
            before the Voronoi step. Tiny clusters (N<50) at wall corners have huge
            Euclidean-Voronoi attribution. Default 50 ≈ 0.05 m³ at vs=0.10 m.
        border_margin_cells: zero frontier voxels within this many cells of the grid
            boundary. Edge voxels "own" outside-grid unknowns → artificially large volume.
        geodesic_voronoi: if True, use scikit-image watershed (propagates only through
            free/unknown voxels, no wall-crossing). ~4× slower but eliminates wall
            attribution artifacts in narrow corridors. Requires skimage.

    Returns:
        List of Frontier3DCluster, sorted descending by unknown_volume_m3.
    """
    if voxel_data.ndim != 3:
        raise ValueError(f"voxel_data must be 3D, got shape {voxel_data.shape}")

    free    = (voxel_data == free_value)
    unknown = (voxel_data == unknown_value)

    # ─── Z-band filter on UNKNOWN voxels ──────────────────────────────
    # Without this, every air voxel above the ramp/platform / under the
    # ceiling stays UNK forever (robot at ground can't probe air at
    # z=2+), forming a giant persistent cluster that the planner keeps
    # chasing. Restricting "valid UNK" to a band the robot can actually
    # reach makes cluster volumes shrink monotonically as exploration
    # progresses, restoring the "frontier consumed by motion" property
    # that 2D frontiers have natively.
    #
    # Band defaults [-0.2, 1.5] m (configurable): covers from just
    # below floor up to platform top + a small clearance. Air above
    # gets ignored — it's not reachable nor actionable for a wheeled
    # ground robot.
    nz = voxel_data.shape[0]
    oz = origin_xyz[2]
    z_world = oz + (np.arange(nz, dtype=np.float32) + 0.5) * voxel_size_m
    z_in_band = (z_world >= z_band_min_m) & (z_world <= z_band_max_m)
    unknown = unknown & z_in_band[:, None, None]

    # Bail fast if there's nothing to find.
    if not unknown.any() or not free.any():
        return []

    # 1. Frontier mask: FREE voxels touching UNKNOWN through a 6-face neighbour.
    struct6 = ndimage.generate_binary_structure(3, 1)  # 6-connectivity
    frontier_mask = free & ndimage.binary_dilation(unknown, structure=struct6)

    # Apply border margin: nullify frontier voxels at grid edges.
    # These voxels border outside-grid space (implicitly unknown) so Voronoi
    # assigns them all out-of-bound "unknown" → wildly inflated volumes.
    if border_margin_cells > 0:
        bm = border_margin_cells
        nz, ny, nx = frontier_mask.shape
        border = np.zeros_like(frontier_mask, dtype=bool)
        border[:bm, :, :] = True; border[nz - bm:, :, :] = True
        border[:, :bm, :] = True; border[:, ny - bm:, :] = True
        border[:, :, :bm] = True; border[:, :, nx - bm:] = True
        frontier_mask = frontier_mask & ~border

    if not frontier_mask.any():
        return []

    # 2. CCL with 26-connectivity — captures oblique frontier surfaces.
    struct26 = np.ones((3, 3, 3), dtype=np.uint8)
    labels, n_clusters = ndimage.label(frontier_mask, structure=struct26)
    if n_clusters == 0:
        return []

    # 3. Pre-filter tiny clusters before Voronoi.
    # A cluster with N=1 (single stray voxel touching a wall corner) gets
    # Euclidean-nearest ownership of all unknown behind that wall → V can be
    # hundreds of m³ from N=1. Drop these before Voronoi so they don't pollute
    # the owner map. Build a remap: old label → new label (0 = dropped).
    sizes = np.bincount(labels.ravel(), minlength=n_clusters + 1)
    # sizes[0] = background count; sizes[1..n_clusters] = cluster sizes
    keep = np.zeros(n_clusters + 1, dtype=np.int32)  # maps old → new label
    new_id = 0
    kept_old_ids: list[int] = []
    for cid in range(1, n_clusters + 1):
        if sizes[cid] >= min_frontier_voxels:
            new_id += 1
            keep[cid] = new_id
            kept_old_ids.append(cid)
    n_kept = new_id
    if n_kept == 0:
        return []
    labels = keep[labels]  # remap; background + dropped clusters → 0

    # 4. Voronoi assignment: every voxel learns its nearest frontier cluster.
    if geodesic_voronoi:
        # Watershed propagates only through walkable voxels (free + unknown).
        # Seeds: frontier cluster labels; mask: free | unknown (don't cross walls).
        try:
            from skimage.segmentation import watershed as skwatershed
        except ImportError as exc:
            raise ImportError(
                "geodesic_voronoi=True requires scikit-image: "
                "pip install scikit-image"
            ) from exc
        # Watershed in skimage: markers=labels (int), mask=bool, connectivity=26.
        # It assigns each voxel inside mask the label of the nearest seed
        # as measured by shortest path through mask.
        walkable = free | unknown
        owner = skwatershed(
            np.zeros(voxel_data.shape, dtype=np.float32),
            markers=labels,
            mask=walkable,
            connectivity=2,  # 26-connectivity in 3D
        ).astype(np.int32)
    else:
        # Euclidean Voronoi via distance_transform_edt (fast, ~10× cheaper).
        # Crosses walls: fine for open scenes, over-attributes in narrow rooms.
        seeds = labels  # already remapped; 0 = non-frontier
        _, nearest_idx = ndimage.distance_transform_edt(
            seeds == 0, return_indices=True)
        # nearest_idx shape: (3, nz, ny, nx) — z, y, x indices of nearest seed.
        owner = labels[nearest_idx[0], nearest_idx[1], nearest_idx[2]]

    # 5. Sum unknown voxels per cluster (1..n_kept; bin 0 is non-unknown).
    owner_unk = np.where(unknown, owner, 0)
    vol_voxels = np.bincount(owner_unk.ravel(), minlength=n_kept + 1)
    vol_m3     = vol_voxels.astype(np.float64) * (voxel_size_m ** 3)

    # 6. Frontier surface area per cluster (vectorised digital-topology count).
    # For each (z,y,x), how many of its 6 face-neighbours are UNKNOWN.
    face_count = np.zeros(voxel_data.shape, dtype=np.int8)
    u8 = unknown.astype(np.int8)
    face_count[1:]   += u8[:-1]    # neighbour at z-1 is unknown
    face_count[:-1]  += u8[1:]     # neighbour at z+1
    face_count[:, 1:]  += u8[:, :-1]
    face_count[:, :-1] += u8[:, 1:]
    face_count[:, :, 1:]  += u8[:, :, :-1]
    face_count[:, :, :-1] += u8[:, :, 1:]
    frontier_now = (labels > 0)  # post-filter frontier mask
    face_count *= frontier_now.astype(np.int8)
    faces_per_c = ndimage.sum(
        face_count, labels=labels, index=np.arange(1, n_kept + 1)
    )
    area_per_c = np.asarray(faces_per_c, dtype=np.float64) * (voxel_size_m ** 2)

    # 7. Build per-cluster output with geometry, then volume-filter.
    # centroid_world: when robot_xy is provided, pick the frontier voxel
    # FURTHEST from the robot rather than the geometric mean. Geometric
    # mean is broken for ring-shaped frontier shells around the robot —
    # the mean lands at the centre of the ring (i.e. ON the robot), so
    # navigating "to centroid" = staying put = no exploration progress.
    # Farthest-voxel always picks a real boundary point in the direction
    # of largest unobserved volume, which is what we actually want to
    # navigate toward.
    ox, oy, oz = origin_xyz
    out: list[Frontier3DCluster] = []
    for new_cid in range(1, n_kept + 1):
        v = float(vol_m3[new_cid])
        if v < min_unknown_volume_m3:
            continue
        mask = (labels == new_cid)
        zs, ys, xs = np.where(mask)
        if len(xs) == 0:
            continue
        if robot_xy is not None:
            # Convert voxel coords to world for distance ranking.
            xs_w = ox + (xs.astype(np.float32) + 0.5) * voxel_size_m
            ys_w = oy + (ys.astype(np.float32) + 0.5) * voxel_size_m
            rx, ry = robot_xy
            d2 = (xs_w - rx) ** 2 + (ys_w - ry) ** 2
            far_idx = int(np.argmax(d2))
            cx = float(xs_w[far_idx])
            cy = float(ys_w[far_idx])
            cz = float(oz + (zs[far_idx] + 0.5) * voxel_size_m)
        else:
            cx = ox + (xs.mean() + 0.5) * voxel_size_m
            cy = oy + (ys.mean() + 0.5) * voxel_size_m
            cz = oz + (zs.mean() + 0.5) * voxel_size_m
        out.append(Frontier3DCluster(
            id=new_cid,
            centroid_world=(float(cx), float(cy), float(cz)),
            unknown_volume_m3=v,
            frontier_voxel_count=int(len(xs)),
            frontier_area_m2=float(area_per_c[new_cid - 1]),
            aabb_voxel=(
                (int(xs.min()), int(ys.min()), int(zs.min())),
                (int(xs.max()), int(ys.max()), int(zs.max())),
            ),
        ))
    out.sort(key=lambda c: c.unknown_volume_m3, reverse=True)
    return out


def project_to_traversability_goal(
    centroid_xyz: tuple[float, float, float],
    trav_grid: np.ndarray,
    trav_resolution_m: float,
    trav_origin_xy: tuple[float, float],
    search_radius_m: float = 2.0,
    free_value: int = 0,
) -> Optional[tuple[float, float]]:
    """
    Project a 3D cluster centroid (x, y, z) down to the closest reachable
    (x, y) FREE cell in the 2.5D traversability grid.

    Args:
        centroid_xyz: from extract_3d_frontiers().
        trav_grid: int8 ndarray, shape (H, W); -1 unknown, 0 free, 100 occ.
        trav_resolution_m: cell size in metres.
        trav_origin_xy: world coords of trav_grid[0,0] cell corner.
        search_radius_m: radius around centroid xy to search.

    Returns:
        (gx, gy) world coords of the chosen FREE cell, or None if no FREE
        cell exists within search_radius.
    """
    cx, cy, _ = centroid_xyz
    ox, oy = trav_origin_xy
    H, W = trav_grid.shape
    ci = int(round((cx - ox) / trav_resolution_m))
    cj = int(round((cy - oy) / trav_resolution_m))
    r = max(1, int(round(search_radius_m / trav_resolution_m)))

    # Bounded search window.
    i0, i1 = max(0, ci - r), min(W, ci + r + 1)
    j0, j1 = max(0, cj - r), min(H, cj + r + 1)
    if i0 >= i1 or j0 >= j1:
        return None
    window = trav_grid[j0:j1, i0:i1]
    free_mask = (window == free_value)
    if not free_mask.any():
        return None

    # Take the FREE cell closest to (ci, cj).
    js, is_ = np.where(free_mask)
    di = (is_ + i0) - ci
    dj = (js + j0) - cj
    best = int(np.argmin(di * di + dj * dj))
    gi = is_[best] + i0
    gj = js[best] + j0
    gx = ox + (gi + 0.5) * trav_resolution_m
    gy = oy + (gj + 0.5) * trav_resolution_m
    return (float(gx), float(gy))
