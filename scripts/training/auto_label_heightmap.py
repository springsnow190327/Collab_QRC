#!/usr/bin/env python3
"""auto_label_heightmap.py — geometric rule-based auto-labelling.

Replaces the manual labeler GUI when the user trusts geometric heuristics:
for each mesh-touched cell of the heightmap, compute (slope, step,
height_above_floor) on a small neighbourhood, then decide:

    LETHAL (1) if any of:
        slope >= slope_lethal_deg
        step  >= step_lethal_m
        height_above_floor >= height_lethal_m

    TRAVERSABLE (0) if all of:
        slope <  slope_trav_deg
        step  <  step_trav_m
        height_above_floor <  height_trav_m

    UNLABELLED otherwise (ambiguous; let original CNN handle)

This mirrors `synth_terrain_dataset.py`'s rule but applied to a REAL
heightmap, so the CNN learns to recognise the noise patterns of our
LiDAR + Fast-LIO + mesh reconstruction pipeline (which synth doesn't
capture). Optional PolyFit overlay forces wall cells lethal.

Output: labels.npz in the same format as trav_labeler.py (so it plugs
straight into build_trav_dataset.py).
"""
import argparse
import re
from pathlib import Path

import numpy as np
from scipy.ndimage import generic_filter, binary_dilation


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--heights", required=True,
                    help="float heightmap .npz from build_float_heightmap")
    ap.add_argument("--out", required=True, help="output labels .npz")
    # Lethal thresholds (any one triggers lethal)
    ap.add_argument("--slope-lethal-deg", type=float, default=35.0)
    ap.add_argument("--step-lethal-m",   type=float, default=0.20)
    ap.add_argument("--height-lethal-m", type=float, default=0.40)
    # Traversable thresholds (all must hold)
    ap.add_argument("--slope-trav-deg", type=float, default=15.0)
    ap.add_argument("--step-trav-m",   type=float, default=0.06)
    ap.add_argument("--height-trav-m", type=float, default=0.10)
    # Floor estimation: bottom-percentile of touched cells in a local window
    ap.add_argument("--floor-window-m", type=float, default=2.0,
                    help="local floor = min over this neighbourhood (m)")
    ap.add_argument("--floor-pct", type=float, default=5.0,
                    help="percentile used for local floor")
    # Cap to balance / sample
    ap.add_argument("--max-per-class", type=int, default=0,
                    help="random subsample each class to this many cells "
                         "(0 = keep all)")
    ap.add_argument("--polyfit", default="",
                    help="(optional) PolyFit walls XML — force wall cells "
                         "lethal regardless of geometry")
    ap.add_argument("--polyfit-radius-m", type=float, default=0.20,
                    help="cells within this XY radius of any wall plane "
                         "footprint become lethal")
    args = ap.parse_args()

    d = np.load(args.heights, allow_pickle=False)
    heights = d["heights"].astype(np.float32)
    touched = d["touched"].astype(bool)
    cell = float(d["cell_size_m"])
    origin = d["origin_xy"].astype(np.float32)
    H, W = heights.shape

    # 1. Local floor estimate (low percentile over a 2m window).
    floor_radius_cells = max(1, int(round(args.floor_window_m / cell)))
    print(f"local floor: {floor_radius_cells}-cell radius, "
          f"pct={args.floor_pct}")

    def pct_floor(window):
        # window is flat 1D from generic_filter
        return np.percentile(window, args.floor_pct)
    # use only touched cells for floor estimation by replacing untouched
    # with a sentinel = +inf and dropping in the filter via a mask later
    heights_for_floor = np.where(touched, heights, np.float32(1e6))
    # scipy generic_filter is CPU and slow — use a coarse stride approach:
    # downsample by floor_radius, take percentile per coarse block, upsample
    stride = floor_radius_cells
    coarse_H = (H + stride - 1) // stride
    coarse_W = (W + stride - 1) // stride
    coarse_floor = np.zeros((coarse_H, coarse_W), dtype=np.float32)
    for cy in range(coarse_H):
        for cx in range(coarse_W):
            y0 = cy * stride; y1 = min(H, y0 + 2 * stride)
            x0 = cx * stride; x1 = min(W, x0 + 2 * stride)
            block = heights_for_floor[y0:y1, x0:x1]
            mtouch = touched[y0:y1, x0:x1]
            if mtouch.any():
                coarse_floor[cy, cx] = np.percentile(block[mtouch], args.floor_pct)
            else:
                coarse_floor[cy, cx] = 0.0
    # Upsample by nearest-neighbour block expansion
    floor_map = np.kron(coarse_floor, np.ones((stride, stride), dtype=np.float32))[:H, :W]
    print(f"  floor map: min={floor_map.min():.2f} max={floor_map.max():.2f} "
          f"median={np.median(floor_map):.2f} m")

    height_above_floor = heights - floor_map

    # 2. Slope: |grad| in degrees
    gy = np.zeros_like(heights)
    gx = np.zeros_like(heights)
    gy[1:-1] = (heights[2:] - heights[:-2]) / (2 * cell)
    gx[:, 1:-1] = (heights[:, 2:] - heights[:, :-2]) / (2 * cell)
    grad_mag = np.sqrt(gx * gx + gy * gy)
    slope_deg = np.degrees(np.arctan(grad_mag))

    # 3. Step = max abs diff in 3x3 neighbourhood
    # use a max-of-abs-of-diffs trick
    H_pad = np.pad(heights, 1, mode="edge")
    diffs = []
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            shifted = H_pad[1 + dy:1 + dy + H, 1 + dx:1 + dx + W]
            diffs.append(np.abs(heights - shifted))
    step_m = np.max(np.stack(diffs, axis=0), axis=0)

    # 4. PolyFit wall mask (optional override)
    wall_mask = np.zeros((H, W), dtype=bool)
    if args.polyfit:
        wall_mask = build_polyfit_mask(
            args.polyfit, origin, cell, H, W, args.polyfit_radius_m)
        print(f"polyfit wall mask: {int(wall_mask.sum()):,} cells")

    # 5. Apply rules
    lethal_geom = (
        (slope_deg >= args.slope_lethal_deg) |
        (step_m >= args.step_lethal_m) |
        (height_above_floor >= args.height_lethal_m)
    )
    trav_geom = (
        (slope_deg <  args.slope_trav_deg) &
        (step_m   <  args.step_trav_m) &
        (height_above_floor < args.height_trav_m)
    )
    # Force untouched cells to unlabelled
    lethal_geom &= touched
    trav_geom   &= touched

    # PolyFit walls override → always lethal
    if wall_mask.any():
        lethal_geom |= wall_mask & touched

    # Where lethal AND trav both trigger (shouldn't happen with the gap
    # in thresholds, but defensively) → lethal wins
    trav_geom &= ~lethal_geom

    labels = np.full((H, W), -1, dtype=np.int8)
    labels[trav_geom] = 0   # FREE
    labels[lethal_geom] = 1 # LETHAL

    n_lethal = int(lethal_geom.sum())
    n_trav   = int(trav_geom.sum())
    n_unk    = int(((labels == -1) & touched).sum())
    print(f"\nlabel distribution:")
    print(f"  LETHAL (1):  {n_lethal:,}  ({100*n_lethal/touched.sum():.1f}% of touched)")
    print(f"  FREE   (0):  {n_trav:,}    ({100*n_trav/touched.sum():.1f}%)")
    print(f"  UNLABEL(-1): {n_unk:,}    ({100*n_unk/touched.sum():.1f}%)")

    # Optional class balance
    if args.max_per_class > 0:
        rng = np.random.default_rng(42)
        for cls in (0, 1):
            idx_y, idx_x = np.where(labels == cls)
            if len(idx_y) > args.max_per_class:
                keep = rng.choice(len(idx_y), args.max_per_class, replace=False)
                drop_y = np.setdiff1d(np.arange(len(idx_y)),
                                       keep, assume_unique=True)
                labels[idx_y[drop_y], idx_x[drop_y]] = -1
        n_lethal = int((labels == 1).sum())
        n_trav   = int((labels == 0).sum())
        print(f"\nafter class cap ({args.max_per_class:,} per class):")
        print(f"  LETHAL: {n_lethal:,}   FREE: {n_trav:,}")

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.out,
        labels=labels,
        heights=heights,
        touched=touched,
        cell_size_m=np.float32(cell),
        origin_xy=origin,
        slope_deg=slope_deg.astype(np.float32),
        step_m=step_m.astype(np.float32),
        height_above_floor=height_above_floor.astype(np.float32),
    )
    print(f"\n✓ wrote {args.out}")
    print(f"\nNext:")
    print(f"  python scripts/training/build_trav_dataset.py \\")
    print(f"      --labels {args.out} \\")
    print(f"      --out training_runs/ops2_auto.npz --augment")


def build_polyfit_mask(polyfit_xml, origin, cell, H, W, radius_m):
    """Rasterise each PolyFit wall (box footprint) into a 2D mask."""
    txt = Path(polyfit_xml).read_text()
    mask = np.zeros((H, W), dtype=bool)
    # parse pos + size + quat
    for m in re.finditer(
        r'pos="([-\d. ]+)"\s+size="([-\d. ]+)"\s+quat="([-\d. ]+)"', txt
    ):
        pos = np.array(list(map(float, m.group(1).split())))
        sz  = np.array(list(map(float, m.group(2).split())))
        quat = np.array(list(map(float, m.group(3).split())))  # wxyz
        # rough footprint = AABB at z ≈ pos[2] with extent 2*sz xy
        # we only need an XY mask; assume axis-aligned bbox after rotation
        # using extent = max(sz[0], sz[1]) for both axes (over-cover slightly)
        ext = max(float(sz[0]), float(sz[1])) + radius_m
        bmin_x = pos[0] - ext; bmax_x = pos[0] + ext
        bmin_y = pos[1] - ext; bmax_y = pos[1] + ext
        ix0 = int(np.floor((bmin_x - origin[0]) / cell)); ix1 = int(np.ceil((bmax_x - origin[0]) / cell))
        iy0 = int(np.floor((bmin_y - origin[1]) / cell)); iy1 = int(np.ceil((bmax_y - origin[1]) / cell))
        ix0 = max(0, ix0); iy0 = max(0, iy0)
        ix1 = min(W, ix1); iy1 = min(H, iy1)
        if ix0 < ix1 and iy0 < iy1:
            mask[iy0:iy1, ix0:ix1] = True
    return mask


if __name__ == "__main__":
    main()
