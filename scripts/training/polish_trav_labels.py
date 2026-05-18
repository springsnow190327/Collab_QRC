#!/usr/bin/env python3
"""polish_trav_labels.py — clean up auto-labelled traversability map.

Two targeted fixes on top of auto_label_heightmap output:

1. SPECKLE REMOVAL — morphological opening on the lethal mask removes
   isolated 1-2 cell "lethal islands" caused by mesh-reconstruction
   noise on smooth floors. Use --speckle-radius 0 to disable.

2. BRIDGE OVERRIDE — cells with low slope (flat) but high z
   (overhead structure like a bridge, awning, walkway) get re-labelled
   as FREE because the robot walks BELOW them, not on them. The 2.5D
   heightmap can't represent the floor under a bridge, but if we know
   the cell is flat+elevated we override it. Disable with
   --bridge-height-m 0 (or set to a very high value).

Diagnostic: emit a PNG showing before/after side-by-side.

Usage:
    python3 polish_trav_labels.py \\
        --in  bags/meshes/ops2_cuda/hfield/ops2_v4_auto_labels.npz \\
        --out bags/meshes/ops2_cuda/hfield/ops2_v4_polished_labels.npz \\
        --speckle-radius 1 \\
        --bridge-height-m 1.2  \\
        --bridge-slope-max-deg 20
"""
import argparse
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def morphological_open(mask: np.ndarray, radius: int) -> np.ndarray:
    """Remove specks <= (2r+1)x(2r+1) by erode then dilate."""
    if radius <= 0:
        return mask.copy()
    from scipy.ndimage import binary_erosion, binary_dilation
    se = np.ones((2 * radius + 1, 2 * radius + 1), dtype=bool)
    return binary_dilation(binary_erosion(mask, structure=se), structure=se)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", required=True)
    ap.add_argument("--out", required=True)
    # Speckle removal
    ap.add_argument("--speckle-radius", type=int, default=1,
                    help="morphological opening radius (cells) on lethal mask. "
                         "0 = disable. 1 removes 1-cell islands; 2 removes "
                         "up to 3x3 (might erode thin walls)")
    ap.add_argument("--min-lethal-component", type=int, default=4,
                    help="drop connected lethal blobs smaller than this. "
                         "0 = disable")
    # Bridge override
    ap.add_argument("--bridge-height-m", type=float, default=1.2,
                    help="cells with height_above_floor >= this AND slope "
                         "below --bridge-slope-max-deg → forced FREE. "
                         "0 = disable")
    ap.add_argument("--bridge-slope-max-deg", type=float, default=20.0)
    ap.add_argument("--bridge-step-max-m", type=float, default=0.20,
                    help="bridges should also have low local step "
                         "(rule out steep awnings)")
    # Diagnostic
    ap.add_argument("--diag-png", default="",
                    help="(optional) save before/after PNG to this path")
    args = ap.parse_args()

    d = np.load(args.inp, allow_pickle=False)
    heights = d["heights"]
    touched = d["touched"]
    labels0 = d["labels"].astype(np.int8)
    cell = float(d["cell_size_m"])
    origin = d["origin_xy"].astype(np.float32)

    # Pull the cached features if present (auto_label_heightmap saves them).
    has_feats = all(k in d.files for k in
                    ("slope_deg", "step_m", "height_above_floor"))
    if has_feats:
        slope = d["slope_deg"]
        step = d["step_m"]
        h_above = d["height_above_floor"]
        print("using cached features")
    else:
        print("WARN: feats not in input npz; bridge override needs them — "
              "re-run auto_label_heightmap on the heightmap first")
        slope = step = h_above = None

    n_let0 = int((labels0 == 1).sum())
    n_tr0  = int((labels0 == 0).sum())
    print(f"input:  lethal={n_let0:,}  free={n_tr0:,}")

    labels = labels0.copy()

    # --- speckle removal ---
    n_specks = 0
    if args.speckle_radius > 0:
        lethal_mask = labels == 1
        opened = morphological_open(lethal_mask, args.speckle_radius)
        # cells that were lethal but didn't survive opening → flip to free
        # (only inside touched region, and only if surrounded mostly by free)
        flipped = lethal_mask & ~opened & touched
        # extra guard: only flip if cell has at least one free neighbour
        # (don't flip lethal cells inside a wall slab)
        from scipy.ndimage import binary_dilation
        free_neighborhood = binary_dilation(labels == 0, iterations=1)
        flipped &= free_neighborhood
        labels[flipped] = 0
        n_specks = int(flipped.sum())

    n_blobs_drop = 0
    if args.min_lethal_component > 0:
        from scipy.ndimage import label as cc_label
        lethal_mask = labels == 1
        cc, n_cc = cc_label(lethal_mask)
        sizes = np.bincount(cc.ravel())
        small_blobs = np.where(sizes < args.min_lethal_component)[0]
        # exclude background (index 0)
        small_blobs = small_blobs[small_blobs != 0]
        if len(small_blobs) > 0:
            mask_small = np.isin(cc, small_blobs)
            # same guard: must be next to a free cell
            from scipy.ndimage import binary_dilation
            free_neighborhood = binary_dilation(labels == 0, iterations=1)
            flip = mask_small & touched & free_neighborhood
            labels[flip] = 0
            n_blobs_drop = int(flip.sum())

    # --- bridge override ---
    n_bridge = 0
    if has_feats and args.bridge_height_m > 0:
        bridge_mask = (
            (h_above >= args.bridge_height_m) &
            (slope <= args.bridge_slope_max_deg) &
            (step  <= args.bridge_step_max_m) &
            touched
        )
        # The whole bridge — including its edges where the auto rule had
        # voted lethal due to height — gets re-labelled FREE.
        labels[bridge_mask] = 0
        n_bridge = int(bridge_mask.sum())

    n_let = int((labels == 1).sum())
    n_tr  = int((labels == 0).sum())
    print(f"output: lethal={n_let:,}  free={n_tr:,}")
    print(f"  speckle flips:    {n_specks:,}")
    print(f"  small-blob drops: {n_blobs_drop:,}")
    print(f"  bridge overrides: {n_bridge:,}")

    # Save
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.out,
        labels=labels,
        heights=heights,
        touched=touched,
        cell_size_m=np.float32(cell),
        origin_xy=origin,
        slope_deg=slope, step_m=step, height_above_floor=h_above,
    )
    print(f"✓ wrote {args.out}")

    # Diagnostic
    if args.diag_png:
        save_diag(labels0, labels, heights, touched, args.diag_png)


def save_diag(lbl0, lbl1, heights, touched, out_path):
    """Side-by-side before/after."""
    fig, axes = plt.subplots(1, 2, figsize=(20, 7))
    disp = heights.copy(); disp[~touched] = np.nan
    for ax, lbl, title in zip(axes, (lbl0, lbl1),
                              ("BEFORE (auto)", "AFTER (polished)")):
        ax.imshow(disp, cmap="gray", origin="lower", alpha=0.6)
        rgba = np.zeros(lbl.shape + (4,), dtype=np.float32)
        rgba[lbl == 1] = [1.0, 0.0, 0.0, 0.7]
        rgba[lbl == 0] = [0.0, 1.0, 0.0, 0.55]
        ax.imshow(rgba, origin="lower")
        n_let = int((lbl == 1).sum()); n_tr = int((lbl == 0).sum())
        ax.set_title(f"{title}\nlethal={n_let:,}  free={n_tr:,}")
        ax.set_xticks([]); ax.set_yticks([])
    plt.tight_layout()
    plt.savefig(out_path, dpi=120, bbox_inches="tight")
    print(f"✓ wrote {out_path}")


if __name__ == "__main__":
    main()
