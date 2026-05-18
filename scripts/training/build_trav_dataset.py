#!/usr/bin/env python3
"""build_trav_dataset.py — labelled heightmap → 7×7 patch training set.

Reads the output of trav_labeler.py (labels.npz which also bundles the
underlying heightmap + cell_size + origin) and produces a CNN-ready
dataset:

    patches : (N, 7, 7) float32 — elevation in meters, NaN-tolerant
    labels  : (N,) float32 ∈ {0.0, 1.0} — 0=lethal, 1=traversable
    classes : (N,) uint8 — 0=free, 1=lethal (for per-class metrics)
    cells   : (N, 2) int32 — (iy, ix) for traceability

Notes:
  - CNN was trained with "1 = traversable" convention (exp(-|.|) ∈ (0,1]
    where 1 means free). We flip labeler's {LETHAL=1, FREE=0} to that.
  - We re-center each patch on its own min during training; here we save
    raw heights so the same dataset works with any center-then-train
    strategy.
  - Patches near borders / on untouched regions are filtered: at least
    60% of the 7×7 must come from mesh-touched cells.

Usage:
    python3 build_trav_dataset.py \\
        --labels bags/meshes/ops2_cuda/hfield/ops2_v4_trav_labels.npz \\
        --out training_runs/ops2_real.npz \\
        --patch-size 7 --min-touched-frac 0.6
"""
import argparse
from pathlib import Path

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--labels", required=True,
                    help="npz from trav_labeler.py (carries heights+labels)")
    ap.add_argument("--out", required=True, help="output .npz")
    ap.add_argument("--patch-size", type=int, default=7)
    ap.add_argument("--min-touched-frac", type=float, default=0.6,
                    help="reject patches where < this fraction of cells "
                         "are mesh-touched (would be all-zero noise)")
    ap.add_argument("--augment", action="store_true",
                    help="add 4-rotation augmentation (x4 dataset size)")
    # Sensor-noise augmentations — match what runtime LiDAR delivers
    ap.add_argument("--flip-lr", action="store_true",
                    help="add left-right flip (x2)")
    ap.add_argument("--flip-ud", action="store_true",
                    help="add up-down flip (x2)")
    ap.add_argument("--noise-rounds", type=int, default=0,
                    help="N copies with Gaussian height noise (default 0)")
    ap.add_argument("--noise-std", type=float, default=0.03,
                    help="Gaussian noise std in meters (default 0.03)")
    ap.add_argument("--dropout-rounds", type=int, default=0,
                    help="N copies with random cell dropout (default 0)")
    ap.add_argument("--dropout-rate", type=float, default=0.10,
                    help="dropout probability per cell (default 0.10)")
    ap.add_argument("--tilt-rounds", type=int, default=0,
                    help="N copies with random tilt (default 0)")
    ap.add_argument("--tilt-max-deg", type=float, default=5.0,
                    help="max tilt magnitude in degrees (default 5)")
    ap.add_argument("--blur-rounds", type=int, default=0,
                    help="N copies with Gaussian blur (default 0)")
    ap.add_argument("--blur-sigma-cells", type=float, default=1.0,
                    help="Gaussian blur std in cells (default 1)")
    args = ap.parse_args()

    d = np.load(args.labels, allow_pickle=False)
    heights = d["heights"]
    touched = d["touched"]
    labels = d["labels"]  # int8: -1 unknown, 0 free, 1 lethal
    cell = float(d["cell_size_m"])
    origin = d["origin_xy"]
    H, W = heights.shape
    ps = args.patch_size
    half = ps // 2

    n_lethal = int((labels == 1).sum())
    n_free   = int((labels == 0).sum())
    print(f"loaded labels: lethal={n_lethal:,}  free={n_free:,}  "
          f"unlabelled={int((labels==-1).sum()):,}")
    if n_lethal + n_free == 0:
        print("no labelled cells — paint some in trav_labeler first")
        return

    # Iterate labelled cells; extract patches.
    iy, ix = np.where(labels >= 0)
    print(f"  candidate cells: {len(iy):,}")
    # In-bounds for the patch window
    in_bounds = (iy >= half) & (iy < H - half) & (ix >= half) & (ix < W - half)
    iy = iy[in_bounds]; ix = ix[in_bounds]
    print(f"  in-bounds after patch border: {len(iy):,}")

    patches = []
    pts_lbl = []
    pts_cls = []
    pts_cells = []
    min_touched = ps * ps * args.min_touched_frac
    skipped_sparse = 0

    for yy, xx in zip(iy, ix):
        win_t = touched[yy - half:yy + half + 1, xx - half:xx + half + 1]
        if win_t.sum() < min_touched:
            skipped_sparse += 1
            continue
        win = heights[yy - half:yy + half + 1, xx - half:xx + half + 1].copy()
        # Replace untouched cells with the min touched value in the window
        # (so they don't pretend to be tall obstacles). Train script also
        # subtracts per-patch min so this is safe.
        if (~win_t).any():
            fill = float(win[win_t].min()) if win_t.any() else 0.0
            win[~win_t] = fill
        patches.append(win.astype(np.float32))
        cls = int(labels[yy, xx])  # 0=free 1=lethal
        # Flip to CNN convention: target=1 means TRAVERSABLE
        target = 0.0 if cls == 1 else 1.0
        pts_lbl.append(target)
        pts_cls.append(cls)
        pts_cells.append((yy, xx))

    if not patches:
        print("no valid patches after filtering — relax --min-touched-frac")
        return

    patches = np.stack(patches)              # (N, 7, 7)
    pts_lbl = np.array(pts_lbl, dtype=np.float32)
    pts_cls = np.array(pts_cls, dtype=np.uint8)
    pts_cells = np.array(pts_cells, dtype=np.int32)

    rng = np.random.default_rng(42)
    cell_size_m = float(cell)

    aug_log = []
    if args.augment:
        rot_p = [patches]
        for k in (1, 2, 3):
            rot_p.append(np.rot90(patches, k=k, axes=(1, 2)).copy())
        patches = np.concatenate(rot_p, axis=0)
        pts_lbl = np.tile(pts_lbl, 4)
        pts_cls = np.tile(pts_cls, 4)
        pts_cells = np.tile(pts_cells, (4, 1))
        aug_log.append(f"rotate ×4 → {len(patches):,}")

    if args.flip_lr:
        flipped = patches[:, :, ::-1].copy()
        patches = np.concatenate([patches, flipped], axis=0)
        pts_lbl = np.tile(pts_lbl, 2)
        pts_cls = np.tile(pts_cls, 2)
        pts_cells = np.tile(pts_cells, (2, 1))
        aug_log.append(f"flip-LR → {len(patches):,}")

    if args.flip_ud:
        flipped = patches[:, ::-1, :].copy()
        patches = np.concatenate([patches, flipped], axis=0)
        pts_lbl = np.tile(pts_lbl, 2)
        pts_cls = np.tile(pts_cls, 2)
        pts_cells = np.tile(pts_cells, (2, 1))
        aug_log.append(f"flip-UD → {len(patches):,}")

    base_p, base_l, base_c, base_x = patches, pts_lbl, pts_cls, pts_cells

    # Noise rounds
    for r in range(args.noise_rounds):
        noisy = base_p + rng.normal(
            0, args.noise_std, base_p.shape).astype(np.float32)
        patches = np.concatenate([patches, noisy], axis=0)
        pts_lbl = np.concatenate([pts_lbl, base_l])
        pts_cls = np.concatenate([pts_cls, base_c])
        pts_cells = np.concatenate([pts_cells, base_x], axis=0)
    if args.noise_rounds > 0:
        aug_log.append(
            f"+gaussian (σ={args.noise_std}m) ×{args.noise_rounds} → {len(patches):,}")

    # Dropout rounds (replace dropped cells with per-patch min)
    for r in range(args.dropout_rounds):
        mask = rng.uniform(0, 1, base_p.shape) < args.dropout_rate
        per_min = base_p.reshape(len(base_p), -1).min(axis=1)[:, None, None]
        dropped = np.where(mask, per_min, base_p).astype(np.float32)
        patches = np.concatenate([patches, dropped], axis=0)
        pts_lbl = np.concatenate([pts_lbl, base_l])
        pts_cls = np.concatenate([pts_cls, base_c])
        pts_cells = np.concatenate([pts_cells, base_x], axis=0)
    if args.dropout_rounds > 0:
        aug_log.append(
            f"+dropout ({args.dropout_rate}) ×{args.dropout_rounds} → {len(patches):,}")

    # Tilt rounds: add a planar slope to the patch
    for r in range(args.tilt_rounds):
        # random pitch + roll
        tilt_deg = rng.uniform(-args.tilt_max_deg, args.tilt_max_deg,
                                (len(base_p), 2)).astype(np.float32)
        # planar offset in meters per cell at each (y, x)
        ps = base_p.shape[-1]
        yy, xx = np.mgrid[0:ps, 0:ps].astype(np.float32) * cell_size_m
        yy = yy - yy.mean()
        xx = xx - xx.mean()
        slope_y = np.tan(np.deg2rad(tilt_deg[:, 0]))[:, None, None]
        slope_x = np.tan(np.deg2rad(tilt_deg[:, 1]))[:, None, None]
        delta = slope_y * yy[None] + slope_x * xx[None]
        tilted = (base_p + delta).astype(np.float32)
        patches = np.concatenate([patches, tilted], axis=0)
        pts_lbl = np.concatenate([pts_lbl, base_l])
        pts_cls = np.concatenate([pts_cls, base_c])
        pts_cells = np.concatenate([pts_cells, base_x], axis=0)
    if args.tilt_rounds > 0:
        aug_log.append(
            f"+tilt (±{args.tilt_max_deg}°) ×{args.tilt_rounds} → {len(patches):,}")

    # Blur rounds (mild gaussian smooth — simulates sensor / fusion smoothing)
    if args.blur_rounds > 0:
        from scipy.ndimage import gaussian_filter
        for r in range(args.blur_rounds):
            blurred = np.stack(
                [gaussian_filter(p, sigma=args.blur_sigma_cells)
                 for p in base_p]).astype(np.float32)
            patches = np.concatenate([patches, blurred], axis=0)
            pts_lbl = np.concatenate([pts_lbl, base_l])
            pts_cls = np.concatenate([pts_cls, base_c])
            pts_cells = np.concatenate([pts_cells, base_x], axis=0)
        aug_log.append(
            f"+blur (σ={args.blur_sigma_cells}c) ×{args.blur_rounds} → {len(patches):,}")

    if aug_log:
        print("augmentations:")
        for line in aug_log:
            print(f"  {line}")

    print(f"  built {len(patches):,} patches  "
          f"(skipped {skipped_sparse:,} for sparsity)")
    print(f"  label distribution: "
          f"trav={int((pts_lbl == 1.0).sum()):,}  "
          f"lethal={int((pts_lbl == 0.0).sum()):,}")
    print(f"  per-class height stats:")
    for cls in (0, 1):
        m = pts_cls == cls
        if m.any():
            sub = patches[m]
            print(f"    cls {cls}: n={int(m.sum()):,}  z=[{sub.min():.2f},{sub.max():.2f}]m  "
                  f"std={sub.std():.2f}")

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.out,
        patches=patches,
        labels=pts_lbl,
        classes=pts_cls,
        cells=pts_cells,
        cell_size_m=np.float32(cell),
        origin_xy=origin,
    )
    print(f"✓ wrote {args.out} ({patches.shape[0]:,} patches)")
    print()
    print("Train with:")
    print(f"  python scripts/training/train_trav_filter.py {args.out} \\")
    print(f"      training_runs/weights_ops2.dat \\")
    print(f"      --init-from training_runs/weights_pretrain.dat \\")
    print(f"      --epochs 200 --lr 1e-4")
    print()
    print("Then in sim:")
    print(f"  TRAV_WEIGHTS=$PWD/training_runs/weights_ops2.dat \\")
    print(f"      ./scripts/launch/nav_test_slam_ops2_v4_go2.sh")


if __name__ == "__main__":
    main()
