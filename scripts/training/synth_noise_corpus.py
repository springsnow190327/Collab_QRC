#!/usr/bin/env python3
"""Generate the pretrain corpus offline from clean heightmaps + GT labels.

For each labelled cell in `<scene>_gtlabel.npy`, extract a 7×7 clean patch
from `<scene>_heightmap.npy`, apply a characterized noise model (LiDAR range
jitter + walking-pitch tilt + Fast-LIO pose drift + Risley dropout +
rot/flip), and emit `(patch, label)` rows.

Noise sources (each configurable, sampled per augmentation):
  1. Gaussian range noise        — add N(0, σ_range) per cell, σ ~ U[σ_lo, σ_hi]
  2. Walking-pitch tilt          — add planar tilt dz = x·tan(pitch) + y·tan(roll)
                                   with pitch, roll ~ U[-tilt_deg, +tilt_deg]
  3. Sub-cell pose drift         — shift the sample window by random sub-cell
                                   offset (bilinear), simulating Fast-LIO trans drift
  4. Risley sparse dropout       — randomly NaN-out `dropout_frac` of cells
                                   (uniform random), simulating Mid-360 misses
  5. Geometric augmentation      — rot ∈ {0, 90, 180, 270}° + optional LR flip
                                   (built-in symmetries; no info loss)

Each labelled cell contributes K augmented samples (K = noise_aug × 8 if
include_geom else noise_aug). Output is one .npz per --output, schema
identical to `merge_trav_corpus.py` (and therefore consumable directly by
`train_trav_filter.py`).

Usage:
    python3 scripts/training/synth_noise_corpus.py \\
        src/go2w/go2_gazebo_sim/mujoco \\
        --output training_runs/data/pretrain_corpus.npz \\
        --noise-aug 4 --include-geom-aug
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

# Label codes — same as mujoco_static_label_map.py.
FREE = 0
LETHAL = 1
INFLATED = 2
UNKNOWN = -1


def _str_bool(s: str) -> bool:
    return s.strip().lower() in ("1", "true", "yes", "on")


def sample_clean_patch(
    heightmap: np.ndarray,
    iy: int,
    ix: int,
    half: int,
    sub_cell_offset: tuple[float, float] = (0.0, 0.0),
) -> np.ndarray | None:
    """Bilinear-sample a (2*half+1)×(2*half+1) patch centered at (iy+dy, ix+dx).

    If the requested window falls outside the heightmap, return None.
    NaN cells in heightmap stay NaN in the patch.
    """
    dy, dx = sub_cell_offset
    H, W = heightmap.shape
    cy = iy + dy
    cx = ix + dx
    y0 = cy - half
    x0 = cx - half
    y1 = cy + half
    x1 = cx + half
    # Conservative bounds: need 1-cell margin for bilinear.
    if y0 < 0 or x0 < 0 or y1 > H - 1 or x1 > W - 1:
        return None

    iy_lo = int(math.floor(y0))
    ix_lo = int(math.floor(x0))
    ay = y0 - iy_lo
    ax = x0 - ix_lo
    side = 2 * half + 1

    out = np.empty((side, side), dtype=np.float32)
    for r in range(side):
        yy = iy_lo + r
        for c in range(side):
            xx = ix_lo + c
            # 4-corner bilinear; NaN propagates if any corner is NaN.
            z00 = heightmap[yy,     xx]
            z10 = heightmap[yy + 1, xx] if yy + 1 < H else z00
            z01 = heightmap[yy,     xx + 1] if xx + 1 < W else z00
            z11 = heightmap[yy + 1, xx + 1] if yy + 1 < H and xx + 1 < W else z00
            out[r, c] = (
                (1 - ay) * (1 - ax) * z00 +
                ay * (1 - ax) * z10 +
                (1 - ay) * ax * z01 +
                ay * ax * z11
            )
    return out


def apply_tilt(patch: np.ndarray, resolution_m: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    """Add a linear plane: dz[r, c] = x(c)·tan(pitch) + y(r)·tan(roll).

    x increases with column, y with row. Patch center is the rotation origin
    (no z-bias at the center cell).
    """
    side = patch.shape[0]
    half = side // 2
    yy, xx = np.mgrid[0:side, 0:side].astype(np.float32)
    x_m = (xx - half) * resolution_m
    y_m = (yy - half) * resolution_m
    tilt = x_m * math.tan(math.radians(pitch_deg)) + y_m * math.tan(math.radians(roll_deg))
    return (patch + tilt).astype(np.float32)


def apply_dropout(patch: np.ndarray, rng: np.random.Generator, frac: float) -> np.ndarray:
    """Set `frac` of cells to NaN uniformly at random."""
    if frac <= 0.0:
        return patch
    side = patch.shape[0]
    n = side * side
    n_drop = int(round(frac * n))
    if n_drop <= 0:
        return patch
    idx = rng.choice(n, n_drop, replace=False)
    out = patch.copy()
    out.flat[idx] = np.nan
    return out


def apply_geom_aug(patch: np.ndarray, k_rot: int, flip_lr: bool) -> np.ndarray:
    """Apply k_rot×90° rotation then optional LR flip. Preserves labels exactly."""
    out = np.rot90(patch, k=k_rot)
    if flip_lr:
        out = np.fliplr(out)
    return np.ascontiguousarray(out)


def find_scene_pairs(scene_dir: Path) -> list[tuple[Path, Path, Path]]:
    """Find (mjcf_stem, heightmap.npy, gtlabel.npy) triplets."""
    out = []
    for hm in sorted(scene_dir.glob("*_heightmap.npy")):
        stem = hm.name.removesuffix("_heightmap.npy")
        lab = scene_dir / f"{stem}_gtlabel.npy"
        if lab.exists():
            out.append((Path(stem), hm, lab))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scene_dir", type=Path,
                    help="Directory of <scene>_heightmap.npy + <scene>_gtlabel.npy pairs")
    ap.add_argument("--output", type=Path, required=True, help="Output .npz path")
    ap.add_argument("--patch-size", type=int, default=7, help="Patch side (must be odd; default 7)")
    ap.add_argument("--noise-aug", type=int, default=4,
                    help="Noise-augmented samples per labelled cell (default 4)")
    ap.add_argument("--include-geom-aug", type=_str_bool, default=True,
                    help="Also rot×4 + flip-LR each noisy patch (default true → 8× multiplier)")
    ap.add_argument("--max-cells-per-scene", type=int, default=0,
                    help="If >0, subsample to at most this many labelled cells per scene")
    ap.add_argument("--cells-per-class", type=int, default=0,
                    help="If >0, take exactly this many of each class (FREE/LETHAL/INFLATED) "
                         "per scene; overrides --max-cells-per-scene")
    ap.add_argument("--drop-inflated", type=_str_bool, default=False,
                    help="If true, skip INFLATED cells (class 2); default false → fold to lethal")
    # Noise knobs (ranges sampled uniformly per augmentation):
    ap.add_argument("--range-noise-lo", type=float, default=0.005,
                    help="Min Gaussian range noise σ in meters (default 5 mm)")
    ap.add_argument("--range-noise-hi", type=float, default=0.04,
                    help="Max σ (default 40 mm)")
    ap.add_argument("--tilt-deg-max", type=float, default=8.0,
                    help="Max walking-pitch tilt in deg (default 8°)")
    ap.add_argument("--pose-drift-cells", type=float, default=1.0,
                    help="Max sub-cell pose drift in cells (default 1.0 = 10 cm at 0.10 m res)")
    ap.add_argument("--dropout-frac-max", type=float, default=0.25,
                    help="Max fraction of cells set to NaN (default 0.25)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    if args.patch_size % 2 == 0:
        sys.exit("patch_size must be odd")
    half = args.patch_size // 2

    rng = np.random.default_rng(args.seed)

    pairs = find_scene_pairs(args.scene_dir)
    if not pairs:
        sys.exit(f"no <scene>_heightmap.npy + _gtlabel.npy pairs in {args.scene_dir}")
    print(f"[synth] found {len(pairs)} scenes", flush=True)

    patches_all: list[np.ndarray] = []
    classes_all: list[int] = []
    scenes_all: list[str] = []

    t0 = time.monotonic()

    for stem, hm_path, lab_path in pairs:
        # Need the label sidecar for resolution.
        lab_json = lab_path.with_suffix(".json")
        if not lab_json.exists():
            print(f"  skip {stem}: missing {lab_json.name}", flush=True)
            continue
        meta = json.loads(lab_json.read_text())
        resolution = float(meta["resolution_m"])

        heightmap = np.load(hm_path).astype(np.float32)
        labels = np.load(lab_path)
        if heightmap.shape != labels.shape:
            print(f"  skip {stem}: shape mismatch hm={heightmap.shape} lab={labels.shape}",
                  flush=True)
            continue

        # Eligible labelled cells (need patch margin in-bounds).
        H, W = labels.shape
        margin = half + 1  # +1 so bilinear with drift can still sample
        # Iterate per-class so --cells-per-class works.
        cells_by_class: dict[int, np.ndarray] = {}
        for cls in (FREE, LETHAL, INFLATED):
            if cls == INFLATED and args.drop_inflated:
                continue
            mask = (labels == cls)
            mask[:margin, :] = False
            mask[-margin:, :] = False
            mask[:, :margin] = False
            mask[:, -margin:] = False
            iy, ix = np.where(mask)
            cells_by_class[cls] = np.stack([iy, ix], axis=1)

        # Down-select cells per scene.
        selected = []
        if args.cells_per_class > 0:
            for cls, arr in cells_by_class.items():
                if len(arr) == 0:
                    continue
                k = min(args.cells_per_class, len(arr))
                idx = rng.choice(len(arr), k, replace=False)
                selected.append((cls, arr[idx]))
        else:
            for cls, arr in cells_by_class.items():
                selected.append((cls, arr))
            if args.max_cells_per_scene > 0:
                total = sum(len(a) for _, a in selected)
                if total > args.max_cells_per_scene:
                    keep = args.max_cells_per_scene / total
                    selected = [(c, a[rng.choice(len(a), int(len(a) * keep), replace=False)])
                                for c, a in selected]

        n_scene_cells = sum(len(a) for _, a in selected)
        n_scene_patches = 0
        for cls, cells in selected:
            for (iy, ix) in cells:
                for _ in range(args.noise_aug):
                    # Sample noise parameters.
                    sigma = rng.uniform(args.range_noise_lo, args.range_noise_hi)
                    pitch = rng.uniform(-args.tilt_deg_max, args.tilt_deg_max)
                    roll  = rng.uniform(-args.tilt_deg_max, args.tilt_deg_max)
                    drift_y = rng.uniform(-args.pose_drift_cells, args.pose_drift_cells)
                    drift_x = rng.uniform(-args.pose_drift_cells, args.pose_drift_cells)
                    drop = rng.uniform(0.0, args.dropout_frac_max)

                    patch = sample_clean_patch(heightmap, int(iy), int(ix), half,
                                                sub_cell_offset=(drift_y, drift_x))
                    if patch is None:
                        continue
                    if not np.all(np.isfinite(patch)):
                        # mesh missed under this cell (shouldn't happen if label is valid)
                        continue
                    patch = patch + rng.normal(0.0, sigma, size=patch.shape).astype(np.float32)
                    patch = apply_tilt(patch, resolution, pitch, roll)
                    patch = apply_dropout(patch, rng, drop)

                    geom_variants = (
                        [(k, f) for k in range(4) for f in (False, True)]
                        if args.include_geom_aug else [(0, False)]
                    )
                    for (k_rot, flip) in geom_variants:
                        out = apply_geom_aug(patch, k_rot, flip)
                        patches_all.append(out)
                        classes_all.append(int(cls))
                        scenes_all.append(str(stem))
                        n_scene_patches += 1

        print(f"  + {stem.name}: cells={n_scene_cells} → patches={n_scene_patches}",
              flush=True)

    patches = np.stack(patches_all, axis=0).astype(np.float32)
    classes = np.array(classes_all, dtype=np.int8)
    labels = (classes == FREE).astype(np.float32)
    scenes_arr = np.array(scenes_all)

    dt = time.monotonic() - t0
    print(f"\n[synth] generated {patches.shape[0]} patches in {dt:.1f}s "
          f"({patches.shape[0]/dt:.0f} patch/s)", flush=True)
    print(f"  FREE={int((classes==0).sum())}  LETHAL={int((classes==1).sum())}  "
          f"INFLATED={int((classes==2).sum())}", flush=True)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.output,
        patches=patches,
        labels=labels,
        classes=classes,
        cell_size_m=np.float32(0.10),
        scenes=scenes_arr,
    )
    print(f"[synth] wrote {args.output}", flush=True)
    print()
    print("Train with:")
    print(f"  python3 scripts/training/train_trav_filter.py {args.output} \\")
    print(f"      training_runs/weights_pretrain_v2.dat \\")
    print(f"      --epochs 150 --lr 2e-4")
    return 0


if __name__ == "__main__":
    sys.exit(main())
