#!/usr/bin/env python3
"""Aggregate per-trial trav_corpus_collector .npz files into one training set.

Reads every <input>/**/*.npz produced by `scripts/bench/trav_corpus_collector.py`,
concatenates the (patches, labels, classes) arrays, applies optional class
re-balancing, and writes a single .npz compatible with
`scripts/training/train_trav_filter.py`.

The collector keeps INFLATED separate from LETHAL via the `classes` key:
  0 = FREE     (label 1.0)
  1 = LETHAL   (label 0.0)
  2 = INFLATED (label 0.0)

By default INFLATED cells are FOLDED INTO LETHAL when training (label=0.0).
That is the safe default for a navigation policy: "robot-center over this
cell results in body overlap with wall" should be lethal in the model.

Output format (matches build_trav_dataset.py / train_trav_filter.py):
    patches    : (N, 7, 7) float32
    labels     : (N,) float32 in {0.0, 1.0}
    classes    : (N,) int8
    cell_size_m: float32
    scenes     : list of scene names (str)  — for traceability

Usage:
    python3 scripts/training/merge_trav_corpus.py \\
        /tmp/trav_corpus \\
        --output training_runs/data/corpus_2026-05-19.npz \\
        --drop-inflated false
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def _str_bool(s: str) -> bool:
    return s.strip().lower() in ("1", "true", "yes", "on")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input_dir", type=Path,
                    help="Directory of per-trial .npz files (recursive glob)")
    ap.add_argument("--output", type=Path, required=True,
                    help="Output merged .npz path")
    ap.add_argument("--drop-inflated", type=_str_bool, default=False,
                    help="If true, discard INFLATED (class=2) rows; "
                         "if false (default) fold them into lethal (label=0)")
    ap.add_argument("--balance", type=_str_bool, default=False,
                    help="If true, subsample majority class so #free ≈ #lethal")
    ap.add_argument("--max-rows", type=int, default=0,
                    help="If >0, randomly subsample to at most this many rows")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)

    files = sorted(args.input_dir.rglob("*.npz"))
    if not files:
        sys.exit(f"no .npz files under {args.input_dir}")
    print(f"[merge] found {len(files)} trial files under {args.input_dir}", flush=True)

    patches_all: list[np.ndarray] = []
    labels_all: list[np.ndarray] = []
    classes_all: list[np.ndarray] = []
    scenes_all: list[str] = []

    n_free = n_lethal = n_inflated = 0
    for f in files:
        try:
            d = np.load(f, allow_pickle=True)
        except Exception as e:
            print(f"  skip {f.name}: {e}", flush=True)
            continue
        if "patches" not in d.files or "classes" not in d.files:
            print(f"  skip {f.name}: missing required keys", flush=True)
            continue
        p = d["patches"].astype(np.float32)
        c = d["classes"].astype(np.int8)
        if p.shape[0] != c.shape[0]:
            print(f"  skip {f.name}: patches/classes length mismatch", flush=True)
            continue
        scene = str(d["scene"]) if "scene" in d.files else f.stem

        if args.drop_inflated:
            keep = (c != 2)
            p = p[keep]
            c = c[keep]
        # label: 1.0 if FREE (class 0), else 0.0
        lab = (c == 0).astype(np.float32)

        patches_all.append(p)
        labels_all.append(lab)
        classes_all.append(c)
        scenes_all.extend([scene] * len(c))
        n_free += int((c == 0).sum())
        n_lethal += int((c == 1).sum())
        n_inflated += int((c == 2).sum())
        print(f"  + {f.name}  rows={len(c)}  scene={scene}  "
              f"(free={int((c==0).sum())} lethal={int((c==1).sum())} infl={int((c==2).sum())})",
              flush=True)

    if not patches_all:
        sys.exit("no usable rows after filtering")

    patches = np.concatenate(patches_all, axis=0)
    labels = np.concatenate(labels_all, axis=0)
    classes = np.concatenate(classes_all, axis=0)
    scenes_arr = np.array(scenes_all)

    n = len(patches)
    print(f"\n[merge] before transforms: {n} rows  "
          f"FREE={n_free} LETHAL={n_lethal} INFLATED={n_inflated}", flush=True)

    if args.balance:
        # Subsample majority so #free ≈ #lethal (lethal includes inflated unless dropped).
        free_idx = np.where(labels == 1.0)[0]
        leth_idx = np.where(labels == 0.0)[0]
        target = min(len(free_idx), len(leth_idx))
        if target == 0:
            sys.exit("balance requested but one class is empty")
        free_keep = rng.choice(free_idx, target, replace=False)
        leth_keep = rng.choice(leth_idx, target, replace=False)
        keep = np.concatenate([free_keep, leth_keep])
        rng.shuffle(keep)
        patches = patches[keep]
        labels = labels[keep]
        classes = classes[keep]
        scenes_arr = scenes_arr[keep]
        print(f"[merge] balanced to {len(keep)} rows ({target} per class)", flush=True)

    if args.max_rows > 0 and len(patches) > args.max_rows:
        idx = rng.choice(len(patches), args.max_rows, replace=False)
        patches = patches[idx]
        labels = labels[idx]
        classes = classes[idx]
        scenes_arr = scenes_arr[idx]
        print(f"[merge] subsampled to {len(patches)} rows (max_rows={args.max_rows})",
              flush=True)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.output,
        patches=patches,
        labels=labels,
        classes=classes,
        cell_size_m=np.float32(0.10),
        scenes=scenes_arr,
    )
    print(f"\n[merge] wrote {args.output}", flush=True)
    print(f"  total rows: {len(patches)}", flush=True)
    print(f"  free={int((classes==0).sum())} "
          f"lethal={int((classes==1).sum())} "
          f"inflated={int((classes==2).sum())}", flush=True)
    print(f"  patches shape: {patches.shape}  dtype: {patches.dtype}", flush=True)
    print()
    print("Train with:")
    print(f"  python3 scripts/training/train_trav_filter.py {args.output} \\")
    print(f"      training_runs/weights_corpus.dat \\")
    print(f"      --init-from training_runs/weights_pretrain.dat \\")
    print(f"      --epochs 200 --lr 1e-4")
    return 0


if __name__ == "__main__":
    sys.exit(main())
