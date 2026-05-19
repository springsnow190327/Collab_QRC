# Traversability CNN training pipeline

End-to-end pipeline for building the training corpus and fine-tuning the
120-parameter traversability CNN that drives `elevation_mapping_cupy`'s
`traversability` layer (consumed by Nav2 + CFPA2 via
`trav_cost_filters/grid_map_to_occupancy_grid`).

The 2026-05-15 ops2 fine-tune (commit `2dd8664`) shipped a CNN that hits
val_mse 0.04 / val_acc 0.95 on the ops2 mesh but is **not robust to noise
across different scenes** — when the elevation map sees a noise distribution
the model wasn't trained on (different LiDAR variance, different walking
pitch, different scene geometry), it produces speckled-lethal artifacts and
mis-classifies bridges / ramps.

This pipeline solves that by giving the CNN broader noise coverage during
training. It has two tiers:

| tier | source | purpose | volume |
|---|---|---|---|
| **pretrain** | MJCF mesh raycast → clean heightmap, then injected sensor + SLAM noise | broad coverage, characterizable noise sweeps | 1-10M+ patches offline in <1 minute |
| **fine-tune** | live `/elevation_map_raw` GridMap captured during 2D bench runs | match the actual joint distribution of `Mid-360 + Risley sampling + Fast-LIO drift + walking pitch + grid_map_filters` | ~100k patches per benchmark batch |

Both tiers produce `.npz` files with identical schema, directly consumable
by `scripts/training/train_trav_filter.py`.

---

## Why this exists

The `elevation_mapping_cupy` traversability CNN sees noisy input at runtime:

1. **Mid-360 LiDAR range noise** (σ ≈ 2 cm, range-dependent)
2. **Risley scan-pattern sparsity** — non-uniform sampling, some cells get
   0-2 hits per frame
3. **Walking-pitch oscillation** — Go2/Go2W body rolls ±5-10° at ~1.5 Hz
   while sensing, biasing the elevation map
4. **Fast-LIO pose drift** — 1-2 cm trans, 0.5° rot per frame (worse during
   turns)
5. **Cross-frame Kalman fusion artifacts** — variance grows / shrinks
   irregularly

The CNN's failure mode under noise it hasn't seen during training looks
like: bridges flipping to lethal, ramps marked free-then-lethal-then-free
as the robot walks, speckled false-lethals on flat ground. This pipeline
gives the model:

- The **clean ground truth** of what every cell SHOULD be classified as
  (from MuJoCo's collision system — no hand-labeled noise).
- Many examples of each cell **with characterized noise applied** so the
  model learns to denoise robustly.

---

## Component overview

### 1. `mujoco_static_label_map.py` — one-shot GT label per scene

For each (x, y) cell in a regular grid over the scene's XY bounds, raycasts
down and classifies:

- `FREE` (0) — floor hit (z < `floor_threshold`)
- `LETHAL` (1) — wall / obstacle hit (z ≥ `floor_threshold`)
- `INFLATED` (2) — FREE cell within `footprint_radius` of a LETHAL cell
  (i.e. robot center here would have body overlap with a wall)
- `UNKNOWN` (-1) — no hit (outside scene)

Output: `<mjcf_stem>_gtlabel.npy` (int8) + `_gtlabel.json` (metadata).

```bash
python3 scripts/training/mujoco_static_label_map.py \
    src/go2w/go2_gazebo_sim/mujoco/demo3_mixed.xml \
    --resolution 0.10 \
    --footprint-radius 0.22
```

Sub-second per scene. The 0.10 m resolution matches
`elevation_mapping_cupy`'s default cell size.

### 2. `mujoco_clean_heightmap.py` — clean reference heightmap

Companion to `mujoco_static_label_map.py`. Same grid + raycast, but stores
the actual hit-z (top surface elevation) per cell instead of a binary
label. Output: `<mjcf_stem>_heightmap.npy` (float32) + `_heightmap.json`.

If the corresponding `_gtlabel.json` already exists, the grid origin /
resolution / dims are inherited so `heightmap[iy, ix]` and
`gtlabel[iy, ix]` describe the same world cell.

```bash
python3 scripts/training/mujoco_clean_heightmap.py \
    src/go2w/go2_gazebo_sim/mujoco/demo3_mixed.xml
```

### 3. `synth_noise_corpus.py` — offline pretrain corpus

For each labelled cell, samples K augmented 7×7 patches from the clean
heightmap and applies characterized noise:

| noise source | distribution | knob |
|---|---|---|
| Gaussian range jitter | per-cell N(0, σ), σ ~ U[lo, hi] | `--range-noise-lo/--range-noise-hi` (default 5-40 mm) |
| Walking-pitch tilt | planar dz = x·tan(pitch) + y·tan(roll), pitch/roll ~ U[±θ] | `--tilt-deg-max` (default 8°) |
| Sub-cell pose drift | bilinear sample offset ±N cells | `--pose-drift-cells` (default 1.0 = 10 cm) |
| Risley dropout | random cells → NaN, frac ~ U[0, max] | `--dropout-frac-max` (default 0.25) |
| Geometric symmetry | rot ∈ {0, 90, 180, 270}° × flip-LR | `--include-geom-aug` (default true → 8×) |

```bash
python3 scripts/training/synth_noise_corpus.py \
    src/go2w/go2_gazebo_sim/mujoco \
    --output training_runs/data/pretrain_corpus.npz \
    --cells-per-class 10000 \
    --noise-aug 4
```

Performance: **~36k patches/sec** on a typical laptop. 6 scenes × 10k
cells/class × 3 classes × 4 noise × 8 geom = 5.76M patches in ~2.5 minutes.

### 4. `trav_corpus_collector.py` + `run_with_corpus.sh` — live-sim capture

ROS 2 node that subscribes to `/<ns>/elevation_map_raw` and samples N
patches per GridMap. Each patch's center is looked up in the static
`<scene>_gtlabel.npy` to assign a ground-truth label. On SIGTERM (the bench
launch's session-duration timer), flushes everything to `.npz`.

Use the shell wrapper to side-car the collector around any benchmark
launch — no edits to launch files required:

```bash
CORPUS_SCENE=demo3_mixed \
CORPUS_OUTPUT=/tmp/trav_corpus/demo3_mixed_t001.npz \
  scripts/bench/run_with_corpus.sh \
    ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio.launch.py \
      session_duration_sec:=120 nav_backend:=nav2_mppi gui:=false rviz:=false
```

The wrapper waits `CORPUS_WARMUP_SEC` (default 5 s) so the elevation
pipeline boots, starts the collector, then exec's the launch. On exit it
sends SIGTERM to the collector for a clean flush.

Bench loops accumulate multiple per-trial `.npz` files under
`/tmp/trav_corpus/<scene>/`.

### 5. `merge_trav_corpus.py` — aggregate per-trial captures

Combines every `<dir>/**/*.npz` into one training set. Handles
class-balancing and inflated-cell folding.

```bash
python3 scripts/training/merge_trav_corpus.py /tmp/trav_corpus \
    --output training_runs/data/finetune_corpus.npz \
    --drop-inflated false \
    --balance true
```

---

## Schema

All four output paths produce `.npz` with these keys (consumable by
`train_trav_filter.py` without modification):

| key | dtype | shape | meaning |
|---|---|---|---|
| `patches` | float32 | (N, 7, 7) | elevation values in meters, NaN for unmapped |
| `labels` | float32 | (N,) | 1.0 if FREE, 0.0 if LETHAL or INFLATED |
| `classes` | int8 | (N,) | 0=FREE, 1=LETHAL, 2=INFLATED (raw class preserved) |
| `cell_size_m` | float32 | scalar | 0.10 m |
| `scenes` | str array | (N,) | scene MJCF stem (for traceability — optional) |

Live-sim `.npz` from `trav_corpus_collector.py` additionally carry
`world_xy` (N, 2) and `t` (N,) for per-row provenance.

---

## End-to-end training recipe

```bash
# ── 1. One-time per scene: label map + clean heightmap ──
for scene in demo3_mixed two_rooms_door_scene lrc_maze_go2w \
             vlm_exploration_scene demo2 demo_ramp; do
  python3 scripts/training/mujoco_static_label_map.py \
    "src/go2w/go2_gazebo_sim/mujoco/${scene}.xml"
  python3 scripts/training/mujoco_clean_heightmap.py \
    "src/go2w/go2_gazebo_sim/mujoco/${scene}.xml"
done

# ── 2. Pretrain corpus (offline; clean + injected noise) ──
python3 scripts/training/synth_noise_corpus.py \
  src/go2w/go2_gazebo_sim/mujoco \
  --output training_runs/data/pretrain_corpus.npz \
  --cells-per-class 10000 --noise-aug 4

# ── 3. Train pretrain weights ──
python3 scripts/training/train_trav_filter.py \
  training_runs/data/pretrain_corpus.npz \
  training_runs/weights_pretrain_v2.dat \
  --epochs 150 --lr 2e-4 \
  --lethal-weight 3.0 --label-smoothing 0.05

# ── 4. Fine-tune corpus (live capture during bench runs) ──
mkdir -p /tmp/trav_corpus
for trial in $(seq 1 20); do
  for scene in demo3_mixed lrc_maze_go2w vlm_exploration_scene; do
    CORPUS_SCENE="${scene}" \
    CORPUS_OUTPUT="/tmp/trav_corpus/${scene}_t$(printf %03d ${trial}).npz" \
      scripts/bench/run_with_corpus.sh \
        ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio.launch.py \
          mujoco_model_path:="src/go2w/go2_gazebo_sim/mujoco/${scene}.xml" \
          session_duration_sec:=120 nav_backend:=nav2_mppi gui:=false rviz:=false
  done
done

# ── 5. Merge fine-tune captures ──
python3 scripts/training/merge_trav_corpus.py /tmp/trav_corpus \
  --output training_runs/data/finetune_corpus.npz \
  --drop-inflated false

# ── 6. Fine-tune from pretrain ──
python3 scripts/training/train_trav_filter.py \
  training_runs/data/finetune_corpus.npz \
  training_runs/weights_corpus_v2.dat \
  --init-from training_runs/weights_pretrain_v2.dat \
  --epochs 100 --lr 1e-4 \
  --pretrain-mix-ratio 0.30 \
  --pretrain-dataset training_runs/data/pretrain_corpus.npz

# ── 7. Deploy ──
# scripts/launch/nav_test_3d_explore.sh auto-loads weights_ops2_tiled.dat by
# default. Set ELEVATION_MAPPING_WEIGHTS=... to point at the new file:
ELEVATION_MAPPING_WEIGHTS=training_runs/weights_corpus_v2.dat \
  ./scripts/launch/nav_test_3d_explore.sh
```

---

## Known limitations + caveats

**Static GT label is purely contact-based.** Dynamic stability (the robot
tips at a ramp foot, the 2026-05-15 open problem at
`docs/claude/ramp_tipover_open_problem.md`) is invisible to a static
collision query. For ramp scenes (`demo_ramp`, `lrc_maze_go2w`), every
elevated cell shows up as LETHAL — conservative but not always right.
**Dynamic-stability annotation is a separate pass** (drive a sim trajectory
and label by outcome) not yet implemented.

**`elevation_mapping_cupy` traversability layer is robot-centric in
config** but published in world frame (`map`). The collector's world-coord
lookup against the static label map is therefore correct, but be aware
that if `pose_topic` updates lag the published GridMap, the lookup can be
off by a few cells during sharp turns. Increase
`CORPUS_WARMUP_SEC` if you see this in the captured rows.

**Noise model in `synth_noise_corpus.py` is independent per cell.** Real
LiDAR errors are spatially correlated (a beam that's off by 1 cm tends to
shift neighbours similarly). The pretrain corpus is intentionally
pessimistic — independent noise per cell is harder to denoise than
correlated noise, so a model trained on it should over-generalize, not
under-generalize. For best-quality fine-tune, lean on the live-sim
collector data.

**Don't commit the generated `.npy` artifacts.** They regenerate in <1 s
per scene from the MJCF + scripts. Re-run step 1 of the recipe after pull.

---

## Adding a new scene

1. Add `<scene>.xml` to `src/go2w/go2_gazebo_sim/mujoco/`.
2. Run `mujoco_static_label_map.py` + `mujoco_clean_heightmap.py` on it.
3. Verify the label map matches expectations:
   ```python
   import numpy as np, json
   lab = np.load("src/go2w/go2_gazebo_sim/mujoco/<scene>_gtlabel.npy")
   meta = json.loads(open("...gtlabel.json").read())
   print(meta["counts"])  # should have realistic free/lethal counts
   # spot-check a known-free coord:
   ox, oy = meta["origin_xy"]; res = meta["resolution_m"]
   wx, wy = 0.0, 0.0  # known floor location
   ix = int((wx - ox) / res); iy = int((wy - oy) / res)
   assert lab[iy, ix] == 0, f"expected FREE at (0,0), got {lab[iy,ix]}"
   ```
4. Re-run `synth_noise_corpus.py` (it auto-discovers new pairs).
5. Bench against the new scene to collect fine-tune rows.

---

## File layout

```
scripts/
├── training/
│   ├── mujoco_static_label_map.py    # ← step 1: GT label per cell
│   ├── mujoco_clean_heightmap.py     # ← step 1: clean reference z(x,y)
│   ├── synth_noise_corpus.py         # ← step 2: pretrain corpus generator
│   ├── merge_trav_corpus.py          # ← step 5: aggregate fine-tune .npz
│   ├── train_trav_filter.py          # (already exists — CNN training loop)
│   ├── build_trav_dataset.py         # (already exists — used for ops2 mesh)
│   └── auto_label_heightmap.py       # (already exists — used for ops2 mesh)
└── bench/
    ├── trav_corpus_collector.py      # ← step 4: live-sim ROS 2 collector node
    ├── run_with_corpus.sh            # ← step 4: wrapper for any benchmark launch
    └── session_reporter.py           # (already exists — bench JSON dumper)

src/go2w/go2_gazebo_sim/mujoco/        # MJCFs + generated artifacts
├── <scene>.xml                        # source MJCF (committed)
├── <scene>_gtlabel.{npy,json}         # generated by mujoco_static_label_map.py
└── <scene>_heightmap.{npy,json}       # generated by mujoco_clean_heightmap.py
```

The `.npy`/`.json` artifacts are **deliberately not committed** — they're
regenerable in <1 s per scene. Re-run `mujoco_static_label_map.py` +
`mujoco_clean_heightmap.py` after pulling.
