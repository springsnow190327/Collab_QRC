#!/usr/bin/env python3
"""Extract the clean ground-truth heightmap of a MuJoCo scene.

Companion to `mujoco_static_label_map.py`: same grid origin + resolution,
but stores the actual hit-z (top surface elevation) per cell instead of a
binary label. This is the noise-free reference that `synth_noise_corpus.py`
applies characterized noise to (LiDAR range jitter, walking-pitch tilt,
Fast-LIO pose drift, Risley dropout) to synthesise the pretrain corpus.

Per cell raycast straight down from z=10:
  - hit on floor / ground plane → store hit_z (typically 0.0 or +ε)
  - hit on wall / obstacle      → store hit_z (e.g. 1.0 m for demo3_mixed walls)
  - no hit                      → store NaN

If a `<scene>_gtlabel.json` sidecar exists alongside the MJCF, this script
inherits its grid origin / resolution / dims so the heightmap and label
arrays index identically: `heightmap[iy, ix]` and `gtlabel[iy, ix]` describe
the same world cell. Otherwise the grid is auto-detected the same way the
label script does.

Output:
    <mjcf_stem>_heightmap.npy   — (H, W) float32, NaN where no hit
    <mjcf_stem>_heightmap.json  — sidecar with origin/resolution/dims +
                                  a `paired_label` key pointing at the .npy
                                  for traceability

Usage:
    python3 scripts/training/mujoco_clean_heightmap.py \\
        src/go2w/go2_gazebo_sim/mujoco/demo3_mixed.xml
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

try:
    import mujoco
except ImportError as e:
    sys.exit(f"mujoco python package missing: {e}")


def scene_xy_bounds(model: mujoco.MjModel) -> tuple[float, float, float, float]:
    """Same logic as mujoco_static_label_map.py — bound by non-plane worldbody geoms."""
    xmin, ymin = math.inf, math.inf
    xmax, ymax = -math.inf, -math.inf
    found_any = False
    for gi in range(model.ngeom):
        if model.geom_bodyid[gi] != 0:
            continue
        if int(model.geom_type[gi]) == int(mujoco.mjtGeom.mjGEOM_PLANE):
            continue
        pos = model.geom_pos[gi]
        size = model.geom_size[gi]
        ex = float(size[0]) if size[0] > 0 else 0.5
        ey = float(size[1]) if size[1] > 0 else 0.5
        xmin = min(xmin, float(pos[0]) - ex)
        xmax = max(xmax, float(pos[0]) + ex)
        ymin = min(ymin, float(pos[1]) - ey)
        ymax = max(ymax, float(pos[1]) + ey)
        found_any = True
    if not found_any:
        raise RuntimeError("No worldbody non-plane geoms found")
    return xmin, ymin, xmax, ymax


def park_robot_far_away(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    park_pos = np.array([1000.0, 1000.0, 1000.0], dtype=np.float64)
    for ji in range(model.njnt):
        if int(model.jnt_type[ji]) == int(mujoco.mjtJoint.mjJNT_FREE):
            qadr = int(model.jnt_qposadr[ji])
            data.qpos[qadr:qadr + 3] = park_pos
            data.qpos[qadr + 3:qadr + 7] = np.array([1.0, 0.0, 0.0, 0.0])
            park_pos = park_pos + np.array([100.0, 0.0, 0.0])
    mujoco.mj_forward(model, data)


def build_heightmap(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    xmin: float, ymin: float, xmax: float, ymax: float,
    resolution: float,
    raycast_height_m: float = 10.0,
) -> tuple[np.ndarray, dict]:
    W = int(math.ceil((xmax - xmin) / resolution))
    H = int(math.ceil((ymax - ymin) / resolution))

    heights = np.full((H, W), np.nan, dtype=np.float32)

    pnt = np.zeros(3, dtype=np.float64)
    vec = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    geomid_buf = np.zeros(1, dtype=np.int32)

    t0 = time.monotonic()
    n_rays = 0
    n_hit = 0

    for iy in range(H):
        y = ymin + iy * resolution
        for ix in range(W):
            x = xmin + ix * resolution
            pnt[0] = x
            pnt[1] = y
            pnt[2] = raycast_height_m
            dist = mujoco.mj_ray(
                model, data, pnt, vec, None, 1, -1, geomid_buf,
            )
            n_rays += 1
            if dist < 0 or geomid_buf[0] < 0:
                continue
            heights[iy, ix] = float(raycast_height_m - dist)
            n_hit += 1

    dt = time.monotonic() - t0
    meta = dict(
        n_rays=n_rays, n_hit=n_hit, n_miss=n_rays - n_hit,
        rays_per_sec=int(n_rays / dt) if dt > 0 else 0,
        elapsed_sec=round(dt, 3),
        z_min=float(np.nanmin(heights)) if n_hit > 0 else None,
        z_max=float(np.nanmax(heights)) if n_hit > 0 else None,
        z_p05=float(np.nanpercentile(heights, 5)) if n_hit > 0 else None,
        z_p95=float(np.nanpercentile(heights, 95)) if n_hit > 0 else None,
    )
    return heights, meta


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mjcf", type=Path)
    ap.add_argument("--resolution", type=float, default=None,
                    help="Grid resolution (default: inherit from label sidecar, else 0.10)")
    ap.add_argument("--pad", type=float, default=0.5,
                    help="XY padding around scene bounds (default 0.5 m)")
    ap.add_argument("--output", type=Path, default=None,
                    help="Output .npy path (default <mjcf_stem>_heightmap.npy)")
    ap.add_argument("--ignore-label-sidecar", action="store_true",
                    help="Don't inherit grid from existing _gtlabel.json")
    args = ap.parse_args()

    if not args.mjcf.exists():
        sys.exit(f"MJCF not found: {args.mjcf}")

    print(f"[clean_hm] loading {args.mjcf}", flush=True)
    model = mujoco.MjModel.from_xml_path(str(args.mjcf))
    data = mujoco.MjData(model)
    park_robot_far_away(model, data)

    label_sidecar = args.mjcf.with_name(args.mjcf.stem + "_gtlabel.json")
    inherited_from = None
    if label_sidecar.exists() and not args.ignore_label_sidecar:
        meta = json.loads(label_sidecar.read_text())
        resolution = float(meta["resolution_m"])
        xmin, ymin = meta["origin_xy"]
        W = int(meta["width"])
        H = int(meta["height"])
        xmax = xmin + W * resolution
        ymax = ymin + H * resolution
        inherited_from = str(label_sidecar)
        print(f"[clean_hm] inherited grid from {label_sidecar.name}: "
              f"{W}×{H} @ {resolution}m", flush=True)
    else:
        xmin, ymin, xmax, ymax = scene_xy_bounds(model)
        xmin -= args.pad
        ymin -= args.pad
        xmax += args.pad
        ymax += args.pad
        resolution = args.resolution if args.resolution is not None else 0.10
        print(f"[clean_hm] auto bounds: x=[{xmin:.2f},{xmax:.2f}] "
              f"y=[{ymin:.2f},{ymax:.2f}] @ {resolution}m", flush=True)

    heights, meta = build_heightmap(model, data, xmin, ymin, xmax, ymax, resolution)
    print(f"[clean_hm] {heights.shape[1]}×{heights.shape[0]} cells  "
          f"{meta['n_rays']} rays in {meta['elapsed_sec']}s "
          f"({meta['rays_per_sec']} ray/s)", flush=True)
    print(f"[clean_hm] hits={meta['n_hit']}  misses={meta['n_miss']}", flush=True)
    if meta["z_min"] is not None:
        print(f"[clean_hm] z range: [{meta['z_min']:.3f}, {meta['z_max']:.3f}]m  "
              f"p5={meta['z_p05']:.3f}  p95={meta['z_p95']:.3f}", flush=True)

    out_path = args.output or args.mjcf.with_name(args.mjcf.stem + "_heightmap.npy")
    json_path = out_path.with_suffix(".json")
    np.save(out_path, heights)

    sidecar = dict(
        scene=str(args.mjcf.name),
        scene_abs_path=str(args.mjcf.resolve()),
        resolution_m=resolution,
        origin_xy=[float(xmin), float(ymin)],
        width=int(heights.shape[1]),
        height=int(heights.shape[0]),
        inherited_grid_from=inherited_from,
        paired_label_npy=str(args.mjcf.stem + "_gtlabel.npy")
            if label_sidecar.exists() else None,
        stats=dict(
            n_rays=meta["n_rays"], n_hit=meta["n_hit"], n_miss=meta["n_miss"],
            z_min=meta["z_min"], z_max=meta["z_max"],
            z_p05=meta["z_p05"], z_p95=meta["z_p95"],
        ),
        created_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
    )
    json_path.write_text(json.dumps(sidecar, indent=2))
    print(f"[clean_hm] wrote {out_path}", flush=True)
    print(f"[clean_hm] wrote {json_path}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
