#!/usr/bin/env python3
"""Pre-compute static traversability ground truth for a MuJoCo scene.

For each (x, y) cell in a regular grid over the scene's XY bounds, cast a
ray straight down from z=10 and classify by what the ray hits:

  - hit z < floor_threshold  → FREE (0)
  - hit z >= floor_threshold → LETHAL (1)
  - no hit                   → UNKNOWN (-1) — outside scene bounds

Then inflate LETHAL by the robot's footprint radius so an "inflated_lethal"
cell means "robot centered at this cell would have its body overlap with
a wall/obstacle." Output cells:

  0  = FREE          (robot can stand here, no overlap with anything tall)
  1  = LETHAL        (cell itself has wall/obstacle above floor)
  2  = INFLATED      (cell is free but within footprint_radius of LETHAL)
  -1 = UNKNOWN       (outside scene XY bounds)

Save as `<mjcf_stem>_gtlabel.npy` (int8, shape H×W) + a sidecar
`<mjcf_stem>_gtlabel.json` with `{resolution, origin, scene, ...}` metadata
to the same directory as the MJCF.

Usage:
    python3 scripts/training/mujoco_static_label_map.py \\
        src/go2w/go2_gazebo_sim/mujoco/demo3_mixed.xml

    # Batch over every MJCF in the directory:
    for mjcf in src/go2w/go2_gazebo_sim/mujoco/*.xml; do
        python3 scripts/training/mujoco_static_label_map.py "$mjcf"
    done

This is a one-shot pre-compute per scene. `trav_corpus_collector.py` reads
the resulting .npy + .json at runtime to label patches sampled from the
live `/elevation_map_raw` during benchmark trials.
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

try:
    from scipy.ndimage import binary_dilation
except ImportError as e:
    sys.exit(f"scipy missing (needed for footprint inflation): {e}")


# Label codes — must stay in sync with trav_corpus_collector.py.
FREE = 0
LETHAL = 1
INFLATED = 2
UNKNOWN = -1


def scene_xy_bounds(model: mujoco.MjModel) -> tuple[float, float, float, float]:
    """Return (xmin, ymin, xmax, ymax) over all worldbody (static) geoms.

    Excludes the floor plane (size can be huge, e.g. 50×50) by checking
    name=='ground' or 'floor' or by clamping plane sizes to the next-largest
    static geom. We instead compute bounds from *non-plane* static geoms
    only — walls + obstacles — which gives the actual reachable area.
    """
    xmin, ymin = math.inf, math.inf
    xmax, ymax = -math.inf, -math.inf
    found_any = False
    for gi in range(model.ngeom):
        if model.geom_bodyid[gi] != 0:
            continue  # not worldbody, skip dynamic
        gtype = int(model.geom_type[gi])
        # Skip plane geoms (they're floors and have giant sizes).
        if gtype == int(mujoco.mjtGeom.mjGEOM_PLANE):
            continue
        pos = model.geom_pos[gi]
        size = model.geom_size[gi]  # half-sizes for box/cylinder/etc.
        # Use size[0], size[1] as XY extent for box; for sphere use size[0]
        # for both. Cheap conservative bound.
        ex = float(size[0]) if size[0] > 0 else 0.5
        ey = float(size[1]) if size[1] > 0 else 0.5
        xmin = min(xmin, float(pos[0]) - ex)
        xmax = max(xmax, float(pos[0]) + ex)
        ymin = min(ymin, float(pos[1]) - ey)
        ymax = max(ymax, float(pos[1]) + ey)
        found_any = True
    if not found_any:
        raise RuntimeError("No worldbody non-plane geoms found — empty scene?")
    return xmin, ymin, xmax, ymax


def park_robot_far_away(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Move every free joint to (1000, 1000, 1000) so robots don't appear in raycasts.

    The scene may have one or two free-joint robots (single Go2 vs dual
    Go2W+Go2). All of them get parked. Static scene geoms are unaffected.
    """
    park_pos = np.array([1000.0, 1000.0, 1000.0], dtype=np.float64)
    for ji in range(model.njnt):
        if int(model.jnt_type[ji]) == int(mujoco.mjtJoint.mjJNT_FREE):
            qadr = int(model.jnt_qposadr[ji])
            # qpos layout for free joint: [x, y, z, qw, qx, qy, qz]
            data.qpos[qadr:qadr + 3] = park_pos
            data.qpos[qadr + 3:qadr + 7] = np.array([1.0, 0.0, 0.0, 0.0])
            park_pos = park_pos + np.array([100.0, 0.0, 0.0])  # space them out
    mujoco.mj_forward(model, data)


def build_label_grid(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    xmin: float, ymin: float, xmax: float, ymax: float,
    resolution: float,
    floor_threshold_m: float,
    raycast_height_m: float = 10.0,
) -> tuple[np.ndarray, dict]:
    """Raster (xmin..xmax) × (ymin..ymax) at `resolution` spacing.

    For each cell, raycast down. Hit above `floor_threshold_m` → LETHAL,
    hit below → FREE, no hit → UNKNOWN.

    Returns:
        labels: int8 array shape (H, W) — row-major; labels[iy, ix] = cell at
                (xmin + ix*resolution, ymin + iy*resolution).
        meta:   diagnostic dict.
    """
    W = int(math.ceil((xmax - xmin) / resolution))
    H = int(math.ceil((ymax - ymin) / resolution))

    labels = np.full((H, W), UNKNOWN, dtype=np.int8)
    hit_heights = np.full((H, W), np.nan, dtype=np.float32)

    pnt = np.zeros(3, dtype=np.float64)
    vec = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    geomid_buf = np.zeros(1, dtype=np.int32)

    t0 = time.monotonic()
    n_rays = 0
    n_lethal = 0
    n_free = 0
    n_unknown = 0

    for iy in range(H):
        y = ymin + iy * resolution
        for ix in range(W):
            x = xmin + ix * resolution
            pnt[0] = x
            pnt[1] = y
            pnt[2] = raycast_height_m
            dist = mujoco.mj_ray(
                model, data, pnt, vec,
                None,  # geomgroup — None means check all groups
                1,     # flg_static — include static worldbody geoms
                -1,    # bodyexclude
                geomid_buf,
            )
            n_rays += 1
            if dist < 0 or geomid_buf[0] < 0:
                labels[iy, ix] = UNKNOWN
                n_unknown += 1
                continue
            hit_z = raycast_height_m - dist
            hit_heights[iy, ix] = hit_z
            if hit_z < floor_threshold_m:
                labels[iy, ix] = FREE
                n_free += 1
            else:
                labels[iy, ix] = LETHAL
                n_lethal += 1

    dt = time.monotonic() - t0
    meta = dict(
        n_rays=n_rays, n_free=n_free, n_lethal=n_lethal, n_unknown=n_unknown,
        rays_per_sec=int(n_rays / dt) if dt > 0 else 0,
        elapsed_sec=round(dt, 3),
        hit_z_min=float(np.nanmin(hit_heights)) if np.any(~np.isnan(hit_heights)) else None,
        hit_z_max=float(np.nanmax(hit_heights)) if np.any(~np.isnan(hit_heights)) else None,
    )
    return labels, meta


def inflate_lethal(labels: np.ndarray, footprint_radius_m: float, resolution: float) -> np.ndarray:
    """Mark FREE cells within `footprint_radius_m` of a LETHAL cell as INFLATED.

    Uses a circular structuring element. Returns a new label array; UNKNOWN
    cells are preserved.
    """
    r = max(1, int(math.ceil(footprint_radius_m / resolution)))
    yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
    selem = (xx * xx + yy * yy) <= r * r

    lethal_mask = (labels == LETHAL)
    if not np.any(lethal_mask):
        return labels.copy()

    dilated = binary_dilation(lethal_mask, structure=selem)

    out = labels.copy()
    # Cells that became dilated but weren't lethal already → INFLATED.
    # Don't touch UNKNOWN cells.
    free_mask = (labels == FREE)
    out[(dilated & free_mask)] = INFLATED
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mjcf", type=Path, help="Path to MuJoCo MJCF scene file")
    ap.add_argument("--resolution", type=float, default=0.10,
                    help="Grid resolution in meters (default 0.10 = elevation_mapping default)")
    ap.add_argument("--floor-threshold", type=float, default=0.05,
                    help="Hit z below this is floor → FREE (default 0.05)")
    ap.add_argument("--footprint-radius", type=float, default=0.22,
                    help="Robot footprint inflation radius in meters (default 0.22 = Go2 ~0.4m diag/2)")
    ap.add_argument("--pad", type=float, default=0.5,
                    help="Padding (m) added to each XY scene bound (default 0.5)")
    ap.add_argument("--output", type=Path, default=None,
                    help="Output .npy path (default <mjcf_stem>_gtlabel.npy alongside MJCF)")
    args = ap.parse_args()

    if not args.mjcf.exists():
        sys.exit(f"MJCF not found: {args.mjcf}")

    print(f"[mujoco_label] loading {args.mjcf}", flush=True)
    model = mujoco.MjModel.from_xml_path(str(args.mjcf))
    data = mujoco.MjData(model)

    park_robot_far_away(model, data)

    xmin, ymin, xmax, ymax = scene_xy_bounds(model)
    xmin -= args.pad
    ymin -= args.pad
    xmax += args.pad
    ymax += args.pad
    print(f"[mujoco_label] scene bounds: x=[{xmin:.2f}, {xmax:.2f}]m  y=[{ymin:.2f}, {ymax:.2f}]m  "
          f"size={xmax-xmin:.1f}×{ymax-ymin:.1f}m", flush=True)

    labels, meta = build_label_grid(
        model, data, xmin, ymin, xmax, ymax,
        args.resolution, args.floor_threshold,
    )
    print(f"[mujoco_label] raster {labels.shape[1]}×{labels.shape[0]}  "
          f"{meta['n_rays']} rays in {meta['elapsed_sec']}s ({meta['rays_per_sec']} ray/s)",
          flush=True)
    print(f"[mujoco_label]   FREE={meta['n_free']}  LETHAL={meta['n_lethal']}  "
          f"UNKNOWN={meta['n_unknown']}", flush=True)
    if meta['hit_z_max'] is not None:
        print(f"[mujoco_label]   hit_z: [{meta['hit_z_min']:.3f}, {meta['hit_z_max']:.3f}]m",
              flush=True)

    labels_inflated = inflate_lethal(labels, args.footprint_radius, args.resolution)
    n_inflated = int(np.sum(labels_inflated == INFLATED))
    print(f"[mujoco_label] inflated by {args.footprint_radius:.2f}m: +{n_inflated} INFLATED cells",
          flush=True)

    out_path = args.output or args.mjcf.with_name(args.mjcf.stem + "_gtlabel.npy")
    json_path = out_path.with_suffix(".json")
    np.save(out_path, labels_inflated)

    sidecar = dict(
        scene=str(args.mjcf.name),
        scene_abs_path=str(args.mjcf.resolve()),
        resolution_m=args.resolution,
        origin_xy=[float(xmin), float(ymin)],
        width=int(labels_inflated.shape[1]),
        height=int(labels_inflated.shape[0]),
        footprint_radius_m=args.footprint_radius,
        floor_threshold_m=args.floor_threshold,
        label_codes=dict(FREE=FREE, LETHAL=LETHAL, INFLATED=INFLATED, UNKNOWN=UNKNOWN),
        counts=dict(
            free=int(np.sum(labels_inflated == FREE)),
            lethal=int(np.sum(labels_inflated == LETHAL)),
            inflated=int(np.sum(labels_inflated == INFLATED)),
            unknown=int(np.sum(labels_inflated == UNKNOWN)),
        ),
        created_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
    )
    json_path.write_text(json.dumps(sidecar, indent=2))
    print(f"[mujoco_label] wrote {out_path}", flush=True)
    print(f"[mujoco_label] wrote {json_path}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
