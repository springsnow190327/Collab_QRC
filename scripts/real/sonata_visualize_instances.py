#!/usr/bin/env python3
"""sonata_visualize_instances.py — top-down PNG of mesh + Sonata instances.

Quick visual triage before relying on the instance labels: render the ops2
mesh footprint (gray) + each instance as a colored AABB + class label, so
the user can spot misclassified clusters (e.g. ScanNet saying "bed" when
it's actually a vending machine or A/C unit) without launching sim.

Output: a single matplotlib PNG showing all 12 instances overlayed on
the mesh top-down footprint.

Usage:
    python3 sonata_visualize_instances.py \\
        --mesh bags/meshes/ops2_cuda/scans_v4_aligned.obj \\
        --summary bags/meshes/ops2_cuda/sonata/instances/ops2_inst_summary.json \\
        --mesh-vert-labels bags/meshes/ops2_cuda/sonata/instances/mesh_vert_labels.npy \\
        --out /tmp/sonata_instances.png
"""
import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import open3d as o3d


CLASS_LABELS_20 = (
    "wall", "floor", "cabinet", "bed", "chair", "sofa", "table", "door",
    "window", "bookshelf", "picture", "counter", "desk", "curtain",
    "refrigerator", "shower_curtain", "toilet", "sink", "bathtub",
    "otherfurniture",
)
# Distinct colors per class for the legend
CLASS_COLORS = {
    "wall": "#999999", "floor": "#cccccc",
    "bed": "#e6194B", "table": "#3cb44b", "chair": "#ffe119",
    "sofa": "#4363d8", "cabinet": "#f58231", "door": "#911eb4",
    "bathtub": "#42d4f4", "otherfurniture": "#f032e6",
    "bookshelf": "#bfef45", "curtain": "#fabed4",
    "refrigerator": "#469990", "picture": "#dcbeff",
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mesh", required=True)
    ap.add_argument("--summary", required=True)
    ap.add_argument("--mesh-vert-labels", required=False,
                    help="(optional) per-vert labels npy → color by class")
    ap.add_argument("--out", required=True)
    ap.add_argument("--scatter-stride", type=int, default=50,
                    help="every N-th mesh vert in scatter (for speed)")
    args = ap.parse_args()

    mesh = o3d.io.read_triangle_mesh(args.mesh)
    verts = np.asarray(mesh.vertices)
    print(f"loaded mesh: {len(verts):,} verts")

    summary = json.loads(Path(args.summary).read_text())
    instances = summary["instances"]
    print(f"loaded {len(instances)} instances")

    # Read instance positions/extents from individual instance JSON-like data
    # The summary doesn't store center; recompute from extents — but we
    # also need the AABB center. Better: read each instance OBJ.
    inst_dir = Path(args.summary).parent / "meshes"
    inst_aabbs = []
    for inst in instances:
        # We saved the full instance mesh as ops2_inst_<class>_NNNN.obj
        op = inst_dir / f"{inst['name']}.obj"
        if not op.exists():
            inst_aabbs.append(None)
            continue
        im = o3d.io.read_triangle_mesh(str(op))
        ivs = np.asarray(im.vertices)
        if len(ivs) == 0:
            inst_aabbs.append(None); continue
        bmin = ivs.min(0); bmax = ivs.max(0)
        inst_aabbs.append((bmin, bmax))

    # Plot
    fig, ax = plt.subplots(1, 1, figsize=(16, 10))

    # Mesh footprint scatter (colored by vert label if available)
    sub = verts[::args.scatter_stride]
    if args.mesh_vert_labels and Path(args.mesh_vert_labels).exists():
        lbl = np.load(args.mesh_vert_labels).astype(np.int32)
        sub_lbl = lbl[::args.scatter_stride]
        # Color floor/wall lightly
        col = np.full((len(sub), 3), 0.85)
        wall_m = sub_lbl == 0
        floor_m = sub_lbl == 1
        obj_m = ~(wall_m | floor_m)
        col[wall_m] = (0.75, 0.75, 0.75)
        col[floor_m] = (0.92, 0.92, 0.85)
        # Per-class color for objects
        for ci, cn in enumerate(CLASS_LABELS_20):
            if ci in (0, 1): continue
            m = (sub_lbl == ci)
            if m.sum() == 0: continue
            c = CLASS_COLORS.get(cn, "#888888")
            from matplotlib.colors import to_rgb
            col[m] = to_rgb(c)
        ax.scatter(sub[:, 0], sub[:, 1], c=col, s=0.6, alpha=0.7, marker='.')
    else:
        ax.scatter(sub[:, 0], sub[:, 1], c="#aaaaaa", s=0.4, alpha=0.5, marker='.')

    # Instance AABBs
    for inst, aabb in zip(instances, inst_aabbs):
        if aabb is None:
            continue
        bmin, bmax = aabb
        cls = inst["class"]
        c = CLASS_COLORS.get(cls, "#000000")
        rect = patches.Rectangle(
            (bmin[0], bmin[1]),
            bmax[0] - bmin[0], bmax[1] - bmin[1],
            linewidth=2, edgecolor=c, facecolor='none', linestyle='-'
        )
        ax.add_patch(rect)
        # label at center
        cx, cy = (bmin[0] + bmax[0]) / 2.0, (bmin[1] + bmax[1]) / 2.0
        ext_z = bmax[2] - bmin[2]
        ax.annotate(
            f"{inst['name'].split('ops2_inst_')[1]}\n"
            f"z={bmin[2]:.1f}-{bmax[2]:.1f}m\n"
            f"({inst['method']})",
            (cx, cy), fontsize=7, ha="center", va="center", color=c,
            bbox=dict(boxstyle="round,pad=0.2", facecolor="white",
                      edgecolor=c, alpha=0.85, linewidth=0.5)
        )

    # Legend
    legend_handles = []
    for cn in ["wall", "floor", "bed", "table", "bathtub", "otherfurniture"]:
        c = CLASS_COLORS.get(cn, "#888888")
        legend_handles.append(
            patches.Patch(facecolor=c, edgecolor='black', label=cn)
        )
    ax.legend(handles=legend_handles, loc="upper right", framealpha=0.9)

    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(
        f"ops2 Sonata instances: {len(instances)} found "
        f"({summary['classes']})"
    )
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(args.out, dpi=150, bbox_inches="tight")
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
