#!/usr/bin/env python3
"""sonata_inference.py — Per-point semantic segmentation of ops2 PCD via
Sonata (Meta self-supervised PTv3) + linear probe head on ScanNet-20.

Mask3D / Pointcept supervised checkpoints expect 6-channel input
(XYZ + RGB) and degrade severely without colour. Sonata is self-supervised
so it relies primarily on geometry (positional encoding + normals); RGB
gets zero-padded with much less hit on accuracy.

Output:
  out_dir/labels.npy   — int8 per-point class index (N,)
  out_dir/labels.json  — class name + count summary
  out_dir/colored.ply  — point cloud coloured by predicted class (visual)

Usage:
    python3 sonata_inference.py input.pcd out_dir
"""
import argparse
import json
import time
import warnings
from pathlib import Path

import numpy as np

warnings.filterwarnings("ignore")


# ScanNet 20-class labels + colours (from Sonata demo)
CLASS_LABELS_20 = (
    "wall", "floor", "cabinet", "bed", "chair", "sofa", "table", "door",
    "window", "bookshelf", "picture", "counter", "desk", "curtain",
    "refrigerator", "shower_curtain", "toilet", "sink", "bathtub",
    "otherfurniture",
)
SCANNET_COLOR_MAP_20 = {
    0: (174, 199, 232), 1: (152, 223, 138), 2: (31, 119, 180), 3: (255, 187, 120),
    4: (188, 189, 34),  5: (140, 86, 75),   6: (255, 152, 150), 7: (214, 39, 40),
    8: (197, 176, 213), 9: (148, 103, 189), 10: (196, 156, 148), 11: (23, 190, 207),
    12: (247, 182, 210), 13: (219, 219, 141), 14: (255, 127, 14), 15: (158, 218, 229),
    16: (44, 160, 44),   17: (112, 128, 144), 18: (227, 119, 194), 19: (82, 84, 163),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="point cloud .pcd / .ply")
    ap.add_argument("out_dir", help="output directory")
    ap.add_argument("--max-points", type=int, default=2_000_000,
                    help="random subsample if PCD has > this many points "
                         "(default 2M — Sonata stable up to ~3M on 32GB VRAM)")
    ap.add_argument("--grid-size", type=float, default=0.04,
                    help="GridSample voxel size in meters (default 0.04 = "
                         "S3DIS scale; ScanNet uses 0.02)")
    ap.add_argument("--repo-id", default="facebook/sonata")
    ap.add_argument("--head", default="sonata_linear_prob_head_sc",
                    help="ScanNet linear probe head id "
                         "(sc=ScanNet20, sc200=ScanNet200)")
    args = ap.parse_args()

    out_dir = Path(args.out_dir); out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[{time.strftime('%H:%M:%S')}] loading point cloud...")
    import open3d as o3d
    pcd = o3d.io.read_point_cloud(args.input)
    coord = np.asarray(pcd.points, dtype=np.float32)
    n = len(coord)
    print(f"  {n:,} points")
    if n > args.max_points:
        rng = np.random.default_rng(42)
        idx = rng.choice(n, args.max_points, replace=False)
        idx.sort()  # preserve some spatial locality
        coord = coord[idx]
        print(f"  subsampled to {len(coord):,} pts (--max-points)")

    print(f"[{time.strftime('%H:%M:%S')}] estimating normals (kNN + tangent plane)...")
    pcd_ds = o3d.geometry.PointCloud()
    pcd_ds.points = o3d.utility.Vector3dVector(coord.astype(np.float64))
    pcd_ds.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.20, max_nn=30))
    normals = np.asarray(pcd_ds.normals, dtype=np.float32)
    # Sign-disambiguate normals to face +z on average so the model has
    # consistent orientation. Indoor scans: floor normals = (0,0,1).
    if normals[:, 2].mean() < 0:
        normals = -normals
    color = np.zeros_like(coord, dtype=np.float32)  # no LiDAR colour → zeros

    point = dict(coord=coord, color=color, normal=normals)
    print(f"  coord {coord.shape}  color zeros  normal {normals.shape}")

    print(f"[{time.strftime('%H:%M:%S')}] loading Sonata + ScanNet linear head...")
    import torch
    import torch.nn as nn
    import sonata

    try:
        import flash_attn  # noqa
        custom_config = {}
    except ImportError:
        custom_config = dict(enc_patch_size=[1024]*5, enable_flash=False)
    model = sonata.load("sonata", repo_id=args.repo_id, custom_config=custom_config).cuda()
    head_ckpt = sonata.load(args.head, repo_id=args.repo_id, ckpt_only=True)
    # Match the demo's SegHead wrapper exactly:
    #   class SegHead(nn.Module):
    #       seg_head: nn.Linear(backbone_out_channels, num_classes)
    class SegHead(nn.Module):
        def __init__(self, backbone_out_channels, num_classes):
            super().__init__()
            self.seg_head = nn.Linear(backbone_out_channels, num_classes)
        def forward(self, x):
            return self.seg_head(x)
    seg_head = SegHead(**head_ckpt["config"]).cuda()
    seg_head.load_state_dict(head_ckpt["state_dict"])
    model.eval(); seg_head.eval()

    print(f"[{time.strftime('%H:%M:%S')}] preprocessing (grid_size={args.grid_size}m)...")
    transform = sonata.transform.default()  # uses 0.02m grid by default; we may override later
    point = transform(point)

    print(f"[{time.strftime('%H:%M:%S')}] forward...")
    with torch.inference_mode():
        for k in list(point.keys()):
            if isinstance(point[k], torch.Tensor):
                point[k] = point[k].cuda(non_blocking=True)
        out = model(point)
        # unroll the pooling hierarchy to map features to input voxels
        while "pooling_parent" in out.keys():
            parent = out.pop("pooling_parent")
            inverse = out.pop("pooling_inverse")
            parent.feat = torch.cat([parent.feat, out.feat[inverse]], dim=-1)
            out = parent
        feat = out.feat
        seg_logits = seg_head(feat)
        # Map per-voxel logits → original (pre-grid) per-point labels
        pred_voxel = seg_logits.argmax(dim=-1).cpu().numpy().astype(np.int8)
        # inverse maps each original point to its voxel index
        inv = point.get("inverse").cpu().numpy() if hasattr(point, "get") and point.get("inverse") is not None else None
        if inv is not None:
            pred = pred_voxel[inv]
        else:
            pred = pred_voxel
    print(f"  predicted {len(pred):,} per-point labels")

    # Stats
    counts = np.bincount(pred, minlength=20)
    summary = {
        "n_points": int(len(pred)),
        "class_counts": {CLASS_LABELS_20[i]: int(counts[i]) for i in range(20)},
        "class_pct": {CLASS_LABELS_20[i]: round(float(counts[i] / max(1, len(pred))), 4) for i in range(20)},
    }
    print(f"[{time.strftime('%H:%M:%S')}] class distribution:")
    for k, v in sorted(summary["class_pct"].items(), key=lambda kv: -kv[1])[:10]:
        print(f"  {k:18s}  {v*100:5.1f}%  ({summary['class_counts'][k]:>10,d} pts)")

    # Save (coord aligned with pred 1:1, so downstream can KDTree-propagate)
    np.save(out_dir / "labels.npy", pred)
    np.save(out_dir / "coord.npy", coord.astype(np.float32))
    (out_dir / "labels.json").write_text(json.dumps(summary, indent=2))
    print(f"\n✓ saved labels.npy + labels.json → {out_dir}")

    # Colored ply for visualisation
    colors_arr = np.array([SCANNET_COLOR_MAP_20[c] for c in pred], dtype=np.uint8)
    out_pcd = o3d.geometry.PointCloud()
    out_pcd.points = o3d.utility.Vector3dVector(coord.astype(np.float64))
    out_pcd.colors = o3d.utility.Vector3dVector(colors_arr.astype(np.float64) / 255.0)
    o3d.io.write_point_cloud(str(out_dir / "colored.ply"), out_pcd)
    print(f"✓ saved colored.ply (open in MeshLab / Open3D to see classes)")


if __name__ == "__main__":
    main()
