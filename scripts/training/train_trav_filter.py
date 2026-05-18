#!/usr/bin/env python3
"""train_trav_filter.py — fine-tune the ETH RSL traversability CNN.

Trains the exact architecture used by elevation_mapping_cupy
(traversability_filter.py): 3 dilated 3x3 convs (1→4 ch, dilation 1/2/3),
concat to 12 ch, 1x1 conv 12→1, exp(-|.|). 120 parameters total.

Input: .npz from synth_terrain_dataset.py (or collect_trav_dataset.py)
  patches: (N, 7, 7) float32 — elevation in meters, NaN-tolerant
  labels:  (N,)     float32 — target trav ∈ [0, 1]

Loss: BCE-equivalent (labels are continuous; we use MSE since the
network's output is bounded by exp(-|.|) and exp's gradient is well
behaved across [0, 1]). Optional class re-weighting via --class-weight.

Output: weights.dat (pickle with keys conv1.weight / conv2.weight /
conv3.weight / conv_final.weight) matching the runtime loader at
src/vendor/elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_cupy/
parameter.py:229.

Usage:
    # pretrain from scratch (or random init):
    python3 train_trav_filter.py out/pretrain.npz weights_pretrain.dat \
        --epochs 500 --lr 1e-3

    # fine-tune from existing weights:
    python3 train_trav_filter.py out/ramp_real.npz weights_ramp.dat \
        --epochs 100 --lr 1e-4 \
        --init-from src/vendor/elevation_mapping_cupy/elevation_mapping_cupy/config/core/weights.dat
"""
import argparse
import pickle
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F


class TraversabilityFilter(nn.Module):
    """Identical math to elevation_mapping_cupy's TraversabilityFilter.

    Input  : (B, 1, H, W) float32. NaN handled by caller (replace with 0).
    Output : (B, 1, H-6, W-6) ∈ (0, 1]. For H=W=7, output is (B, 1, 1, 1).
    """

    def __init__(self, use_bias: bool = False):
        super().__init__()
        self.conv1 = nn.Conv2d(1, 4, 3, dilation=1, padding=0, bias=use_bias)
        self.conv2 = nn.Conv2d(1, 4, 3, dilation=2, padding=0, bias=use_bias)
        self.conv3 = nn.Conv2d(1, 4, 3, dilation=3, padding=0, bias=use_bias)
        self.conv_out = nn.Conv2d(12, 1, 1, bias=use_bias)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        out1 = self.conv1(x)
        out2 = self.conv2(x)
        out3 = self.conv3(x)
        # Crop to common spatial size (matches runtime cropping).
        out1 = out1[:, :, 2:-2, 2:-2] if out1.shape[-1] > 4 else out1
        out2 = out2[:, :, 1:-1, 1:-1] if out2.shape[-1] > 2 else out2
        # For H=W=7: out1=(_, 4, 5, 5)→(_, 4, 1, 1), out2=(_, 4, 3, 3)→(_, 4, 1, 1),
        # out3=(_, 4, 1, 1). All same after crop.
        out = torch.cat((out1, out2, out3), dim=1)  # (B, 12, h, w)
        out = self.conv_out(out.abs())
        return torch.exp(-out)


def load_dataset(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    data = np.load(path)
    patches = data["patches"].astype(np.float32)
    labels = data["labels"].astype(np.float32)
    classes = data["classes"].astype(np.uint8) if "classes" in data.files \
        else np.zeros(len(labels), dtype=np.uint8)
    # NaN → 0 (zero contribution to conv sum). The Kalman map fills these
    # with mean elevation eventually; for training, 0 is a benign sentinel.
    patches = np.where(np.isfinite(patches), patches, 0.0).astype(np.float32)
    return patches, labels, classes


def init_from_pickle(model: TraversabilityFilter, weights_path: Path) -> None:
    """Load (conv1/conv2/conv3/conv_final).weight into model. Skip on failure."""
    with weights_path.open("rb") as f:
        w = pickle.load(f)
    with torch.no_grad():
        model.conv1.weight.copy_(torch.from_numpy(np.asarray(w["conv1.weight"], dtype=np.float32)))
        model.conv2.weight.copy_(torch.from_numpy(np.asarray(w["conv2.weight"], dtype=np.float32)))
        model.conv3.weight.copy_(torch.from_numpy(np.asarray(w["conv3.weight"], dtype=np.float32)))
        model.conv_out.weight.copy_(torch.from_numpy(np.asarray(w["conv_final.weight"], dtype=np.float32)))


def save_pickle(model: TraversabilityFilter, out_path: Path) -> None:
    """Save in the exact format parameter.py:229 expects."""
    state = {
        "conv1.weight":      model.conv1.weight.detach().cpu().numpy().astype(np.float32),
        "conv2.weight":      model.conv2.weight.detach().cpu().numpy().astype(np.float32),
        "conv3.weight":      model.conv3.weight.detach().cpu().numpy().astype(np.float32),
        "conv_final.weight": model.conv_out.weight.detach().cpu().numpy().astype(np.float32),
    }
    with out_path.open("wb") as f:
        pickle.dump(state, f, protocol=pickle.HIGHEST_PROTOCOL)


def compute_loss(pred: torch.Tensor, y: torch.Tensor, args) -> torch.Tensor:
    """Loss with optional class weighting + label smoothing.

    Convention: y=1.0 is traversable (good), y=0.0 is lethal (bad).
    We weight lethal errors higher so missed obstacles hurt more.
    """
    eps = float(args.label_smoothing)
    if eps > 0:
        # y -> (1-eps) for trav, eps for lethal (asymmetric smoothing OK
        # since 0/1 are the only label values)
        y = y * (1.0 - 2 * eps) + eps  # y=1 → 1-eps, y=0 → eps

    if args.loss_type == "mse":
        if args.lethal_weight == 1.0:
            return F.mse_loss(pred, y)
        # Per-sample weight: lethal (y<0.5) gets lethal_weight
        w = torch.where(y < 0.5,
                        torch.tensor(args.lethal_weight, device=y.device),
                        torch.tensor(1.0, device=y.device))
        return (w * (pred - y) ** 2).mean()
    if args.loss_type == "bce":
        # pred ∈ (0,1]; clamp away from 0 for log stability
        pc = pred.clamp(1e-7, 1.0 - 1e-7)
        pos = -y * torch.log(pc)              # missing trav (y=1)
        neg = -(1 - y) * torch.log(1 - pc)    # missing lethal (y=0)
        return (pos + args.lethal_weight * neg).mean()
    if args.loss_type == "focal":
        # Binary focal: down-weight easy examples
        pc = pred.clamp(1e-7, 1.0 - 1e-7)
        gamma = float(args.focal_gamma)
        # pt is the predicted prob of the target class
        pt = torch.where(y > 0.5, pc, 1 - pc)
        focal_w = (1 - pt) ** gamma
        bce = -(y * torch.log(pc) + (1 - y) * torch.log(1 - pc))
        # Asymmetric weighting via lethal_weight
        cls_w = torch.where(y < 0.5,
                            torch.tensor(args.lethal_weight, device=y.device),
                            torch.tensor(1.0, device=y.device))
        return (focal_w * cls_w * bce).mean()
    raise ValueError(f"unknown loss_type {args.loss_type}")


def train_one_epoch(
    model: TraversabilityFilter,
    optimizer: torch.optim.Optimizer,
    patches: torch.Tensor,
    labels: torch.Tensor,
    batch_size: int,
    rng: np.random.Generator,
    args,
) -> float:
    n = len(patches)
    perm = rng.permutation(n)
    total_loss = 0.0
    n_batches = 0
    model.train()
    for start in range(0, n, batch_size):
        idx = perm[start : start + batch_size]
        x = patches[idx]  # (B, 1, 7, 7)
        y = labels[idx]   # (B,)
        # Re-center each patch on its own min so the network learns
        # relative geometry, not absolute height. Matches runtime input.
        x_min = x.reshape(x.shape[0], -1).min(dim=1).values[:, None, None, None]
        x = x - x_min

        optimizer.zero_grad()
        pred = model(x).squeeze()
        loss = compute_loss(pred, y, args)
        loss.backward()
        optimizer.step()
        total_loss += float(loss.item())
        n_batches += 1
    return total_loss / max(1, n_batches)


@torch.no_grad()
def evaluate(
    model: TraversabilityFilter,
    patches: torch.Tensor,
    labels: torch.Tensor,
    classes: np.ndarray,
) -> dict:
    model.eval()
    x = patches
    x_min = x.reshape(x.shape[0], -1).min(dim=1).values[:, None, None, None]
    x = x - x_min
    pred = model(x).squeeze().cpu().numpy()
    y = labels.cpu().numpy()
    mse = float(np.mean((pred - y) ** 2))
    # Binary accuracy at 0.5 threshold (treat label > 0.5 as traversable).
    acc = float(np.mean((pred > 0.5) == (y > 0.5)))
    per_class = {}
    for k in np.unique(classes):
        m = classes == k
        per_class[int(k)] = {
            "n": int(m.sum()),
            "pred_mean": float(pred[m].mean()),
            "label_mean": float(y[m].mean()),
            "mse": float(np.mean((pred[m] - y[m]) ** 2)),
        }
    return {"mse": mse, "acc@0.5": acc, "per_class": per_class}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dataset", help="input .npz from synth_terrain_dataset.py")
    ap.add_argument("output", help="output weights.dat (pickle)")
    ap.add_argument("--init-from", default=None,
                    help="warm-start from existing weights.dat")
    ap.add_argument("--epochs", type=int, default=500)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--batch-size", type=int, default=4096)
    ap.add_argument("--val-frac", type=float, default=0.1)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--report-every", type=int, default=25)
    # ---- new knobs (loss + anti-forgetting + anti-overfit) ----
    ap.add_argument("--loss-type", choices=("mse", "bce", "focal"), default="mse",
                    help="Loss flavor. weighted-mse default; bce/focal for "
                         "stronger asymmetry on lethal misses.")
    ap.add_argument("--lethal-weight", type=float, default=3.0,
                    help="Multiply lethal-class loss by this. Higher = "
                         "more conservative (prefers FP-lethal over FN-lethal).")
    ap.add_argument("--focal-gamma", type=float, default=2.0)
    ap.add_argument("--label-smoothing", type=float, default=0.05,
                    help="ε for label smoothing — guards against the ~10% "
                         "noise in our auto+polish labels.")
    ap.add_argument("--mix-pretrain", default=None,
                    help="Path to pretrain .npz to mix in (catastrophic "
                         "forgetting mitigation). Roughly 30% of every "
                         "epoch comes from this dataset.")
    ap.add_argument("--mix-ratio", type=float, default=0.30,
                    help="Fraction of each epoch sampled from --mix-pretrain")
    ap.add_argument("--weight-decay", type=float, default=1e-4,
                    help="Adam L2 weight decay")
    ap.add_argument("--early-stop-patience", type=int, default=0,
                    help="Stop if val_mse doesn't improve for this many "
                         "report intervals (0 = disabled)")
    ap.add_argument("--ckpt-every", type=int, default=0,
                    help="Save weights every N epochs as "
                         "<output>_ep<N>.dat. 0 = only save final.")
    args = ap.parse_args()

    print(f"loading dataset {args.dataset}...")
    patches_np, labels_np, classes_np = load_dataset(Path(args.dataset))
    print(f"  primary: {len(patches_np):,} patches, "
          f"label mean {labels_np.mean():.3f}")

    rng = np.random.default_rng(args.seed)
    perm = rng.permutation(len(patches_np))
    n_val = int(len(perm) * args.val_frac)
    val_idx = perm[:n_val]
    train_idx = perm[n_val:]

    x_train = torch.from_numpy(patches_np[train_idx, None, :, :]).to(args.device)
    y_train = torch.from_numpy(labels_np[train_idx]).to(args.device)
    x_val = torch.from_numpy(patches_np[val_idx, None, :, :]).to(args.device)
    y_val = torch.from_numpy(labels_np[val_idx]).to(args.device)
    c_val = classes_np[val_idx]

    # Optional pretrain mix (anti catastrophic forgetting)
    x_pre = y_pre = None
    if args.mix_pretrain:
        pre_p, pre_l, _pre_c = load_dataset(Path(args.mix_pretrain))
        print(f"  pretrain mix: {len(pre_p):,} patches "
              f"(will be sampled at ratio {args.mix_ratio:.2f} per epoch)")
        x_pre = torch.from_numpy(pre_p[:, None, :, :]).to(args.device)
        y_pre = torch.from_numpy(pre_l).to(args.device)

    model = TraversabilityFilter().to(args.device)
    if args.init_from:
        print(f"warm-starting from {args.init_from}...")
        try:
            init_from_pickle(model, Path(args.init_from))
        except Exception as e:
            print(f"  WARNING: warm-start failed ({e}); using random init")

    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr,
                                 weight_decay=args.weight_decay)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=args.epochs)

    print(f"training: {args.epochs} epochs, batch={args.batch_size}, lr={args.lr}, "
          f"wd={args.weight_decay}")
    print(f"  loss={args.loss_type}  lethal_w={args.lethal_weight}  "
          f"smooth={args.label_smoothing}  mix={args.mix_ratio if args.mix_pretrain else 0:.2f}")
    print(f"  device: {args.device}, params: {sum(p.numel() for p in model.parameters())}")
    print()

    best_val_mse = float("inf")
    best_state = None
    plateau_count = 0
    for ep in range(args.epochs):
        # Build the per-epoch training set: optionally mix in pretrain
        if x_pre is not None and args.mix_ratio > 0:
            n_pre = int(args.mix_ratio * len(x_train))
            pre_perm = rng.permutation(len(x_pre))[:n_pre]
            x_ep = torch.cat([x_train, x_pre[pre_perm]], dim=0)
            y_ep = torch.cat([y_train, y_pre[pre_perm]], dim=0)
        else:
            x_ep, y_ep = x_train, y_train

        train_loss = train_one_epoch(
            model, optimizer, x_ep, y_ep, args.batch_size, rng, args)
        scheduler.step()
        if (ep + 1) % args.report_every == 0 or ep == args.epochs - 1:
            metrics = evaluate(model, x_val, y_val, c_val)
            improved = metrics["mse"] < best_val_mse
            if improved:
                best_val_mse = metrics["mse"]
                best_state = {k: v.detach().clone() for k, v in model.state_dict().items()}
                plateau_count = 0
            else:
                plateau_count += 1
            tag = "*" if improved else " "
            print(f"  ep {ep+1:4d}/{args.epochs}  train_loss={train_loss:.4f}  "
                  f"val_mse={metrics['mse']:.4f}  val_acc={metrics['acc@0.5']:.3f}  {tag}")
            if args.early_stop_patience > 0 and plateau_count >= args.early_stop_patience:
                print(f"  early stopping at epoch {ep+1} "
                      f"(no val improvement for {plateau_count} reports)")
                break

        # Periodic checkpoint (overwrites prior ckpts at the same epoch)
        if args.ckpt_every > 0 and (ep + 1) % args.ckpt_every == 0:
            out_path_base = Path(args.output)
            ckpt_path = out_path_base.with_name(
                f"{out_path_base.stem}_ep{ep+1:03d}{out_path_base.suffix}")
            save_pickle(model, ckpt_path)
            print(f"     → ckpt: {ckpt_path}")

    if best_state is not None:
        model.load_state_dict(best_state)
    print()
    print("final per-class validation metrics:")
    metrics = evaluate(model, x_val, y_val, c_val)
    for k, st in metrics["per_class"].items():
        print(f"  class {k} n={st['n']:5d}  pred_mean={st['pred_mean']:.3f}  "
              f"label_mean={st['label_mean']:.3f}  mse={st['mse']:.4f}")

    out_path = Path(args.output).resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    baseline = Path(
        "src/vendor/elevation_mapping_cupy/elevation_mapping_cupy/"
        "config/core/weights.dat"
    ).resolve()
    if out_path == baseline:
        print(f"REFUSING to overwrite baseline weights at {baseline}.")
        print("Pick a different --output path (e.g. weights_ramp.dat).")
        sys.exit(2)
    save_pickle(model, out_path)
    print()
    print(f"✓ wrote {out_path} ({out_path.stat().st_size} bytes)")
    print()
    print("Baseline weights are preserved at:")
    print(f"  {baseline}")
    print()
    print("To A/B test the NEW weights without touching the baseline, point")
    print("elevation_mapping_cupy at the new file via its `weight_file` ROS")
    print("param. Easiest: add this line to")
    print("  src/collaborative_exploration/trav_cost_filters/config/elevation_mapping.yaml")
    print("under /**: ros__parameters:")
    print(f"  weight_file: \"{out_path}\"")
    print()
    print("Then restart the elevation_mapping_node (no rebuild needed if you")
    print("only edited the yaml). Revert by removing that line.")


if __name__ == "__main__":
    main()
