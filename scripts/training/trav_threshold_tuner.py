#!/usr/bin/env python3
"""trav_threshold_tuner.py — interactive slope/step/height threshold tuner.

Instead of painting per-cell, expose the 6 thresholds used by
auto_label_heightmap as sliders. The label overlay updates LIVE on each
slider move. Adjust until happy, press 's' to save the .npz.

Precomputes the three geometric features once (slope, step,
height_above_floor); slider changes only re-evaluate boolean masks →
millisecond response on the full 321×807 grid.

Output: labels.npz in the same format as trav_labeler / auto_label
(so it plugs straight into build_trav_dataset.py).

Usage:
    python3 trav_threshold_tuner.py \\
        --heights bags/meshes/ops2_cuda/hfield/ops2_v4_heights.npz \\
        --out    bags/meshes/ops2_cuda/hfield/ops2_v4_tuned_labels.npz
"""
import argparse
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("TkAgg", force=True)
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button


def compute_features(heights, touched, cell, floor_window_m, floor_pct):
    H, W = heights.shape
    # Local floor (coarse stride percentile)
    stride = max(1, int(round(floor_window_m / cell)))
    cH = (H + stride - 1) // stride
    cW = (W + stride - 1) // stride
    coarse_floor = np.zeros((cH, cW), dtype=np.float32)
    for cy in range(cH):
        for cx in range(cW):
            y0 = cy * stride; y1 = min(H, y0 + 2 * stride)
            x0 = cx * stride; x1 = min(W, x0 + 2 * stride)
            block_t = touched[y0:y1, x0:x1]
            block_h = heights[y0:y1, x0:x1]
            if block_t.any():
                coarse_floor[cy, cx] = np.percentile(block_h[block_t], floor_pct)
            else:
                coarse_floor[cy, cx] = 0.0
    floor_map = np.kron(coarse_floor, np.ones((stride, stride), dtype=np.float32))[:H, :W]
    h_above = heights - floor_map

    # Slope
    gy = np.zeros_like(heights, dtype=np.float32)
    gx = np.zeros_like(heights, dtype=np.float32)
    gy[1:-1] = (heights[2:] - heights[:-2]) / (2 * cell)
    gx[:, 1:-1] = (heights[:, 2:] - heights[:, :-2]) / (2 * cell)
    slope_deg = np.degrees(np.arctan(np.sqrt(gx * gx + gy * gy)))

    # Step (max abs diff in 3x3)
    H_pad = np.pad(heights, 1, mode="edge")
    diffs = []
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            shifted = H_pad[1 + dy:1 + dy + H, 1 + dx:1 + dx + W]
            diffs.append(np.abs(heights - shifted))
    step_m = np.max(np.stack(diffs, axis=0), axis=0)

    return slope_deg.astype(np.float32), step_m.astype(np.float32), h_above.astype(np.float32)


class Tuner:
    def __init__(self, heights_npz, out_path, init):
        d = np.load(heights_npz, allow_pickle=False)
        self.heights = d["heights"]
        self.touched = d["touched"]
        self.cell = float(d["cell_size_m"])
        self.origin = d["origin_xy"].astype(np.float32)
        self.H, self.W = self.heights.shape
        self.out_path = out_path

        print("computing slope/step/height_above_floor (one-shot)...")
        self.slope, self.step, self.h_above = compute_features(
            self.heights, self.touched, self.cell,
            floor_window_m=2.0, floor_pct=5.0,
        )
        print(f"  slope:   [{self.slope[self.touched].min():.1f}, {self.slope[self.touched].max():.1f}] deg")
        print(f"  step:    [{self.step[self.touched].min():.3f}, {self.step[self.touched].max():.3f}] m")
        print(f"  h_above: [{self.h_above[self.touched].min():.2f}, {self.h_above[self.touched].max():.2f}] m")

        self.init = init
        self._build_fig()
        self._recompute()

    def _build_fig(self):
        self.fig = plt.figure(figsize=(18, 10))
        # Main image
        self.ax_main = self.fig.add_axes([0.05, 0.35, 0.70, 0.60])
        disp = self.heights.copy()
        disp[~self.touched] = np.nan
        self.img_height = self.ax_main.imshow(
            disp, cmap="gray", origin="lower",
            extent=self._extent(), alpha=0.6,
        )
        self.img_label = self.ax_main.imshow(
            np.zeros((self.H, self.W, 4), dtype=np.float32),
            origin="lower", extent=self._extent(), interpolation="nearest",
        )
        self.ax_main.set_xlabel("x [m]"); self.ax_main.set_ylabel("y [m]")
        self.ax_main.set_aspect("equal")

        # Side: feature views (small)
        self.ax_slope = self.fig.add_axes([0.78, 0.66, 0.18, 0.26])
        self.ax_step = self.fig.add_axes([0.78, 0.36, 0.18, 0.26])

        for ax, data, label, cmap, vmax in [
            (self.ax_slope, self.slope, "slope [deg]", "hot", 60),
            (self.ax_step,  self.step,  "step [m]",   "magma", 0.5),
        ]:
            disp = data.copy(); disp[~self.touched] = np.nan
            ax.imshow(disp, origin="lower", cmap=cmap, vmin=0, vmax=vmax)
            ax.set_title(label, fontsize=9)
            ax.set_xticks([]); ax.set_yticks([])

        # Sliders (6) — lethal thresholds + traversable thresholds
        slider_axes = []
        bottom = 0.02
        height = 0.025
        gap = 0.005
        for i in range(6):
            ax = self.fig.add_axes([0.10, bottom + i * (height + gap), 0.65, height])
            slider_axes.append(ax)

        self.s_slope_let = Slider(slider_axes[5], "slope_lethal_deg",
                                  10, 60, valinit=self.init["slope_lethal_deg"])
        self.s_step_let  = Slider(slider_axes[4], "step_lethal_m",
                                  0.05, 0.50, valinit=self.init["step_lethal_m"])
        self.s_h_let     = Slider(slider_axes[3], "height_lethal_m",
                                  0.10, 1.50, valinit=self.init["height_lethal_m"])
        self.s_slope_tr  = Slider(slider_axes[2], "slope_trav_deg",
                                  0, 30, valinit=self.init["slope_trav_deg"])
        self.s_step_tr   = Slider(slider_axes[1], "step_trav_m",
                                  0.01, 0.30, valinit=self.init["step_trav_m"])
        self.s_h_tr      = Slider(slider_axes[0], "height_trav_m",
                                  0.02, 0.50, valinit=self.init["height_trav_m"])

        for s in (self.s_slope_let, self.s_step_let, self.s_h_let,
                  self.s_slope_tr, self.s_step_tr, self.s_h_tr):
            s.on_changed(self._on_slider)

        # Save button
        self.ax_save = self.fig.add_axes([0.78, 0.02, 0.18, 0.05])
        self.btn_save = Button(self.ax_save, "SAVE labels (.npz)", color="lightgreen")
        self.btn_save.on_clicked(self._on_save)

        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

    def _extent(self):
        x0 = float(self.origin[0]); y0 = float(self.origin[1])
        return (x0, x0 + self.W * self.cell, y0, y0 + self.H * self.cell)

    def _recompute(self):
        sl_let = self.s_slope_let.val
        st_let = self.s_step_let.val
        h_let  = self.s_h_let.val
        sl_tr  = self.s_slope_tr.val
        st_tr  = self.s_step_tr.val
        h_tr   = self.s_h_tr.val

        lethal = (self.slope >= sl_let) | (self.step >= st_let) | (self.h_above >= h_let)
        trav   = (self.slope <  sl_tr) & (self.step <  st_tr) & (self.h_above <  h_tr)
        lethal &= self.touched
        trav   &= self.touched & (~lethal)

        self.labels = np.full((self.H, self.W), -1, dtype=np.int8)
        self.labels[trav] = 0
        self.labels[lethal] = 1

        # Overlay RGBA
        rgba = np.zeros((self.H, self.W, 4), dtype=np.float32)
        rgba[lethal] = [1.0, 0.0, 0.0, 0.55]
        rgba[trav]   = [0.0, 1.0, 0.0, 0.45]
        self.img_label.set_data(rgba)

        n_let = int(lethal.sum()); n_tr = int(trav.sum())
        n_total = int(self.touched.sum())
        n_unk = n_total - n_let - n_tr
        self.ax_main.set_title(
            f"LETHAL {n_let:,} ({100*n_let/n_total:.1f}%)  "
            f"FREE {n_tr:,} ({100*n_tr/n_total:.1f}%)  "
            f"UNK {n_unk:,} ({100*n_unk/n_total:.1f}%)"
        )
        self.fig.canvas.draw_idle()

    def _on_slider(self, _val):
        self._recompute()

    def _on_save(self, _evt):
        self._save()

    def _on_key(self, ev):
        if ev.key == "s":
            self._save()

    def _save(self):
        Path(self.out_path).parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            self.out_path,
            labels=self.labels,
            heights=self.heights,
            touched=self.touched,
            cell_size_m=np.float32(self.cell),
            origin_xy=self.origin,
            slope_deg=self.slope,
            step_m=self.step,
            height_above_floor=self.h_above,
            # Save thresholds for reproducibility
            slope_lethal_deg=np.float32(self.s_slope_let.val),
            step_lethal_m=np.float32(self.s_step_let.val),
            height_lethal_m=np.float32(self.s_h_let.val),
            slope_trav_deg=np.float32(self.s_slope_tr.val),
            step_trav_m=np.float32(self.s_step_tr.val),
            height_trav_m=np.float32(self.s_h_tr.val),
        )
        n_let = int((self.labels == 1).sum())
        n_tr  = int((self.labels == 0).sum())
        print(f"✓ saved {self.out_path}  lethal={n_let:,}  free={n_tr:,}")
        print(f"  thresholds: slope_let={self.s_slope_let.val:.1f}  "
              f"step_let={self.s_step_let.val:.3f}  h_let={self.s_h_let.val:.2f}  "
              f"slope_tr={self.s_slope_tr.val:.1f}  step_tr={self.s_step_tr.val:.3f}  "
              f"h_tr={self.s_h_tr.val:.2f}")

    def run(self):
        plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--heights", required=True)
    ap.add_argument("--out", required=True)
    # Initial threshold values
    ap.add_argument("--slope-lethal-deg", type=float, default=35.0)
    ap.add_argument("--step-lethal-m",   type=float, default=0.20)
    ap.add_argument("--height-lethal-m", type=float, default=0.40)
    ap.add_argument("--slope-trav-deg",  type=float, default=15.0)
    ap.add_argument("--step-trav-m",     type=float, default=0.06)
    ap.add_argument("--height-trav-m",   type=float, default=0.10)
    args = ap.parse_args()

    init = dict(
        slope_lethal_deg=args.slope_lethal_deg,
        step_lethal_m=args.step_lethal_m,
        height_lethal_m=args.height_lethal_m,
        slope_trav_deg=args.slope_trav_deg,
        step_trav_m=args.step_trav_m,
        height_trav_m=args.height_trav_m,
    )
    Tuner(args.heights, args.out, init).run()


if __name__ == "__main__":
    main()
