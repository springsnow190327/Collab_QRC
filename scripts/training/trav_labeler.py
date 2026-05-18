#!/usr/bin/env python3
"""trav_labeler.py — 2D paint-brush + z-band slice + line tool for ops2 trav labels.

Loads a float32 heightmap (from build_float_heightmap.py / polish_trav_labels.py)
and lets you paint each cell as:

    LMB drag     → 1 (lethal)        - red overlay
    RMB drag     → 0 (free)          - green overlay
    MMB drag     → -1 (unlabelled)   - clears

Shift-click straight-line tool:
    Shift+LMB    → set start (or end) of LETHAL line
    Shift+RMB    → set start (or end) of FREE   line
    Esc          → cancel pending line

Z-band slice (peek under roofs):
    Optional --mesh loads the source mesh. Two sliders set [z_lo, z_hi].
    Key 'b' toggles background:
        all   : normal max-z heightmap (default)
        band  : density of mesh verts in [z_lo, z_hi] only

Keys:
    + / -        brush radius
    s            save labels
    u            undo last stroke
    p            toggle PolyFit / Sonata overlay
    b            toggle band view
    q            quit

Usage:
    python3 trav_labeler.py \\
        --heights bags/meshes/.../ops2_v4_polished_labels.npz \\
        --labels-out bags/meshes/.../ops2_v4_final_labels.npz \\
        --mesh bags/meshes/ops2_cuda/scans_v4_aligned.obj \\
        --polyfit bags/meshes/.../ops2_polyfit_walls.xml
"""
import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("TkAgg", force=True)
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import matplotlib.patches as mpatches


UNLBL = -1
FREE = 0
LETHAL = 1


def bresenham_cells(y0, x0, y1, x1):
    """Yield (y, x) cells on the straight line from (y0,x0) to (y1,x1)."""
    dy = abs(y1 - y0); dx = abs(x1 - x0)
    sy = 1 if y0 < y1 else -1
    sx = 1 if x0 < x1 else -1
    err = dx - dy
    y, x = y0, x0
    while True:
        yield y, x
        if y == y1 and x == x1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy; x += sx
        if e2 < dx:
            err += dx; y += sy


class Labeler:
    def __init__(self, heights_npz, labels_out, mesh_path,
                 polyfit_xml, sonata_summary, sonata_meshes_dir):
        d = np.load(heights_npz, allow_pickle=False)
        self.heights = d["heights"]
        self.touched = d["touched"]
        self.cell = float(d["cell_size_m"])
        self.origin = d["origin_xy"].astype(np.float32)
        self.H, self.W = self.heights.shape
        self.labels = np.full((self.H, self.W), UNLBL, dtype=np.int8)
        self.labels_out = labels_out

        # Resume — try loading from labels_out first, else from input npz
        loaded_from = None
        if Path(labels_out).exists():
            try:
                prev = np.load(labels_out, allow_pickle=False)
                self.labels = prev["labels"].astype(np.int8)
                loaded_from = labels_out
            except Exception:
                pass
        if loaded_from is None and "labels" in d.files:
            self.labels = d["labels"].astype(np.int8)
            loaded_from = heights_npz
        if loaded_from is not None:
            print(f"loaded labels from {loaded_from}  "
                  f"lethal={int((self.labels == LETHAL).sum()):,}  "
                  f"free={int((self.labels == FREE).sum()):,}")

        # State
        self.brush_radius = 2
        self.painting_btn = 0
        self.show_overlays = True
        self.line_pending = None  # (iy, ix, label)
        self.undo_stack = []

        # Z-band: keep all mesh verts in memory for fast filter
        self.verts = None
        if mesh_path:
            try:
                import open3d as o3d
                m = o3d.io.read_triangle_mesh(mesh_path)
                self.verts = np.asarray(m.vertices, dtype=np.float32)
                print(f"loaded mesh verts {len(self.verts):,}")
            except Exception as e:
                print(f"WARN: mesh load failed: {e}")

        # Polyfit + Sonata overlays
        self.wall_boxes = self._load_polyfit(polyfit_xml) if polyfit_xml else []
        self.sonata_aabbs = self._load_sonata(
            sonata_summary, sonata_meshes_dir
        ) if sonata_summary else []

        self._build_fig()

    def _load_polyfit(self, xml_path):
        import re
        try:
            txt = Path(xml_path).read_text()
        except Exception:
            return []
        boxes = []
        for m in re.finditer(
            r'pos="([-\d. ]+)"\s+size="([-\d. ]+)"\s+quat="([-\d. ]+)"', txt
        ):
            pos = list(map(float, m.group(1).split()))
            sz = list(map(float, m.group(2).split()))
            boxes.append((pos[0], pos[1], sz[0], sz[1]))
        return boxes

    def _load_sonata(self, summary_path, meshes_dir):
        try:
            s = json.loads(Path(summary_path).read_text())
        except Exception:
            return []
        import open3d as o3d
        aabbs = []
        for inst in s["instances"]:
            op = Path(meshes_dir) / f"{inst['name']}.obj"
            if not op.exists():
                continue
            m = o3d.io.read_triangle_mesh(str(op))
            v = np.asarray(m.vertices)
            if len(v) == 0:
                continue
            aabbs.append((v.min(0), v.max(0), inst["name"]))
        return aabbs

    def _build_fig(self):
        self.fig = plt.figure(figsize=(18, 10))
        self.ax = self.fig.add_axes([0.05, 0.20, 0.92, 0.75])

        # Layer 1: BASE heightmap, max-z within slider-controlled z range.
        # We build it on demand in _update_band(); start with full-range
        # max-z from the pre-baked heightmap as a sane initial display.
        disp = self.heights.copy(); disp[~self.touched] = np.nan
        self.img_height = self.ax.imshow(
            disp, cmap="viridis", origin="lower",
            extent=self._extent(), interpolation="nearest",
        )
        # Layer 2: semi-transparent labels on top
        self.img_overlay = self.ax.imshow(
            self._labels_rgba(), origin="lower",
            extent=self._extent(), interpolation="nearest",
        )
        # Pending-line preview
        self.line_artist, = self.ax.plot([], [], "y-", lw=2.0, alpha=0.8)
        # Whether sliders control the base (off until first slider move if
        # no mesh; on by default if mesh loaded so band-base shows up)
        self.show_band = self.verts is not None

        # Overlay artists
        self.overlay_artists = []
        self._draw_overlays()

        self.ax.set_xlabel("x [m]"); self.ax.set_ylabel("y [m]")
        self.ax.set_aspect("equal")
        self.ax.set_title(self._status_text())

        # Z-band sliders (bottom)
        z_lo_init = 0.0
        z_hi_init = 0.5
        self.ax_zlo = self.fig.add_axes([0.10, 0.10, 0.75, 0.02])
        self.ax_zhi = self.fig.add_axes([0.10, 0.06, 0.75, 0.02])
        z_arr = self.verts[:, 2] if self.verts is not None else np.array([0.0, 5.0])
        z_min_v = float(np.percentile(z_arr, 0.5)) if self.verts is not None else 0.0
        z_max_v = float(np.percentile(z_arr, 99.5)) if self.verts is not None else 5.0
        # Default: full range (matches old max-z view at startup). User
        # tightens to peek at a specific layer (e.g. [0.10, 0.80] for racks).
        z_lo_init = z_min_v
        z_hi_init = z_max_v
        self.s_zlo = Slider(self.ax_zlo, "z_lo [m]", z_min_v, z_max_v, valinit=z_lo_init)
        self.s_zhi = Slider(self.ax_zhi, "z_hi [m]", z_min_v, z_max_v, valinit=z_hi_init)
        self.s_zlo.on_changed(self._on_band_slider)
        self.s_zhi.on_changed(self._on_band_slider)

        # Initial band paint
        self._update_band()

        # Events
        self.fig.canvas.mpl_connect("button_press_event", self._on_press)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

        print()
        print("=== controls ===")
        print("  LMB drag       : LETHAL (red) brush")
        print("  RMB drag       : FREE (green) brush")
        print("  MMB drag       : clear brush")
        print("  Shift+LMB      : straight LETHAL line (2 clicks)")
        print("  Shift+RMB      : straight FREE   line (2 clicks)")
        print("  Esc            : cancel pending line")
        print("  + / -          : brush radius (current {})".format(self.brush_radius))
        print("  z_lo / z_hi    : sliders set the z range used for the BASE")
        print("                   heightmap.  [0.10, 0.80] = ground-to-rack")
        print("                   layer only, roof becomes invisible.")
        print("  b              : toggle band-base on/off (back to full max-z)")
        print("  s              : save  |  u : undo  |  p : refs  |  q : quit")
        print()
        if self.verts is None:
            print("  ⚠ no --mesh given; z-band view will be empty")

    def _extent(self):
        x0 = float(self.origin[0]); y0 = float(self.origin[1])
        return (x0, x0 + self.W * self.cell, y0, y0 + self.H * self.cell)

    def _labels_rgba(self):
        rgba = np.zeros((self.H, self.W, 4), dtype=np.float32)
        rgba[self.labels == LETHAL] = [1.0, 0.0, 0.0, 0.55]
        rgba[self.labels == FREE]   = [0.0, 1.0, 0.0, 0.45]
        return rgba

    def _status_text(self):
        n_lethal = int((self.labels == LETHAL).sum())
        n_free = int((self.labels == FREE).sum())
        n_total = int(self.touched.sum())
        pending = f"  PENDING LINE @ ({self.line_pending[0]},{self.line_pending[1]})" \
            if self.line_pending else ""
        if hasattr(self, "s_zlo") and self.verts is not None:
            band = f"z-band [{self.s_zlo.val:.2f}, {self.s_zhi.val:.2f}]m"
            band += " ON" if self.show_band else " OFF"
        else:
            band = "no mesh"
        return (
            f"{band}  brush r={self.brush_radius}c  "
            f"lethal={n_lethal:,}  free={n_free:,}  "
            f"({100*(n_lethal+n_free)/max(1,n_total):.1f}% labelled){pending}"
        )

    def _update_band(self):
        """Rebuild the BASE heightmap as max-z within slider z range.

        Cells with no verts in [z_lo, z_hi] go transparent (NaN), so the
        user can isolate any height layer — e.g. set [0.10, 0.80] to see
        bike racks without the roof in the way.
        """
        if not self.show_band or self.verts is None:
            disp = self.heights.copy(); disp[~self.touched] = np.nan
            self.img_height.set_data(disp)
            if self.touched.any():
                self.img_height.set_clim(
                    vmin=float(self.heights[self.touched].min()),
                    vmax=float(self.heights[self.touched].max()))
            return
        z_lo = float(self.s_zlo.val)
        z_hi = float(self.s_zhi.val)
        if z_hi < z_lo:
            z_lo, z_hi = z_hi, z_lo
        in_band = (self.verts[:, 2] >= z_lo) & (self.verts[:, 2] <= z_hi)
        v = self.verts[in_band]
        # Rasterize max-z per cell
        grid = np.full((self.H, self.W), -np.inf, dtype=np.float32)
        if len(v) > 0:
            ix = ((v[:, 0] - self.origin[0]) / self.cell).astype(np.int64).clip(0, self.W - 1)
            iy = ((v[:, 1] - self.origin[1]) / self.cell).astype(np.int64).clip(0, self.H - 1)
            np.maximum.at(grid, (iy, ix), v[:, 2])
        valid = grid > -np.inf
        disp = grid.copy()
        disp[~valid] = np.nan
        self.img_height.set_data(disp)
        if valid.any():
            self.img_height.set_clim(vmin=z_lo, vmax=max(z_hi, z_lo + 0.01))

    def _draw_overlays(self):
        for a in self.overlay_artists:
            a.remove()
        self.overlay_artists.clear()
        if not self.show_overlays:
            return
        for bmin, bmax, _name in self.sonata_aabbs:
            r = mpatches.Rectangle(
                (bmin[0], bmin[1]), bmax[0] - bmin[0], bmax[1] - bmin[1],
                fill=False, edgecolor="purple", linewidth=1.0, linestyle="--",
            )
            self.ax.add_patch(r); self.overlay_artists.append(r)
        for cx, cy, hx, hy in self.wall_boxes:
            r = mpatches.Rectangle(
                (cx - hx, cy - hy), 2 * hx, 2 * hy,
                fill=False, edgecolor="white", linewidth=0.4, alpha=0.5,
            )
            self.ax.add_patch(r); self.overlay_artists.append(r)

    def _world_to_cell(self, wx, wy):
        ix = int((wx - self.origin[0]) / self.cell)
        iy = int((wy - self.origin[1]) / self.cell)
        return ix, iy

    def _cell_to_world(self, iy, ix):
        wx = self.origin[0] + (ix + 0.5) * self.cell
        wy = self.origin[1] + (iy + 0.5) * self.cell
        return wx, wy

    def _stamp(self, wx, wy, label):
        if wx is None or wy is None:
            return False
        ix, iy = self._world_to_cell(wx, wy)
        return self._stamp_cell(iy, ix, label)

    def _stamp_cell(self, iy, ix, label):
        r = self.brush_radius
        x0 = max(0, ix - r); x1 = min(self.W, ix + r + 1)
        y0 = max(0, iy - r); y1 = min(self.H, iy + r + 1)
        if x0 >= x1 or y0 >= y1:
            return False
        sub_t = self.touched[y0:y1, x0:x1]
        sub_l = self.labels[y0:y1, x0:x1]
        yy, xx = np.ogrid[y0:y1, x0:x1]
        in_brush = ((yy - iy) ** 2 + (xx - ix) ** 2 <= r * r) & sub_t
        sub_l[in_brush] = label
        self.labels[y0:y1, x0:x1] = sub_l
        return True

    def _stamp_line(self, y0, x0, y1, x1, label):
        any_touched = False
        for yy, xx in bresenham_cells(y0, x0, y1, x1):
            if self._stamp_cell(yy, xx, label):
                any_touched = True
        return any_touched

    def _snapshot(self):
        self.undo_stack.append(self.labels.copy())
        if len(self.undo_stack) > 20:
            self.undo_stack.pop(0)

    def _refresh(self):
        self.img_overlay.set_data(self._labels_rgba())
        self.ax.set_title(self._status_text())
        if self.line_pending is not None:
            wy0, wx0 = self.line_pending[0], self.line_pending[1]
            self.line_artist.set_data([wx0], [wy0])
        else:
            self.line_artist.set_data([], [])
        self.fig.canvas.draw_idle()

    # ------- mouse handlers --------
    def _on_press(self, event):
        if event.inaxes != self.ax or event.xdata is None:
            return
        shift = (event.key is not None and "shift" in event.key)
        if shift and event.button in (1, 3):
            # Line tool
            label = LETHAL if event.button == 1 else FREE
            ix, iy = self._world_to_cell(event.xdata, event.ydata)
            if self.line_pending is None:
                self.line_pending = (iy, ix, label)
                self._refresh()
            else:
                y0, x0, lbl0 = self.line_pending
                # If labels conflict (LMB then RMB), use the second
                self._snapshot()
                self._stamp_line(y0, x0, iy, ix, label)
                self.line_pending = None
                self._refresh()
            return
        # Normal brush
        self._snapshot()
        if event.button == 1:
            self.painting_btn = LETHAL
        elif event.button == 3:
            self.painting_btn = FREE
        elif event.button == 2:
            self.painting_btn = UNLBL
        else:
            return
        if self._stamp(event.xdata, event.ydata, self.painting_btn):
            self._refresh()

    def _on_release(self, _event):
        self.painting_btn = 0

    def _on_motion(self, event):
        if self.painting_btn == 0 or event.inaxes != self.ax:
            return
        if event.xdata is None:
            return
        if self._stamp(event.xdata, event.ydata, self.painting_btn):
            self._refresh()

    # ------- key handler --------
    def _on_key(self, event):
        k = event.key
        if k in ("+", "="):
            self.brush_radius = min(20, self.brush_radius + 1)
        elif k in ("-", "_"):
            self.brush_radius = max(1, self.brush_radius - 1)
        elif k == "s":
            self.save()
        elif k == "u":
            if self.undo_stack:
                self.labels = self.undo_stack.pop()
        elif k == "p":
            self.show_overlays = not self.show_overlays
            self._draw_overlays()
        elif k == "b":
            self.show_band = not self.show_band
            self._update_band()
        elif k == "escape":
            self.line_pending = None
        elif k == "q":
            plt.close(self.fig)
            return
        self._refresh()

    def _on_band_slider(self, _val):
        if self.show_band:
            self._update_band()
        self._refresh()

    def save(self):
        Path(self.labels_out).parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            self.labels_out,
            labels=self.labels,
            heights=self.heights,
            touched=self.touched,
            cell_size_m=np.float32(self.cell),
            origin_xy=self.origin,
        )
        n_lethal = int((self.labels == LETHAL).sum())
        n_free = int((self.labels == FREE).sum())
        print(f"✓ saved {self.labels_out}  "
              f"lethal={n_lethal:,}  free={n_free:,}")

    def run(self):
        plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--heights", required=True)
    ap.add_argument("--labels-out", required=True)
    ap.add_argument("--mesh", default="",
                    help="(optional) mesh .obj for z-band slicing")
    ap.add_argument("--polyfit", default="")
    ap.add_argument("--sonata-summary", default="")
    ap.add_argument("--sonata-meshes-dir", default="")
    args = ap.parse_args()

    Labeler(args.heights, args.labels_out, args.mesh,
            args.polyfit, args.sonata_summary,
            args.sonata_meshes_dir).run()


if __name__ == "__main__":
    main()
