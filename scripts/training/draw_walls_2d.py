#!/usr/bin/env python3
"""Hand-draw wall segments on a 2D top-down heightmap → MJCF collision boxes.

PolyFit auto-RANSAC overfits (catches coplanar walls across corridors, makes
47m slabs). This tool lets you trace the real walls by hand on a top-down
view of the building heightmap, then extrudes each 2D segment into a thin
vertical MJCF <geom type="box"> for sim-side collision.

Background reference: a top-down z(x,y) heightmap (from
mujoco_clean_heightmap.py) — bright = tall (walls), dark = floor. Trace along
the bright wall ridges.

Controls:
  Left-click           add a vertex to the current polyline (wall run)
  Right-click / Enter  finish current polyline, start a new one
  'u'                  undo last vertex (or last finished polyline if current empty)
  'c'                  clear everything
  'h'                  cycle wall height (2 / 3 / 4 m)
  's'                  save → MJCF snippet + segments JSON
  'q' / close window   quit (auto-saves first)

Each consecutive vertex pair in a polyline → one wall box:
  center = midpoint, z = base_z + height/2
  length = segment length (local x), thickness = --thickness (local y),
  height = current wall height (local z), yaw = atan2(dy, dx)

Output:
  <out>_walls.xml   — paste into MJCF <worldbody> (geoms named <name>_pNNN)
  <out>_walls.json  — segment list, reload with --load to continue editing

Usage:
  python3 scripts/training/draw_walls_2d.py \\
      --heightmap /tmp/ops2_nopoly_heightmap.npy \\
      --out /tmp/ops2_hand_walls \\
      --overlay-mjcf src/go2w/go2_gazebo_sim/mujoco/slam_ops2_v4_go2_clustered.xml
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

try:
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
except ImportError as e:
    sys.exit(f"matplotlib required: {e}")


def quat_from_yaw(yaw: float):
    """wxyz quaternion for a rotation about world-z by yaw."""
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def load_overlay_walls(mjcf_path: Path):
    """Parse existing polyfit/clustered wall geoms for visual reference.

    Returns list of (cx, cy, half_len, half_thick, yaw) projected to XY.
    We only approximate orientation from the quaternion's yaw component.
    """
    if not mjcf_path or not mjcf_path.exists():
        return []
    txt = mjcf_path.read_text()
    pat = re.compile(
        r'name="ops2_poly_\w+"[^>]*pos="([-\d. ]+)"[^>]*size="([-\d. ]+)"[^>]*quat="([-\d. ]+)"')
    walls = []
    for m in pat.finditer(txt):
        pos = [float(v) for v in m.group(1).split()]
        size = [float(v) for v in m.group(2).split()]
        quat = [float(v) for v in m.group(3).split()]
        qw, qx, qy, qz = quat
        # yaw of the box's wide axis (approx from quat z component)
        yaw = math.atan2(2 * (qw * qz + qx * qy),
                         1 - 2 * (qy * qy + qz * qz))
        walls.append((pos[0], pos[1], size[0], size[1], yaw))
    return walls


class WallDrawer:
    def __init__(self, args):
        self.args = args
        hm_path = Path(args.heightmap)
        if hm_path.suffix == ".npz":
            # build_float_heightmap.py format: heights/touched/origin_xy/cell_size_m
            d = np.load(hm_path)
            self.hm = d["heights"].astype(np.float32)
            if "touched" in d.files:
                # mask untouched cells to NaN so the building geometry stands out
                self.hm = np.where(d["touched"], self.hm, np.nan)
            self.res = float(d["cell_size_m"])
            self.ox, self.oy = [float(v) for v in d["origin_xy"]]
        else:
            # mujoco_clean_heightmap.py format: .npy + .json sidecar
            self.heights_meta = json.loads(hm_path.with_suffix(".json").read_text())
            self.hm = np.load(hm_path)
            self.res = float(self.heights_meta["resolution_m"])
            self.ox, self.oy = self.heights_meta["origin_xy"]
        H, W = self.hm.shape
        self.extent = [self.ox, self.ox + W * self.res,
                       self.oy, self.oy + H * self.res]

        self.wall_heights = [2.0, 3.0, 4.0]
        self.wall_height_idx = 1  # default 3 m

        # Edit-mode state
        self.edit_mode = False
        self.selected_seg = None   # (polyline_idx, seg_idx)
        self._dragging = None      # (polyline_idx, vertex_idx)

        # polylines: list of list of (x, y) world points
        self.polylines = []
        self.current = []
        if args.load and Path(args.load).exists():
            data = json.loads(Path(args.load).read_text())
            self.polylines = [[tuple(p) for p in pl] for pl in data.get("polylines", [])]
            print(f"loaded {len(self.polylines)} polylines from {args.load}")

        self._setup_plot()

    def _band_display(self):
        """Heightmap masked to the [z_lo, z_hi] band (trav_labeler-style).

        Cells whose max-z falls in the band are shown; others go NaN (grey).
        Isolating e.g. [0.3, 2.5] hides floor + ceiling so wall footprints
        stand out for tracing.
        """
        disp = np.where(np.isfinite(self.hm), self.hm, np.nan)
        in_band = (disp >= self.z_lo) & (disp <= self.z_hi)
        return np.where(in_band, disp, np.nan)

    def _setup_plot(self):
        from matplotlib.widgets import Slider

        finite = self.hm[np.isfinite(self.hm)]
        self.z_floor = float(finite.min()) if finite.size else 0.0
        self.z_ceil = float(np.nanpercentile(finite, 99)) if finite.size else 5.0
        # Default band: 0.3 m (above floor) → ceiling, isolates walls.
        self.z_lo = 0.30
        self.z_hi = self.z_ceil

        self.fig, self.ax = plt.subplots(figsize=(14, 10))
        plt.subplots_adjust(bottom=0.16)

        # Magma cmap clipped to band for max wall contrast.
        self.img = self.ax.imshow(
            self._band_display(), origin="lower", extent=self.extent,
            cmap="magma", aspect="equal", interpolation="nearest",
            vmin=self.z_lo, vmax=self.z_hi)
        self.fig.colorbar(self.img, ax=self.ax, label="elevation z (m)")

        # Overlay existing polyfit walls (reference, semi-transparent red)
        for (cx, cy, hl, ht, yaw) in load_overlay_walls(
                Path(self.args.overlay_mjcf) if self.args.overlay_mjcf else None):
            dx = hl * math.cos(yaw)
            dy = hl * math.sin(yaw)
            self.ax.plot([cx - dx, cx + dx], [cy - dy, cy + dy],
                         color="red", alpha=0.35, lw=2, zorder=2)

        self.ax.set_title(self._title())
        self.ax.set_xlabel("world x (m)")
        self.ax.set_ylabel("world y (m)")

        # z-band sliders (trav_labeler style)
        ax_lo = self.fig.add_axes([0.15, 0.06, 0.6, 0.025])
        ax_hi = self.fig.add_axes([0.15, 0.02, 0.6, 0.025])
        self.s_lo = Slider(ax_lo, "z_lo", self.z_floor, self.z_ceil,
                           valinit=self.z_lo, color="orange")
        self.s_hi = Slider(ax_hi, "z_hi", self.z_floor, self.z_ceil,
                           valinit=self.z_hi, color="orange")
        self.s_lo.on_changed(self._on_band)
        self.s_hi.on_changed(self._on_band)

        self._artist_lines = []
        self._redraw_polylines()

        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("close_event", self._on_close)

    def _on_band(self, _val):
        self.z_lo = float(self.s_lo.val)
        self.z_hi = float(self.s_hi.val)
        if self.z_hi <= self.z_lo:
            self.z_hi = self.z_lo + 0.05
        self.img.set_data(self._band_display())
        self.img.set_clim(self.z_lo, self.z_hi)
        self.fig.canvas.draw_idle()

    def _title(self):
        h = self.wall_heights[self.wall_height_idx]
        n_seg = sum(max(0, len(pl) - 1) for pl in self.polylines) + max(0, len(self.current) - 1)
        if self.edit_mode:
            return (f"[EDIT] walls={n_seg} h={h}m | L-drag vertex · L-click segment then "
                    f"d=delete · R-click vertex=delete · e=draw mode · s save · q quit")
        return (f"[DRAW] walls={n_seg} h={h}m thick={self.args.thickness}m | "
                f"L add · R/Enter new run · u undo · e=EDIT mode · h height · s save · q quit")

    def _redraw_polylines(self):
        for ln in self._artist_lines:
            ln.remove()
        self._artist_lines = []
        # Finished polylines (cyan in draw mode, green in edit mode)
        base_color = "lime" if self.edit_mode else "cyan"
        for pi, pl in enumerate(self.polylines):
            if len(pl) >= 1:
                xs = [p[0] for p in pl]; ys = [p[1] for p in pl]
                ln, = self.ax.plot(xs, ys, "-o", color=base_color, lw=2, ms=4, zorder=5)
                self._artist_lines.append(ln)
        if self.current:
            xs = [p[0] for p in self.current]; ys = [p[1] for p in self.current]
            ln, = self.ax.plot(xs, ys, "-o", color="yellow", lw=2, ms=5, zorder=6)
            self._artist_lines.append(ln)
        # Highlight selected segment (magenta thick)
        if self.selected_seg is not None:
            pi, si = self.selected_seg
            if pi < len(self.polylines) and si + 1 < len(self.polylines[pi]):
                p1 = self.polylines[pi][si]; p2 = self.polylines[pi][si + 1]
                ln, = self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]],
                                   "-", color="magenta", lw=4, zorder=7)
                self._artist_lines.append(ln)
        self.ax.set_title(self._title())
        self.fig.canvas.draw_idle()

    def _grab_radius(self):
        """Pick radius in world meters, scaled to current view."""
        xlim = self.ax.get_xlim()
        return abs(xlim[1] - xlim[0]) * 0.012  # ~1.2% of view width

    def _nearest_vertex(self, x, y):
        """Return (polyline_idx, vertex_idx, dist) of nearest vertex."""
        best = (None, None, 1e9)
        for pi, pl in enumerate(self.polylines):
            for vi, (px, py) in enumerate(pl):
                d = math.hypot(px - x, py - y)
                if d < best[2]:
                    best = (pi, vi, d)
        return best

    def _nearest_segment(self, x, y):
        """Return (polyline_idx, seg_idx, dist) of nearest segment (point-to-line)."""
        best = (None, None, 1e9)
        for pi, pl in enumerate(self.polylines):
            for si in range(len(pl) - 1):
                ax_, ay_ = pl[si]; bx_, by_ = pl[si + 1]
                dx, dy = bx_ - ax_, by_ - ay_
                L2 = dx * dx + dy * dy
                if L2 < 1e-9:
                    continue
                t = max(0.0, min(1.0, ((x - ax_) * dx + (y - ay_) * dy) / L2))
                projx, projy = ax_ + t * dx, ay_ + t * dy
                d = math.hypot(projx - x, projy - y)
                if d < best[2]:
                    best = (pi, si, d)
        return best

    def _on_click(self, event):
        if event.inaxes != self.ax:
            return
        if self.edit_mode:
            x, y = float(event.xdata), float(event.ydata)
            r = self._grab_radius()
            if event.button == 1:  # left: grab nearest vertex to drag
                pi, vi, d = self._nearest_vertex(x, y)
                if pi is not None and d < r:
                    self._dragging = (pi, vi)
                else:
                    # not near a vertex → select nearest segment
                    pi, si, d = self._nearest_segment(x, y)
                    self.selected_seg = (pi, si) if (pi is not None and d < r) else None
                    self._redraw_polylines()
            elif event.button == 3:  # right: delete nearest vertex
                pi, vi, d = self._nearest_vertex(x, y)
                if pi is not None and d < r:
                    self._delete_vertex(pi, vi)
            return
        # draw mode
        if event.button == 1:
            self.current.append((float(event.xdata), float(event.ydata)))
            self._redraw_polylines()
        elif event.button == 3:
            self._finish_current()

    def _on_motion(self, event):
        if not self.edit_mode or self._dragging is None or event.inaxes != self.ax:
            return
        pi, vi = self._dragging
        self.polylines[pi][vi] = (float(event.xdata), float(event.ydata))
        self._redraw_polylines()

    def _on_release(self, event):
        self._dragging = None

    def _delete_vertex(self, pi, vi):
        pl = self.polylines[pi]
        pl.pop(vi)
        if len(pl) < 2:
            self.polylines.pop(pi)  # too short to be a wall
        self.selected_seg = None
        self._redraw_polylines()
        print(f"deleted vertex; {sum(max(0,len(p)-1) for p in self.polylines)} segments left")

    def _delete_selected_segment(self):
        if self.selected_seg is None:
            return
        pi, si = self.selected_seg
        pl = self.polylines[pi]
        # Split polyline at the segment: drop the edge si→si+1, keep both halves.
        left = pl[:si + 1]
        right = pl[si + 1:]
        new = []
        if len(left) >= 2:
            new.append(left)
        if len(right) >= 2:
            new.append(right)
        self.polylines = self.polylines[:pi] + new + self.polylines[pi + 1:]
        self.selected_seg = None
        self._redraw_polylines()
        print(f"deleted segment; {sum(max(0,len(p)-1) for p in self.polylines)} segments left")

    def _finish_current(self):
        if len(self.current) >= 2:
            self.polylines.append(self.current)
            print(f"finished polyline: {len(self.current)} pts "
                  f"({len(self.current)-1} segments)")
        self.current = []
        self._redraw_polylines()

    def _on_key(self, event):
        if event.key == "e":
            self.edit_mode = not self.edit_mode
            self.selected_seg = None
            self._dragging = None
            print(f"mode → {'EDIT (drag vertex / select+del segment)' if self.edit_mode else 'DRAW'}")
            self._redraw_polylines()
        elif event.key in ("d", "delete", "backspace") and self.edit_mode:
            self._delete_selected_segment()
        elif event.key == "enter":
            self._finish_current()
        elif event.key == "u":
            if self.current:
                self.current.pop()
            elif self.polylines:
                self.polylines.pop()
            self._redraw_polylines()
        elif event.key == "c":
            self.current = []
            self.polylines = []
            self.selected_seg = None
            self._redraw_polylines()
        elif event.key == "h":
            self.wall_height_idx = (self.wall_height_idx + 1) % len(self.wall_heights)
            self._redraw_polylines()
        elif event.key == "s":
            self.save()
        elif event.key == "q":
            plt.close(self.fig)

    def _on_close(self, event):
        # Guard against clobbering a good save with an empty canvas: if there's
        # nothing drawn (e.g. a load failed and the window was just closed),
        # do NOT auto-save over the file. This prevented the 2026-05-20
        # "线段全没了" data loss.
        if not self.polylines and not self.current:
            print("close: canvas empty — NOT saving (kept existing file).")
            return
        self.save()

    def save(self):
        self._finish_current()
        if not self.polylines:
            print("save: nothing to save (no polylines).")
            return
        out = Path(self.args.out)
        out.parent.mkdir(parents=True, exist_ok=True)

        # segments JSON (reloadable)
        seg_json = out.with_name(out.name + "_walls.json")
        seg_json.write_text(json.dumps({
            "polylines": self.polylines,
            "wall_height_m": self.wall_heights[self.wall_height_idx],
            "thickness_m": self.args.thickness,
            "base_z_m": self.args.base_z,
        }, indent=2))

        # MJCF snippet
        h = self.wall_heights[self.wall_height_idx]
        thick = self.args.thickness
        base_z = self.args.base_z
        lines = ['    <!-- hand-drawn walls (draw_walls_2d.py) -->']
        idx = 0
        for pl in self.polylines:
            for i in range(len(pl) - 1):
                x1, y1 = pl[i]; x2, y2 = pl[i + 1]
                dx, dy = x2 - x1, y2 - y1
                length = math.hypot(dx, dy)
                if length < 1e-3:
                    continue
                cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                cz = base_z + h / 2.0
                yaw = math.atan2(dy, dx)
                qw, qx, qy, qz = quat_from_yaw(yaw)
                lines.append(
                    f'    <geom name="{self.args.name}_p{idx:03d}" type="box" '
                    f'pos="{cx:.4f} {cy:.4f} {cz:.4f}" '
                    f'size="{length/2.0:.4f} {thick/2.0:.4f} {h/2.0:.4f}" '
                    f'quat="{qw:.6f} {qx:.6f} {qy:.6f} {qz:.6f}" '
                    f'rgba="0.65 0.6 0.55 0.6" contype="1" conaffinity="1" '
                    f'condim="3" friction="0.8 0.02 0.01"/>')
                idx += 1
        xml = out.with_name(out.name + "_walls.xml")
        xml.write_text("\n".join(lines) + "\n")
        print(f"\n✓ saved {idx} wall boxes")
        print(f"  MJCF:     {xml}")
        print(f"  segments: {seg_json}  (reload with --load {seg_json})")

    def run(self):
        plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--heightmap", required=True,
                    help="top-down heightmap .npy (from mujoco_clean_heightmap.py)")
    ap.add_argument("--out", required=True, help="output path prefix")
    ap.add_argument("--load", default="", help="reload a previously-saved _walls.json")
    ap.add_argument("--overlay-mjcf", default="",
                    help="MJCF whose ops2_poly_* walls to show as red reference")
    ap.add_argument("--thickness", type=float, default=0.10, help="wall thickness (m)")
    ap.add_argument("--base-z", type=float, default=0.0, help="wall base z (m)")
    ap.add_argument("--name", default="ops2_hand", help="geom name prefix")
    args = ap.parse_args()

    drawer = WallDrawer(args)
    drawer.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
