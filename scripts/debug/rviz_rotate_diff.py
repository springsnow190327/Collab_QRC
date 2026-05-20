#!/usr/bin/env python3
"""Capture RViz before/after a programmatic rotation, stitch side-by-side.

For diagnosing pose-related trav-grid smear: we want to see the OccupancyGrid
state at robot-yaw=0 vs robot-yaw=90°, side-by-side. The smear (if any) shows
up as cells that move/appear/grow between the two frames.

Pipeline:
  1. Find the RViz main window via xwininfo + grab its bbox.
  2. PIL.ImageGrab the bbox → before.png.
  3. ros2 topic pub /robot/cmd_vel_legged at 0.7 rad/s yaw for `--rotate-sec`
     (default 4 s, gives ~160° rotation).
  4. Wait `--settle-sec` (default 3 s) for filter chain output to settle.
  5. ImageGrab again → after.png.
  6. ImageMagick convert -append (vertical) or +append (horizontal) → diff.png.
  7. Print absolute path so Read tool can load it.

Output: /tmp/rviz_diff_<timestamp>.png

Usage:
  python3 scripts/debug/rviz_rotate_diff.py
  python3 scripts/debug/rviz_rotate_diff.py --rotate-sec 6 --settle-sec 5
  python3 scripts/debug/rviz_rotate_diff.py --no-rotate  # just before/after with same view
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import time
from pathlib import Path
from PIL import Image, ImageGrab, ImageDraw, ImageFont


def find_rviz_bbox():
    """Parse xwininfo -root -tree for the RViz main window's geometry."""
    out = subprocess.check_output(["xwininfo", "-root", "-tree"], text=True)
    # Match lines like:  0x6a00106 "...nav_test.rviz... - RViz": (...)  4980x2752+0+74  +140+128
    # The first +X+Y is offset relative to parent; second +X+Y is absolute on root.
    candidates = []
    for line in out.splitlines():
        # Filter to only RViz main window: title contains ".rviz" + size > 500x500
        if "RViz" not in line and "rviz2" not in line:
            continue
        m = re.search(
            r'0x[0-9a-fA-F]+\s+"([^"]*)"\s*:\s*\([^)]*\)\s+(\d+)x(\d+)\+(-?\d+)\+(-?\d+)\s+\+(-?\d+)\+(-?\d+)',
            line)
        if not m:
            continue
        title, w, h, x_local, y_local, x_root, y_root = m.groups()
        w, h = int(w), int(h)
        x_root, y_root = int(x_root), int(y_root)
        if w < 500 or h < 500:
            continue  # skip tiny owner / aux windows
        if ".rviz" in title.lower() or "rviz" in title.lower():
            candidates.append((title, x_root, y_root, w, h))
    if not candidates:
        raise RuntimeError("No RViz window found. Is RViz running and visible?")
    # Take the LARGEST candidate (the outer main window, not inner viewport)
    candidates.sort(key=lambda c: c[3] * c[4], reverse=True)
    title, x, y, w, h = candidates[0]
    return title, (x, y, x + w, y + h)


def capture(bbox: tuple[int, int, int, int], label: str, out_path: Path):
    """Grab the bbox and overlay a label in the top-left for clarity."""
    im = ImageGrab.grab(bbox=bbox)
    draw = ImageDraw.Draw(im)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 48)
    except OSError:
        font = ImageFont.load_default()
    # Outlined text for visibility on any background
    txt = label
    pad = 14
    box_w = draw.textlength(txt, font=font) + pad * 2
    draw.rectangle([(0, 0), (box_w, 80)], fill=(0, 0, 0))
    draw.text((pad, pad), txt, fill=(255, 255, 0), font=font)
    im.save(out_path)
    return im


def publish_yaw(rate_rad: float, seconds: float, namespace: str = "robot",
                topic: str = "cmd_vel_legged"):
    """ros2 topic pub --rate at angular yaw for `seconds` then stop."""
    full_topic = f"/{namespace}/{topic}"
    msg = (
        f'{{linear: {{x: 0.0, y: 0.0, z: 0.0}}, '
        f'angular: {{x: 0.0, y: 0.0, z: {rate_rad}}}}}'
    )
    # Spawn ros2 topic pub at 20 Hz; kill after `seconds`
    proc = subprocess.Popen(
        ["ros2", "topic", "pub", "--rate", "20",
         full_topic, "geometry_msgs/msg/Twist", msg],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(seconds)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
    # Send one zero-twist to stop
    subprocess.run(
        ["ros2", "topic", "pub", "--once", full_topic, "geometry_msgs/msg/Twist",
         '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rotate-sec", type=float, default=4.0)
    ap.add_argument("--settle-sec", type=float, default=3.0,
                    help="Wait time after rotation stops, before AFTER snapshot")
    ap.add_argument("--yaw-rate", type=float, default=0.7,
                    help="Angular velocity in rad/s (default 0.7 ≈ 40°/s)")
    ap.add_argument("--no-rotate", action="store_true",
                    help="Skip rotation; capture two snapshots back-to-back")
    ap.add_argument("--namespace", default="robot")
    ap.add_argument("--out-prefix", default="/tmp/rviz_diff")
    ap.add_argument("--layout", choices=("h", "v"), default="v",
                    help="Stitch horizontally (h) or vertically (v, default)")
    args = ap.parse_args()

    title, bbox = find_rviz_bbox()
    print(f"[capture] RViz window: {title}")
    print(f"[capture] bbox: {bbox}  size={bbox[2]-bbox[0]}×{bbox[3]-bbox[1]}")

    ts = time.strftime("%Y%m%d_%H%M%S")
    before_png = Path(f"{args.out_prefix}_before_{ts}.png")
    after_png  = Path(f"{args.out_prefix}_after_{ts}.png")
    diff_png   = Path(f"{args.out_prefix}_{ts}.png")

    print("[capture] BEFORE snapshot...")
    im_b = capture(bbox, "BEFORE rotation", before_png)

    if not args.no_rotate:
        print(f"[rotate] yaw {args.yaw_rate:.2f} rad/s for {args.rotate_sec:.1f} s "
              f"(~{args.yaw_rate * args.rotate_sec * 180 / 3.14159:.0f}° total)...")
        publish_yaw(args.yaw_rate, args.rotate_sec, namespace=args.namespace)
        print(f"[settle] waiting {args.settle_sec:.1f} s for trav grid to update...")
        time.sleep(args.settle_sec)
    else:
        time.sleep(args.settle_sec)

    print("[capture] AFTER snapshot...")
    im_a = capture(bbox, "AFTER rotation", after_png)

    # Stitch
    if args.layout == "h":
        canvas = Image.new("RGB",
                           (im_b.width + im_a.width, max(im_b.height, im_a.height)),
                           (40, 40, 40))
        canvas.paste(im_b, (0, 0))
        canvas.paste(im_a, (im_b.width, 0))
    else:
        canvas = Image.new("RGB",
                           (max(im_b.width, im_a.width), im_b.height + im_a.height),
                           (40, 40, 40))
        canvas.paste(im_b, (0, 0))
        canvas.paste(im_a, (0, im_b.height))
    # Downscale if huge (Read tool dislikes 5K-wide PNGs)
    max_dim = 2400
    if max(canvas.size) > max_dim:
        ratio = max_dim / max(canvas.size)
        canvas = canvas.resize(
            (int(canvas.width * ratio), int(canvas.height * ratio)),
            Image.LANCZOS)
    canvas.save(diff_png, optimize=True)
    print(f"[stitch] wrote {diff_png}  size={canvas.size}")
    print(f"\n  view: {diff_png.absolute()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
