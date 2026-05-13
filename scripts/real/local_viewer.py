#!/usr/bin/env python3
"""
local_viewer.py — Laptop-side matplotlib renderer for stream_map_live.sh

Left  panel  : 2D current LiDAR scan slice + pose trail  (5 Hz)
Right panel  : 3D voxblox occupancy map, third-person follow-cam  (~2 Hz)
                Camera: 30° elevation, azimuth tracks robot heading from behind.

Binary protocol:
    'C' 0x43  Current scan XY  [N:uint32][N×2 float32]
    'P' 0x50  Pose             [x y z yaw : 4×float32]
    'M' 0x4D  Map increment    [N:uint32][N×3 float32 xyz]  — new voxels only
    'T' 0x54  Path XY          [N:uint32][N×2 float32]

Usage:
    python3 local_viewer.py --fifo /tmp/go2_map_stream.fifo
    python3 local_viewer.py           # stdin (test)
"""

import sys, struct, threading, collections, math, time, argparse
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D   # registers 3d projection

_ap = argparse.ArgumentParser()
_ap.add_argument('--fifo', default=None)
_args = _ap.parse_args()
FIFO_PATH = _args.fifo

# ── Tunables ──────────────────────────────────────────────────────────────────
MAP_VOX_RES   = 0.15      # must match voxblox / onboard_projector
MAX_TRAIL     = 8_000
SCAN_WIN      = 12.0      # ±m around robot for 2D scan panel
VIEW_RADIUS   = 6.0       # ±m around robot to render in 3D panel
# Camera offset: 1.5 m above robot, looking down 30° (elev=30 in matplotlib)
CAM_Z_ABOVE   = 2.0       # Z above robot body in view frustum
CAM_Z_BELOW   = 0.5       # Z below robot body (see floor context)
_CAM_Z_RANGE  = CAM_Z_ABOVE + CAM_Z_BELOW  # = 2.5 m total Z window
RENDER_MAX    = 12_000    # max voxels passed to scatter (subsample if more)
FPS           = 5
MAP_UPDATE_N  = 3         # update 3D scatter every N animation frames (~1.7 Hz)

# ── Shared state ──────────────────────────────────────────────────────────────
_lock = threading.Lock()
_curr_xy  = np.empty((0, 2), dtype=np.float32)
_path_xy  = np.empty((0, 2), dtype=np.float32)
_trail_x  = collections.deque(maxlen=MAX_TRAIL)
_trail_y  = collections.deque(maxlen=MAX_TRAIL)
_pose_x   = 0.0;  _pose_y = 0.0;  _pose_z = 0.3;  _pose_yaw = 0.0
_running   = True
_connected = False

# Incremental 3D voxblox map: dict (ix,iy,iz) → True for dedup;
# _map_xyz_cache is rebuilt (lazily) from the dict when _map_dirty.
_map_cells:    dict = {}
_map_xyz_cache: np.ndarray = np.empty((0, 3), dtype=np.float32)
_map_dirty = False

# ── Protocol helpers ──────────────────────────────────────────────────────────
def _read_exact(stream, n):
    buf = bytearray(n)
    mv  = memoryview(buf)
    got = 0
    try:
        while got < n:
            c = stream.readinto(mv[got:])
            if not c:
                return None
            got += c
    except OSError:
        return None
    return bytes(buf)

def _parse_cloud(payload):
    global _curr_xy
    if len(payload) < 4:
        return
    N = struct.unpack_from('<I', payload, 0)[0]
    if N == 0:
        return
    xy = np.frombuffer(payload, dtype=np.float32, offset=4).reshape(N, 2).copy()
    with _lock:
        _curr_xy = xy

def _parse_pose(payload):
    global _pose_x, _pose_y, _pose_z, _pose_yaw
    if len(payload) < 16:
        return
    x, y, z, yaw = struct.unpack_from('<ffff', payload, 0)
    with _lock:
        _pose_x, _pose_y, _pose_z, _pose_yaw = x, y, z, yaw
        _trail_x.append(x)
        _trail_y.append(y)

def _parse_map_increment(payload):
    global _map_dirty
    if len(payload) < 4:
        return
    N = struct.unpack_from('<I', payload, 0)[0]
    if N == 0:
        return
    xyz = np.frombuffer(payload, dtype=np.float32, offset=4).reshape(N, 3)
    ix  = np.floor(xyz[:,0] / MAP_VOX_RES).astype(np.int32)
    iy  = np.floor(xyz[:,1] / MAP_VOX_RES).astype(np.int32)
    iz  = np.floor(xyz[:,2] / MAP_VOX_RES).astype(np.int32)
    added = False
    with _lock:
        for i in range(N):
            k = (int(ix[i]), int(iy[i]), int(iz[i]))
            if k not in _map_cells:
                _map_cells[k] = True
                added = True
        if added:
            _map_dirty = True

def _parse_path(payload):
    global _path_xy
    if len(payload) < 4:
        return
    N = struct.unpack_from('<I', payload, 0)[0]
    xy = (np.frombuffer(payload, dtype=np.float32, offset=4).reshape(N,2).copy()
          if N else np.empty((0,2), dtype=np.float32))
    with _lock:
        _path_xy = xy

# ── Reader thread ─────────────────────────────────────────────────────────────
def _reader_thread():
    global _running, _connected
    fc = 0
    while _running:
        if FIFO_PATH:
            print('[viewer] waiting for connection…', file=sys.stderr, flush=True)
            try:
                stream = open(FIFO_PATH, 'rb')
            except OSError as e:
                print(f'[viewer] FIFO error: {e}', file=sys.stderr, flush=True)
                time.sleep(1)
                continue
            with _lock:
                _connected = True
            print('[viewer] connected', file=sys.stderr, flush=True)
        else:
            stream = sys.stdin.buffer
            with _lock:
                _connected = True

        while _running:
            hdr = _read_exact(stream, 5)
            if hdr is None:
                break
            tb  = hdr[0]
            plen = struct.unpack_from('<I', hdr, 1)[0]
            if plen > 8_000_000:
                _read_exact(stream, plen)
                continue
            payload = _read_exact(stream, plen)
            if payload is None:
                break
            if   tb == ord('C'): _parse_cloud(payload)
            elif tb == ord('P'): _parse_pose(payload)
            elif tb == ord('M'): _parse_map_increment(payload)
            elif tb == ord('T'): _parse_path(payload)
            fc += 1
            if fc == 1:
                print('[viewer] first frame', file=sys.stderr, flush=True)
            elif fc % 200 == 0:
                print(f'[viewer] {fc} frames  cells={len(_map_cells)}',
                      file=sys.stderr, flush=True)

        try:
            stream.close()
        except OSError:
            pass
        with _lock:
            _connected = False
        if FIFO_PATH:
            print('[viewer] disconnected', file=sys.stderr, flush=True)
        else:
            _running = False

threading.Thread(target=_reader_thread, daemon=True).start()

# ── Map cache rebuild (called from animation thread, under _lock) ──────────────
def _maybe_rebuild_map_cache():
    global _map_xyz_cache, _map_dirty
    if not _map_dirty or not _map_cells:
        return
    _map_dirty = False
    half = MAP_VOX_RES * 0.5
    keys = np.array(list(_map_cells.keys()), dtype=np.float32)  # (K,3)
    _map_xyz_cache = keys * MAP_VOX_RES + half

# ── Figure ────────────────────────────────────────────────────────────────────
BG          = '#111111'
FG          = '#cccccc'
SCAN_COLOR  = '#00ccff'
TRAIL_COLOR = '#ff7700'
POSE_COLOR  = '#ffff00'
PATH_COLOR  = '#ff44ff'
VOX_COLOR   = '#4477aa'

fig = plt.figure(figsize=(15, 7), facecolor=BG)
ax_scan = fig.add_subplot(1, 2, 1)
ax_map  = fig.add_subplot(1, 2, 2, projection='3d')

# 2D scan panel styling
ax_scan.set_facecolor(BG)
ax_scan.tick_params(colors='#666666', labelsize=7)
for sp in ax_scan.spines.values():
    sp.set_color('#333333')
ax_scan.set_aspect('equal', adjustable='datalim')
ax_scan.set_title('Current scan + trail', color=FG, fontsize=9, pad=4)

# 3D map panel styling
ax_map.set_facecolor(BG)
ax_map.xaxis.pane.fill = False
ax_map.yaxis.pane.fill = False
ax_map.zaxis.pane.fill = False
ax_map.xaxis.pane.set_edgecolor('#222222')
ax_map.yaxis.pane.set_edgecolor('#222222')
ax_map.zaxis.pane.set_edgecolor('#222222')
ax_map.tick_params(colors='#555555', labelsize=6)
ax_map.set_xlabel('X (map)', color='#888888', fontsize=7, labelpad=-4)
ax_map.set_ylabel('Y (map)', color='#888888', fontsize=7, labelpad=-4)
ax_map.set_zlabel('Z (up)',  color='#888888', fontsize=7, labelpad=-4)
ax_map.set_title('Voxblox 3D  (follow-cam)', color=FG, fontsize=9, pad=4)

# World-frame alignment: pin box_aspect to data range so 1 m in X = 1 m in Y =
# 1 m in Z on screen.  Without this, matplotlib stretches each axis to fill a
# unit cube, distorting heights vs. ground extent (Z would appear ~6× too tall).
ax_map.set_box_aspect((2 * VIEW_RADIUS, 2 * VIEW_RADIUS, _CAM_Z_RANGE))

_title = fig.suptitle('Go2 live map  ○ connecting…', color=FG, fontsize=11)

# Lock the 3D view: prevent matplotlib's interactive rotate/pan from
# competing with our follow-cam view_init updates each frame.
ax_map.mouse_init = lambda *a, **kw: None
ax_map.disable_mouse_rotation()

# ── 2D scan artists ───────────────────────────────────────────────────────────
scan_scatter = ax_scan.scatter([], [], s=1.2, c=SCAN_COLOR, alpha=0.45,
                               linewidths=0, rasterized=True)
trail_s, = ax_scan.plot([], [], '-', color=TRAIL_COLOR, lw=0.8, alpha=0.7)
path_s,  = ax_scan.plot([], [], '--', color=PATH_COLOR,  lw=1.4, alpha=0.85, zorder=4)
_arrow   = [None]

# ── 3D map artists ────────────────────────────────────────────────────────────
map_scat   = ax_map.scatter([], [], [], s=3, c=VOX_COLOR, alpha=0.35,
                            depthshade=False, linewidths=0)
trail_m,   = ax_map.plot([], [], [], '-',  color=TRAIL_COLOR, lw=1.0, alpha=0.8)
path_m,    = ax_map.plot([], [], [], '--', color=PATH_COLOR,  lw=1.6, alpha=0.9)
robot_m,   = ax_map.plot([], [], [], 'o',  color=POSE_COLOR,  ms=7, zorder=5)
# Heading arrow in 3D (a short quiver)
_quiver    = [None]

cell_text = ax_map.text2D(0.02, 0.02, '', transform=ax_map.transAxes,
                          color='#555555', fontsize=7)

coord_text = fig.text(0.5, 0.004, 'Click to read (x,y)  |  frame: map',
                      ha='center', color='#555555', fontsize=8)

def _on_click(ev):
    if ev.inaxes == ax_scan and ev.xdata is not None:
        coord_text.set_text(f'  ({ev.xdata:.3f}, {ev.ydata:.3f})  frame: map')
        fig.canvas.draw_idle()
fig.canvas.mpl_connect('button_press_event', _on_click)

# ── Animation ─────────────────────────────────────────────────────────────────
_frame_n = [0]

def _update(_fn):
    fn = _frame_n[0]
    _frame_n[0] += 1

    with _lock:
        curr  = _curr_xy.copy() if len(_curr_xy) > 0 else None
        path  = _path_xy.copy() if len(_path_xy) > 0 else None
        tx    = list(_trail_x);  ty = list(_trail_y)
        px, py, pz, pyaw = _pose_x, _pose_y, _pose_z, _pose_yaw
        live  = _connected
        nc    = len(_map_cells)
        # Rebuild cache if new voxels arrived
        _maybe_rebuild_map_cache()
        vox_all = _map_xyz_cache.copy() if len(_map_xyz_cache) > 0 else None

    # Title
    _title.set_text(f'Go2 live map  {"● LIVE" if live else "○ reconnecting…"}')
    _title.set_color('#00ff88' if live else '#ff8800')

    # ── Left: 2D scan ─────────────────────────────────────────────────────────
    if curr is not None and len(curr) > 0:
        scan_scatter.set_offsets(curr)
        ax_scan.set_xlim(px - SCAN_WIN, px + SCAN_WIN)
        ax_scan.set_ylim(py - SCAN_WIN, py + SCAN_WIN)
    if tx:
        trail_s.set_data(tx, ty)
    if _arrow[0] is not None:
        try: _arrow[0].remove()
        except Exception: pass
        _arrow[0] = None
    _arrow[0] = ax_scan.annotate('',
        xy=(px + 0.6*math.cos(pyaw), py + 0.6*math.sin(pyaw)), xytext=(px, py),
        arrowprops=dict(arrowstyle='->', color=POSE_COLOR, lw=1.5), zorder=6)
    path_s.set_data(*(zip(*path.tolist()) if path is not None and len(path) >= 2
                      else ([], [])))

    # ── Right: 3D map — update at ~1.7 Hz ────────────────────────────────────
    if fn % MAP_UPDATE_N == 0:
        if vox_all is not None and len(vox_all) > 0:
            # Crop to robot neighbourhood for rendering
            R = VIEW_RADIUS * 1.5
            dx = np.abs(vox_all[:,0] - px)
            dy = np.abs(vox_all[:,1] - py)
            vox = vox_all[(dx < R) & (dy < R)]
            # Subsample if still too large
            if len(vox) > RENDER_MAX:
                idx = np.random.choice(len(vox), RENDER_MAX, replace=False)
                vox = vox[idx]
            if len(vox) > 0:
                map_scat._offsets3d = (vox[:,0], vox[:,1], vox[:,2])

        # Trail at robot z
        if len(tx) >= 2:
            tz = [pz] * len(tx)
            trail_m.set_data(tx, ty)
            trail_m.set_3d_properties(tz)
        robot_m.set_data([px], [py])
        robot_m.set_3d_properties([pz])

        # Path at robot z + small offset so it floats above floor
        if path is not None and len(path) >= 2:
            pz_path = pz + 0.1
            path_m.set_data(path[:,0], path[:,1])
            path_m.set_3d_properties([pz_path] * len(path))
        else:
            path_m.set_data([], [])
            path_m.set_3d_properties([])

        # Heading indicator: quiver from robot in facing direction
        if _quiver[0] is not None:
            try: _quiver[0].remove()
            except Exception: pass
            _quiver[0] = None
        _quiver[0] = ax_map.quiver(
            px, py, pz + 0.05,
            math.cos(pyaw)*0.8, math.sin(pyaw)*0.8, 0,
            color=POSE_COLOR, arrow_length_ratio=0.4, linewidth=1.5)

        # Robot-centred view bounds — all robot-relative so robot is always centred
        ax_map.set_xlim(px - VIEW_RADIUS, px + VIEW_RADIUS)
        ax_map.set_ylim(py - VIEW_RADIUS, py + VIEW_RADIUS)
        ax_map.set_zlim(pz - CAM_Z_BELOW, pz + CAM_Z_ABOVE)
        # Refresh isotropic box_aspect so 1 m in X = 1 m in Z on screen
        ax_map.set_box_aspect((2 * VIEW_RADIUS, 2 * VIEW_RADIUS, _CAM_Z_RANGE))

        # Third-person follow-cam: fixed relative to robot body
        #   elev=30 → camera 30° above horizontal, line-of-sight looks down 30°
        #   azim = yaw+180° → camera sits behind robot facing forward
        azim_deg = math.degrees(pyaw) + 180.0
        ax_map.view_init(elev=30, azim=azim_deg)

        cell_text.set_text(f'{nc:,} voxels')

ani = animation.FuncAnimation(fig, _update, interval=1000//FPS,
                              blit=False, cache_frame_data=False)

plt.tight_layout(rect=[0, 0.02, 1, 0.96])
try:
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    _running = False
    print('[viewer] closed', file=sys.stderr, flush=True)
