#!/usr/bin/env python3
"""Generate LRC 2026 Confined Configuration world files.

Produces Gazebo SDF (.world) and MuJoCo MJCF (.xml) for the 6-lane
continuous L-shaped LRC 2026 confined configuration course.

Lanes: A (Ramps) -> B (Pallets) -> C (K-Rails) -> D (Stepfields)
    -> E (Pallet Climb) -> F (Stairs Up+Down)

Layout (Option B — balanced L):
    C ── D ── E ── F
    |
    B
    |
    A
    |
  ENTRY

Usage:
    python3 scripts/generate_lrc_2025_world.py          # flat
    python3 scripts/generate_lrc_2025_world.py --slope   # 15deg cross-slope
"""

import argparse
import math
import os
import random
from dataclasses import dataclass
from typing import List, Tuple, Union

# ── Constants ──────────────────────────────────────────────────────
WALL_H = 1.2        # wall height (m)
WALL_T = 0.15       # wall thickness (m)
DOOR_W = 1.0        # doorway opening width (m)

PASS_W = 1.2        # pass width (corridor width from PDF)
LANE_LEN = 3.6      # pass run length (40% shorter than original 6m)
DIV_THICK = 0.15    # divider wall thickness
SNAKE_W = 3 * PASS_W + 2 * DIV_THICK  # 3.9m total cross-pass width
HALF_LEN = LANE_LEN / 2      # 1.8
HALF_SNAKE = SNAKE_W / 2     # 1.95

SLOPE_DEG = 15.0
SLOPE_RAD = math.radians(SLOPE_DEG)

STEPFIELD_SEED = 2026

# Zig-zag pass layout (offsets from lane center, in stacking direction)
PASS_OFF = [-1.35, 0.0, 1.35]
DIV_OFF = [-0.675, 0.675]
GAP_LEN = 1.2

# Colors (RGBA)
C_WALL   = (0.60, 0.55, 0.45, 1.0)
C_RAMP   = (0.55, 0.45, 0.35, 1.0)
C_PALLET = (0.65, 0.50, 0.30, 1.0)
C_PIPE   = (0.40, 0.40, 0.40, 1.0)
C_CONCR  = (0.55, 0.55, 0.55, 1.0)
C_KRAIL  = (0.50, 0.50, 0.50, 1.0)
C_FLOOR  = (0.45, 0.40, 0.35, 1.0)
C_STEP   = (0.50, 0.50, 0.55, 1.0)   # cubic stepfield blocks
C_CLIMB  = (0.60, 0.45, 0.25, 1.0)   # pallet climb (darker wood)
C_QR     = (0.0, 0.8, 0.0, 1.0)
C_HAZ    = (1.0, 0.5, 0.0, 1.0)
C_ESTOP  = (0.9, 0.9, 0.0, 1.0)
C_EBTN   = (1.0, 0.0, 0.0, 1.0)


# ── Data classes ───────────────────────────────────────────────────
@dataclass
class Box:
    name: str
    pos: Tuple[float, float, float]
    size: Tuple[float, float, float]   # FULL size (sx, sy, sz)
    euler: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # RPY radians
    color: Tuple[float, float, float, float] = C_WALL
    mu: float = 1.0

@dataclass
class Cyl:
    name: str
    pos: Tuple[float, float, float]
    radius: float
    length: float
    euler: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    color: Tuple[float, float, float, float] = C_PIPE
    mu: float = 1.0

Geom = Union[Box, Cyl]


# ── Lane configuration ────────────────────────────────────────────
# Balanced L: 2 vertical (A, B EW) + elbow (C NS) + 3 horizontal (D, E, F NS)
LANES = {
    'A': dict(cx=0,     cy=-8.65, orient='EW', terrain='ramps'),
    'B': dict(cx=0,     cy=-3.75, orient='EW', terrain='pallets_ew'),
    'C': dict(cx=0,     cy=1.0,   orient='NS', terrain='krails'),
    'D': dict(cx=4.9,   cy=1.0,   orient='NS', terrain='stepfields'),
    'E': dict(cx=9.8,   cy=1.0,   orient='NS', terrain='pallet_climb'),
    'F': dict(cx=14.7,  cy=1.0,   orient='NS', terrain='stairs'),
}

DOORS = {
    'A': dict(south=0.0,   north=0.0,   east=None, west=None),
    'B': dict(south=0.0,   north=-1.35, east=None, west=None),
    'C': dict(south=-1.35, north=None,  east=2.3,  west=None),
    'D': dict(south=None,  north=None,  east=-0.3, west=2.3),
    'E': dict(south=None,  north=None,  east=2.3,  west=-0.3),
    'F': dict(south=None,  north=None,  east=-0.3, west=2.3),
}

# Divider gap side per lane
# EW: 'min'=west, 'max'=east.  NS: 'min'=south, 'max'=north.
DIV_GAPS = {
    'A': ('max', 'min'),
    'B': ('min', 'max'),
    'C': ('max', 'min'),
    'D': ('min', 'max'),
    'E': ('max', 'min'),
    'F': ('min', 'max'),
}

CORRIDORS = [
    dict(cx=0.0,   cy=-6.20, orient='V', w=DOOR_W, l=1.0),   # A→B
    dict(cx=-1.35, cy=-1.30, orient='V', w=DOOR_W, l=1.0),   # B→C
    dict(cx=2.45,  cy=2.3,   orient='H', w=DOOR_W, l=1.0),   # C→D (north)
    dict(cx=7.35,  cy=-0.3,  orient='H', w=DOOR_W, l=1.0),   # D→E (south)
    dict(cx=12.25, cy=2.3,   orient='H', w=DOOR_W, l=1.0),   # E→F (north)
]


# ── Builder helpers ────────────────────────────────────────────────

def _lane_extents(cx, cy, orient):
    if orient == 'EW':
        return (cx - HALF_LEN, cx + HALF_LEN,
                cy - HALF_SNAKE, cy + HALF_SNAKE)
    else:
        return (cx - HALF_SNAKE, cx + HALF_SNAKE,
                cy - HALF_LEN, cy + HALF_LEN)


def _wall_segs(name, axis, perp, a0, a1, door_pos, wh):
    segs = []
    if door_pos is not None:
        d0 = door_pos - DOOR_W / 2
        d1 = door_pos + DOOR_W / 2
        parts = []
        if d0 - a0 > 0.01:
            parts.append((f"{name}_a", a0, d0))
        if a1 - d1 > 0.01:
            parts.append((f"{name}_b", d1, a1))
    else:
        parts = [(name, a0, a1)]

    for sn, s0, s1 in parts:
        ln = s1 - s0
        mid = (s0 + s1) / 2
        if axis == 'X':
            segs.append(Box(sn, pos=(mid, perp, wh / 2),
                            size=(ln, WALL_T, wh)))
        else:
            segs.append(Box(sn, pos=(perp, mid, wh / 2),
                            size=(WALL_T, ln, wh)))
    return segs


def _slope_z(slope, orient, cx, cy, px, py):
    if not slope:
        return 0.0
    pt = 0.10
    x0, _, y0, _ = _lane_extents(cx, cy, orient)
    if orient == 'EW':
        dy = py - y0
        return dy * math.sin(SLOPE_RAD) + pt * math.cos(SLOPE_RAD)
    else:
        dx = px - x0
        return dx * math.sin(SLOPE_RAD) + pt * math.cos(SLOPE_RAD)


# ── Main builder ──────────────────────────────────────────────────

def build_all(slope: bool) -> List[Geom]:
    gs: List[Geom] = []
    wh = 2.5 if slope else WALL_H

    for lid, L in LANES.items():
        cx, cy, orient = L['cx'], L['cy'], L['orient']
        doors = DOORS[lid]
        x0, x1, y0, y1 = _lane_extents(cx, cy, orient)

        # ── outer walls ──
        for prefix, axis, perp, a_start, a_end, dkey in [
            (f"L{lid}_S", 'X', y0, x0, x1, 'south'),
            (f"L{lid}_N", 'X', y1, x0, x1, 'north'),
            (f"L{lid}_W", 'Y', x0, y0, y1, 'west'),
            (f"L{lid}_E", 'Y', x1, y0, y1, 'east'),
        ]:
            gs.extend(_wall_segs(prefix, axis, perp, a_start, a_end,
                                 doors[dkey], wh))

        # ── internal dividers ──
        for di, (d_off, gap_side) in enumerate(
                zip(DIV_OFF, DIV_GAPS[lid])):
            if orient == 'EW':
                d_perp = cy + d_off
                e_min, e_max = x0, x1
                if gap_side == 'min':
                    w_s, w_e = e_min + GAP_LEN, e_max
                else:
                    w_s, w_e = e_min, e_max - GAP_LEN
                w_len = w_e - w_s
                w_mid = (w_s + w_e) / 2
                gs.append(Box(f"L{lid}_d{di}",
                              pos=(w_mid, d_perp, wh / 2),
                              size=(w_len, DIV_THICK, wh)))
            else:
                d_perp = cx + d_off
                e_min, e_max = y0, y1
                if gap_side == 'min':
                    w_s, w_e = e_min + GAP_LEN, e_max
                else:
                    w_s, w_e = e_min, e_max - GAP_LEN
                w_len = w_e - w_s
                w_mid = (w_s + w_e) / 2
                gs.append(Box(f"L{lid}_d{di}",
                              pos=(d_perp, w_mid, wh / 2),
                              size=(DIV_THICK, w_len, wh)))

        # ── slope floor plate ──
        if slope:
            pt = 0.10
            if orient == 'EW':
                sz_x, sz_y = LANE_LEN, SNAKE_W
                z_c = HALF_SNAKE * math.sin(SLOPE_RAD) + (pt / 2) * math.cos(SLOPE_RAD)
                gs.append(Box(f"L{lid}_floor", pos=(cx, cy, z_c),
                              size=(sz_x, sz_y, pt),
                              euler=(SLOPE_RAD, 0, 0), color=C_FLOOR))
            else:
                sz_x, sz_y = SNAKE_W, LANE_LEN
                z_c = HALF_SNAKE * math.sin(SLOPE_RAD) + (pt / 2) * math.cos(SLOPE_RAD)
                gs.append(Box(f"L{lid}_floor", pos=(cx, cy, z_c),
                              size=(sz_x, sz_y, pt),
                              euler=(0, SLOPE_RAD, 0), color=C_FLOOR))

        _place_terrain(gs, lid, cx, cy, orient, L['terrain'], slope)
        _place_tasks(gs, lid, cx, cy, orient)

    # ── corridors ──
    for ci, c in enumerate(CORRIDORS):
        hw = c['w'] / 2
        if c['orient'] == 'V':
            gs.append(Box(f"cor{ci}_W",
                          pos=(c['cx'] - hw, c['cy'], wh / 2),
                          size=(WALL_T, c['l'], wh)))
            gs.append(Box(f"cor{ci}_E",
                          pos=(c['cx'] + hw, c['cy'], wh / 2),
                          size=(WALL_T, c['l'], wh)))
        else:
            gs.append(Box(f"cor{ci}_S",
                          pos=(c['cx'], c['cy'] - hw, wh / 2),
                          size=(c['l'], WALL_T, wh)))
            gs.append(Box(f"cor{ci}_N",
                          pos=(c['cx'], c['cy'] + hw, wh / 2),
                          size=(c['l'], WALL_T, wh)))

    # ── START room (south of Lane A) ──
    # Lane A is EW: x in [-1.8, 1.8], south wall at y = cy_A - HALF_SNAKE
    # The start room matches Lane A's full x-extent and extends south.
    # Lane A's south wall (with door) serves as the shared north boundary.
    cy_A = LANES['A']['cy']
    cx_A = LANES['A']['cx']
    entry_y = cy_A - HALF_SNAKE        # -10.6
    ROOM_L = 2.5                        # room depth (y direction)
    sr_x0 = cx_A - HALF_LEN            # -1.8  (match Lane A x-extent)
    sr_x1 = cx_A + HALF_LEN            # 1.8
    sr_y0 = entry_y - ROOM_L           # -13.1
    sr_y1 = entry_y                     # -10.6
    sr_w = sr_x1 - sr_x0               # 3.6
    sr_cy = (sr_y0 + sr_y1) / 2
    # West wall
    gs.append(Box("start_W", pos=(sr_x0, sr_cy, wh / 2),
                  size=(WALL_T, ROOM_L, wh)))
    # East wall
    gs.append(Box("start_E", pos=(sr_x1, sr_cy, wh / 2),
                  size=(WALL_T, ROOM_L, wh)))
    # South wall (closed)
    gs.append(Box("start_S", pos=(cx_A, sr_y0, wh / 2),
                  size=(sr_w, WALL_T, wh)))
    # No north wall — Lane A's south wall already has the door
    # Floor label (green marker)
    gs.append(Box("start_marker", pos=(cx_A, sr_cy, 0.005),
                  size=(0.6, 0.6, 0.01), color=(0.0, 0.7, 0.2, 1.0)))

    # ── FINISH room (east of Lane F) ──
    # Lane F is NS: y in [-0.8, 2.8], east wall at x = cx_F + HALF_SNAKE
    # The finish room matches Lane F's full y-extent and extends east.
    # Lane F's east wall (with door) serves as the shared west boundary.
    cx_F = LANES['F']['cx']
    cy_F = LANES['F']['cy']
    f_east = cx_F + HALF_SNAKE          # 16.65
    fin_y0 = cy_F - HALF_LEN           # -0.8
    fin_y1 = cy_F + HALF_LEN           # 2.8
    fin_h = fin_y1 - fin_y0            # 3.6
    fin_x0 = f_east                     # 16.65
    fin_x1 = f_east + ROOM_L           # 19.15
    fin_cx = (fin_x0 + fin_x1) / 2
    fin_cy = (fin_y0 + fin_y1) / 2
    # South wall
    gs.append(Box("finish_S", pos=(fin_cx, fin_y0, wh / 2),
                  size=(ROOM_L, WALL_T, wh)))
    # North wall
    gs.append(Box("finish_N", pos=(fin_cx, fin_y1, wh / 2),
                  size=(ROOM_L, WALL_T, wh)))
    # East wall (closed)
    gs.append(Box("finish_E", pos=(fin_x1, fin_cy, wh / 2),
                  size=(WALL_T, fin_h, wh)))
    # No west wall — Lane F's east wall already has the door
    # Floor label (red marker for finish)
    gs.append(Box("finish_marker", pos=(fin_cx, fin_cy, 0.005),
                  size=(0.6, 0.6, 0.01), color=(0.8, 0.1, 0.1, 1.0)))

    return gs


# ── Terrain placement ─────────────────────────────────────────────

def _place_terrain(gs, lid, cx, cy, orient, terrain, slope):
    if terrain == 'ramps':
        _terrain_ramps(gs, lid, cx, cy, orient, slope)
    elif terrain == 'pallets_ew':
        _terrain_pallets_ew(gs, lid, cx, cy, orient, slope)
    elif terrain == 'krails':
        _terrain_krails(gs, lid, cx, cy, orient, slope)
    elif terrain == 'stepfields':
        _terrain_stepfields(gs, lid, cx, cy, orient, slope)
    elif terrain == 'pallet_climb':
        _terrain_pallet_climb(gs, lid, cx, cy, orient, slope)
    elif terrain == 'stairs':
        _terrain_stairs(gs, lid, cx, cy, orient, slope)


def _terrain_ramps(gs, lid, cx, cy, orient, slope):
    """Lane A: 60cm pitch/roll ramp tiles — 2 rows per pass to fill 1.2m width."""
    ts = 0.60
    th = 0.16
    a15 = math.radians(15)
    n_tiles = int(LANE_LEN / ts)  # 6

    for pi, p_off in enumerate(PASS_OFF):
        if orient == 'EW':
            for ri, r_off in enumerate([-0.3, 0.3]):
                py = cy + p_off + r_off
                for ti in range(n_tiles):
                    tx = cx - HALF_LEN + ts / 2 + ti * ts
                    tz = th / 2 + _slope_z(slope, orient, cx, cy, tx, py)
                    pat = (pi + ri + ti) % 4
                    if pat == 0:   e = (a15,  0,    0)
                    elif pat == 1: e = (0,    a15,  0)
                    elif pat == 2: e = (-a15, 0,    0)
                    else:          e = (0,    -a15, 0)
                    gs.append(Box(f"ramp_{pi}_{ri}_{ti}", pos=(tx, py, tz),
                                  size=(ts, ts, th), euler=e, color=C_RAMP))


def _terrain_pallets_ew(gs, lid, cx, cy, orient, slope):
    """Lane B (EW): pallet slat groups + pipe proxies, passes stacking in Y."""
    obs_w = 1.1
    for pi, p_off in enumerate(PASS_OFF):
        if orient == 'EW':
            py = cy + p_off
            for gi, x_off in enumerate([-0.6, 0.6]):
                px = cx + x_off
                double = (gi == 1)
                sh = 0.40 if double else 0.20
                sz = _slope_z(slope, orient, cx, cy, px, py)

                for si in range(5):
                    sx = px - 0.36 + si * 0.18
                    gs.append(Box(f"plt_{pi}_{gi}_{si}",
                                  pos=(sx, py, sh / 2 + sz),
                                  size=(0.10, obs_w, sh), color=C_PALLET))

                gs.append(Cyl(f"pipe_{pi}_{gi}",
                              pos=(px - 0.50, py, sh + 0.05 + sz),
                              radius=0.05, length=obs_w,
                              euler=(math.pi / 2, 0, 0), color=C_PIPE))


def _terrain_krails(gs, lid, cx, cy, orient, slope):
    """Lane C: diagonal K-rails at 45deg yaw, incremental heights."""
    heights = [0.05, 0.10, 0.15, 0.20]
    for pi, p_off in enumerate(PASS_OFF):
        if orient == 'NS':
            px = cx + p_off
            for ri, y_off in enumerate([-0.6, 0.6]):
                ry = cy + y_off
                h = heights[(pi * 2 + ri) % 4]
                sz = _slope_z(slope, orient, cx, cy, px, ry)
                gs.append(Box(f"kr_{pi}_{ri}",
                              pos=(px, ry, h / 2 + sz),
                              size=(1.7, 0.15, h),
                              euler=(0, 0, math.pi / 4), color=C_KRAIL))


def _terrain_stepfields(gs, lid, cx, cy, orient, slope):
    """Lane D: cubic stepfield — 30cm square steps at 3 height levels."""
    step_side = 0.30
    heights = [0.0, 0.15, 0.30]
    rng = random.Random(STEPFIELD_SEED)

    n_cols = int(PASS_W / step_side)      # 4
    n_rows = int(LANE_LEN / step_side)    # 12

    for pi, p_off in enumerate(PASS_OFF):
        if orient == 'NS':
            pass_cx = cx + p_off
            x_start = pass_cx - PASS_W / 2 + step_side / 2
            y_start = cy - HALF_LEN + step_side / 2

            for col in range(n_cols):
                bx = x_start + col * step_side
                for row in range(n_rows):
                    by = y_start + row * step_side
                    h = rng.choice(heights)
                    if h == 0.0:
                        continue
                    sz = _slope_z(slope, orient, cx, cy, bx, by)
                    gs.append(Box(f"sf_{pi}_{col}_{row}",
                                  pos=(bx, by, h / 2 + sz),
                                  size=(step_side, step_side, h),
                                  color=C_STEP))


def _terrain_pallet_climb(gs, lid, cx, cy, orient, slope):
    """Lane E: pallet & pipe climb — taller stacks, 2 pipes per group."""
    obs_w = 1.1
    stack_heights = [0.40, 0.60, 0.80]

    for pi, p_off in enumerate(PASS_OFF):
        if orient == 'NS':
            px = cx + p_off
            for gi, y_off in enumerate([-0.6, 0.6]):
                py = cy + y_off
                sh = stack_heights[(pi * 2 + gi) % 3]
                sz = _slope_z(slope, orient, cx, cy, px, py)

                for si in range(5):
                    sy = py - 0.36 + si * 0.18
                    gs.append(Box(f"pclimb_{pi}_{gi}_{si}",
                                  pos=(px, sy, sh / 2 + sz),
                                  size=(obs_w, 0.10, sh), color=C_CLIMB))

                for pipe_i, pipe_dx in enumerate([-0.15, 0.15]):
                    gs.append(Cyl(f"cpipe_{pi}_{gi}_{pipe_i}",
                                  pos=(px + pipe_dx, py - 0.50, sh + 0.05 + sz),
                                  radius=0.05, length=obs_w * 0.8,
                                  euler=(0, math.pi / 2, 0), color=C_PIPE))


def _terrain_stairs(gs, lid, cx, cy, orient, slope):
    """Lane F: stairs up-landing-down — 3 up, flat landing, 3 down per pass."""
    rise = 0.15
    run = 0.30
    n_up = 3
    landing_len = 0.30
    obs_w = 1.1
    total_span = n_up * run + landing_len + n_up * run  # 2.10m
    y_offset = -total_span / 2

    for pi, p_off in enumerate(PASS_OFF):
        if orient == 'NS':
            px = cx + p_off
            sz = _slope_z(slope, orient, cx, cy, px, cy)
            y_base = cy + y_offset

            # ascending steps
            for si in range(n_up):
                sh = (si + 1) * rise
                sy = y_base + si * run + run / 2
                gs.append(Box(f"stup_{pi}_{si}",
                              pos=(px, sy, sh / 2 + sz),
                              size=(obs_w, run, sh), color=C_CONCR))

            # flat landing at top
            landing_h = n_up * rise
            landing_y = y_base + n_up * run + landing_len / 2
            gs.append(Box(f"stland_{pi}",
                          pos=(px, landing_y, landing_h / 2 + sz),
                          size=(obs_w, landing_len, landing_h), color=C_CONCR))

            # descending steps
            down_y_base = y_base + n_up * run + landing_len
            for si in range(n_up):
                sh = (n_up - si) * rise
                sy = down_y_base + si * run + run / 2
                gs.append(Box(f"stdn_{pi}_{si}",
                              pos=(px, sy, sh / 2 + sz),
                              size=(obs_w, run, sh), color=C_CONCR))

            # debris rail on ascending section
            dz_up = 2 * rise + 0.02 + sz
            gs.append(Cyl(f"debup_{pi}",
                          pos=(px, y_base + 1 * run, dz_up),
                          radius=0.02, length=obs_w,
                          euler=(0.35, math.pi / 2, 0), color=C_PIPE))

            # debris rail on descending section
            dz_dn = 2 * rise + 0.02 + sz
            gs.append(Cyl(f"debdn_{pi}",
                          pos=(px, down_y_base + 1 * run, dz_dn),
                          radius=0.02, length=obs_w,
                          euler=(-0.35, math.pi / 2, 0), color=C_PIPE))


# ── Mission task placeholders ─────────────────────────────────────

def _place_tasks(gs, lid, cx, cy, orient):
    x0, x1, y0, y1 = _lane_extents(cx, cy, orient)
    if lid == 'A':
        gs.append(Box(f"qr_{lid}", pos=(x0 + 0.15, cy, 0.60),
                      size=(0.02, 0.15, 0.15), color=C_QR))
        gs.append(Box(f"haz_{lid}", pos=(cx + 0.5, cy + PASS_OFF[0], 0.06),
                      size=(0.12, 0.12, 0.12), color=C_HAZ))
    elif lid == 'B':
        gs.append(Box(f"qr_{lid}", pos=(x1 - 0.15, cy, 0.60),
                      size=(0.02, 0.15, 0.15), color=C_QR))
        gs.append(Box(f"haz_{lid}", pos=(cx - 0.5, cy + PASS_OFF[2], 0.06),
                      size=(0.12, 0.12, 0.12), color=C_HAZ))
    elif lid == 'C':
        gs.append(Box(f"qr_{lid}", pos=(cx, y1 - 0.15, 0.60),
                      size=(0.15, 0.02, 0.15), color=C_QR))
        gs.append(Box(f"haz_{lid}", pos=(cx + PASS_OFF[0], cy + 0.5, 0.06),
                      size=(0.12, 0.12, 0.12), color=C_HAZ))
    elif lid == 'D':
        gs.append(Box(f"qr_{lid}", pos=(cx, y0 + 0.15, 0.60),
                      size=(0.15, 0.02, 0.15), color=C_QR))
        gs.append(Box(f"haz_{lid}", pos=(cx + PASS_OFF[1], cy + 0.3, 0.06),
                      size=(0.12, 0.12, 0.12), color=C_HAZ))
    elif lid == 'E':
        gs.append(Box(f"qr_{lid}", pos=(cx, y1 - 0.15, 0.60),
                      size=(0.15, 0.02, 0.15), color=C_QR))
        gs.append(Box(f"haz_{lid}", pos=(cx + PASS_OFF[0], cy - 0.3, 0.06),
                      size=(0.12, 0.12, 0.12), color=C_HAZ))
    elif lid == 'F':
        gs.append(Box(f"qr_{lid}", pos=(x1 - 0.15, cy, 0.60),
                      size=(0.02, 0.15, 0.15), color=C_QR))
        gs.append(Box(f"haz_{lid}", pos=(cx + PASS_OFF[0], cy - 0.5, 0.06),
                      size=(0.12, 0.12, 0.12), color=C_HAZ))


# ── SDF writer ────────────────────────────────────────────────────

def _f(v):
    return f"{v:.4f}" if isinstance(v, float) else str(v)


def write_sdf(geoms: List[Geom], path: str):
    L: List[str] = []
    a = L.append

    a('<?xml version="1.0" ?>')
    a('<sdf version="1.6">')
    a('<world name="lrc_2026">')
    a('')
    a('  <!-- ROS Gazebo plugins for spawn_entity and clock -->')
    a('  <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"/>')
    a('  <plugin name="gazebo_ros_factory" filename="libgazebo_ros_factory.so"/>')
    a('')
    a('  <physics type="ode">')
    a('    <max_step_size>0.002</max_step_size>')
    a('    <real_time_factor>1.0</real_time_factor>')
    a('    <real_time_update_rate>500</real_time_update_rate>')
    a('    <ode>')
    a('      <solver>')
    a('        <type>quick</type>')
    a('        <iters>25</iters>')
    a('      </solver>')
    a('    </ode>')
    a('  </physics>')
    a('')
    a('  <scene>')
    a('    <ambient>0.6 0.6 0.6 1</ambient>')
    a('    <background>0.7 0.8 0.9 1</background>')
    a('  </scene>')
    a('')
    a('  <light name="sun" type="directional">')
    a('    <cast_shadows>true</cast_shadows>')
    a('    <pose>0 0 10 0 0 0</pose>')
    a('    <diffuse>0.8 0.8 0.8 1</diffuse>')
    a('    <specular>0.2 0.2 0.2 1</specular>')
    a('    <direction>-0.5 0.1 -0.9</direction>')
    a('  </light>')
    a('')
    a('  <model name="ground_plane">')
    a('    <static>true</static>')
    a('    <link name="link">')
    a('      <collision name="collision">')
    a('        <geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>')
    a('      </collision>')
    a('      <visual name="visual">')
    a('        <geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>')
    a('        <material>')
    a('          <ambient>0.8 0.8 0.8 1</ambient>')
    a('          <diffuse>0.8 0.8 0.8 1</diffuse>')
    a('        </material>')
    a('      </visual>')
    a('    </link>')
    a('  </model>')
    a('')

    for g in geoms:
        if isinstance(g, Box):
            _sdf_box(L, g)
        else:
            _sdf_cyl(L, g)

    a('')
    a('</world>')
    a('</sdf>')

    with open(path, 'w') as fh:
        fh.write('\n'.join(L) + '\n')
    print(f"  SDF:  {path}")


def _sdf_box(L, b: Box):
    x, y, z = b.pos
    sx, sy, sz = b.size
    rx, ry, rz = b.euler
    r, g, bl, al = b.color
    L.append(f'  <model name="{b.name}">')
    L.append(f'    <static>true</static>')
    L.append(f'    <pose>{_f(x)} {_f(y)} {_f(z)} {_f(rx)} {_f(ry)} {_f(rz)}</pose>')
    L.append(f'    <link name="link">')
    L.append(f'      <collision name="col">')
    L.append(f'        <geometry><box><size>{_f(sx)} {_f(sy)} {_f(sz)}</size></box></geometry>')
    if b.mu != 1.0:
        L.append(f'        <surface><friction><ode>')
        L.append(f'          <mu>{b.mu}</mu><mu2>{b.mu}</mu2>')
        L.append(f'        </ode></friction></surface>')
    L.append(f'      </collision>')
    L.append(f'      <visual name="vis">')
    L.append(f'        <geometry><box><size>{_f(sx)} {_f(sy)} {_f(sz)}</size></box></geometry>')
    L.append(f'        <material>')
    L.append(f'          <ambient>{r} {g} {bl} {al}</ambient>')
    L.append(f'          <diffuse>{r} {g} {bl} {al}</diffuse>')
    L.append(f'        </material>')
    L.append(f'      </visual>')
    L.append(f'    </link>')
    L.append(f'  </model>')


def _sdf_cyl(L, c: Cyl):
    x, y, z = c.pos
    rx, ry, rz = c.euler
    r, g, bl, al = c.color
    L.append(f'  <model name="{c.name}">')
    L.append(f'    <static>true</static>')
    L.append(f'    <pose>{_f(x)} {_f(y)} {_f(z)} {_f(rx)} {_f(ry)} {_f(rz)}</pose>')
    L.append(f'    <link name="link">')
    L.append(f'      <collision name="col">')
    L.append(f'        <geometry><cylinder><radius>{c.radius}</radius><length>{c.length}</length></cylinder></geometry>')
    L.append(f'      </collision>')
    L.append(f'      <visual name="vis">')
    L.append(f'        <geometry><cylinder><radius>{c.radius}</radius><length>{c.length}</length></cylinder></geometry>')
    L.append(f'        <material>')
    L.append(f'          <ambient>{r} {g} {bl} {al}</ambient>')
    L.append(f'          <diffuse>{r} {g} {bl} {al}</diffuse>')
    L.append(f'        </material>')
    L.append(f'      </visual>')
    L.append(f'    </link>')
    L.append(f'  </model>')


# ── MJCF writer ───────────────────────────────────────────────────

def write_mjcf(geoms: List[Geom], path: str):
    L: List[str] = []
    a = L.append

    a('<mujoco model="lrc_2026">')
    a('  <compiler angle="radian" coordinate="local"/>')
    a('  <option timestep="0.002" gravity="0 0 -9.81"/>')
    a('')
    a('  <asset>')
    a('    <texture name="grid" type="2d" builtin="checker"')
    a('             rgb1="0.8 0.8 0.8" rgb2="0.7 0.7 0.7"')
    a('             width="512" height="512"/>')
    a('    <material name="ground_mat" texture="grid" texrepeat="10 10"/>')
    a('  </asset>')
    a('')
    a('  <worldbody>')
    a('    <light pos="0 0 10" dir="-0.5 0.1 -0.9" diffuse="0.8 0.8 0.8"/>')
    a('    <geom name="ground" type="plane" size="25 25 0.1"')
    a('          material="ground_mat"/>')
    a('')

    for g in geoms:
        if isinstance(g, Box):
            _mjcf_box(L, g)
        else:
            _mjcf_cyl(L, g)

    a('  </worldbody>')
    a('</mujoco>')

    with open(path, 'w') as fh:
        fh.write('\n'.join(L) + '\n')
    print(f"  MJCF: {path}")


def _mjcf_box(L, b: Box):
    x, y, z = b.pos
    hx, hy, hz = b.size[0] / 2, b.size[1] / 2, b.size[2] / 2
    r, g, bl, al = b.color
    parts = [f'name="{b.name}"', 'type="box"',
             f'pos="{_f(x)} {_f(y)} {_f(z)}"',
             f'size="{_f(hx)} {_f(hy)} {_f(hz)}"',
             f'rgba="{r} {g} {bl} {al}"']
    if any(v != 0 for v in b.euler):
        rx, ry, rz = b.euler
        parts.append(f'euler="{_f(rx)} {_f(ry)} {_f(rz)}"')
    if b.mu != 1.0:
        parts.append(f'friction="{b.mu} {b.mu} 0.001"')
    L.append(f'    <geom {" ".join(parts)}/>')


def _mjcf_cyl(L, c: Cyl):
    x, y, z = c.pos
    hl = c.length / 2
    r, g, bl, al = c.color
    parts = [f'name="{c.name}"', 'type="cylinder"',
             f'pos="{_f(x)} {_f(y)} {_f(z)}"',
             f'size="{c.radius} {_f(hl)}"',
             f'rgba="{r} {g} {bl} {al}"']
    if any(v != 0 for v in c.euler):
        rx, ry, rz = c.euler
        parts.append(f'euler="{_f(rx)} {_f(ry)} {_f(rz)}"')
    L.append(f'    <geom {" ".join(parts)}/>')


# ── Main ──────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="Generate LRC 2026 Confined Configuration world files")
    ap.add_argument('--slope', action='store_true',
                    help='Add 15deg cross-slope to lane floors')
    args = ap.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(script_dir)
    worlds_dir = os.path.join(repo_root, 'src', 'go2w', 'go2_gazebo_sim', 'worlds')

    suffix = '_slope' if args.slope else ''
    sdf_path = os.path.join(worlds_dir, f'lrc_terrain_maze{suffix}.world')
    mjcf_path = os.path.join(worlds_dir, f'lrc_terrain_maze{suffix}.xml')

    variant = '15deg slope' if args.slope else 'flat'
    print(f"Generating LRC 2026 Confined Configuration ({variant})...")
    print(f"  6 lanes: A(Ramps) B(Pallets) C(K-Rails) D(Stepfields) E(PalletClimb) F(Stairs)")
    print(f"  Lane length: {LANE_LEN}m, Pass width: {PASS_W}m, Snake width: {SNAKE_W:.1f}m")

    geoms = build_all(args.slope)
    boxes = sum(1 for g in geoms if isinstance(g, Box))
    cyls = sum(1 for g in geoms if isinstance(g, Cyl))
    print(f"  Elements: {len(geoms)} total ({boxes} boxes, {cyls} cylinders)")

    write_sdf(geoms, sdf_path)
    write_mjcf(geoms, mjcf_path)

    cy_A = LANES['A']['cy']
    entry_y = cy_A - HALF_SNAKE
    spawn_y = entry_y - 1.25  # center of start room (ROOM_L=2.5)
    cx_F = LANES['F']['cx']
    finish_x = cx_F + HALF_SNAKE + 1.0
    finish_y = LANES['F']['cy'] + DOORS['F']['east']
    print(f"\nLayout:")
    print(f"    C ── D ── E ── F ── [FINISH]")
    print(f"    |")
    print(f"    B")
    print(f"    |")
    print(f"    A")
    print(f"    |")
    print(f"  [START]")
    print(f"\nRobot spawn: (0, {spawn_y:.1f}) facing north  (start room)")
    print(f"Finish room: ({finish_x:.1f}, {finish_y:.1f})")
    print("Done.")


if __name__ == '__main__':
    main()
