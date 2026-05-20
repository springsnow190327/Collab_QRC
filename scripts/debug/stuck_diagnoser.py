#!/usr/bin/env python3
"""stuck_diagnoser — auto-classify WHY the robot stopped exploring.

Runs alongside the live Nav2 + CFPA2 + trav-grid stack. When the robot is
stuck (either the stuck_watchdog emits /<ns>/recovery_event "stuck_detected",
or this node self-detects no motion for `still_sec` with an active goal), it
snapshots every layer of the autonomy chain and prints a single verdict
naming the FIRST layer that broke, plus the evidence from each layer.

Failure modes it distinguishes (the user's three buckets + finer grain):

  NO_GOAL          CFPA2 isn't assigning a goal (no_frontiers / no_reachable /
                   silent). Exploration logic, not navigation.
  NO_PLAN          A goal exists but Nav2's planner produces no path (or a
                   stale/empty one). Probes the costmap at the goal + around
                   the robot to say whether the goal is lethal or the robot
                   is boxed in.
  CONTROLLER_IDLE  Goal + plan exist but cmd_vel ≈ 0 — MPPI is rejecting every
                   trajectory (usually footprint vs inflation: robot can't
                   find a collision-free rollout). "Stuck because nav2 won't
                   move" even though it has a plan.
  WALL_HIT         Goal + plan + cmd_vel command motion, but odom isn't moving
                   — the robot is physically pushing against geometry (footprint
                   wedged / collision).
  TRAV_CORRUPT     The traversability grid itself is bad around the robot
                   (robot boxed in by a lethal ring, huge lethal blob, NaN /
                   garbage). This is UPSTREAM of NO_PLAN/CONTROLLER_IDLE, so
                   when detected it is reported as the PRIMARY cause.

Output sinks (all three, every diagnosis):
  - stdout, formatted block (greppable: "STUCK-DIAGNOSIS").
  - /<ns>/stuck_diagnosis (std_msgs/String, JSON) for downstream logging.
  - append JSONL to --log-file (default
    /tmp/collab_qrc_logs/stuck_diagnosis_<ns>.jsonl) so a desktop run keeps a
    durable record across restarts.

Usage:
  ./scripts/debug/stuck_diagnoser.py --ns robot
  ./scripts/debug/stuck_diagnoser.py --ns robot --cmd-vel-topic cmd_vel_legged
  # Wired automatically into nav_test_mujoco_fastlio.launch.py (explore mode).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from collections import deque
from typing import Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from std_msgs.msg import String


def _split_ros_argv(argv):
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        return argv[:i], argv[i:]
    return argv, []


class StuckDiagnoser(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("stuck_diagnoser")
        self.ns = args.ns.strip("/")
        self.still_sec = float(args.still_sec)
        self.still_thresh_m = float(args.still_thresh_m)
        self.v_eps = float(args.v_eps)
        self.w_eps = float(args.w_eps)
        self.cooldown_sec = float(args.cooldown_sec)
        self.goal_stale_sec = float(args.goal_stale_sec)
        self.plan_stale_sec = float(args.plan_stale_sec)

        self.log_file = args.log_file or os.path.join(
            "/tmp/collab_qrc_logs", f"stuck_diagnosis_{self.ns}.jsonl")
        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)

        # ── latest snapshots ───────────────────────────────────────────
        self.last_odom: Optional[Odometry] = None
        self.pose_hist: deque = deque()  # (t_wall, x, y)
        self.last_goal: Optional[PoseStamped] = None
        self.last_goal_t: float = 0.0
        self.last_plan: Optional[Path] = None
        self.last_plan_t: float = 0.0
        self.last_costmap: Optional[OccupancyGrid] = None
        self.last_trav: Optional[OccupancyGrid] = None
        self.last_status: Optional[str] = None
        self.last_status_t: float = 0.0
        self.cmd_hist: deque = deque()  # (t_wall, v, w)
        # Real collisions from MuJoCo /mujoco/contacts. Each entry:
        # (t_wall, body_pairs:set[str], foot_wall_pairs:set[str]). A benign
        # contact is floor↔foot (normal walking); anything else is the robot
        # actually touching geometry it shouldn't.
        self.contact_hist: deque = deque()
        self._last_collision_log: float = 0.0

        self._last_diag_t: float = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        n = self.ns
        self.create_subscription(Odometry, f"/{n}/odom/nav", self._cb_odom, sensor_qos)
        self.create_subscription(PoseStamped, f"/{n}/goal_pose", self._cb_goal, reliable)
        self.create_subscription(Path, f"/{n}/plan", self._cb_plan, reliable)
        self.create_subscription(
            OccupancyGrid, f"/{n}/global_costmap/costmap", self._cb_costmap, latched)
        self.create_subscription(
            OccupancyGrid, f"/{n}/traversability_grid", self._cb_trav, latched)
        self.create_subscription(
            String, f"/{n}/exploration_status", self._cb_status, reliable)
        self.create_subscription(
            String, f"/{n}/recovery_event", self._cb_recovery, 10)
        # cmd_vel: Go2 (legged) → cmd_vel_legged; Go2W → cmd_vel. Subscribe to
        # the configured one AND the alternate, take whichever publishes.
        cmd_topic = args.cmd_vel_topic.strip("/")
        self.create_subscription(
            Twist, f"/{n}/{cmd_topic}", self._cb_cmd_twist, sensor_qos)
        alt = "cmd_vel" if cmd_topic != "cmd_vel" else "cmd_vel_legged"
        self.create_subscription(
            Twist, f"/{n}/{alt}", self._cb_cmd_twist, sensor_qos)
        # Some stacks publish TwistStamped mirrors.
        self.create_subscription(
            TwistStamped, f"/{n}/cmd_vel_stamped", self._cb_cmd_stamped, sensor_qos)
        # Real-collision sensing (sim only): MuJoCo publishes every contact pair
        # on the GLOBAL /mujoco/contacts topic (not namespaced). Format per line
        # "geom1|geom2|px,py,pz|fx". floor↔foot is benign walking; anything else
        # is the robot touching geometry.
        self.create_subscription(
            String, "/mujoco/contacts", self._cb_contacts, sensor_qos)

        self.diag_pub = self.create_publisher(String, f"/{n}/stuck_diagnosis", 10)

        # Self-detect timer + heartbeat.
        self.create_timer(1.0, self._tick)
        self._boot_t = time.monotonic()
        self.get_logger().info(
            f"stuck_diagnoser armed ns=/{self.ns} still={self.still_sec}s/"
            f"{self.still_thresh_m}m log={self.log_file}")

    # ── callbacks ──────────────────────────────────────────────────────
    def _cb_odom(self, m: Odometry) -> None:
        self.last_odom = m
        t = time.monotonic()
        self.pose_hist.append((t, m.pose.pose.position.x, m.pose.pose.position.y))
        cutoff = t - max(self.still_sec, 15.0)
        while self.pose_hist and self.pose_hist[0][0] < cutoff:
            self.pose_hist.popleft()

    def _cb_goal(self, m: PoseStamped) -> None:
        self.last_goal = m
        self.last_goal_t = time.monotonic()

    def _cb_plan(self, m: Path) -> None:
        self.last_plan = m
        self.last_plan_t = time.monotonic()

    def _cb_costmap(self, m: OccupancyGrid) -> None:
        self.last_costmap = m

    def _cb_trav(self, m: OccupancyGrid) -> None:
        self.last_trav = m

    def _cb_status(self, m: String) -> None:
        self.last_status = m.data
        self.last_status_t = time.monotonic()

    def _cb_cmd_twist(self, m: Twist) -> None:
        self._push_cmd(m.linear.x, m.angular.z)

    def _cb_cmd_stamped(self, m: TwistStamped) -> None:
        self._push_cmd(m.twist.linear.x, m.twist.angular.z)

    def _push_cmd(self, v: float, w: float) -> None:
        t = time.monotonic()
        self.cmd_hist.append((t, v, w))
        cutoff = t - max(self.still_sec, 15.0)
        while self.cmd_hist and self.cmd_hist[0][0] < cutoff:
            self.cmd_hist.popleft()

    def _cb_recovery(self, m: String) -> None:
        if m.data == "stuck_detected":
            self._maybe_diagnose("watchdog:stuck_detected")

    # foot collision-geom names (MuJoCo Go2: FL/FR/RL/RR; some MJCFs *_foot).
    _FOOT_TOKENS = frozenset({"FL", "FR", "RL", "RR"})

    @classmethod
    def _classify_contact(cls, g1: str, g2: str) -> Optional[Tuple[str, str]]:
        """None=benign (floor↔foot). Else (severity, "g1|g2") where severity is
        'body' (a non-foot robot part / chassis touching a wall or floor — a
        real crash) or 'foot_wall' (a foot brushing a wall base — minor)."""
        is_foot = lambda t: t in cls._FOOT_TOKENS or "foot" in t.lower()
        is_floor = lambda t: t.lower() == "floor"
        if (is_floor(g1) and is_foot(g2)) or (is_floor(g2) and is_foot(g1)):
            return None  # normal walking
        pair = f"{g1}|{g2}"
        if is_foot(g1) or is_foot(g2):
            return ("foot_wall", pair)   # a foot touched a wall (minor)
        return ("body", pair)            # chassis/leg touched wall or fell

    def _cb_contacts(self, m: String) -> None:
        body: set = set()
        foot_wall: set = set()
        for line in m.data.splitlines():
            line = line.strip()
            if not line:
                continue
            parts = line.split("|")
            if len(parts) < 2:
                continue
            cls = self._classify_contact(parts[0], parts[1])
            if cls is None:
                continue
            (sev, pair) = cls
            (body if sev == "body" else foot_wall).add(pair)
        if not body and not foot_wall:
            return
        t = time.monotonic()
        self.contact_hist.append((t, body, foot_wall))
        cutoff = t - max(self.still_sec, 15.0)
        while self.contact_hist and self.contact_hist[0][0] < cutoff:
            self.contact_hist.popleft()
        # A real body/chassis collision is serious — trigger a diagnosis
        # immediately (cooldown-gated), don't wait for the still-window.
        if body and (t - self._last_collision_log) > 3.0:
            self._last_collision_log = t
            self.get_logger().warn(
                f"REAL COLLISION contact(s): {sorted(body)} — robot body "
                f"touching geometry (not floor↔foot)")
            self._maybe_diagnose("contact:body_collision")

    def _recent_contacts(self, window: float) -> Tuple[set, set]:
        """Union of body + foot_wall collision pairs within `window` sec."""
        t_now = time.monotonic()
        body: set = set()
        foot_wall: set = set()
        for (t, b, fw) in self.contact_hist:
            if t >= t_now - window:
                body |= b
                foot_wall |= fw
        return body, foot_wall

    # ── self-detection + heartbeat ─────────────────────────────────────
    def _tick(self) -> None:
        if time.monotonic() - self._boot_t < 20.0:
            return  # let the stack warm up
        moved = self._displacement(self.still_sec)
        have_goal = self._goal_active()
        if have_goal and moved is not None and moved < self.still_thresh_m:
            self._maybe_diagnose(f"self:still({moved*100:.0f}cm/{self.still_sec:.0f}s)")

    def _maybe_diagnose(self, trigger: str) -> None:
        now = time.monotonic()
        if now - self._last_diag_t < self.cooldown_sec:
            return
        self._last_diag_t = now
        self._diagnose(trigger)

    # ── helpers ────────────────────────────────────────────────────────
    def _robot_xy(self) -> Optional[Tuple[float, float]]:
        if self.last_odom is None:
            return None
        return (self.last_odom.pose.pose.position.x,
                self.last_odom.pose.pose.position.y)

    def _displacement(self, window: float) -> Optional[float]:
        if len(self.pose_hist) < 2:
            return None
        t_now = self.pose_hist[-1][0]
        x1, y1 = self.pose_hist[-1][1], self.pose_hist[-1][2]
        # earliest sample within the window
        x0 = y0 = None
        for (t, x, y) in self.pose_hist:
            if t >= t_now - window:
                x0, y0 = x, y
                break
        if x0 is None:
            return None
        if (t_now - self.pose_hist[0][0]) < window * 0.8:
            return None  # not enough history yet
        return math.hypot(x1 - x0, y1 - y0)

    def _goal_active(self) -> bool:
        if self.last_goal is None:
            return False
        return (time.monotonic() - self.last_goal_t) < self.goal_stale_sec

    def _cmd_motion(self) -> Tuple[float, float]:
        """Return (max|v|, max|w|) commanded over the still window."""
        if not self.cmd_hist:
            return 0.0, 0.0
        t_now = self.cmd_hist[-1][0]
        vs = [abs(v) for (t, v, w) in self.cmd_hist if t >= t_now - self.still_sec]
        ws = [abs(w) for (t, v, w) in self.cmd_hist if t >= t_now - self.still_sec]
        return (max(vs) if vs else 0.0, max(ws) if ws else 0.0)

    @staticmethod
    def _grid_at(g: OccupancyGrid, wx: float, wy: float) -> Optional[int]:
        if g is None:
            return None
        res = g.info.resolution
        if res <= 0:
            return None
        i = int((wx - g.info.origin.position.x) / res)
        j = int((wy - g.info.origin.position.y) / res)
        if 0 <= i < g.info.width and 0 <= j < g.info.height:
            return int(g.data[j * g.info.width + i])
        return None

    def _grid_window_stats(self, g: OccupancyGrid, cx: float, cy: float,
                           half_m: float) -> dict:
        """Class fractions in a square window around (cx,cy)."""
        if g is None:
            return {}
        res = g.info.resolution
        if res <= 0:
            return {}
        w, h = g.info.width, g.info.height
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        ci = int((cx - ox) / res)
        cj = int((cy - oy) / res)
        r = max(1, int(half_m / res))
        i0, i1 = max(0, ci - r), min(w, ci + r + 1)
        j0, j1 = max(0, cj - r), min(h, cj + r + 1)
        if i0 >= i1 or j0 >= j1:
            return {}
        d = np.array(g.data, dtype=np.int16).reshape(h, w)[j0:j1, i0:i1]
        tot = d.size
        lethal = int((d >= 99).sum())
        occ_mid = int(((d >= 50) & (d < 99)).sum())
        unk = int((d < 0).sum())
        free = int((d == 0).sum())
        return {
            "total": tot,
            "lethal_pct": 100.0 * lethal / tot,
            "midcost_pct": 100.0 * occ_mid / tot,
            "unknown_pct": 100.0 * unk / tot,
            "free_pct": 100.0 * free / tot,
        }

    def _boxed_in(self, g: OccupancyGrid, cx: float, cy: float) -> Tuple[bool, int]:
        """Is the robot ringed by lethal cells within ~0.6 m? Returns
        (boxed, n_open_directions of 8)."""
        if g is None:
            return False, -1
        open_dirs = 0
        for ang in range(0, 360, 45):
            rad = math.radians(ang)
            # probe at 0.5 m in this direction
            v = self._grid_at(g, cx + 0.5 * math.cos(rad), cy + 0.5 * math.sin(rad))
            if v is not None and 0 <= v < 70:
                open_dirs += 1
        return (open_dirs == 0), open_dirs

    # ── the diagnosis ──────────────────────────────────────────────────
    def _diagnose(self, trigger: str) -> None:
        rxy = self._robot_xy()
        rx, ry = rxy if rxy else (float("nan"), float("nan"))
        moved = self._displacement(self.still_sec)
        max_v, max_w = self._cmd_motion()
        have_goal = self._goal_active()
        status = self.last_status or "?"
        status_age = time.monotonic() - self.last_status_t if self.last_status_t else 1e9
        plan_age = time.monotonic() - self.last_plan_t if self.last_plan_t else 1e9
        plan_len = len(self.last_plan.poses) if self.last_plan else 0
        goal_age = time.monotonic() - self.last_goal_t if self.last_goal_t else 1e9

        gx = gy = float("nan")
        if self.last_goal:
            gx = self.last_goal.pose.position.x
            gy = self.last_goal.pose.position.y

        # ── layer probes ──
        trav_stats = self._grid_window_stats(self.last_trav, rx, ry, 1.5) if rxy else {}
        cost_stats = self._grid_window_stats(self.last_costmap, rx, ry, 1.5) if rxy else {}
        boxed, open_dirs = self._boxed_in(self.last_costmap, rx, ry) if rxy else (False, -1)
        robot_cell_cost = self._grid_at(self.last_costmap, rx, ry) if rxy else None
        goal_cell_cost = (self._grid_at(self.last_costmap, gx, gy)
                          if self.last_goal else None)

        # trav-grid corruption heuristics (UPSTREAM root)
        trav_corrupt = False
        trav_reason = ""
        if trav_stats:
            if trav_stats.get("lethal_pct", 0) > 60.0:
                trav_corrupt = True
                trav_reason = (f"lethal {trav_stats['lethal_pct']:.0f}% of 3 m window "
                               f"around robot — robot buried in lethal")
        # robot's own costmap cell lethal = boxed in by perception
        if robot_cell_cost is not None and robot_cell_cost >= 99:
            trav_corrupt = True
            trav_reason = (f"robot's own costmap cell = {robot_cell_cost} (lethal); "
                           f"perception painted the robot's footprint as obstacle")
        if boxed:
            trav_corrupt = True
            trav_reason = (trav_reason + "; " if trav_reason else "") + \
                "all 8 compass directions blocked within 0.5 m (lethal ring)"

        # real collisions (sim ground truth from /mujoco/contacts)
        coll_body, coll_foot_wall = self._recent_contacts(self.still_sec)

        # ── verdict tree ──
        verdict = "UNKNOWN"
        detail = ""
        fix = ""
        if coll_body:
            verdict = "REAL_COLLISION"
            detail = (f"MuJoCo reports the robot BODY/chassis touching geometry "
                      f"(not floor↔foot): {sorted(coll_body)}. The robot is "
                      f"physically colliding — distinct from being merely wedged. "
                      f"cmd: max|v|={max_v:.3f} max|w|={max_w:.3f}, "
                      f"moved={(moved or 0)*100:.0f}cm/{self.still_sec:.0f}s.")
            fix = ("A real crash. Either Nav2 drove the body into a wall (footprint "
                   "too small vs body, or the wall is z-band-filtered out of the "
                   "trav grid so the planner can't see it), or the robot tipped/fell. "
                   "Check the trav grid HAS the wall at the contact point; widen "
                   "footprint or restore the filtered wall geom.")
        elif trav_corrupt:
            verdict = "TRAV_CORRUPT"
            detail = trav_reason
            fix = ("Upstream perception. Check elevation_mapping / filter_chain: "
                   "stray z outlier blowing wall_cost (re-add wall_cost_clamp_hi), "
                   "SLAM z-drift painting the floor as wall, or robot_self_filter "
                   "not removing the robot body. Run trav_grid_diag.py for cells.")
        elif not have_goal or status in ("no_frontiers", "no_reachable", "paused"):
            verdict = "NO_GOAL"
            detail = (f"CFPA2 status='{status}' (age {status_age:.0f}s), "
                      f"goal_age={goal_age:.0f}s. Exploration layer assigned no "
                      f"reachable frontier.")
            if status == "no_reachable":
                fix = ("Frontiers exist but none reachable through KNOWN-FREE space "
                       "(allow_unknown=false working as intended). Robot may be "
                       "walled off, OR reachability occ_thresh too strict (corridor "
                       "blocked). Check costmap around robot for a free path.")
            elif status == "no_frontiers":
                fix = ("No frontiers survived filtering. Either area explored, or "
                       "frontier filters (min_unknown_cells / cluster_area / "
                       "obstacle_clearance) too strict, or max_goal_distance capped.")
            else:
                fix = "CFPA2 not publishing goals — check it's alive + has a map."
        elif plan_age > self.plan_stale_sec or plan_len < 2:
            verdict = "NO_PLAN"
            detail = (f"goal=({gx:+.1f},{gy:+.1f}) active but plan stale "
                      f"(age={plan_age:.0f}s, len={plan_len}). Nav2 planner found "
                      f"no path. goal_cell_cost={goal_cell_cost}, "
                      f"robot_cell_cost={robot_cell_cost}, open_dirs={open_dirs}/8.")
            if goal_cell_cost is not None and goal_cell_cost >= 99:
                fix = ("Goal cell is LETHAL/inscribed — CFPA2 picked a goal Nav2 "
                       "can't accept. Tighten CFPA2 occ_thresh / clearance so it "
                       "stops picking near-wall goals, or it's a footprint vs "
                       "inflation mismatch.")
            elif open_dirs == 0:
                fix = ("Robot boxed in costmap — likely transient inflation around a "
                       "noisy obstacle; check trav grid + let costmap clear.")
            else:
                fix = ("Goal looks free but planner fails — check SmacHybrid "
                       "tolerance/footprint, costmap continuity between robot and "
                       "goal, or unknown space blocking (allow_unknown in planner).")
        else:
            # plan exists — is the controller commanding motion?
            if max_v <= self.v_eps and max_w <= self.w_eps:
                verdict = "CONTROLLER_IDLE"
                detail = (f"goal+plan(len={plan_len}) exist but cmd_vel≈0 "
                          f"(max|v|={max_v:.3f}, max|w|={max_w:.3f}) over "
                          f"{self.still_sec:.0f}s. MPPI rejecting all rollouts.")
                fix = ("MPPI ObstaclesCritic rejects every trajectory — footprint "
                       "vs inflation/collision_margin too tight for the corridor. "
                       "Lower collision_margin_distance, verify consider_footprint "
                       "polygon, or the local costmap has a spurious obstacle "
                       "ahead. cost window: " + json.dumps(cost_stats))
            else:
                verdict = "WALL_HIT"
                detail = (f"goal+plan+cmd_vel commanding motion "
                          f"(max|v|={max_v:.3f}, max|w|={max_w:.3f}) but robot moved "
                          f"only {(moved or 0)*100:.0f} cm in {self.still_sec:.0f}s. "
                          f"Physically wedged.")
                fix = ("Robot pushing against geometry it doesn't see as lethal. "
                       "Footprint too small vs body, MuJoCo collision wall not in "
                       "trav grid (z-band filtered), or CHAMP gait can't overcome "
                       "the contact. Check trav grid has the wall the robot is "
                       "touching.")

        # ── emit ──
        report = {
            "schema": "stuck_diagnoser/v1",
            "ns": self.ns,
            "trigger": trigger,
            "verdict": verdict,
            "detail": detail,
            "fix_hint": fix,
            "t_wall": time.time(),
            "robot": [round(rx, 2), round(ry, 2)],
            "goal": [round(gx, 2), round(gy, 2)] if self.last_goal else None,
            "moved_m": round(moved, 3) if moved is not None else None,
            "cmd_max_v": round(max_v, 3),
            "cmd_max_w": round(max_w, 3),
            "cfpa2_status": status,
            "plan_len": plan_len,
            "plan_age_s": round(plan_age, 1),
            "goal_age_s": round(goal_age, 1),
            "robot_cell_cost": robot_cell_cost,
            "goal_cell_cost": goal_cell_cost,
            "open_dirs_8": open_dirs,
            "trav_window": trav_stats,
            "cost_window": cost_stats,
            "collision_body": sorted(coll_body),
            "collision_foot_wall": sorted(coll_foot_wall),
        }
        block = [
            "",
            "█" * 72,
            f" STUCK-DIAGNOSIS [{self.ns}]  trigger={trigger}",
            f"   robot=({rx:+.2f},{ry:+.2f})  goal=" +
            (f"({gx:+.2f},{gy:+.2f})" if self.last_goal else "NONE") +
            f"  moved={(moved or 0)*100:.0f}cm/{self.still_sec:.0f}s",
            f"   cmd: max|v|={max_v:.3f} max|w|={max_w:.3f}   "
            f"cfpa2='{status}'  plan_len={plan_len}(age{plan_age:.0f}s)",
            f"   costmap: robot_cell={robot_cell_cost} goal_cell={goal_cell_cost} "
            f"open_dirs={open_dirs}/8  trav_window={trav_stats}",
            f"   contacts: body={sorted(coll_body) if coll_body else 'none'}  "
            f"foot_wall={sorted(coll_foot_wall) if coll_foot_wall else 'none'}",
            "─" * 72,
            f" ► VERDICT: {verdict}",
            f"   {detail}",
            f"   FIX: {fix}",
            "█" * 72,
        ]
        self.get_logger().warn("\n".join(block))
        s = String()
        s.data = json.dumps(report)
        self.diag_pub.publish(s)
        try:
            with open(self.log_file, "a") as f:
                f.write(json.dumps(report) + "\n")
        except OSError as e:
            self.get_logger().error(f"log write failed: {e}")


def main(argv=None):
    user_argv, ros_argv = _split_ros_argv(sys.argv if argv is None else argv)
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns", default="robot")
    ap.add_argument("--cmd-vel-topic", default="cmd_vel_legged",
                    help="primary cmd_vel topic (Go2 legged=cmd_vel_legged, "
                         "Go2W=cmd_vel). The alternate is also subscribed.")
    ap.add_argument("--still-sec", type=float, default=10.0)
    ap.add_argument("--still-thresh-m", type=float, default=0.20)
    ap.add_argument("--v-eps", type=float, default=0.02)
    ap.add_argument("--w-eps", type=float, default=0.05)
    ap.add_argument("--cooldown-sec", type=float, default=12.0)
    ap.add_argument("--goal-stale-sec", type=float, default=8.0)
    ap.add_argument("--plan-stale-sec", type=float, default=6.0)
    ap.add_argument("--log-file", default=None)
    args = ap.parse_args(user_argv[1:] if len(user_argv) > 1 else [])

    rclpy.init(args=ros_argv)
    node = StuckDiagnoser(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
