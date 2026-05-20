# ops2 Bidirectional Exploration — Debugging Journey & Deployment Notes

**Goal:** Go2 (no wheels) autonomously explores the ops2 SLAM-reconstructed building on the **desktop standalone** stack (`scripts/launch/nav_test_slam_ops2_v4_go2.sh` → `nav_test_3d_explore.launch.py`: MuJoCo + Fast-LIO + elevation/trav + Nav2 + CFPA2). Success = the **physical trajectory** spans **x = +35 → −35 within 10 % tolerance** (x_max ≥ +31.5 AND x_min ≤ −31.5), **real traversal, no goal-tolerance cheating**. This is a real-robot-deployment dry run.

**Building shape (critical):** ops2 is **V-shaped**, spawn at the middle (0,0). Handwall extent x = [−37.3, +41.7], y = [−29.4, +1.5] (`mujoco/handwalls/ops2_hand_walls.json`).
- **+x branch** = down-RIGHT to x ≈ +41 (wide, easy).
- **−x branch** = down-LEFT, a **winding diagonal maze** to x ≈ −37, physical corridor ≈ **1.0 m** wide.

---

## TL;DR — the full fix stack to deploy

**Validation status (honest, at commit time):** +x end is solid (x_max ≈ 32–34 across many runs). The bidirectional **breakthrough is proven to x_min = −8.4** (run29): the robot beelines +x to ≈ +32, **turns around at +31.5** while still mobile, drives back through spawn, and enters the −x branch. **Full −x to −31.5 is NOT yet validated** — run29 stalled at −8.4 (inflation pinch), and the two follow-up fixes (inflation 0.28→0.16, IG-override disabled) are committed but **pending a clean validation run** (run31 was killed mid-run for the manual-goal isolation test). Treat the committed config as "best-known, partially validated".

Three independent problem layers had to be solved in order:

1. **Get it moving at all** (runs ~1–14): an 11-bug cascade (config divergence, broken Python node, oversized costmap, Fast-LIO z-drift, trav self-paint, cross-host Jetson ghost, BT global-clear, blind-zone disconnect, `ig_mode:floodfill` stub). → robot explores frontier-by-frontier. *(Detailed in CLAUDE.md "Active state 2026-05-20 cont.".)*
2. **Reach +x robustly** (runs 15–23): `allow_unknown=false` reachability + **fallback snap-to-reachable** → no freeze, path 80 m+, x_max ≈ 32.
3. **Reach −x too** (runs 24–30, this doc): **extent-seek** strategy + **deterministic stable fallback** + **inflation re-tune**. ← the bidirectional breakthrough.

**Deploy-relevant knobs (all in config, no code recompile needed to tune):**

| Knob | File | Value | Why |
|---|---|---|---|
| `cfpa2_extent_seek_enabled` | `cfpa2_single_robot_ops2.yaml` | `true` | turn around once one ±x extreme reached |
| `cfpa2_extent_target_x` | ″ | `31.5` | = 10 % tol of 35; robot crosses it with momentum while beelining |
| `cfpa2_reachability_allow_unknown` | ″ | `false` | never plan optimistically into unknown / behind walls |
| `occupancy_block_threshold` / `cfpa2_reachability_occ_threshold` | ″ | `70` | reachability == Nav2-plannable |
| `planning_map_topic_suffix` | ″ | `/global_costmap/costmap` | CFPA2 reads the SAME inflated map Nav2 plans on |
| `cfpa2_ig_mode` | ″ | `"local"` | floodfill single-call is a stub returning 0 (footgun) |
| `inflation_radius` (both costmaps) | `nav2_go2_full_stack.yaml` | `0.16` | 1.0 m corridor → 0.68 m cost-0 lane ≥ 0.64 m turning envelope |
| `cost_scaling_factor` | ″ | `6.0` | steeper falloff beyond the thin halo |
| `consider_footprint` | ″ | `true` | EXACT collision guard (the real safety, not inflation) |
| `footprint` | ″ | `0.64 × 0.36 m` | real Go2 0.31 m body + leg-spread margin |
| `minimum_turning_radius` | ″ | `0.05` | Go2 pivots in place via CHAMP |
| `visited_corridor_enabled` | `grid_map_to_occupancy_grid.py` | `true` | stamp swept path FREE forever (ground truth) |

**Code changes that need a build** (`colcon build --packages-select cfpa2_collaborative_autonomy trav_cost_filters`):
- `cfpa2_coordinator.cpp`: extent-seek block, directional guard, IG-only stable fallback, high-IG unreachable override, `forced_goal_by_ns` policy bypass.
- `distance_transform.cpp`: `is_free` lambda (allow_unknown actually works).
- `grid_map_to_occupancy_grid.py`: persistent visited-corridor mask, two-tier footprint seed.

---

## Run-by-run journey

| run | change under test | x_max | x_min | verdict |
|---|---|---|---|---|
| ~1–14 | 11-bug cascade fixes | — | — | robot unfrozen, explores |
| 15/17/19/21 | `allow_unknown=false`, seed disk, ig=local | ~32 | ~0 | +x reached; froze at +x corner (~56 m path) |
| 23 | fallback snap-to-reachable | 32.3 | −2.5 | +x robust (80 m+, no freeze); −x never entered |
| 24 | persistent visited-corridor + stable fallback | 32.6 | −2.9 | greedy grinds +x pockets forever (path 54→176 m); empty-util fallback rarely fires |
| 25 | + high-IG unreachable override | 34.4 | −1.8 | **wedged at +x corner (x=34)**; IG-override fooled by high-IG perimeter frontiers (exterior unknown) |
| 26 | + extent-seek (threshold 32.0) | 31.8 | −1.8 | extent-seek **never fired** (x_max plateaued 31.79 < 32.0) |
| 27 | extent-seek threshold → 31.5 | 32.2 | −1.0 | extent-seek **fired** but goal **thrashed** −8.35 ↔ +10.25 (branch-1 picked min-x *reachable* frontier = still +x) |
| 28 | + directional guard | 33.9 | −1.2 | still thrashed −8.35 ↔ +36.05 (the −x goal got blacklisted on slow progress → primary +x back in) |
| 29 | + always-commit (drop blacklist) + bypass goal-policy | 31.7 | **−8.4** | **BREAKTHROUGH** — robot turned at +31.7, drove through spawn, first real −x traversal. Then **STALLED at −8.4** |
| 30 | inflation 0.28 → 0.16 | 28.6 | −1.7 | **regression**: with more −x now reachable, the IG-override (gated on `best_reach_ig`, not extent) fired DURING the +x beeline and thrashed +37 ↔ −16 → robot stuck creeping at +28, never hit the 31.5 extent-seek trigger. Zero real collisions. |
| 31 | + IG-override **disabled** (extent-seek only) | *not validated* | | killed mid-run for the manual-goal isolation test (below); needs a clean re-run |

### The −x failure modes, in the order they were peeled back

- **run24 — greedy pocket-grind.** The explored +x building always retains *some* tiny reachable frontier, so the empty-util fallback (which only fires when `util.empty()`) almost never triggers. The robot grinds +x nooks forever (path grew with zero new extent) and never returns for −x.
- **run25 — +x corner wedge + IG-override fooled.** Added an "information-gain-greedy" override (prefer the big unexplored region when reachable frontiers are low-IG). But frontiers at the building **perimeter** have *high* IG (the exterior is permanent unknown), so `best_reach_ig` stayed high → override never fired. The robot drove to the far +x corner and physically wedged (WALL_HIT: commanding turn, moving 3 cm/10 s).
- **run26 — threshold off by 0.21 m.** Extent-seek (turn around once x_max ≥ target) never fired because x_max plateaued at 31.79, just under the 32.0 target.
- **run27 — branch-1 wrong-direction pick.** With target 31.5 it fired, but "reachable frontier furthest −x" returns the **min-x reachable** frontier — which is still **+x** (all −x is unreachable) → goal alternated −8.35 ↔ +10.25 every tick → Nav2 never committed.
- **run28 — blacklist fallthrough.** Added a directional guard (only accept goals −x of the robot). Killed the +10.25, but the −x goal got **blacklisted** when progress was slow (false positive), which broke the challenger streak → primary +x leaked back → −8.35 ↔ +36.05 thrash, robot net-drifted to the +x corner.
- **run29 — decisive fix → breakthrough.** Three changes made the −x redirect unconditional: (a) extent-seek **always commits** a −x goal (a guaranteed furthest-(-x) reachable cell, blacklist check dropped — the strategic goal must not be blacklisted away), (b) **bypass `apply_goal_policy`** for the single deterministic seek goal (no challenger streak for the blacklist to break), (c) directional guard from run28. **Verified with `ComputePathToPose`: Nav2 CAN path to −8.35 (OK 364 poses)** — so the only thing stopping the robot was the thrash. x_min reached −8.38.
- **run29 STALL → run30 inflation fix.** The robot wedged at x = −8.4 in the winding maze. Measurement (RViz screenshot from the user + costmap probe):

  | | physical corridor (trav grid, lethal-to-lethal) | after Nav2 inflation (costmap) |
  |---|---|---|
  | robot cell cost | 0 (free) | 57 (in inflation) |
  | narrowest run over 3 m ahead | **1.00 m** | **0.50 m** |

  The corridor is physically 1.0 m, but `inflation_radius=0.28` left only a **0.44 m** cost-0 lane (1.0 − 2·0.28), narrower than the robot's **0.64 m** turning envelope, so SmacHybrid couldn't turn through the chicanes. At corridor center, `cost = 252·exp(−4.0·(0.5−0.18)) ≈ 70` = the block threshold → no plannable lane. **Fix: `inflation_radius → 0.16`** → 0.68 m cost-0 lane (≥ 0.64 m), `consider_footprint=true` stays as the exact guard.

### Manual-goal isolation test (CFPA2 off) — there is no shortcut to unexplored −x

To separate navigation from exploration, CFPA2 + bridge + watchdog were killed and goals sent by hand via `NavigateToPose`:

| goal | result |
|---|---|
| `(-30,-10)` deep unexplored | `ComputePathToPose` EMPTY; robot did not go −x (no path through unmapped space) |
| `(-15,-11)` (transiently OK 257 earlier) | robot did not move from a static pose |

**Conclusion:** you cannot drive to space the robot hasn't mapped — the planner has no path there. And you can't map it without driving (exploring) into it. So navigation and exploration are inseparable; the **only** way to deep −x is the incremental loop (move → sense → map grows → plan a bit further). This validates extent-seek + snap-to-reachable and rules out "just send waypoints to −35" as a shortcut. *(First attempt drove +x — the stuck_watchdog was republishing a stale +x goal; killed it for the clean test.)*

### Traversability / inflation is over-wide (operator observation, 2026-05-20)

The robot is **not hitting walls** (MuJoCo `contacts: 2` = the 2 foot-floor pairs; `/mujoco/contacts` shows only `floor↔foot`), but the **lethal/high-cost region over-paints free space** — the red band in RViz is too wide, eating the green corridor.

**Causal chain (operator correction — the trav grid is the root, not just inflation):** the global costmap's `static_layer` reads `map_topic: /robot_b/traversability_grid` and **copies the trav grid's lethal cells verbatim**; the `inflation_layer` then inflates *around* them. So any free→lethal over-painting in the trav grid is not merely inherited by the costmap — it is **amplified** (a false-lethal cell gets a full inflation halo on top). Tuning `inflation_radius` only treats the symptom.

Two distinct effects, don't conflate them:
1. **At the −8.4 chicane** the trav corridor was genuinely **1.0 m lethal-to-lethal** (those lethal cells = real walls), and Nav2 inflation crushed the threadable lane to 0.5 m → `inflation_radius 0.28 → 0.16` is the correct fix *there*.
2. **In open areas** (the operator's room screenshot) flat free floor shows red — the **trav grid itself marks free→lethal**. Inflation tuning will NOT fix this; it's upstream in the trav pipeline.

**ROOT FOUND + FIXED (probed `/robot/elevation_map_filtered` layers live):** the OccupancyGrid is thresholded from `trav_eth = (1−slope_cost)·(1−step_cost)·(1−wall_cost)` (lethal when < 0.05). Probe of the lethal cells: **91% were wall_cost-driven** (mean wall_cost 0.95); only 2% were open-floor slope/step noise; open floor (wall_cost<0.2) was 99.3% non-lethal (mean slope 2.4°, step 1.8cm). So it is NOT noise — it is **`wall_cost_dilated` (filter10, `maxOfFinites` `window_size:5` = ±0.20m at 0.10m/cell)** spreading real-wall lethal **±0.20m into adjacent free space** (every wall painted 0.40m fatter than physical). The costmap static_layer copies it and inflation halos around it → amplified. **Fix: `window_size 5 → 3`** (±0.10m) — keeps the wall rim lethal (the unreachable wall-top interior reading free is harmless behind the rim), frees 0.10m/side back into the corridor. Re-probe confirmed wall-driven lethal dropped 91%→70%. `consider_footprint` + Nav2 inflation remain the clearance guards.

**Remaining secondary (minor):** ~4% of open floor near the robot reads lethal during *early* sensing (sparse Mid-360 coverage → noisy normals → slope_cost up to ~0.19). Principled fix if it matters on the real robot: a **variance/confidence gate** — mark cells with high elevation `variance` (the layer exists) as UNKNOWN, not lethal, so sparse/noisy sensing never becomes a hard obstacle. Left as a next step (it's gradient, not lethal, for 96% of open floor).

### Waypoint navigation idea (`navigate_through_poses`) — analysis (operator proposal)

**Proposal:** instead of sending CFPA2's single far goal and planning one path to it, sample **waypoints from the planned path** (≈ 1 m) and navigate through them with replanning between — so as the robot passes each waypoint and senses more, a dynamic replan may open a route that was blocked before.

**Assessment — a good complementary robustness layer, with caveats:**
- **Real value:** (a) *committed incremental progress* — if the far goal momentarily fails to plan (transient inflation blob, goal edging into unknown), the robot still advances to the next achievable waypoint instead of aborting the whole goal (kills the abort→re-snap→churn cycle); (b) *per-meter replan* in the winding maze, where line-of-sight changes a lot each meter, can route around what looked blocked.
- **Already partly present:** `navigate_to_pose` already replans the full path ~1 Hz (BT RateController + ComputePathToPose), so fresh sensor data already updates the path. The upgrade is the *committed-waypoint* behavior, not replanning per se.
- **Limits:** (1) needs an initial path to extract waypoints — deep-unexplored has none, so CFPA2's snap-to-reachable must still provide the reachable goal first; waypoints navigate to *that*. (2) Does **not** fix geometric infeasibility (a too-tight corridor) — that's the inflation/footprint fix. (3) Adds bridge complexity + needs re-plan throttling to avoid churn when the CFPA2 goal moves each tick.
- **Implementation sketch (in `cfpa2_to_nav2_bridge.py`):** on a new/sufficiently-moved goal → `ComputePathToPose(goal, allow_unknown)` → sample poses at 1 m with tangent orientations → `NavigateThroughPoses(poses)`; fall back to `navigate_to_pose(goal)` if no path; throttle re-extraction (goal moved > X m or every N s); keep the action-status → `nav_status` feedback for CFPA2's fast-blacklist.
- **Simpler alternative:** a "lookahead carrot" — plan the path, send `navigate_to_pose` to the point ~3–5 m along it, re-evaluate each cycle. Less code, achieves committed near-progress, loses multi-waypoint replan.
- **Recommended order:** validate the inflation + extent-seek + override-disabled config first (a clean run31), THEN add `navigate_through_poses` as the robustness layer — so we know what it adds vs. masks.

### The core principle (carry to the real robot)

> **"traversable in elevation mapping" ≠ "reachable".** A cell free in the trav grid can be on the wrong side of a wall.

Two-layer handling:
1. **CFPA2 reads the inflated global costmap** (`planning_map_topic_suffix: /global_costmap/costmap`), not the raw trav grid → its notion of reachable == what Nav2 can plan.
2. **Reachability is a connectivity BFS with `allow_unknown=false`** (`goal_reachable` checks the frontier is in the BFS component from the robot; `distance_transform` only flows through known-free cells, cost < 70). A frontier behind a wall is a *different* component → rejected (`rej_reach`).

**The double-edged cost:** `allow_unknown=false` also rejects genuinely-reachable-but-unsensed regions (the −x corridor reads unreachable because unknown lies between). That is why the robot can't directly target −x and needs **snap-to-reachable** (target the nearest reachable cell *toward* the unreachable region → drive there → sense more → the region progressively joins the reachable component) + **extent-seek** (commit to that incremental push once one extreme is done).

> `allow_unknown=true` was tried and is a trap: the BFS leaks through unknown around the finite ends of thin walls → frontiers *behind* walls read reachable → Nav2's footprint planner can't path there → 8 s stuck-recovery → blacklist → cross-wall oscillation. **Do not re-enable it.**

---

## Real-robot deployment notes

### Sim-specific vs production
- **`SIM_GT_ODOM=1`** (ops2 launcher default) feeds `/odom/nav` from MuJoCo ground truth — **sim test harness only**. The real robot / HIL keeps **lifecycle-gated Fast-LIO** (z-drift fix is the gate, not GT odom). The trav grid is REAL perception in both.
- **`/mujoco/contacts`** (collision detection in `stuck_diagnoser.py`) is sim-only ground truth. On the real robot there is no equivalent; rely on the IMU/foot-force + the supervisor panic override.
- MuJoCo Mid-360 sim uses Risley CSV replay; the real Mid-360 mount is calibrated (pitch +15.10°, roll −2.11°).

### What carries over unchanged
- All CFPA2 logic (extent-seek, reachability, snap, IG-only fallback) — pure algorithm, no sim assumptions. The C++ port runs identically on the Orin (tick p95 ≈ 1.1 ms).
- The inflation/footprint tuning is geometry, not sim — but **re-verify on the real building**: the real corridor widths may differ from the SLAM mesh. If a real corridor is < 1.0 m, lower `inflation_radius` further OR narrow the footprint toward the real 0.31 m body width.
- `consider_footprint=true` is the real collision guard. With `inflation_radius=0.16` the robot drives ~0.16 m from walls in cost terms but the **footprint check** prevents any actual overlap. Validate clearance physically before trusting it in tight real corridors.

### Deployment checklist
1. `colcon build --packages-select cfpa2_collaborative_autonomy trav_cost_filters` (C++ + copy-installed Python).
2. Confirm the ops2 overlay is the one loaded (`cfpa2_config_overlay:=…/cfpa2_single_robot_ops2.yaml`) and the C++ binary is selected (`cfpa2_executable_suffix:=_cpp`).
3. Real robot: **do NOT set `SIM_GT_ODOM`**; ensure Fast-LIO lifecycle gate fires (z drift < 0.2 m after stand-up).
4. Verify `consider_footprint: true` + the footprint polygon match the real robot before entering tight corridors.
5. Watch `stuck_diagnoser` verdicts: `REAL_COLLISION` (body touching geometry — abort), `WALL_HIT` (wedged, not a real hit), `NO_PLAN`/`NO_GOAL`/`TRAV_CORRUPT`.
6. Re-tune `cfpa2_extent_target_x` if the real building's reachable extent differs (it must be a value the robot reliably crosses while beelining).

### Known residual / next levers (if a corridor still pinches on the real robot)
- Narrow `footprint` width 0.36 → 0.31 (real body) for extra turning room — at the cost of leg-spread margin.
- Lower `inflation_radius` below the inscribed radius (0.21) — Nav2 warns, but `consider_footprint` still guards; only do this if a real corridor is genuinely < 0.9 m.
- The `cfpa2_ig_mode: floodfill` single-call path is still a **stub returning 0** — implement it or remove the param (currently pinned to `local`; a footgun if someone flips it).

---

## Debug / automation toolchain (reusable)

- `scripts/debug/stuck_diagnoser.py` — auto-classifies why the robot stopped: `NO_GOAL / NO_PLAN / CONTROLLER_IDLE / WALL_HIT / TRAV_CORRUPT / REAL_COLLISION`. Subscribes recovery_event + self-detects stillness; probes costmap/trav/plan/cmd_vel + `/mujoco/contacts`; JSONL log. Auto-wired into the explore launch (`STUCK_DIAGNOSER=0` to disable).
- `scripts/debug/trajectory_monitor.py` — tracks min/max x (the ±35 oracle), path length, goal_met; JSON summary.
- `scripts/debug/explore_autorun.sh` — one-command run (GUI + RViz **on by default**) + monitor + diagnoser + heartbeat + final report. Logs in `logs/explore_autorun/<ts>/`.
- `scripts/debug/kill_sim.sh` — hardened two-host teardown (desktop + Jetson SSH) + verify (PID/comm/arg kills; `pkill -f` alternation silently misses; nav2 comm names truncate > 15 chars).

**Live probes used this session** (ad-hoc, kept for reference):
- `ComputePathToPose` action client to test if Nav2 can *plan* to a point (read-only, doesn't move the robot, doesn't fight CFPA2) — decisive for "is it unreachable geometry or just unexplored unknown?".
- Coarse ASCII renders of `/robot/traversability_grid` and `/robot/global_costmap/costmap` + connectivity BFS — to see the corridor structure and measure lethal-to-lethal vs cost<70 widths.
