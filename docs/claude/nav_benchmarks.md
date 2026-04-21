# Nav Stack Benchmarking (Phase 5)

Rigorous headless benchmarking of the CMU autonomy stack on the `demo1` scene (12 m × 8 m inner room = **96 m² GT**). Goal: real-robot-safe configuration with zero wall contacts and ≥90% coverage in 120-second bounded sessions.

## Stack under test

**CMU autonomy stack** (FAR V-graph global planner + CFPA2 frontier exploration + Cartographer 2D SLAM + CHAMP locomotion).

Alternative evaluated: Fast-LIO2 + SC-PGO (see `slam_and_scenes.md`).

## FULL PASS criterion

```
PASS = outcome=completed ∧ coverage_ratio ≥ 0.90 ∧ contacts == 0 ∧ ¬tipped
```

5-trial samples = too small for reliability claims. 10-trial runs preferred for any "robust" claim.

## Infrastructure

### `scripts/bench/session_reporter.py` (~420 lines)

Bounded session metrics node. Subscribes to `/{ns}/map`, `/{ns}/odom/nav`, `/{ns}/odom/ground_truth`, `/mujoco/contacts`. Tracks per trial:
- Coverage: explored_area from OccupancyGrid free+occupied cells; `coverage_ratio = area / scene_area_m2` (CLI arg, default 96.0)
- Progress: distance, peak speed, start/end pose, peak roll/pitch, tip-over flag
- SLAM drift: peak/mean/final trans + yaw error between Cartographer odom vs ground-truth odom, computed as **relative drift from each stream's own start pose** (so map-frame vs world-frame origin offsets don't show up as fake error)
- Safety: named-geom attribution of every contact in `/mujoco/contacts`

JSON written every 10 s + final flush. Uses `os._exit` in `_finalize_and_exit` to bypass rclpy executor swallowing SystemExit.

### `scripts/benchmark_far_nav.sh`

Multi-trial headless runner. Env-overridable: `NUM_TRIALS`, `DURATION_SEC`, `OUT_DIR`, `SCENE_AREA_M2`, `COVERAGE_PASS_FRACTION`. Per-trial: targeted `pkill` cleanup → `timeout`-bounded `ros2 launch` → session_reporter JSON. Aggregate summary with FULL PASS criterion.

### Launch file additions (`nav_test_mujoco.launch.py`)

- `session_duration_sec`, `session_output_path`, `scene_area_m2`, `enable_wall_checker` launch args
- `ExecuteProcess(session_reporter.py)` + `OnProcessExit → Shutdown` handler
- `enable_wall_checker:=false` during benchmarks so the session runs to completion and captures ALL contact events (vs terminal wall_checker which aborts on first hit)

### Named collision geoms

`vlm_exploration_scene_no_artifacts.xml`: added `name=` attrs to 19 previously-unnamed `class="collision"` geoms so `/mujoco/contacts` reports `wall_X|FL_wheel_collision|...` instead of unattributable pair. Failure modes diagnosable (rear-scrape vs front-wedge vs head face-plant).

### `/mujoco/contacts` is benchmark-only

Audited: referenced in 4 files only (plugin publisher, launch-file wiring to validator exec, session_reporter, far_wall_checker). **Nothing in nav/planning/control/CHAMP path reads it.** Tuning results are real-robot transferable.

## Six initial 5-trial iterations

| Run | twd | infl | rot | width | speed | PASS | Zero cts | Contacts | Avg m² |
|---|---|---|---|---|---|---|---|---|---|
| baseline | F | 1 | F | 0.40 | 0.2 | 3/5 | 3/5 | 1668 | 95.34 |
| twd=true | T | 1 | F | 0.40 | 0.2 | 4/5 | 4/5 | 1092 | 89.86 |
| infl=2 | T | 2 | F | 0.40 | 0.2 | 4/5 | 4/5 | 78 | 101.91 |
| rot+w=.50 | T | 2 | T | 0.50 | 0.2 | 3/5 | 3/5 | 506 | 93.19 |
| **config A** | **T** | **2** | **T** | **0.40** | **0.2** | **5/5** | **5/5** | **0** | **97.85** |
| speed=.4 | T | 2 | T | 0.40 | 0.4 | 3/5 | 3/5 | 2748 | 102.38 |

All failures were spatially concentrated at `divider_h_east` until the speed bump.

### Diagnoses

- **Baseline trial 3 stuck** (1640 contacts, `divider_h_east`): FAR routed through narrow gap, localPlanner had no valid forward primitive, pathFollower emitted zero velocity while wheels ground against divider. Fix: `obs_inflate_size=2` (FAR re-routes around thin obstacles globally) + `twoWayDrive=true` (restores CMU stack's built-in reverse primitives for local stuck recovery). twoWayDrive was disabled in iter-1 to stop reverse-thrash from waypoint jitter; iter 2-5 FAR tuning had addressed that root cause, so the disable was stale.
- **infl=2 trial 1 rear scrape** (78 contacts, FR_wheel dominating): robot rotated with rear body swinging into wall. Fix: `checkRotObstacle=true` (localPlanner sweeps each rotation primitive through `/terrain_map` and rejects any whose rotational footprint intersects — pure LiDAR terrain cloud, real-robot safe). Was disabled in iter-1 as "too conservative near walls" but with `infl=2` the upstream plan already stays wider.
- **rot+w=.50 regression** (506 contacts, head_lower + FL_wheel face-planting `wall_east`): widening `vehicleWidth` to 0.50 m overconstrained path library in narrow spots. When no forward primitive was valid, pathFollower defaulted to "continue current direction at reduced speed", committing robot into `wall_east`. Revert to 0.40 m.
- **speed=.4 regression** (2748 contacts, new RL_thigh scrape + catastrophic trial_4 `wall_west` stuck): none of the safety checks are velocity-aware. Rotation sweep approves primitives on static geometry; at 0.4 m/s the robot carries ~2× momentum, overshoots approved primitive by ~4 cm per tick. SLAM drift also doubled (1.48 m → 3.02 m avg peak trans) — Cartographer struggles at faster locomotion + reverse maneuvers. **Pushing speed requires a velocity-aware safety envelope**, not just higher `maxSpeed`.

## Config A — winning knobs

All CMU-stack defaults or LiDAR-only; nothing sim-specific.

| Knob | Value | Role | Real-robot safe? |
|---|---|---|---|
| `twoWayDrive` | **true** | Reverse primitives in path library + pure-pursuit; local stuck recovery | ✓ CMU default |
| `checkRotObstacle` | **true** | Rejects rotation primitives whose swept footprint hits `/terrain_map` | ✓ terrain cloud is LiDAR only |
| `util/obs_inflate_size` | **2** | FAR V-graph inflates obstacle cells by 2 grid cells (~0.10 m) | ✓ V-graph from terrain cloud |
| `vehicleWidth` / `vehicleLength` | **0.40 / 0.70** | Path-library collision footprint | ✓ close to true Go2W dims |
| `far_max_speed` | **0.2 m/s** | Below point where position-based safety loses grip; real Go2W cruises 1-2 m/s | conservative |

**5-trial result:** 5/5 FULL PASS, 0 contacts, 97.85 m² avg (101.9% of 96 m² GT), 31.77 m avg distance, SLAM drift avg peak 1.48 m trans / 11.48° yaw.

## Iter 7 — 10-trial config A reliability

Reality check. The 5/5 result was an optimistic sample.

- **7/10 FULL PASS**, 8/10 coverage, 9/10 zero-contact
- **Two distinct failure modes** surfaced at n=10 hidden at n=5:
  1. **Trial 6 corner wedge** (1/10): pinned at (7.51, -0.46) — NE corner between `wall_east` and `wall_north`. First contact at t=64.88 s, then `RL_thigh_collision` scraped `wall_east` 1060 times over 55 s while distance froze at 17.3 m. Both forward and reverse primitives blocked at chosen angle — FAR emitted zero velocity and supervisor had nothing to do. **Local stuck-recovery can't escape a two-wall corner; needs global backtrack.**
  2. **Trials 4 and 9 SLAM-drift stalls** (2/10): yaw drift peaks 27.7° and 19.3°, means 23.2° and 15.4°. No wall contacts but map rotated enough that CFPA2's frontier picks landed wrong. Distance was normal; robot kept moving, just to wrong places.

JSONs: `/tmp/far_bench/run_cfgA_10trial/` + `summary.json`.

## Iter 8 — velocity-aware supervisor (cone v1) + speed=0.4

Standalone `scripts/runtime/velocity_safety_supervisor.py` between pathFollower and twist_bridge. Caps `cmd_vel` to `v_cap = sqrt(2·a_max·(d_nearest − d_safe))` where `d_nearest` is closest scan point within **±60° forward cone**. Enabled via `enable_velocity_supervisor:=true`; remap flips pathFollower's `/cmd_vel` to `cmd_vel_stamped_raw`, supervisor publishes supervised `cmd_vel_stamped` that twist_bridge consumes. `far_max_speed` auto-bumps to 0.4.

**5-trial result:** 4/5 FULL PASS, 1108 contacts in one failure.

- Coverage passes 5/5 (avg 98.57 m², best productivity) — faster speed outruns drift-induced stall.
- Front-on face-plants eliminated.
- Trial 2 failure: 1108 contacts on `divider_h_east`, rear-right dominating (`RR_calf_upper: 956`, `RR_wheel: 117`). Side-scrape during forward motion: robot drove *past* divider, rear-right corner caught edge. Forward cone doesn't see obstacles at 90°; point-robot model ignores rear extending 0.35 m behind LiDAR.

## Iter 9 — body-aware corridor supervisor (reverted)

**v1 attempt**: rectangular "swept corridor" check. Project each scan beam into body-centre frame (using `lidar_offset_x_m=0.16`), classify by region (ahead/behind/abreast). Hard-stop forward motion on abreast threat.

5-trial regressed to 3/5 PASS, 2417 contacts across 4 walls. Root cause: **scan sees robot's own legs**. LiDAR at (0.16, 0, 0.12) picks up hip/thigh cylinders at body-frame (0.19, 0.14) → satisfies `|x|<L/2 ∧ |y|<corridor_hw` → "abreast threat" → hard-stop on every rotation.

**v2 attempt**: self-return filter — any scan with `|x|<half_L ∧ |y|<half_W` is a self-return and skipped. Smoke passed (10.92 m / 30 s vs cone 8.85 m). 5-trial tipped over in trial 1. Reverted.

**Diagnosis**: abreast-threat hard-stop produces step-function `cmd_vel` discontinuities exceeding CHAMP's gait stability envelope. Stance phase needs ~0.2 s to settle each footfall; hard-stopping mid-stride at 0.4 m/s pitches base forward and robot loses standup pose. A softer variant (ramp-to-stop, or N-tick debouncer) might work. **Kept cone supervisor as shipped prototype; body-aware rewrite reverted.**

## Iter 10 — `maxYawRate 45 → 25` (reverted)

Hypothesis: aggressive yaw → SLAM scan-match drift → tip-over + coverage stall. Tried config A + `maxYawRate: 25`.

Catastrophic SLAM breakdown — 4/5 trials had Cartographer lock onto **180°-flipped submap**:
```
trial     PASS    dist m    contacts   drift m   drift °
trial_1    ✗      15.31       521       2.98     33.3°
trial_2    ✓     332.36         0       3.50    180.0°
trial_3    ✓       1.07         0       3.14    180.0°
trial_4    ✓    1529.54         0       3.15    179.9°
trial_5    ✗       0.65         0       3.22    177.5°
```

Nonsensical distance numbers are session reporter integrating `/odom/nav` pose jumps of ~0.25 m/tick (scan matcher oscillating between submap hypotheses at 50 Hz).

**Diagnosis**: CMU-shipped 2D Cartographer Lua uses pure scan matching against current submap (`use_odometry=false`). In near-symmetric rectangular room, scan facing +X looks almost identical to −X. During **long pure-rotation windows** (180° in 7.2 s @ 25°/s vs 4 s @ 45°/s), matcher has time to accumulate gradient for the flip. **Fast yaw was *protecting* against symmetry ambiguity, not causing it.** `maxYawRate=45` is right for this SLAM + scene; revisit only after Cartographer Lua gains odometry prior, or in asymmetric env.

## Final shipping state

`nav_test_mujoco.launch.py` at config A:
- `twoWayDrive: True`
- `checkRotObstacle: True`
- `util/obs_inflate_size: 2`
- `vehicleWidth: 0.40`, `vehicleLength: 0.70`
- `far_max_speed: 0.2` (flips to 0.4 if `enable_velocity_supervisor:=true`)
- `maxYawRate: 45.0` (iter-5 value, restored after iter-10 revert)
- `enable_velocity_supervisor: False` (default)

`scripts/runtime/velocity_safety_supervisor.py` present as cone-based prototype, disabled by default. Body-aware rewrite reverted; history in docstring for future.

**10-trial config A numbers:** 7/10 PASS, 9/10 zero contacts, 8/10 coverage ≥90%, avg explored 91.93 m² (95.8% of GT), SLAM drift avg peak 0.85 m / 9.0° yaw.

## Coverage/distance efficiency

Config A's safety costs efficiency: coverage-per-distance dropped from baseline 3.82 m²/m to **3.08 m²/m** (−19%).

Timeline (10 s checkpoints, trial 1):
```
config        t=10   t=20   t=30   t=40   t=50   t=60   t=70   t=80   t=90  t=100  t=110
baseline      54.5   72.0   82.4   82.8   83.2   83.3   89.3   89.5   89.5   89.5   89.5
config A      74.8   75.2   75.6   78.3   79.4   87.8   90.3   91.6   92.7   92.8   93.0
```

Time to 80 m² coverage: baseline ~27 s, config A ~46 s (70% slower). The extra distance is NOT loops — it's longer global paths from `obs_inflate_size=2` routing around thin obstacles + reverse-drive safety cycles. Legitimate safety detours.

A 0.4 m/s speed bump recovered efficiency on clean trials (111.57 m² avg, 118% of GT) but broke safety on 2/5. **Right solution is velocity-aware safety, not higher maxSpeed with static checks.**

## FAR teammate-awareness

Researched: **FAR is strictly single-agent.**

- `src/vendor/far_planner/route_planner/far_planner/src/far_planner.cpp:26-33` — subscribes only to self odom, terrain cloud, scan cloud, goal point. No peer topics.
- `graph_msger.cpp:99` — `robot_id` is commented-out dead code: `// const std::size_t robot_id = msg->robot_id;`. `graph_msger/robot_id` launch param never consumed.
- `far_planner.cpp:772-775` — dynamic obstacles from local scan only.

Two robots running FAR would collide with each other. CFPA2 (`cfpa2_collaborative_autonomy`) is the external frontier allocator for multi-robot; FAR executes whatever CFPA2 emits, blind to teammates.

**Implication for dual-robot**: multi-robot safety must come from (a) CFPA2 frontier partitioning that prevents overlap, or (b) mixing both robots' LiDAR into each planner's `/scan_cloud` before terrain analysis. Neither wired up today.

## Forward-bias tuning (iter 7, not yet benchmarked on main)

Three param changes to bias CMU stack toward forward driving:

| Param | Old | New | Effect |
|---|---|---|---|
| `localPlanner/dirWeight` | 0.02 | **0.5** | Penalises reverse primitives |
| `pathFollower/dirDiffThre` | 0.8 rad | **1.2 rad** | Raises heading-error before reverse mode |
| `far_planner/path_momentum_thred` | 10 | **25** | FAR commits harder to current path |

**Immediate next experiment**: 5-trial on main with these values.

## Why demo1 is unusually hard

Six compounding factors:
1. **Near-symmetric geometry** → Cartographer 2D scan matcher degeneracy (+X ≈ −X)
2. **Thin dividers (15 cm)** inflated 2-3× by voxel + FAR margins → 1 m corridor becomes ~0.5 m passable
3. **MuJoCo soft contact** → sub-mm penetration every step → 1000+ contact grinding looks catastrophic even for gentle bumps
4. **CMU stack was tuned on Gazebo** — hard LCP, clean LiDAR, thin voxels; MuJoCo thicker voxels + softer contacts break the tuning
5. **Quadruped body** — legs/wheels/head extend every direction; rectangular footprint approximation misses leg-swing during locomotion
6. **Forward/reverse ambiguity** with `twoWayDrive=true` + low `dirWeight=0.02` — frequent backward driving degrades SLAM + wastes distance

## Reactive RPP branch (`nav2-local-controller`) — shelved

Replaced `localPlanner + pathFollower` with ~270-line Python pure-pursuit (`scripts/runtime/reactive_rpp_controller.py`) reading raw LaserScan directly — no terrain_analysis, no path library, no fwd/rev switching.

**What worked:** SLAM drift dropped to **0.24 m / 3.0°** (3.5× better than config A). Forward-only driving keeps scan-match frame-to-frame consistency high.

**What didn't:**
1. **Close-range blindness** — `min_valid_range=0.20` filtered close obstacles to avoid own legs. At `v_max=0.2 m/s` braking distance is only 1 cm (v²/2a) → deceleration only activates at <11 cm. Points <20 cm filtered → NEVER saw close obstacle → straight into walls.
2. **LiDAR position** — moved L1 from top-body (0.16, 0, +0.12) to chin (0.29, 0, -0.04); lower height caused downward rays to hit ground within 1.5 m → terrain_analysis flooded with ground-as-obstacle → FAR broke. Reverted to head-top (0.29, 0, +0.12). Same ground-hit issue as real robot; real-robot fix is `min_height`/`max_height` z-band filter in `pointcloud_to_laserscan` (0.05-0.60).
3. **Waypoint-stop dead time** — v1 stopped at each FAR waypoint (goal_tol=0.25 m), waited for next. ~50% stationary, only 12 m in 120 s.
4. **Hard-stop without replan** — v2 added two-tier avoidance (hard stop 0.25 m, ramp 0.25-0.50 m). Stopping at wall without replan → just sits. Added rotate-after-1s to trigger FAR replan, still hit at t=10.5 s.
5. **No scan-available guard** — controller inits `corridor_clearance=inf`, starts driving immediately on first FAR waypoint before any scan arrives. Drives blind for 1-2 s.

**Takeaways:** SLAM improvement from forward-only is real and worth pursuing. Controller needs (a) wait-for-first-scan guard, (b) deceleration model for low speeds (hard cutoff, not sqrt(2ad) with 1 cm window), (c) integration with FAR replan cycle on stop, (d) LiDAR self-body filtering by ANGLE not RANGE.

## Known limits / next work

- **Corner-wedge stuck** (1/10 in config A) — need stuck detector watching `|cmd_vel| vs |odom_vel|`, publishes forced retreat waypoint. ~80 lines Python, nav2/MARBLE pattern. Cheapest single fix for highest-impact failure. **Schedule as immediate next experiment.**
- **SLAM yaw-drift stall** (2/10) — set `use_odometry=true` in `cartographer_sim_2d.lua`, feed wheel encoder into submap constraint builder. Harder: needs reading Lua config + tuning docs + 2-3 iterations. Fixes both trial 4/9 stall AND iter-10 flip failure. Unlocks independent `maxYawRate` tuning.
- **Velocity-aware supervisor** — 4/5 at 0.4 m/s with one diagnosable regression. Pick back up after stuck detector; try softer body-aware variant (ramp-to-stop or N-tick debouncer).
- **Efficiency tax** (−19% m²/m) — only speed bump recovers, needs supervisor + stuck detector robust first.
- **5-trial samples too small.** 10-trial config A (7/10) is best sample. Future "robust" claims need ≥10 trials with varied spawn poses (`spawn_x ∈ {2,4,10}`, `spawn_yaw ∈ {0, π/2, π}`).

## Commands

```bash
# One-time: rebuild plugin if contacts publisher missing
rm -rf ~/Collab_QRC/build/mujoco_ros2_control ~/Collab_QRC/install/mujoco_ros2_control
cd ~/Collab_QRC && colcon build --packages-select mujoco_ros2_control

# Smoke test (30 s / 1 trial)
NUM_TRIALS=1 DURATION_SEC=30 OUT_DIR=/tmp/far_bench/smoke ./scripts/benchmark_far_nav.sh

# Full 5-trial
./scripts/benchmark_far_nav.sh

# 10-trial reliability at config A
NUM_TRIALS=10 DURATION_SEC=120 OUT_DIR=/tmp/far_bench/cfgA_10 ./scripts/benchmark_far_nav.sh

# Inspect a trial
jq . /tmp/far_bench/<dir>/trial_1.json | less
```
