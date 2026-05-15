# 3D Frontier Exploration Debugging — 2026-05-14

Day-long debug pass on `nav_test_3d_explore.sh` (demo_ramp scene + nvblox_frontend + CFPA2 3D IG). Robot kept getting stuck on the same goal forever, oscillating around a few cells without making progress. Six distinct compounding bugs found and fixed in sequence; this doc captures them in the order that mattered for diagnosis.

The fundamental insight (last bug, by far the most important): **the mean of frontier voxels in a ring-shaped frontier is geometrically at the centre of the ring, which is the robot**. Every other fix below was preliminary cleanup — without this one the system literally cannot explore.

## TL;DR sequence

1. Octomap-style trav_grid projection (drop fan-fill shortcut, query nvblox 3D state directly)
2. Mid-360 geometric blind zone — 3 m forced-FREE disk in trav_grid
3. Dead-frontier filter incompatible with 3D mode — skip it
4. trav_grid not persistent — switch to world-fixed 40 m × 40 m grid with `cls_persist_` accumulating across frames
5. 3D cluster volume never shrinks — z-band filter [-0.2, 1.5] m so air voxels above ramp don't dominate
6. **Ring-frontier centroid = robot location** — use farthest-from-robot frontier voxel instead of mean

Plus supporting:
- MuJoCo Mid-360 sim ray density (`MUJOCO_LIDAR_VT_SAMPLES` env: 20 → 96)
- ClusterTracker cross-frame identity matching for non-actionable detection
- Preflight script `set -o pipefail` bug on clean systems

## 1. Octomap-style trav_grid (drop fan-fill shortcut)

**Symptom**: free space leaking past walls. Cells behind the wall showed as FREE in trav_grid even though no rays reached them in 3D.

**Original design** (broken): per-scan polar fan-fill — bin every hit into N=720 angular bins keyed off sensor xy, compute `r_min[bin]` = closest hit per bin, mark cell as FREE if its 2D `(r, θ)` from sensor is `r < r_min[bin]`. The shortcut works when every angular bin has a hit, but Mid-360's non-repetitive rosette leaves bins empty per scan → `r_min[bin] = ∞` → fan extends to infinity → cells past unsampled bins get false FREE → `cls_persist_` locks the leak in forever.

**The right thing** (it was sitting right there in nvblox the whole time): just project nvblox's persistent 3D occupancy layer to 2D, octomap-style. nvblox's `integrateDepth` already does proper 3D raycasting per scan — free voxels get carved along ray paths, hit voxels get OCC log-odds, behind-wall voxels stay log_odds=0 (UNKNOWN). Querying that per column gives correct 2D occupancy with no leak.

Code in [`mapper_node.cpp`](../../src/collaborative_exploration/nvblox_frontend/src/mapper_node.cpp): replaced ~200 lines of fan-fill + persistent ray_covered + 1-cell dilation with a single column-query per cell against nvblox's `occupancy_layer`. Walls block rays in 3D ⇒ `free_bits == 0` behind walls ⇒ stays UNK. Slope/step filter on the per-column highest occupied z (`H[idx]`) still gates ramp-traversal vs cliff-block, which Octomap proper doesn't do.

**Lesson**: when you have a leak-proof 3D representation already (nvblox), don't reinvent its 2D projection with a 2D heuristic. The 3D Bayesian update IS the leak-proof part. 2D Bresenham/fan-fill on its own is fundamentally lossy w.r.t. 3D occlusion.

## 2. Mid-360 geometric blind zone

**Symptom**: empty trav_grid in a 3 m donut around the robot. Robot's immediate surroundings shown as UNKNOWN even though it's obviously open floor.

**Math**: Mid-360 V-FOV is -7° to +52° (asymmetric, mostly up). With sensor mounted at world z = 0.4 m on Go2W, the lowest beam (-7°) intersects the floor at horizontal distance `d = 0.4 / tan(7°) ≈ 3.25 m`. So even with infinite ray density there are **no Mid-360 ground returns within ~3.25 m radius of the robot** — geometric fact, not a tuning issue.

**Fix** ([`mapper_node.cpp:624-660`](../../src/collaborative_exploration/nvblox_frontend/src/mapper_node.cpp#L624-L660)): force a 3 m disk around the robot to FREE in `publish_traversability`. Preserves OCC verdicts so walls within 3 m stay OCC. Inside the disk: no Mid-360 evidence is even possible, so assume FREE unless explicit OCC exists.

**Side-effect that fired bug #3**: this fill caused all 3D cluster ground projections within 3 m to land in FREE space, breaking 2D-mode-style dead-frontier checks.

## 3. Dead-frontier filter incompatible with 3D mode

**Symptom**: `frontier_3d_test_node` found 5 valid 3D clusters, but CFPA2 reported `no_frontiers` every tick.

**Cause**: CFPA2's `_filter_dead_frontiers` checks that each frontier goal has ≥ 20 "live UNKNOWN" cells within a 0.40 m radius in the 2D trav_grid. Designed for 2D mode where frontier goals lie ON the FREE/UNK boundary. In 3D mode the goal is the GROUND-PROJECTION of a 3D cluster centroid, which lies INSIDE the FREE area (especially after the 3 m blind-zone fill above) → all neighbours FREE → `live_n = 0` → "dead frontier" → dropped.

**Fix** ([`cfpa2_single_robot_node.py:262-268`](../../src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_single_robot_node.py#L262-L268)): skip the filter entirely in 3D mode; the 3D extractor's `min_unknown_volume_m3 + min_frontier_voxels` already gate against trivial clusters.

## 4. trav_grid not persistent

**Symptom**: as the robot moves, cells behind it revert to UNKNOWN. The trav_grid is a "rolling window" pinned to robot pose.

**Original** (robot-centric): `ox = robot_xyz.x() - half_extent`. Window slides with robot → cells outside the current 20 m window get dropped → "memory" is at most 20 m wide and never accumulates.

**Fix** ([`mapper_node.cpp`](../../src/collaborative_exploration/nvblox_frontend/src/mapper_node.cpp)):
- 40 m × 40 m world-fixed grid (`trav_xy_extent_m: 40.0`)
- Origin locked from first odom (or from `trav_world_origin_x/y` params)
- `cls_persist_` member buffer keeps non-UNK values across frames; new-frame `cls == -1` (UNKNOWN) does NOT overwrite — only fresh observations win

Result: every cell the robot has ever observed retains its FREE/OCC class. The robot's path stays painted FREE behind it; walls stay painted OCC after the robot moves past them.

## 5. 3D cluster volume never shrinks — z-band filter

**Symptom**: after fixing 1–4, the robot moved a few metres, then froze. CFPA2 reported one giant 3D cluster (V=1085 m³, centroid at (5.4, -0.1, 0.5)) whose volume DID NOT DECREASE no matter how the robot moved. Goal was always the same projection, robot reached it within 0.6 m, CFPA2 blacklisted it, then immediately re-issued the same goal because the cluster was still huge.

**Cause**: `extract_3d_frontiers` counts all UNKNOWN voxels in the cluster's Voronoi-owned region, including voxels at z > 1.5 m — air voxels above the ramp / under the ceiling that a Mid-360 at z ≈ 0.4 m on the ground cannot probe (no rays go straight up far enough to hit anything, returns never come back, log_odds stays at 0). Those voxels are PERMANENTLY UNKNOWN regardless of where the robot drives. Including them in the cluster's IG makes the cluster look infinitely valuable forever.

**Fix** ([`frontier_3d.py:88-105`](../../src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/frontier_3d.py#L88-L105) and CFPA2 params): add z_band [-0.2, 1.5] m to `extract_3d_frontiers`. Mask `unknown = unknown & (z_world in band)` before clustering. Now the cluster's volume reflects only voxels the robot can plausibly observe by moving on the ground → volume actually shrinks as exploration progresses → CFPA2's "no progress" detection can work.

After this fix: V went from 1085 m³ (stuck) to 628 → 565 → 541 m³ as the robot moved forward 2 m. **First time** that volume meaningfully tracked exploration progress.

## 6. Ring-frontier centroid = robot location (the real one)

**Symptom**: even after fixes 1–5, robot would advance ~2 m and then stop. Cluster centroid stable at (5.4, 0, 0.5). Goal stable at "ramp base", robot reached it, no new motion.

**The key insight from the user**: *"the unknown is ring-shaped, so the centroid falls inside the known area?"*

Yes — and that's exactly what was happening, hidden in plain sight all along.

In a typical exploration state, the robot has carved a roughly circular FREE region around itself, surrounded by UNKNOWN. The **frontier voxels** (FREE adjacent to UNK) form a **shell** around the robot. The geometric centre of a shell sits AT the centre of the shell — i.e. roughly at the robot's current position, in the FREE region.

So `centroid_world = (xs.mean(), ys.mean(), zs.mean())` of the frontier voxel mask gives a point that's at the robot's xy. Projecting that to a ground goal returns a cell within ~1 m of where the robot already is. "Navigate to goal" = "stay put". Tracker can never detect progress because there is no progress.

**This is independent of bugs 1–5**: even with perfect 3D occupancy carving, z-band filtered IG, persistent grid, all of them — the mean-centroid metric is geometrically wrong for ring-shaped frontiers. And ring-shaped is the default topology of any incremental exploration.

**Fix** ([`frontier_3d.py:193-225`](../../src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/frontier_3d.py#L193-L225)): replace mean with **farthest-from-robot frontier voxel pick**. Added `robot_xy` parameter to `extract_3d_frontiers`; when supplied, pick the frontier voxel with maximum `(x - rx)² + (y - ry)²` as the cluster's "centroid". This always returns a point at the boundary in the direction of largest unexplored extent → goal lies somewhere the robot has never been → navigating there actually expands the frontier.

Behavioural change after this fix: robot pose immediately starts moving (vx=0.43 m/s, real translation), goals appear far from the robot (e.g. (7.8, -3.9) when robot is at (2, 1)), cluster volume actually tracks exploration progress, robot traverses the demo_ramp scene.

**Why this matters more than every other fix combined**: a frontier-based explorer that uses "mean of frontier voxels" as the goal is fundamentally broken for any topology except a sharply-pointed frontier (like a hallway extending in one direction). For any compact / wraparound free region — which is what you have at the start of every exploration, and for most of its duration — mean centroid = self-position. Fixing the carving (1–5) just lets the system reach this fundamental wall faster.

## Supporting fixes (touched today but ancillary)

### MuJoCo Mid-360 sim ray density

Hardcoded default in `mujoco_ros2_sensors.cpp:176-177`: `MUJOCO_LIDAR_HZ_SAMPLES=1000`, `MUJOCO_LIDAR_VT_SAMPLES=20`. 20 vertical rays across 59° V-FOV = 2.95° spacing → walls show 1-3 voxels tall in nvblox at 1 m. Bumped via env vars in [`nav_test_3d_explore.sh`](../../scripts/launch/nav_test_3d_explore.sh): `1024 × 96 = 98 304 rays/scan`, ~0.6° vertical resolution. Cloud point count 4880 → 23 000 per scan. Walls now fill all 10 z-layers from floor to z = 1 m. See [`feedback_mid360_sim_density.md`](../../../.claude/projects/-home-hanszhu-Research-Collab-QRC/memory/feedback_mid360_sim_density.md) memory.

### ClusterTracker (cross-frame cluster identity)

Added [`cluster_tracker.py`](../../src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cluster_tracker.py): per-frame extract_3d_frontiers returns fresh `Frontier3DCluster` objects with no identity. ClusterTracker matches new clusters to existing trackers via world-coord AABB overlap, accumulates volume history and attempt count, and exposes a `non_actionable` flag (attempt_count ≥ N and no volume shrink for stale_after_sec). CFPA2 skips non_actionable clusters → no more whack-a-mole on dead-end clusters.

Critical implementation detail: AABBs must be matched in **world coords**, not voxel-index coords. The voxels_3d grid is robot-centric (origin shifts with robot), so a stable world cluster has drifting voxel-index AABBs across frames. Tracker converts on update.

Event-driven attempt-crediting: hooked into coordinator's `_update_reached_goal_blacklist` via `_on_reached_blacklist` callback so each blacklist event = one genuine attempt (rather than incrementing every tick).

### Preflight kill `set -o pipefail` bug

Symptom: `nav_test_*.sh` prints "[preflight] killing stale processes..." then dies silently on a CLEAN system. `pgrep ... | awk ... | head -1` in `_preflight_stop_ros2_daemon` returns 1 from pgrep when no match → pipefail propagates → set -e aborts. Counterintuitively fires on a clean system (nothing to kill) — exact opposite of when you'd expect preflight to do anything. Workaround: `PREFLIGHT_KILL=0 ./scripts/launch/nav_test_3d_explore.sh`. Proper fix not yet committed. See [`feedback_preflight_pipefail.md`](../../../.claude/projects/-home-hanszhu-Research-Collab-QRC/memory/feedback_preflight_pipefail.md) memory.

## Files touched

- `src/collaborative_exploration/nvblox_frontend/src/mapper_node.cpp` (octomap-style projection, world-fixed grid, blind-zone fill, persistent cls)
- `src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/frontier_3d.py` (z-band filter, farthest-voxel centroid)
- `src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_single_robot_node.py` (skip dead-frontier in 3D, wire ClusterTracker + robot_xy)
- `src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cluster_tracker.py` (new)
- `src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_coordinator_node.py` (`_on_reached_blacklist` hook)
- `src/collaborative_exploration/cfpa2_collaborative_autonomy/config/cfpa2_single_robot.yaml` (loosened goal-reaching dead-band)
- `src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py` (40 m trav extent)
- `src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio.launch.py` (gate octomap_server on `nav_costmap_mode != "3d"`)
- `src/go2w/go2_gazebo_sim/rviz/nav_test.rviz` (disable legacy Map display)
- `src/go2w/go2w_config/config/nav/nav2_go2w_full_stack.yaml` (`xy_goal_tolerance: 0.40 → 0.50`, `yaw_goal_tolerance: 0.50 → 3.14`)
- `scripts/launch/nav_test_3d_explore.sh` (env vars: LD_LIBRARY_PATH, MUJOCO_LIDAR_*_SAMPLES)

## Verification

`nav_test_3d_explore.sh` on demo_ramp 2026-05-14 16:10–17:35:
- Robot actively navigates: pose changes from (1.8, 0) → (3.7, 0.8) → broader scene traversal
- Goal coordinates land FAR from robot (e.g. goal=(7.8, -3.9) at robot=(2, 1))
- Cluster volume meaningfully decreases over time (628 → 541 m³ in first minute)
- Cmd_vel non-zero throughout active exploration (vx=0.43 m/s observed)
- `cls_persist_` retains FREE behind the robot's trail; walls stay OCC

Open follow-ups not yet implemented:
- Slope-aware projection (push goal ONTO the ramp surface when cluster z>0.3 m, currently lands at ramp base)
- Per-axis frontier wedging (one goal per direction sector, so the robot doesn't just chase the single farthest voxel)
- Adaptive z-band ceiling (raise to z=2 m once robot reaches platform top, so platform-top exploration works)
