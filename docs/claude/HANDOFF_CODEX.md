# Codex Agent Handoff — Collab_QRC (2026-05-13)

## Who you're working with

**Hz** is a robotics PhD-level engineer. Communication style:
- Short, direct messages. Questions are signals, not requests for tutorials.
- When he says "我确定是X导致的" ("I'm certain it's X"), treat it as a strong hypothesis — validate or disprove with evidence, not conjecture.
- "继续" = resume exactly where you left off, no recap.
- "还是不行" = don't repeat the same fix. Go deeper, find root cause.
- He switches languages mid-sentence (Chinese/English) fluidly — match whichever he's using.
- Expects you to **run commands and show results**, not narrate plans.
- Hates: summaries of what you just did, multi-paragraph explanations when one line suffices, and hypothetical abstractions he didn't ask for.

---

## Repo: `/home/hz/Collab_QRC`

Multi-robot autonomy stack. Unitree Go2W (wheeled-leg) + Go2 (walking) quadrupeds.  
**ROS 2 Humble** + **MuJoCo 3.6** sim. Real robot runs on Jetson Orin.

Build:
```bash
micromamba activate cmu_env
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select <pkg>
# Python-only packages: no rebuild needed after edits (symlink-install)
```

Active branch: `main`. Current git status: `M scripts/real/deploy_noetic_to_jetson.sh` + new Point-LIO vendor sources.

---

## Current active work: 3D Frontier Exploration (nvblox path)

### Pipeline overview

```
MuJoCo LiDAR
    ↓
nvblox_frontend mapper_node (C++)
    ├─ /robot/voxels_3d       (VoxelGrid3D, BEST_EFFORT)   ← 3D occupancy
    └─ /robot/traversability_grid  (OccupancyGrid, TRANSIENT_LOCAL)  ← 2.5D nav surface
            ↓                                    ↓
    frontier_3d.py (CCL+Voronoi)       Nav2 StaticLayer costmap
            ↓                                    ↓
    frontier_3d_test_node              SmacPlannerLattice + MPPI
            ↓
    /robot/frontier_3d_markers  (RViz MarkerArray)
```

**NOT YET DONE**: frontier_3d clusters → CFPA2 utility scoring. Currently the 3D frontier extractor runs as a standalone validator only.

### Key source files

| File | Role |
|---|---|
| `src/collaborative_exploration/nvblox_frontend/src/mapper_node.cpp` | C++ mapper: ground filter, log-odds threshold, slope/step filter, median filter, TRANSIENT_LOCAL publisher |
| `src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/frontier_3d.py` | Pure Python 3D frontier extractor (no ROS) |
| `src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/frontier_3d_test_node.py` | ROS test node: subscribes voxels_3d + trav_grid, publishes markers |
| `src/go2w/go2w_config/config/nav/nav2_3d_costmap_overlay.yaml` | Nav2 overlay: replaces octomap costmap with traversability_grid |
| `src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio.launch.py` | Main sim launch (loads 3D overlay when `explore_3d:=true`) |
| `docs/claude/nvblox_3d_frontier.md` | All 8 bug fixes, architecture diagram, math formulation |

---

## mapper_node.cpp — what was changed and why

All changes are in the `cloud_cb` callback and `publish_traversability` method.

### Fix 1: Ground point pre-filter (scatter noise)
LiDAR grazing returns hit the floor at various world-Z heights → scattered voxels → salt-and-pepper in traversability.

```cpp
// Parameter: ground_z_max_m_ = 0.15
// In cloud_cb, before nvblox integration:
const float gz_max = static_cast<float>(ground_z_max_m_);
const Eigen::Matrix3f R = T.linear();
const Eigen::Vector3f t = T.translation();
// world_z = R(2,0)*x + R(2,1)*y + R(2,2)*z + t.z()
// skip point if world_z < gz_max
```

### Fix 2: Log-odds threshold (occ_lo_thresh_)
Default `lo > 0.0f` marks a voxel as surface after just 1 LiDAR hit. Raised to `0.7` (≈2 hits).
```cpp
// Parameter: occ_lo_thresh_ = 0.7
// In surface-finding pass: if (lo > occ_lo_thresh_) { surface_z = voxel_z; }
// Same threshold in clearance pass.
```

### Fix 3: 3×3 median filter (residual noise)
After traversability grid computation, apply median filter to remove isolated noise cells.
```cpp
std::vector<int8_t> filtered(cls);
for (int j = 1; j < nxy-1; ++j)
  for (int i = 1; i < nxy-1; ++i) {
    int8_t nb[9]; int n=0;
    for (int dj=-1; dj<=1; ++dj)
      for (int di=-1; di<=1; ++di)
        nb[n++] = cls[(j+dj)*nxy+(i+di)];
    std::sort(nb, nb+9);
    filtered[j*nxy+i] = nb[4];
  }
cls = std::move(filtered);
```

### Fix 4: Robot footprint force-FREE
Robot's own body creates occupancy around it → traversability shows robot cell as OCC.
```cpp
// Force 3×3 cells around robot position to FREE (0)
const int ci_r = (robot_xyz.x() - ox) / vs;
const int cj_r = (robot_xyz.y() - oy) / vs;
for (int dj=-1; dj<=1; ++dj)
  for (int di=-1; di<=1; ++di) {
    int ci=ci_r+di, cj=cj_r+dj;
    if (ci>=0 && ci<nxy && cj>=0 && cj<nxy)
      cls[cj*nxy+ci] = 0;
  }
```

### Fix 5: Slope check 3-cell baseline (ramp = staircase bug)
0.10 m voxel quantization + adjacent-cell slope check: ramp at 30° → dh=0.10m/0.10m=1.0 > tan(30°)=0.577 → blocked. Fix: use 3-cell (0.30 m) baseline for slope, keep 1-cell for step.
```cpp
const int sw = 3;  // 0.30 m at vs=0.10
// step check: abs(h_adj - h_center) > step_max_m → OCC
// slope check: abs(h_sw - h_center) / (sw * vs) > tan(smax) → OCC
```

### Fix 6: TRANSIENT_LOCAL publisher
Nav2 StaticLayer subscribes after mapper already published → misses first (and only) map update → robot has no costmap at startup.
```cpp
auto trav_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
trav_pub_ = create_publisher<OccupancyGrid>("traversability_grid", trav_qos);
```
Also required: `map_subscribe_transient_local: true` in `nav2_3d_costmap_overlay.yaml`.

### Fix 7: Wrong map_topic in Nav2 overlay
`RewrittenYaml` only rewrites keys explicitly listed in `param_rewrites` — it does NOT do free-form string substitution. The YAML had `/robot_a/traversability_grid` hardcoded.  
Fix: change YAML to `/robot/traversability_grid` AND add `"map_topic": f"/{robot_ns}/traversability_grid"` to `param_rewrites` in the launch file.

### Fix 8: Ramp slope baseline (same as Fix 5, applied to second slope window too)
See Fix 5. Both the forward and lateral slope checks need the `sw=3` baseline.

---

## frontier_3d.py — algorithm + recent fixes

**Algorithm** (pure Python, no ROS):
1. `frontier_mask = FREE & binary_dilation(UNKNOWN, struct6)` — 6-face adjacency
2. `labels, n = ndimage.label(frontier_mask, struct26)` — 26-conn CCL
3. **Pre-filter**: drop clusters with `N < min_frontier_voxels` (default 50) BEFORE Voronoi
4. **Border margin**: zero frontier voxels within `border_margin_cells=3` of grid edge
5. Voronoi: `distance_transform_edt(seeds==0, return_indices=True)` → `owner[voxel]`
   - Optional `geodesic_voronoi=True`: use `skimage.segmentation.watershed` — no wall-crossing, ~4× slower
6. `owner_unk = where(UNKNOWN, owner, 0)` → `bincount` → `vol_m3 per cluster`
7. Frontier area: vectorized face-count (Klette & Rosenfeld 2004)
8. Volume filter: keep `V > min_unknown_volume_m3` (default 1.0 m³, interpretation B)

**Current output** (robot at start position, ~half scene unexplored):
```
6 clusters (total 1128.9 m³)
  #1 N=23187, V=918.5m³, A=377m²  →ground(+9.69,+3.68)   ← main unexplored region
  #2 N=155,   V=177.1m³, A=4.8m²  →ground(+10.79,+8.38)  ← suspect: low A/V ratio
  #5 N=231,   V=15.2m³,  A=6.9m²  →ground(+12.99,+6.68)
  #4 N=69,    V=7.2m³,   A=2.7m²  →ground(+8.49,+3.38)
  ...
```

Was 48 clusters before fixes (many N=1–4 owning 50–176 m³ via Euclidean wall-crossing).

**Known remaining issue**: Cluster #2 has A=4.8 m² for 177 m³ — low area-to-volume ratio suggests Euclidean Voronoi is still attributing unknowns through a wall. Fix: `geodesic_voronoi=True` or `min_frontier_voxels` tuning. Not critical for demo.

---

## What is NOT done (priority order)

### P1: Integrate 3D frontier into CFPA2 utility scoring
Currently `frontier_3d.py` is a standalone validator. The coordinator needs to call it.

Target file: `src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_single_robot_node.py`

The CFPA2 single-robot node currently:
1. Subscribes `/robot/map` (OccupancyGrid) 
2. Runs 2D frontier extraction
3. Scores frontiers by 2D IG estimate
4. Sends best frontier to Nav2 as `goal_pose`

To integrate 3D:
- Subscribe `/robot/voxels_3d` (VoxelGrid3D, BEST_EFFORT)
- On each tick, call `extract_3d_frontiers()` → get `Frontier3DCluster` list
- Replace or augment 2D IG with `cluster.unknown_volume_m3` 
- Use `cluster.centroid_world` as goal (already has ground projection via `project_to_traversability_goal`)
- The ground-projected `(gx, gy)` is the `goal_pose` to Nav2

Key design question Hz hasn't decided yet: **replace** 2D frontier scoring entirely, or use 3D volume as a multiplicative weight on top of 2D candidates?

### P2: S3/S4 experiment
Run full exploration with 3D IG enabled vs 2D baseline. Compare:
- Does robot navigate up the ramp to the upper platform?
- Coverage at convergence
- Exploration time

Launch: `./scripts/launch/nav_test_3d_explore.sh` (check if this script exists, might need creation).

### P3: coverage_stagnant fires too early
`exploration_metrics_logger.py` has `coverage_stagnant_threshold_pct=0.5%` over `30s`. With slow 3D planning, this fires before the robot has finished. Raise threshold or window.

### P4 (deferred): Jetson port
nvblox_frontend is ROS 2 Humble C++. Jetson runs ROS 1 Noetic for gbplanner3. These are parallel stacks, not yet integrated.

---

## Debug workflow Hz uses

**Standard debug loop:**
1. `ros2 topic hz /robot/<topic>` — verify data is flowing at expected rate
2. `ros2 topic echo /robot/<topic> --no-arr -n 1` — inspect one message
3. `ros2 topic info /robot/<topic> -v` — check publisher_count > 0 (catches dead topics)
4. Look at the lowest layer first. If "everyone says they're publishing correctly", read the source of the lowest component.
5. `tail -f /tmp/<node>.log` for nodes launched with `> /tmp/xxx.log 2>&1 &`

**Common pitfalls in this repo** (from CLAUDE.md golden rules):
- QoS mismatch: publisher BEST_EFFORT + subscriber RELIABLE → silent data loss
- TRANSIENT_LOCAL: any latched topic (costmap, map) must use it on both ends
- Namespaced TF: `/tf` is always empty in dual-robot; must remap to `/{ns}/tf`
- `publisher_count=0` on a topic: node subscribed to wrong (absolute vs relative) topic name
- `RewrittenYaml` only rewrites keys in `param_rewrites`; won't substitute arbitrary strings in values
- MuJoCo zombie: kill before re-launch (`pkill -f mujoco` or check `pgrep -fa mujoco`)
- `colcon build` needed for C++ changes; Python symlink-install = instant

**Test node restart pattern:**
```bash
pkill -f frontier_3d_test_node
source /opt/ros/humble/setup.bash && source /home/hz/Collab_QRC/install/setup.bash
ros2 run cfpa2_collaborative_autonomy frontier_3d_test_node \
  --ros-args -p robot_namespace:=robot \
             -p min_frontier_voxels:=50 \
             -p border_margin_cells:=3 \
  > /tmp/frontier_3d_test.log 2>&1 &
tail -f /tmp/frontier_3d_test.log
```

---

## Key ROS message types

```
nvblox_frontend_msgs/VoxelGrid3D:
  std_msgs/Header header
  float32 voxel_size          # typically 0.10 m
  uint32  size_x, size_y, size_z
  geometry_msgs/Point origin  # world coord of voxel (0,0,0) corner
  int8[]  data                # row-major (z,y,x): -1=unknown, 0=free, 100=occ
```

Layout: `data[z*ny*nx + y*nx + x]` → reshape as `np.frombuffer(bytes(v.data), dtype=np.int8).reshape(nz, ny, nx)`

---

## Env / launch

```bash
# Sim with 3D frontier:
./scripts/launch/nav_test_mujoco_fastlio.launch.py  # or equivalent sh wrapper
# with explore_3d:=true to load nav2_3d_costmap_overlay.yaml

# Check sim is alive:
ros2 topic hz /robot/voxels_3d          # expect ~1 Hz
ros2 topic hz /robot/traversability_grid # expect ~1 Hz

# frontier_3d_test_node log:
tail -f /tmp/frontier_3d_test.log

# CFPA2 status:
ros2 topic echo /robot/exploration_status --no-arr
```

---

## Formal math (for reference in CFPA2 integration)

**Walkable surface** (Wermelinger 2016):
```
S = ∂F ∩ K(α_max, s_max, c_min, r_robot)
  where ∂F = boundary of free space
        α_max = max slope angle (30° for Go2W)
        s_max = max step height (0.15 m)
        c_min = min clearance (robot_height + margin)
        r_robot = footprint radius
```

**Frontier 3D cluster volume** (interpretation B, Dai 2020):
```
V(C) = vs³ × |{u ∈ UNKNOWN : owner(u) = C}|
```
where `owner` is determined by Euclidean (fast) or geodesic (accurate) Voronoi.

**Frontier surface area** (Klette & Rosenfeld 2004):
```
A(C) = vs² × Σ_{v ∈ frontier(C)} |{face-neighbours of v that are UNKNOWN}|
```



# Session Summary:

### P1: Integrate 3D frontier into CFPA2 utility scoring — CODE DONE, NOT VALIDATED
`cfpa2_single_robot_node.py` now calls `extract_3d_frontiers()` when `ig_dimension=3d` and `/robot/voxels_3d` is available. Falls back to 2D if voxels not yet received. Ground-projected centroid used as goal_pose, `unknown_volume_m3` used as IG.

New parameters (all have defaults, no launch arg changes needed):
- `frontier_3d_min_unknown_volume_m3` (1.0)
- `frontier_3d_min_frontier_voxels` (50)
- `frontier_3d_border_margin_cells` (3)
- `frontier_3d_geodesic_voronoi` (false)
- `frontier_3d_goal_search_radius_m` (2.0)

**Remaining**: full sim smoke test. Last run used stale mapper_node binary → ramp still stairs. Must relaunch clean.

---

## Session log

### 2026-05-13
- P1 code integration done (cfpa2_single_robot_node.py)
- First run: ramp still discretized as stairs → diagnosed as stale C++ binary (mapper_node not relaunched after rebuild)
- Live topic inspection confirmed frontiers DO exist: 639 2D cells, 1 large 3D cluster (918.5 m³, centroid ~(3.74, 0.16, 0.49))
- "No frontier on open ground" was visual misread — all frontiers merged into one big cluster
- Next: clean relaunch → verify ramp → full 3D explore smoke test
- If ramp still stairs after relaunch: inspect per-cell height deltas to check if sw=3 baseline is sufficient

### 2026-05-13 (cont.)
- Relaunched with rebuilt mapper_node binary. Noise significantly reduced (median filter + log-odds working).
- **New issue**: voxel grid has almost no FREE voxels; free space only appears near walls. Suggests free-space carving (raycasting along beam to mark traversed voxels as free) is broken or missing. This starves the frontier mask (`FREE & dilate(UNKNOWN)` → near-empty).
- **Ramp still blocked**: confirmed not stale binary. Likely related to free-space issue — if voxels along ramp surface never become FREE, slope check has no continuous surface to evaluate.
- Suspected root causes: (a) ground_z_max filter skipping low-z points prevents those voxels from ever being updated (stuck at UNKNOWN), (b) mapper_node may only increment log-odds on hit without decrementing on ray-through, (c) nvblox integration call may not perform free-space carving at all.
- Next: audit mapper_node.cpp free-space carving logic, check ground_z_max interaction with ray updates.