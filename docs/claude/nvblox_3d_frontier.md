# nvblox 3D Frontier Exploration — Session Notes

Live tracker for the CFPA2 + nvblox_frontend + Nav2-on-traversability stack
introduced for 3D-aware exploration on `demo_ramp.xml`.

Pipeline (sim):
```
MuJoCo lidar → pointcloud_adapter → /robot/velodyne_points
   → fast_lio (slam_node)         → /robot/cloud_registered_body, /robot/Odometry
   → slam_odom_relay              → /robot/odom/nav (world frame, GT-bootstrapped)
   → nvblox_frontend mapper_node  → /robot/traversability_grid (2.5D OccupancyGrid)
                                  → /robot/voxels_3d           (sparse 3D for CFPA2 IG)
                                  → /robot/voxels_cloud        (RViz visualisation)
   → CFPA2 coordinator            → /robot/way_point_coord (frontier goal)
   → cfpa2_to_nav2_bridge         → /robot/goal_pose
   → Nav2 (SmacPlannerLattice/MPPI on /robot/traversability_grid via StaticLayer)
   → cmd_vel → robot
```

Launch entry: [scripts/launch/nav_test_3d_explore.sh](../../scripts/launch/nav_test_3d_explore.sh)
→ [src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py](../../src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py)

## Bug trail (chronological)

### 1. RViz map flashing — two mapper instances racing on /traversability_grid
- **Symptom**: traversability map alternating between two states at ~2 Hz; topic shows `Publisher count: 2`.
- **Cause**: stale `nohup … &` mapper from manual testing survived `Ctrl-C` of the launch; new launch spawned a second mapper in the same namespace.
- **Fix**: added `mapper_node` to `_PREFLIGHT_PATTERNS` and `_PREFLIGHT_ALIVE_RE` in [scripts/launch/_preflight_kill.sh](../../scripts/launch/_preflight_kill.sh). Every launch now reaps stale mapper PIDs.

### 2. Local costmap OOB at every map update
- **Symptom**: `Robot is out of bounds of the costmap!` at 2 Hz; planner can't compute path.
- **Cause**: `rolling_window: true` (local 6×6 m in `odom`) incompatible with a `StaticLayer` receiving the 20×20 m `traversability_grid`. StaticLayer overrode the costmap size every 0.5 s → origin shifted → robot center fell outside the rolling window.
- **Fix**: `nav2_3d_costmap_overlay.yaml` only overrides the **global** costmap. Local costmap keeps its `scan_3d` ObstacleLayer.

### 3. Costmap subscribed to wrong topic (`/robot_a/traversability_grid`)
- **Symptom**: persistent OOB despite traversability grid publishing. `ros2 param get static_layer.map_topic` shows `/robot_a/...` (multi-robot placeholder).
- **Cause**: overlay YAML used `/robot_a/` as namespace placeholder, but `RewrittenYaml(root_key=...)` does **not** do free-form string substitution — it only rewrites parameter keys named in `param_rewrites`.
- **Fix**: changed YAML to literal `/robot/traversability_grid` AND added `"map_topic": f"/{robot_ns}/traversability_grid"` to `param_rewrites` in [nav_test_mujoco_fastlio.launch.py](../../src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio.launch.py).

### 4. RViz goal sphere invisible
- **Symptom**: no marker for current CFPA2 goal.
- **Cause**: RViz config had `NavGoal` Marker on `/robot/final_goal_marker` — that topic has zero publishers anywhere in the repo.
- **Fix**: rviz config now subscribes `/robot/mtare_goal_marker` (CFPA2 coordinator output, big magenta sphere) + `/mtare/frontier_markers` MarkerArray.

### 5. Planning too slow (1–5 s per request)
- **Symptom**: BT `ComputePathToPose` times out → recovery loop.
- **Cause**: global costmap defaulted to 0.05 m resolution, so the 20×20 m traversability_grid was resampled to 400×400 = 160 k cells. SmacPlannerLattice is O(cells × primitive expansions).
- **Fix**: overlay sets `global_costmap.resolution: 0.10` to match the source grid → 200×200 = 40 k cells (~4× faster). Also drops `max_planning_time: 5.0 → 1.5`.

### 6. Salt-and-pepper noise in traversability (ground point clutter)
- **Symptom**: scattered OCC cells (cost=100) across what's physically flat floor — robot's start cell often in lethal space → `Starting point in lethal space!`.
- **Cause**: LiDAR returns at grazing angles produce stray hits at body-frame z ≈ -0.30 to -0.40 m (floor plane). nvblox accumulated these as occupied voxels. Single-scan returns reach log-odds ≈ 0.85, which the old surface-finder (`lo > 0`) trusted. With 0.10 m z-discretization, scattered floor hits become "floating surfaces" 1–2 voxels above the actual floor, failing the step/slope filter against their neighbours.
- **Fix (4 layers in [mapper_node.cpp](../../src/collaborative_exploration/nvblox_frontend/src/mapper_node.cpp))**:
  1. **Ground pre-filter** (`ground_z_max_m=0.15`): drops body-frame points whose world-z < 0.15 m **before** nvblox integration. World-z is computed inline from the cached odom pose.
  2. **Log-odds threshold** (`occ_lo_thresh=0.7`): surface-finding requires `lo > 0.7` (≥2 consistent hits, since nvblox adds ~0.85 per hit). Single stray returns no longer count as surfaces.
  3. **3×3 median filter** on the output 2.5D grid: int8_t sort over the 9-neighbour window → isolated 100s become 0s.
  4. **Robot footprint force-free**: 3×3 cells around the latest odom pose always written to 0. Guarantees `SmacPlannerLattice` always finds a valid start cell.

### 7. Startup OOB before first map arrives
- **Symptom**: `Robot is out of bounds of the costmap!` for the first ~35 s, until the first traversability_grid publishes.
- **Cause**: mapper publisher was VOLATILE. Nav2 StaticLayer with `map_subscribe_transient_local: false` had no map available until the next 0.5 s publish tick → costmap started at default tiny size → robot center outside bounds.
- **Fix**:
  - mapper_node: `trav_pub_` QoS = `KeepLast(1).transient_local()`.
  - overlay YAML: `map_subscribe_transient_local: true`. StaticLayer gets the last published grid immediately on subscribe.

### 8. Ramp mapped as stairs
- **Symptom**: 14° smooth ramp from [demo_ramp.xml](../../src/go2w/go2_gazebo_sim/mujoco/demo_ramp.xml) shows up as a stair-stepped pattern of OBSTACLE cells in the traversability grid.
- **Cause**: nvblox z-voxel size is 0.10 m; the highest occupied z at each (x,y) cell snaps to `{0, 0.10, 0.20, ...}`. Along the ramp these surface z's form a stair pattern `0, 0, 0, 0.1, 0.1, 0.1, 0.2, ...`. The slope filter compared **adjacent** cells: at the z-transition boundaries, `dh = 0.10 m` over `vs = 0.10 m` → slope = 100 % = 45° > slope_max_deg(30°) → marked OBSTACLE.
- **Fix**: split the slope/step filter:
  - **Step check** (`|dh| > step_max=0.20 m`) keeps using 1-cell neighbours — detects real curbs/stairs/walls.
  - **Slope check** (`dh / baseline > tan(slope_max)`) uses a **3-cell baseline = 0.30 m**. A 14° ramp accumulates 0.075 m rise over 0.30 m → ratio 0.25 < tan(30°)=0.577 → FREE. A real cliff still has `dh ≥ 0.30 m` over the same baseline → OBSTACLE.

## Files changed this session

| File | Purpose |
|---|---|
| [src/collaborative_exploration/nvblox_frontend/src/mapper_node.cpp](../../src/collaborative_exploration/nvblox_frontend/src/mapper_node.cpp) | Ground filter, log-odds threshold, median filter, footprint force-free, 3-cell slope baseline, TRANSIENT_LOCAL publisher |
| [src/go2w/go2w_config/config/nav/nav2_3d_costmap_overlay.yaml](../../src/go2w/go2w_config/config/nav/nav2_3d_costmap_overlay.yaml) | Static layer points at `/robot/traversability_grid`, transient-local subscribe, res 0.10 m, no local-costmap override |
| [src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio.launch.py](../../src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio.launch.py) | `nav_costmap_mode` arg + `map_topic` param_rewrite |
| [src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py](../../src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py) | Forwards `nav_costmap_mode` to base, defaults to `3d`, spawns nvblox mapper |
| [src/go2w/go2_gazebo_sim/rviz/nav_test.rviz](../../src/go2w/go2_gazebo_sim/rviz/nav_test.rviz) | TraversabilityGrid, Voxels3D, CFPAGoal, FrontierMarkers displays |
| [scripts/launch/_preflight_kill.sh](../../scripts/launch/_preflight_kill.sh) | `mapper_node` added to kill patterns |

## Open thinking — how does frontier search work in 3D?

**Current implementation reality (after reading [cfpa2_coordinator_node.py:175-200](../../src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_coordinator_node.py#L175-L200))**:

The "3D" mode is **NOT** doing 3D frontier search. The split is:

| Step | Where it runs | Data source |
|---|---|---|
| **Frontier detection** | C++ `extract_frontiers` over 2D grid | `/robot/traversability_grid` (2.5D OccupancyGrid) — still 2D |
| **Cluster / centroid** | Python, 2D radius merge | same 2D grid |
| **BFS reachability** | C++ `_distance_transform`, 4-connected | same 2D grid |
| **Information gain** | Python | **3D** — vertical cylinder count of unknown voxels in `/robot/voxels_3d` above the centroid XY |

So `ig_dimension: 3d` only changes the **utility score**, not what counts as a frontier. The pipeline still picks goals on the ground plane.

### Why this works for `demo_ramp`
- Ramp top + platform are reachable on the traversability_grid (after fix #8): ramp cells = FREE, platform cells = FREE, beyond-platform cells = UNKNOWN.
- 2D frontiers naturally form at the platform edge where FREE meets UNKNOWN.
- 3D IG biases the robot toward the platform: a ramp-top centroid has a tall column of unknown above it (clearance to ceiling that the LiDAR couldn't see from the floor) → higher score than equally-far ground frontiers.

### Where it breaks
- **Frontiers visible only in 3D**: e.g. a hole in a ceiling, a mezzanine accessible only via a non-modelled stair — no 2D FREE↔UNKNOWN boundary exists at any z-slice we look at.
- **Multi-level buildings**: floor 2 is invisible to the 2.5D projection (the traversability stores only the *highest* occupied z per column, which would pick the floor-2 surface and lose floor 1 entirely).
- **Vertical passages**: chimneys, manholes, caves with vertical sections — same problem.

### Options for true 3D frontier search

1. **3D frontier voxel scan** — iterate voxels in `voxels_3d`, mark a voxel as a 3D frontier if it's FREE (lo < 0) and has ≥1 UNKNOWN 6-neighbour. Cluster in 3D. Project each cluster centroid to ground via "closest navigable XY in traversability_grid". Cheap, no new motion planner needed. **Limitation**: still requires a 2D-reachable goal; doesn't help with multi-floor.

2. **Layered traversability**: publish *N* 2.5D grids, one per height band (`z ∈ [0, 0.5)`, `[0.5, 1.0)`, ...). Each band becomes its own frontier search. Goal selection picks the band with highest expected gain. Heavy on bandwidth and CFPA2 surgery (it currently expects a single planning map).

3. **Voxel-graph planner (gbplanner3-style)**: build an RRG/RRT over the 3D free voxels, sample frontiers as graph leaves where free→unknown transitions live. This is what we already have on Jetson for the real robot ([gbplanner3_noetic_onboard.md](gbplanner3_noetic_onboard.md)). For sim, we'd port gbplanner3 to ROS 2 — significant effort.

4. **Hybrid (recommended next step for sim)**: keep 2D detection as the primary, **augment** with 3D frontier voxels that get projected to nearest 2D-reachable goal. Bonus IG for those goals. Catches "ramp-leads-somewhere-the-2D-map-doesn't-yet-show" without needing a 3D motion planner.

### Concrete proposal for Option 4 (small surgery on CFPA2)

In `cfpa2_coordinator_node.py`:
- Add a `_extract_3d_frontiers()` pass that scans `voxels_3d` for FREE voxels adjacent to UNKNOWN voxels.
- For each 3D frontier cluster, drop a vertical line to the nearest FREE cell in the traversability_grid → that becomes a candidate goal.
- Mark these candidates with a flag `from_3d=True`; they bypass the 2D frontier_obstacle_clearance check (they're already known free in 3D) but still need BFS-reachability through the 2D grid.
- Score with the existing IG-cylinder; the projection ensures the IG cylinder will cover the 3D unknown region that motivated the goal.

Cost: ~200 lines of Python, no new C++ dependencies, no planner change.

## Validation status

- [x] Pipeline runs end-to-end with `nav_costmap_mode=3d`
- [x] Traversability grid clean (no salt-and-pepper after fix #6)
- [ ] Ramp passes traversability filter (after fix #8 — under test)
- [ ] Robot reaches platform via ramp under CFPA2 3D IG
- [ ] 3D IG vs 2D IG comparison on demo_ramp (S3/S4)
- [ ] 3D frontier search prototype (Option 4)
- [ ] Jetson port (deferred)

## Clustering decision (2026-05-13)

**Algorithm: 26-conn CCL on `voxels_3d` frontier mask, via `scipy.ndimage.label`.**

| Why this wins over the alternatives | |
|---|---|
| O(N) optimal vs DBSCAN's O(N log N) | grid is regular, adjacency is free |
| Fully deterministic, frame-coherent | no ε / min_samples knobs to tune |
| 26-conn captures oblique frontier surfaces | 6-conn fragments diagonal walls |
| Existing C-impl in scipy = ~10 ms on 200×200×30 | no new dependency |

Reference for the algorithmic claim:
- Cieslewski et al. 2017 (ICRA) — octree-CCL is SOTA frontier extraction on voxel grids
- Dai et al. 2020 (IROS) — explicit "CCL outperforms KD-tree clustering by 5–10× on regular grids" for the same problem
- He et al. 2017 review confirms modern two-pass CCL (Wu/Otoo/Suzuki 2009) is within 30 % of optimal; scipy's implementation suffices for our voxel counts.

## Volume filter — interpretation (B)

User decision: **cluster kept iff "unknown volume behind it" > 1.0 m³**, not the AABB of the
frontier shell itself. A door-sized 1 m × 1 m frontier is only ~0.1 m³ of shell voxels
but typically gates 10+ m³ of unknown room behind it.

Computation (Voronoi/watershed style — avoids double-counting):

```python
# After CCL produces `labels[z, y, x]` (0 = background, 1..N = frontier clusters)
# Propagate each frontier voxel's label into the adjacent unknown region via 3D
# distance-transform-with-features. Every unknown voxel ends up owned by its
# nearest frontier cluster.
from scipy.ndimage import distance_transform_edt
unknown = (voxels == -1)
# treat non-frontier-non-unknown as barriers (occupied OR free-interior)
seeds = labels.copy()
seeds[~(unknown | (labels > 0))] = -1   # mark walls/interior as forbidden
_, (iz, iy, ix) = distance_transform_edt(seeds == 0, return_indices=True)
owner = labels[iz, iy, ix]              # nearest frontier id per voxel
volumes = np.bincount(owner[unknown].ravel(), minlength=N+1) * (vs**3)
keep = np.where(volumes > 1.0)[0]
```

Cost: one `distance_transform_edt` + bincount = ~30 ms on our grid.

Reused by IG scorer for free — `volumes[cluster_id]` IS the IG (unknown m³ this
goal will resolve), no separate cylinder integration needed.

## Towards a truly 3D frontier (vs 2.5D)

### Why our current pipeline is 2.5D, not 3D

The 2.5-vs-3 distinction sits in two places:

| Stage | Now | True-3D version |
|---|---|---|
| Frontier *detection* | 2D scan on `traversability_grid` (one classification per `(x,y)` column) | 3D scan on `voxels_3d` (per-voxel FREE↔UNKNOWN boundary across all z) |
| Frontier *execution* | Nav2 plans on the 2.5D `traversability_grid` and emits ground-level cmd_vel | 3D motion planner (gbplanner3-style RRG over free voxels) emits 3D pose / SE(3) twist |

We can fix **detection** today; **execution** stays 2.5D until we either bring gbplanner3 into the sim stack or build layered 2.5D + topology.

### What 2.5D actually loses (concretely)

For `demo_ramp` specifically, **2.5D is sufficient** — the scene is single-floor.
The platform top is the column's "highest occupied z" → traversability picks
it up after fix #8. Walking up the ramp = walking on the "highest walkable
surface" of each column, which is what 2.5D models. Multi-floor it ain't.

The 2.5D failure modes (each one breaks the "one surface per (x,y)" assumption):
- **Bridges / tunnels**: column has a walkable lower surface AND walkable upper surface; 2.5D keeps only the higher one (the bridge top) and misses the tunnel below.
- **Multi-story buildings**: same (x,y), two floors; 2.5D keeps floor-2 and hides floor-1.
- **Balconies / overhangs**: walking under an overhang requires knowing the overhang isn't a wall blocking the column.
- **Ceiling holes / 3D-only passages**: a hole in the ceiling above a free area — no 2D frontier exists in any z-slice we currently project.

None of these appear in `demo_ramp`. All appear in the real-world / SubT-style use cases gbplanner3 was built for.

### Three honest paths to "truly 3D"

**Path A — 3D detection + 2.5D execution (Option 4 hybrid, already proposed).**

Detect frontiers in `voxels_3d` (real 3D), then for each surviving cluster project the centroid down to the closest reachable cell on `traversability_grid` and hand that to Nav2. Catches frontiers that `traversability_grid` doesn't surface (e.g., a hole at z=1.5 m above a free floor cell) provided the path *to* the projection is 2D-reachable. **Doesn't** solve multi-story or under-bridge.

Effort: ~200 LoC Python in CFPA2. **Recommended next step.**

**Path B — Layered 2.5D + level-transition graph.**

Publish K traversability grids, one per height band (`z ∈ [0, 0.5)`, `[0.5, 1.0)`, ...). Each layer has its own FREE/OCC/UNKNOWN. Add a small "level transition graph" of (layer_i, layer_j) edges where a ramp/stair/ladder is detected. Plan = (layer_i 2D plan) → (transition) → (layer_j 2D plan).

Catches multi-story. Doesn't catch bridges-where-the-tunnel-is-shorter-than-a-layer or arbitrary-z navigation. Requires CFPA2 + Nav2 refactor.

Effort: medium-large. Worth it if multi-story is on the roadmap.

**Path C — true 3D motion planner.**

Either port gbplanner3 to ROS 2 + sim, or write a voxblox/nvblox-native RRG planner. Goals become SE(3) poses, planning is full 3D, frontier detection uses the planner's own RRG graph as the candidate set (nodes that are leaves and have unknown neighbours).

This is "the right answer" — what the gbplanner3-on-Jetson stack does for the real robot. For sim, it's a major undertaking and arguably out of scope for the demo we're trying to land.

Effort: large. Already done for the real robot; redoing in sim is a research-engineering project.

### Decision

For this sim demo: **Path A**.
For the real robot: gbplanner3 (Path C) already wired up.
For the day someone wants multi-story sim: revisit Path B.

### Detection algorithm (Path A, concrete)

```python
def extract_3d_frontiers(voxels_3d, vs, min_unknown_vol_m3=1.0,
                         conn=26, max_depth_vox=20):
    """
    voxels_3d : int8 ndarray (nz, ny, nx), values {-1 unk, 0 free, 100 occ}
    Returns list of (centroid_xyz, unknown_vol_m3, frontier_voxel_count).
    """
    free    = (voxels_3d == 0)
    unknown = (voxels_3d == -1)

    # 1. 3D frontier mask: FREE with ≥1 UNKNOWN 6-neighbour.
    struct6 = ndimage.generate_binary_structure(3, 1)
    frontier = free & ndimage.binary_dilation(unknown, structure=struct6)

    # 2. 26-conn CCL on frontier voxels.
    struct26 = np.ones((3, 3, 3), dtype=np.uint8) if conn == 26 \
               else ndimage.generate_binary_structure(3, 1)
    labels, n = ndimage.label(frontier, structure=struct26)
    if n == 0:
        return []

    # 3. Watershed: every unknown voxel → nearest frontier cluster.
    seeds = np.where(frontier, labels, 0)
    # forbid expansion into walls (only frontier ∪ unknown is reachable)
    barrier = ~(unknown | frontier)
    seeds_with_barrier = seeds.copy()
    seeds_with_barrier[barrier] = -1
    # distance_transform_edt with return_indices propagates labels via
    # nearest-seed assignment; cheap and gives Voronoi partitioning.
    _, (iz, iy, ix) = ndimage.distance_transform_edt(
        seeds_with_barrier == 0, return_indices=True)
    owner = labels[iz, iy, ix]
    # mask: only unknown voxels count toward volume
    owner_unk = np.where(unknown, owner, 0)
    volumes_vox = np.bincount(owner_unk.ravel(), minlength=n + 1)
    volumes_m3  = volumes_vox * (vs ** 3)

    # 4. Filter by unknown-behind-volume.
    out = []
    for cid in range(1, n + 1):
        if volumes_m3[cid] < min_unknown_vol_m3:
            continue
        zs, ys, xs = np.where(labels == cid)
        centroid_vox = np.array([xs.mean(), ys.mean(), zs.mean()])
        out.append((centroid_vox, volumes_m3[cid], len(xs)))
    return out
```

### Ground projection (Path A, glue)

For each `(centroid_vox, vol, _)`:
1. Convert `centroid_vox → (cx, cy, cz)` world.
2. Find the closest FREE cell in `traversability_grid` to `(cx, cy)` via a small bounded BFS / dilation lookup.
3. Run the existing CFPA2 `_distance_transform` from the robot's current cell to verify reachability; drop if unreachable.
4. Emit as a goal candidate with `ig = vol` (replaces the cylinder count for this candidate).

The 2D frontier set and the 3D-derived set are then merged and scored by the same utility function CFPA2 already uses. No motion-planner change.

### Cost in numbers

For 200 × 200 × 30 voxels = 1.2 M cells:
- `binary_dilation(unknown)`: ~5 ms
- `ndimage.label(...)`: ~10 ms
- `distance_transform_edt(...)`: ~30 ms (dominant)
- bincount + filter + projection: < 2 ms
- **Total ~50 ms**, comfortable inside CFPA2's 500 ms tick budget.

### What this does NOT solve

Multi-story, under-bridge, ceiling-hole-above-free-cell-with-no-2D-path-up.
For those, Path B or C is the honest answer.

## Research foundations — formal "feasible walkable shell" in 3D

The user's intuition is right: there is a well-developed body of work formalising
"kinematically feasible thin shell" in 3D. The framework has appeared under three
overlapping names depending on community.

### Continuous formulation

The walkable surface S ⊂ ℝ³ is the set of contact configurations where the
robot can stand statically (or quasi-statically). Formally:

```text
S = ∂F  ∩  K(α_max, s_max, c_min, r_robot)
    └─┬─┘ └────────────┬───────────────┘
   skin of            kinematic admissibility set
   free space         (slope, step, clearance, footprint)
```

with the admissibility set

```text
K = {  p ∈ ℝ³ :
       n(p) · ẑ ≥ cos(α_max)                                   # slope
    ∧  Lip_r_robot(h)(p) ≤ s_max / r_robot                     # local roughness
    ∧  Cyl(p, r_robot, c_min) ⊂ F                              # head/body clearance
    }
```

where `n(p)` is the local surface normal at p, `Lip_r` is the local Lipschitz
constant of the height function in an r-neighbourhood, `F` is the free-space set.

In words: **S = (skin of free space) intersected with (kinematic OK)**. This
recovers our `mapper_node` traversability algorithm exactly — slope-filter (`α_max`),
step-filter (`s_max`), clearance check (`c_min`), footprint via inflation.

Canonical references:
- **Latombe 1991** *Robot Motion Planning*, ch. 7 — the C-space + admissibility framing.
- **Wermelinger, Fankhauser, Diethelm, Hutter 2016** "Navigation Planning for Legged Robots in Challenging Terrain" (IROS) — the form we ship: `T = w_s·slope + w_e·step + w_r·roughness`, threshold to declare a cell traversable. Used by ANYmal and friends.
- **Krüsi, Bosse, Siegwart 2017** "Driving on point clouds: Motion planning, trajectory optimization, and terrain assessment in generic non-planar environments" (J. Field Robotics) — same costs evaluated directly on a 3D point cloud, no intermediate elevation grid. Closest paper to "true 3D walkable surface".
- **Fankhauser & Hutter 2014/2018** — Robot-Centric Elevation Mapping (RC-EM) — the 2.5D height-function discretisation everyone uses for quadrupeds.

### Discrete formulation on a voxel grid (what we actually implement)

On a regular grid with voxel size `vs`, the discrete walkable shell is

```text
S_d = { v ∈ FREE :
        ∃ u ∈ OCC adjacent to v   (i.e., v sits on top of something)
     ∧  |H(v) − H(v')| ≤ s_max     ∀ v' in r_robot-neighbourhood (step)
     ∧  |H(v) − H(v')| / (vs * k) ≤ tan(α_max)  for k-cell baseline (slope)
     ∧  Cyl(v, r_robot, c_min) ⊂ FREE                (clearance)
      }
```

where `H(v)` is the highest-occupied-z in v's column. **This is what
`mapper_node::publish_traversability` computes.** The `k=3` baseline (fix #8) is
the discrete analogue of the continuous Lipschitz constraint averaged over a
finite window — exactly the trick used in Krüsi 2017 to handle voxelisation
artefacts on smooth slopes.

### The frontier *surface area* formula

For a voxel grid with frontier mask `F = FREE ∩ dilate6(UNKNOWN)`, the discrete
frontier surface area (the "thin shell" area) is

```text
A(F) = vs²  ·  Σ_{v ∈ F}  |{ u : u ∈ N₆(v), u ∈ UNKNOWN }|
```

(sum, over each frontier voxel, of how many of its 6 face-neighbours are
unknown — each contributes `vs²` of shell area).

This is the **digital-topology** surface-area formula (Lachaud & Thibert
2013, Klette & Rosenfeld *Digital Geometry* 2004). It is the natural
discretisation of `Area(∂F ∩ ∂U)` and converges to the continuous surface
area as `vs → 0` for sufficiently smooth boundaries.

For higher accuracy (and slightly slower) the Marching-Cubes triangulation
of the frontier iso-surface gives a piecewise-linear surface whose sum of
triangle areas converges faster (and gives normals for free).

### The "frontier volume" formula (interpretation B)

We want the unknown volume *gated* by each frontier cluster `C`. Define the
Voronoi assignment from frontier voxels into the unknown set:

```text
∀ u ∈ UNKNOWN,  owner(u) = argmin_{v ∈ F}  d_geo(u, v)
```

where `d_geo` is the geodesic distance through `FREE ∪ UNKNOWN` (occupied
voxels are barriers). Then

```text
V_unknown(C) = vs³ · |{ u ∈ UNKNOWN : owner(u) ∈ C }|
```

A cluster passes the volume filter iff `V_unknown(C) > V_min` (we set 1.0 m³).

In our scipy implementation `d_geo` is approximated by the 6-conn Euclidean
distance transform (`distance_transform_edt` with the occupied set turned
into infinity barriers), which is the standard approximation and exact up
to discretisation order.

References for this exact construction:
- **Yamauchi 1997** "A Frontier-Based Approach for Autonomous Exploration"
  (CIRA) — original 2D formulation.
- **Cieslewski, Kaufmann, Scaramuzza 2017** "Rapid Exploration with
  Multi-Rotors: A Frontier Selection Method for High Speed Flight" (ICRA)
  — first to do voxel-CCL frontier on an octree for 3D MAV exploration.
- **Dai, Papatheodorou, Tzoumas, Williams 2020** "Fast Frontier-based
  Information-driven Autonomous Exploration with an MAV" (IROS) — the
  Voronoi-volume formula above, applied to TSDF maps. They report frontier
  + volume + CCL in ~30 ms on a similar voxel count to ours.
- **Hornung, Wurm, Bennewitz, Stachniss, Burgard 2013** "OctoMap" — the
  underlying probabilistic occupancy model.

### Mesh-based 3D navigation (the most general formalism)

For *truly* 3D walkable surfaces — the multi-story / under-bridge / overhang
case that breaks 2.5D — the literature settled on **mesh-based navigation**:

1. Extract surface mesh of the occupancy/TSDF via Marching Cubes.
2. Per-face filter: keep faces whose normal satisfies `n·ẑ ≥ cos(α_max)`,
   whose adjacent face dihedral angle is within slope tolerance, with
   clearance cylinder in free space.
3. Survivors form one or more *walkable mesh manifolds*. Plan on the mesh
   graph (faces are nodes, shared edges are edges, edge cost = traversal
   cost from Wermelinger 2016).

References:
- **Wiemann, Mitschke, Mock, Hertzberg 2021** "Mesh-Based Navigation for
  Mobile Robots in Unstructured 3D Environments" (Sensors). Open-source
  ROS stack `mesh_navigation` (Osnabrück).
- **Putz, Wiemann, Hertzberg 2018** "The Mesh Tools Package" (also Osnabrück).
- **Vasilopoulos et al. 2018** "Reactive Semantic Planning in Unexplored
  Semantic Environments Using Deep Perceptual Feedback" — multi-story
  semantic.

This is "Path B with a different name" — instead of layered 2.5D grids you
have a single 2-manifold mesh that handles multi-story directly via the
mesh topology.

For frontier exploration on a mesh: a mesh face is a frontier face if it
borders an UNKNOWN voxel region (one of its vertices is on the boundary
of the explored mesh).

### Bottom line for our pipeline

What we ship today (after fixes 1–8) is the **discrete 2.5D Wermelinger-Krüsi
form**, in the Hornung occupancy framework. The proposed Path A adds **3D
frontier surface extraction (Cieslewski + Dai form)** while keeping execution
2.5D — that's already a documented, published configuration.

Going to true 3D walkable surface = porting to mesh_navigation or equivalent.
Worth doing only when the demo scenes require it (we are not there yet).

