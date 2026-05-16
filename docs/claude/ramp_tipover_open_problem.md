# Open problem — Go2W tips over near ramp/platform edges

**Status:** open as of 2026-05-15
**Scene:** `demo_ramp.xml` (single ramp + adjacent elevated platform with sharp cliff edge)
**Launch:** `./scripts/launch/nav_test_3d_explore.sh`

## Symptom

The robot autonomously explores via CFPA2 frontier selection → Nav2 planning on
the fused traversability grid. Path planning succeeds (no PLAN_FAILED events,
plan times 20ms p95) and the robot drives forward. But the global plan and
MPPI execution route the robot too close to:

1. the ramp foot transition zone (slope ramps from 0° to 14° within ~30 cm)
2. the cliff edge of the elevated platform alongside the ramp

The Go2W is wheeled-legged and tips when straddling the foot transition or
hitting a sharp z-discontinuity at the platform edge.

## Attempted fixes (all applied, none sufficient on their own)

| Layer | Fix | Result |
|---|---|---|
| Traversability filter chain | `ramp_safe = slope_margin·step_margin·100` clamped, rescues CNN on slopes < 30° AND step_residual < 6 cm. `trav_fused = max(CNN, ramp_safe)`. | Ramp body becomes uniformly free (verified 943/945 cells > 0.95 on demo_ramp). |
| Trav→OccupancyGrid threshold | Bumped `free_threshold` 0.30 → **0.60**, `lethal_threshold` 0.15 → **0.30**. Mid-band [0.30, 0.60] gets costly 1–99 interpolation. | Tightens but doesn't catch ramp-foot cells which `ramp_safe` rescues to ≈ 1.0 (above 0.60 → still free). |
| Height-based cost layer | Added `elevation_cost_enabled` to `grid_map_to_occupancy_grid`. Cells at z=0.05m → 0, z=1.0m → 90 (capped just below lethal). Combined with trav cost via `max()`. Live values: ramp_foot mean cost 0.2, ramp_mid 34, ramp_top 82. | Planner now prefers flat ground when alternatives exist, but ramp foot and cliff-adjacent platform cells still show mostly free. |
| Nav2 InflationLayer | `inflation_radius` 0.30 → **0.60 m**, `cost_scaling_factor` 5.0 → **2.5** (more gradual decay). Doubles the keep-clear halo around any lethal cell. | Pushes the planner further from walls, but the ramp foot itself isn't lethal (it's traversable), so inflation doesn't expand from it. |
| Ramp ascent helper | Removed `ramp_ascent_goal_node` + `ramp_cmd_vel_assist_node` from launch. | Eliminates the scripted ascent that was previously driving the robot into the foot; now CFPA2/Nav2 plan it themselves. |

## Why none of these fully solves it

The root issue is that the **ramp foot transition** and **platform cliff
edges** are *legitimately traversable* per every sensor: slope is moderate,
step_residual is small, elevation is finite. The Go2W's tipping failure mode
is dynamic-stability — a function of base orientation × CoM × wheel contact
geometry — which our 2.5D static cost layer cannot observe.

Specifically:

- **Ramp foot:** the 14° ramp starts within 1 cell of flat ground. A cell on
  the ramp at slope = 8° has `slope_margin = (0.5236 − 0.14)/0.5236 = 0.73`,
  which after the gain-100 amplification clamps to **ramp_safe = 1.0**. Even
  the height-cost layer gives it only `(0.10 − 0.05)/(1.00 − 0.05) × 90 ≈ 5`
  cost. The planner sees it as ≈ free.
- **Platform top:** flat (slope ≈ 0°) → CNN says ≈ 1.0 → free. Cells one
  voxel inside the cliff edge are flat-on-top but the *neighbouring* cell
  is a 1 m vertical drop. step_residual is computed in a 3 × 3 window, so
  the cliff-edge cell itself has a tiny window crossing the discontinuity →
  caught as lethal. But the cell *one inside* sees only flat top → free.
  At 10 cm resolution that's a 10 cm safety margin; Go2W's CoM swings
  further than that during a stop.

## Candidate next steps

Ordered roughly by implementation cost:

1. **Tighter `ramp_safe` envelope.** Require slope > 8° to rescue (not 0°).
   This prevents the rescue from lifting the ramp-foot transition cells.
   Add a slope-floor margin: `slope_floor_margin = clamp((slope − 0.14)/0.07, 0, 1)`,
   then `ramp_safe = slope_floor_margin · slope_ceil_margin · step_margin`.
   Cells with slope < 8° fall through to raw CNN (which says low trav near
   the gradient) → mid-band cost.
2. **Cliff-edge detector layer.** Add a filter that flags cells whose
   neighbouring step_height (over a *5 × 5* window, not 3 × 3) exceeds
   30 cm. This catches the platform top one cell inside the cliff. Inflate
   the resulting "cliff_proximity" cost by the robot's CoM-to-wheel offset
   (~20 cm).
3. **Per-cell footprint shrinkage.** Already enabled
   (`consider_footprint: true`) but the footprint polygon is symmetric.
   A pose-dependent footprint expansion on slopes (anticipating CoM shift)
   would model the tipping geometry; requires custom plugin.
4. **MPPI critic for slope traversal direction.** Penalise paths that
   approach a slope obliquely; reward paths that hit the slope perpendicular
   so the Go2W climbs straight up rather than crab-walking sideways across
   it. Implementable as a custom critic in `nav2_mppi_controller` plugins.
5. **Train CNN on tilted terrain.** ETH RSL released training scripts;
   augmenting the dataset with tilted-but-flat patches labelled traversable
   would let the CNN learn to distinguish "tilted but smooth = OK" from
   "tilted with discontinuity = lethal" without needing the analytical
   ramp_safe rescue at all.

## Where the relevant code lives

- `src/collaborative_exploration/trav_cost_filters/config/grid_map_filters.yaml` —
  filter chain that computes `slope_margin`, `step_margin`, `ramp_safe`,
  `trav_fused`.
- `src/collaborative_exploration/trav_cost_filters/trav_cost_filters/grid_map_to_occupancy_grid.py` —
  trav → cost converter, also publishes elevation-based extra cost.
- `src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py` — threshold
  knobs for the occupancy adapter.
- `src/go2w/go2w_config/config/nav/nav2_3d_costmap_overlay.yaml` — Nav2
  inflation settings.
- `src/vendor/elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_cupy/traversability_filter.py` —
  `get_filter_cupy()` Blackwell-native CNN backend.
