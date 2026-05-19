# Exploration Planner Benchmark

Common-executor benchmark for `cfpa2`, `gbplanner2`, `gbplanner3`, and `mtare` in the
`demo3_mixed` dual-robot MuJoCo scene family.

## Launch One GUI Run

```bash
./scripts/launch/nav_test_demo3_mixed.sh gui:=true rviz:=true exploration_planner:=cfpa2
./scripts/launch/nav_test_demo3_mixed.sh gui:=true rviz:=true exploration_planner:=gbplanner2
./scripts/launch/nav_test_demo3_mixed.sh gui:=true rviz:=true exploration_planner:=gbplanner3
./scripts/launch/nav_test_demo3_mixed.sh gui:=true rviz:=true exploration_planner:=mtare
```

All planner modes execute through the same Nav2 MPPI stack.  External planners
must only publish high-level waypoints/trajectories:

- GBPlanner2/3: `/robot_a/command/trajectory`, `/robot_b/command/trajectory`
- MTARE/TARE: `/robot_a/mtare/way_point`, `/robot_b/mtare/way_point`

Adapters relay those outputs to `/<robot_ns>/way_point_coord`.

GBPlanner2/3 use `scripts/sim/gbplanner3_mujoco/launch_dual_common_executor.sh`
by default.  The wrapper checks out `origin/gbplanner2` or
`origin/gbplanner3_test` in the Unified Autonomy Stack GBPlanner workspace and
rebuilds when that ref changes.  Set `UAS_REPO_ROOT` if the Unified Autonomy
Stack checkout is not at `~/Research/uas_deploy/unified_autonomy_stack`.

MTARE uses the local ROS2 common-executor fallback by default.  To run upstream
ROS1 MTARE instead, pass `mtare_external_cmd:=...` or set
`MTARE_EXTERNAL_CMD=...` in the benchmark runner.

## Generate Maze Variants

```bash
python3 scripts/bench/generate_exploration_mazes.py
```

This writes deterministic MJCF scenes under:

```text
src/go2w/go2_gazebo_sim/mujoco/generated/
```

## Run Benchmark

Smoke:

```bash
NUM_TRIALS=1 DURATION_SEC=60 PLANNERS=cfpa2 \
  ./scripts/bench/benchmark_exploration_planners.sh
```

GBPlanner2 single-planner smoke:

```bash
NUM_TRIALS=1 DURATION_SEC=150 SCENE_FILTER=demo3_mixed PLANNERS=gbplanner2 \
  ./scripts/bench/benchmark_exploration_planners.sh
```

Full matrix:

```bash
./scripts/bench/benchmark_exploration_planners.sh
```

Default full budget is `600 sim-seconds x 10 trials` for each
`env/planner`.  Results are written to:

```text
/tmp/exploration_bench/<timestamp>/<env>/<planner>/trial_<n>/
```

The runner writes `summary.json` and `summary.csv` via
`aggregate_exploration_benchmark.py`.
