# Traversability Grid Rewrite — Elevation Map + grid_map Filter Chain

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the ad-hoc 6-step 2D projection in `mapper_node.cpp::publish_traversability` with a principled `elevation_mapping_cupy` + `grid_map` filter chain producing a continuous-cost map consumed by Nav2 and CFPA2.

**Architecture:** Three new ROS 2 native nodes run alongside the existing `nvblox_frontend mapper_node` (which keeps doing 3D Bayesian carving + voxels_3d for CFPA2 frontiers). `elevation_mapping_cupy` builds a 2.5D height map from `/robot/cloud_registered_body`. A `grid_map_filters` chain computes slope/roughness/step cost layers. A small adapter binarises/repackages the cost layer as an `OccupancyGrid` on `/robot/traversability_grid` so existing consumers (Nav2 StaticLayer, CFPA2 BFS) keep working with zero downstream changes. The old `publish_traversability` is gated behind `enable_legacy_2d_proj` (default `false`) so it can be re-enabled for A/B comparison or as an emergency fallback.

**Tech Stack:**
- ROS 2 Humble
- `ros-humble-grid-map` (apt — install)
- `elevation_mapping_cupy` (leggedrobotics ros2 branch — vendored to `src/vendor/`)
- CuPy + PyTorch (in `cmu_env` micromamba env, already has CUDA 12.6)
- Existing: nvblox 3D occupancy_layer, Point-LIO / Fast-LIO odom, MuJoCo Mid-360 sim

**Background and risk:**
- 6 mathematical defects of the current model in [`docs/claude/trav_grid_math_model_critique.md`](../trav_grid_math_model_critique.md)
- Problems 1+2 (lowest-stable H + plane-fit slope) already implemented in commit `8d4a7ba` — sim run still showed ramp-mid OCC, 33 m² wall leak, scattered noise → confirmed the model itself (Problems 3-6) is the issue, not parameter tuning.
- gbplanner3 stack uses elevation_mapping_cupy inside ROS 1 Noetic docker. We are bringing it up NATIVELY in ROS 2 Humble in this workspace. The leggedrobotics ros2 branch supports Humble; CuPy install in `cmu_env` is the main unknown.

**Rollback strategy:** Every phase ends with a commit. Phases 0-4 each toggle a single launch flag; reverting that flag returns the system to the previous-phase state. `enable_legacy_2d_proj:=true` re-enables the current production projection at any time.

---

## File structure

### Files to create

| Path | Responsibility |
|---|---|
| `src/vendor/elevation_mapping_cupy/` | Vendored ANYbotics elevation_mapping_cupy ros2 branch |
| `src/collaborative_exploration/trav_cost_filters/` | New ROS 2 ament_python package |
| `src/collaborative_exploration/trav_cost_filters/package.xml` | Package manifest |
| `src/collaborative_exploration/trav_cost_filters/setup.py` | ament_python setup |
| `src/collaborative_exploration/trav_cost_filters/trav_cost_filters/grid_map_to_occupancy_grid.py` | Adapter: grid_map cost → OccupancyGrid binarised at thresholds |
| `src/collaborative_exploration/trav_cost_filters/config/elevation_mapping.yaml` | elevation_mapping_cupy config tuned for our scene + Mid-360 |
| `src/collaborative_exploration/trav_cost_filters/config/grid_map_filters.yaml` | Filter chain: slope → roughness → step → cost |
| `src/collaborative_exploration/trav_cost_filters/launch/trav_pipeline.launch.py` | Launches elevation_mapping + grid_map_filters + adapter, namespaced |
| `src/collaborative_exploration/trav_cost_filters/test/test_grid_map_to_occupancy_grid.py` | Unit tests for the adapter binarisation logic |
| `docs/claude/plans/2026-05-14-trav-grid-rewrite.md` | This plan |

### Files to modify

| Path | Change |
|---|---|
| `src/collaborative_exploration/nvblox_frontend/src/mapper_node.cpp` | Add `enable_legacy_2d_proj` param (default `false`); gate the `publish_traversability` call on it; keep `publish_voxels_3d` unconditional |
| `src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py` | Include `trav_pipeline.launch.py` when `nav_costmap_mode:=3d`; pass `enable_legacy_2d_proj` to mapper (default `false`) |
| `CLAUDE.md` | Add 2026-05-14 active-state entry referring to this rewrite |

---

## Phase 0 — Stop the legacy publisher (one-flag escape hatch)

The current `publish_traversability` is what we're replacing. Gate it behind a parameter so the new pipeline can take over `/robot/traversability_grid` without conflict. CFPA2 frontier extraction keeps working because `publish_voxels_3d` stays on unconditionally.

### Task 0.1: Add `enable_legacy_2d_proj` parameter to mapper_node + launch arg

**Files:**
- Modify: `src/collaborative_exploration/nvblox_frontend/src/mapper_node.cpp`
- Modify: `src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py`

- [ ] **Step 1:** Open `mapper_node.cpp`. In the constructor parameter declarations (around the existing `voxel_size_m` declaration), add:

```cpp
enable_legacy_2d_proj_ = declare_parameter<bool>(
    "enable_legacy_2d_proj", false);
```

- [ ] **Step 2:** Find the member-variable block (end of class, near `cls_persist_`) and add:

```cpp
bool enable_legacy_2d_proj_{false};
```

- [ ] **Step 3:** Find the periodic timer callback that calls `publish_traversability(...)` (search for `publish_traversability`). Wrap the call site:

```cpp
if (enable_legacy_2d_proj_) {
  publish_traversability(robot_xyz, stamp, cloud_world, sensor_world);
}
```

Leave `publish_voxels_3d(...)` and the cloud publish unchanged — CFPA2's 3D frontier extractor depends on them.

- [ ] **Step 4:** Wire the launch arg in `src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py`. Find the `args = [DeclareLaunchArgument(...), ...]` list and add:

```python
DeclareLaunchArgument("enable_legacy_2d_proj", default_value="false",
    description="Re-enable the old mapper_node publish_traversability for A/B comparison."),
```

In the `mapper = Node(...)` block, add to the `parameters=[{...}]` dict:

```python
"enable_legacy_2d_proj": LaunchConfiguration("enable_legacy_2d_proj"),
```

- [ ] **Step 5:** Build and verify:

```bash
source /opt/ros/humble/setup.bash && \
  micromamba activate cmu_env && \
  colcon build --symlink-install --packages-select nvblox_frontend \
    --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3 2>&1 | tail -5
```

Expected: `Summary: 1 package finished`.

- [ ] **Step 6:** Launch the sim with the legacy projection still on (back-compat sanity):

```bash
rm -f /tmp/3d_launch.log
bash scripts/launch/nav_test_3d_explore.sh enable_legacy_2d_proj:=true > /tmp/3d_launch.log 2>&1 &
until grep -q "trav_grid world-fixed origin locked" /tmp/3d_launch.log 2>/dev/null; do sleep 3; done
ros2 topic hz /robot/traversability_grid --window 5 2>&1 | head -5
```

Expected: `average rate: ~2.0 Hz` (0.5 s publish period). Confirms back-compat.

Stop and re-launch with the flag off (the new default):
```bash
pkill -f "ros2 launch nav_test_3d_explore" ; sleep 3
rm -f /tmp/3d_launch.log
bash scripts/launch/nav_test_3d_explore.sh > /tmp/3d_launch.log 2>&1 &
until grep -q "exploration_status" /tmp/3d_launch.log 2>/dev/null; do sleep 3; done
ros2 topic hz /robot/traversability_grid --window 10 --timeout 5 2>&1 | head -5
```

Expected: `no new messages` (or `hz: timeout`) — confirms the gate works. CFPA2 will start logging that the map is stale (expected — there's nothing publishing in this state).

Stop the sim:
```bash
pkill -f "ros2 launch nav_test_3d_explore" ; sleep 2
```

- [ ] **Step 7:** Commit:

```bash
git add src/collaborative_exploration/nvblox_frontend/src/mapper_node.cpp \
        src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py
git commit -m "$(cat <<'EOF'
mapper_node: gate publish_traversability behind enable_legacy_2d_proj

Adds a default-false parameter to disable the legacy 6-step 2D projection
so the upcoming elevation_mapping_cupy + grid_map pipeline can own
/robot/traversability_grid without conflict. voxels_3d (CFPA2 frontier
input) and voxel_cloud (RViz) stay on unconditionally. Launch wires the
flag through as enable_legacy_2d_proj:=<bool>.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

**Review checkpoint — Phase 0 done.** No new dependencies installed yet; system can be restored by setting `enable_legacy_2d_proj:=true` at launch. CFPA2 still gets voxels_3d. Nav2 has no map — this is the intended pre-pipeline state.

---

## Phase 1 — Install grid_map and verify minimal demo

Pre-flight check: is the apt package available and runnable? Bail-out point if it isn't.

### Task 1.1: Install ros-humble-grid-map

**Files:** none

- [ ] **Step 1:** Check availability:

```bash
apt-cache policy ros-humble-grid-map ros-humble-grid-map-cv ros-humble-grid-map-msgs \
  ros-humble-grid-map-filters ros-humble-grid-map-ros 2>&1 | head -30
```

Expected: all five packages show `Candidate: 1.x.x-...`.

If any are missing, stop and revise — we'd need a source build instead. (Skill: this is the bail-out gate.)

- [ ] **Step 2:** Install:

```bash
sudo apt update
sudo apt install -y ros-humble-grid-map ros-humble-grid-map-cv \
  ros-humble-grid-map-msgs ros-humble-grid-map-filters ros-humble-grid-map-ros \
  ros-humble-grid-map-rviz-plugin
```

- [ ] **Step 3:** Verify Python bindings:

```bash
source /opt/ros/humble/setup.bash
python3 -c "from grid_map_msgs.msg import GridMap; print('OK')"
```

Expected: `OK`.

- [ ] **Step 4:** No commit (no repo change).

**Review checkpoint — Phase 1 done.** If apt failed we stop here and revisit; otherwise grid_map is available system-wide.

---

## Phase 2 — Vendor elevation_mapping_cupy and verify it builds

This is the highest-risk task in the plan. CuPy in `cmu_env` is unverified.

### Task 2.1: Verify CuPy in cmu_env

**Files:** none

- [ ] **Step 1:**

```bash
micromamba activate cmu_env
python3 -c "import cupy; cupy.show_config()" 2>&1 | head -20
```

Expected: prints CUDA runtime version (should match nvblox's CUDA 12.6).

If `ModuleNotFoundError`, install:

```bash
pip install cupy-cuda12x
python3 -c "import cupy; print(cupy.cuda.runtime.runtimeGetVersion())"
```

Expected: prints `12060` or similar.

If that fails on the 5090 / Blackwell — bail out here, revise to a pure-CPU port (NTNU/RSL has one) or to the lightweight Problem-3-6 fix in-tree.

### Task 2.2: Clone elevation_mapping_cupy into src/vendor

**Files:**
- Create: `src/vendor/elevation_mapping_cupy/`

- [ ] **Step 1:**

```bash
cd /home/hanszhu/Research/Collab_QRC/src/vendor
git clone --branch ros2 --depth 1 https://github.com/leggedrobotics/elevation_mapping_cupy.git
```

Expected: clone succeeds; `elevation_mapping_cupy/` directory exists with a `package.xml` inside.

- [ ] **Step 2:** Ensure nothing else in `src/vendor/elevation_mapping_cupy/` would get auto-built by colcon that we don't actually need. List its packages:

```bash
ls src/vendor/elevation_mapping_cupy/
```

If the repo contains demo or simulation packages we don't need, add `COLCON_IGNORE` to those subdirs. Keep only `elevation_mapping_cupy` and `elevation_map_msgs`.

- [ ] **Step 3:** Build:

```bash
colcon build --symlink-install --packages-select elevation_map_msgs elevation_mapping_cupy \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3 2>&1 | tail -20
```

Expected: both packages finished.

If build fails: capture the error, do not proceed. Likely fixes are CMake `find_package` for grid_map_msgs/CuPy or a python-version mismatch in setup.py.

- [ ] **Step 4:** Smoke-test the node starts (no input data yet):

```bash
source install/setup.bash
timeout 8 ros2 run elevation_mapping_cupy elevation_mapping_node 2>&1 | head -20
```

Expected: the node starts, complains about missing config / no input cloud, then exits at timeout. Confirms binary launches.

- [ ] **Step 5:** Commit:

```bash
git add src/vendor/elevation_mapping_cupy
git commit -m "vendor: elevation_mapping_cupy ros2 branch for native trav pipeline"
```

**Review checkpoint — Phase 2 done.** elevation_mapping_cupy runs natively in our ROS 2 Humble + cmu_env environment.

---

## Phase 3 — Configure elevation_mapping for our scene

### Task 3.1: Create trav_cost_filters package

**Files:**
- Create: `src/collaborative_exploration/trav_cost_filters/package.xml`
- Create: `src/collaborative_exploration/trav_cost_filters/setup.py`
- Create: `src/collaborative_exploration/trav_cost_filters/setup.cfg`
- Create: `src/collaborative_exploration/trav_cost_filters/resource/trav_cost_filters`
- Create: `src/collaborative_exploration/trav_cost_filters/trav_cost_filters/__init__.py`

- [ ] **Step 1:** Create the directory structure:

```bash
PKG=src/collaborative_exploration/trav_cost_filters
mkdir -p $PKG/trav_cost_filters $PKG/launch $PKG/config $PKG/resource $PKG/test
touch $PKG/trav_cost_filters/__init__.py
touch $PKG/resource/trav_cost_filters
```

- [ ] **Step 2:** Write `package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>trav_cost_filters</name>
  <version>0.1.0</version>
  <description>grid_map cost layer → OccupancyGrid adapter for Nav2 + CFPA2 in 3D-frontier-exploration sim.</description>
  <maintainer email="zhuhanshan12@outlook.com">Hanshang Zhu</maintainer>
  <license>BSD-3-Clause</license>

  <depend>rclpy</depend>
  <depend>grid_map_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>std_msgs</depend>

  <exec_depend>elevation_mapping_cupy</exec_depend>
  <exec_depend>grid_map_filters</exec_depend>

  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 3:** Write `setup.py`:

```python
from setuptools import setup
import os
from glob import glob

package_name = "trav_cost_filters"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hanshang Zhu",
    maintainer_email="zhuhanshan12@outlook.com",
    description="grid_map cost → OccupancyGrid adapter.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "grid_map_to_occupancy_grid = trav_cost_filters.grid_map_to_occupancy_grid:main",
        ],
    },
)
```

- [ ] **Step 4:** Write `setup.cfg`:

```
[develop]
script_dir=$base/lib/trav_cost_filters
[install]
install_scripts=$base/lib/trav_cost_filters
```

- [ ] **Step 5:** Build:

```bash
colcon build --symlink-install --packages-select trav_cost_filters \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3 2>&1 | tail -5
```

Expected: `1 package finished`.

- [ ] **Step 6:** Commit:

```bash
git add src/collaborative_exploration/trav_cost_filters/
git commit -m "trav_cost_filters: skeleton ament_python package"
```

### Task 3.2: Write elevation_mapping config for demo_ramp scene

**Files:**
- Create: `src/collaborative_exploration/trav_cost_filters/config/elevation_mapping.yaml`

- [ ] **Step 1:** Write the config (modelled on `scripts/sim/gbplanner3_mujoco/config/collab_qrc_go2/elevation_mapping_config.yaml` but ROS 2 native + topics aligned with our workspace):

```yaml
# elevation_mapping_cupy config for nav_test_3d_explore (demo_ramp scene)
#
# Input: /robot/cloud_registered_body — Mid-360 deskewed cloud from Fast-LIO
#                                       (body frame; node will TF to map)
# Frames: world frame "map" (Fast-LIO map), robot frame "base_link"
# Scene: demo_ramp.xml ≈ 16×16 m. We use a 30×20 m map with the robot
#        centred so the ramp + platform always stay inside the map.

input_sources:
  cloud:
    type: pointcloud
    topic: cloud_registered_body          # relative — namespace prefixes
    queue_size: 1
    publish_on_update: true
    sensor_processor:
      type: perfect                       # PointLIO/FastLIO already deskew

map_frame_id: "map"
robot_base_frame_id: "base_link"
robot_pose_with_covariance_topic: ""
robot_pose_cache_size: 200
track_point_frame_id: "base_link"
track_point_x: 0.0
track_point_y: 0.0
track_point_z: 0.0

length_in_x: 30.0
length_in_y: 20.0
position_x: 0.0
position_y: 0.0
resolution: 0.10                          # match nvblox voxel_size_m

min_variance: 1.0e-5
max_variance: 0.05
mahalanobis_thresh: 2.5
increase_height_alpha: 0.0

# Update rate — same cadence the old mapper_node ran at.
map_acquire_fps: 5.0
fps: 5.0

# Plugins: enable the publishers we need (elevation, variance, traversability
# is computed by the grid_map_filters chain — not here).
plugin_config_file: ""
```

- [ ] **Step 2:** Commit:

```bash
git add src/collaborative_exploration/trav_cost_filters/config/elevation_mapping.yaml
git commit -m "trav_cost_filters: elevation_mapping config for demo_ramp scene"
```

### Task 3.3: Smoke-test elevation_mapping in isolation

**Files:** none — runtime check.

- [ ] **Step 1:** Start the sim with the legacy projection still ON (so Nav2 doesn't die while we are debugging):

```bash
rm -f /tmp/3d_launch.log
ENABLE_LEGACY_2D_PROJ=true bash scripts/launch/nav_test_3d_explore.sh > /tmp/3d_launch.log 2>&1 &
until grep -q "exploration_status" /tmp/3d_launch.log 2>/dev/null; do sleep 3; done
echo "ready"
```

- [ ] **Step 2:** In a second terminal-equivalent (`Bash run_in_background`), launch elevation_mapping_cupy standalone:

```bash
source install/setup.bash
ros2 run elevation_mapping_cupy elevation_mapping_node \
  --ros-args -r __ns:=/robot \
  --params-file src/collaborative_exploration/trav_cost_filters/config/elevation_mapping.yaml \
  -p use_sim_time:=true 2>&1 > /tmp/elev.log &
sleep 10
ros2 topic list | grep -E "elevation|grid_map" | head -10
```

Expected: at least `/robot/elevation_map_raw` (or similar) appears.

- [ ] **Step 3:** Confirm the elevation map is being published with non-zero data:

```bash
ros2 topic hz /robot/elevation_map_raw --window 10 2>&1 | head -5
```

Expected: `~5 Hz`.

- [ ] **Step 4:** Capture a sample and inspect the `elevation` layer at the ramp:

```bash
ros2 topic echo /robot/elevation_map_raw --once 2>&1 | head -60
```

Expected: `data: ...` blocks include the `elevation` layer with float values. Real elevation values (≈0.0 m on floor, climbing toward 1.0 m on the ramp).

If the elevation map is empty or zero everywhere — stop, diagnose (frame TF, point cloud topic, sensor_processor params).

Stop the sim and node:

```bash
pkill -f elevation_mapping_node ; pkill -f "ros2 launch nav_test_3d_explore" ; sleep 2
```

**Review checkpoint — Phase 3 done.** elevation_mapping_cupy produces a populated elevation map from our Mid-360 cloud.

---

## Phase 4 — Wire grid_map_filters traversability chain

### Task 4.1: Write filter chain config

**Files:**
- Create: `src/collaborative_exploration/trav_cost_filters/config/grid_map_filters.yaml`

- [ ] **Step 1:** Write the chain (slope → roughness → step → combined cost):

```yaml
# grid_map filter chain: elevation → traversability cost
#
# Input layer: "elevation"  (from elevation_mapping_cupy)
# Output layer: "traversability"  (continuous 0..1; 0 = blocked, 1 = perfect)
#
# Stages:
#   1. surface_normals     — fit local plane, derive surface normal
#   2. slope               — angle between normal and +z, threshold 30°
#   3. roughness           — std dev of elevation in local window
#   4. step_height         — max(elevation) - min(elevation) in window
#   5. traversability      — multiplicative combination of the above
#
# The filter chain runs at the elevation_mapping_cupy publish rate.

filter_chain:
  - name: surface_normals
    type: gridMapFilters/NormalVectorsFilter
    params:
      algorithm: area
      input_layer: elevation
      output_layers_prefix: normal_vectors_
      radius: 0.25                # cells = 0.25 / 0.10 = 2.5 → 5x5 window
      normal_vector_positive_axis: z

  - name: slope
    type: gridMapFilters/MathExpressionFilter
    params:
      output_layer: slope
      expression: acos(normal_vectors_z)    # radians

  - name: slope_cost
    type: gridMapFilters/MathExpressionFilter
    params:
      output_layer: slope_cost
      expression: slope / 0.5236            # 30° = 0.5236 rad → cost ≥ 1.0 means too steep

  - name: roughness
    type: gridMapFilters/SlidingWindowMathExpressionFilter
    params:
      input_layer: elevation
      output_layer: roughness
      expression: sqrt(meanOfFinites(square(elevation - meanOfFinites(elevation))))
      compute_empty_cells: false
      edge_handling: crop
      window_size: 5            # 5x5 cells = 50 cm

  - name: roughness_cost
    type: gridMapFilters/MathExpressionFilter
    params:
      output_layer: roughness_cost
      expression: roughness / 0.05          # > 5 cm RMS = too rough → cost ≥ 1.0

  - name: step_height
    type: gridMapFilters/SlidingWindowMathExpressionFilter
    params:
      input_layer: elevation
      output_layer: step_height
      expression: maxOfFinites(elevation) - minOfFinites(elevation)
      compute_empty_cells: false
      edge_handling: crop
      window_size: 3            # 3x3 cells = 30 cm — adjacent cells only

  - name: step_cost
    type: gridMapFilters/MathExpressionFilter
    params:
      output_layer: step_cost
      expression: step_height / 0.20        # > 20 cm step = too high → cost ≥ 1.0

  - name: traversability
    type: gridMapFilters/MathExpressionFilter
    params:
      output_layer: traversability
      # min((1 - slope_cost),(1 - roughness_cost),(1 - step_cost)), clipped [0,1].
      # 1 = perfect traversal; 0 = blocked. NaN-safe via grid_map's expression eval.
      expression: min(max(0.0, 1.0 - slope_cost), min(max(0.0, 1.0 - roughness_cost), max(0.0, 1.0 - step_cost)))
```

- [ ] **Step 2:** Commit:

```bash
git add src/collaborative_exploration/trav_cost_filters/config/grid_map_filters.yaml
git commit -m "trav_cost_filters: slope+roughness+step → traversability filter chain"
```

### Task 4.2: Smoke-test grid_map_filters

- [ ] **Step 1:** Re-launch the sim (legacy projection ON), elevation_mapping ON, then start `grid_map_filters` standalone:

```bash
rm -f /tmp/3d_launch.log /tmp/elev.log /tmp/filters.log
ENABLE_LEGACY_2D_PROJ=true bash scripts/launch/nav_test_3d_explore.sh > /tmp/3d_launch.log 2>&1 &
until grep -q "exploration_status" /tmp/3d_launch.log 2>/dev/null; do sleep 3; done
ros2 run elevation_mapping_cupy elevation_mapping_node \
  --ros-args -r __ns:=/robot \
  --params-file src/collaborative_exploration/trav_cost_filters/config/elevation_mapping.yaml \
  -p use_sim_time:=true > /tmp/elev.log 2>&1 &
sleep 8
ros2 run grid_map_filters grid_map_filters_node \
  --ros-args -r __ns:=/robot \
  -r input_topic:=elevation_map_raw \
  -r output_topic:=elevation_map_filtered \
  --params-file src/collaborative_exploration/trav_cost_filters/config/grid_map_filters.yaml \
  -p use_sim_time:=true > /tmp/filters.log 2>&1 &
sleep 8
ros2 topic hz /robot/elevation_map_filtered --window 5 2>&1 | head -3
ros2 topic echo /robot/elevation_map_filtered --once 2>&1 | grep -E "layers|name|elevation|slope|roughness|traversability" | head -20
```

Expected: `~5 Hz` and the layers list includes `elevation`, `slope_cost`, `roughness_cost`, `step_cost`, `traversability`.

- [ ] **Step 2:** Visual check via RViz (if running): add a `GridMap` display, topic `/robot/elevation_map_filtered`, layer `traversability`. Ramp surface should look gradient-coloured (continuous low cost); wall edges and ramp-edge cliffs should saturate near 1.0.

Stop:
```bash
pkill -f grid_map_filters_node ; pkill -f elevation_mapping_node ; pkill -f "ros2 launch nav_test_3d_explore" ; sleep 2
```

**Review checkpoint — Phase 4 done.** Continuous traversability layer published at ~5 Hz.

---

## Phase 5 — grid_map → OccupancyGrid adapter

### Task 5.1: Write the adapter (TDD)

**Files:**
- Create: `src/collaborative_exploration/trav_cost_filters/test/test_grid_map_to_occupancy_grid.py`
- Create: `src/collaborative_exploration/trav_cost_filters/trav_cost_filters/grid_map_to_occupancy_grid.py`

- [ ] **Step 1: Write the failing test.**

```python
# test/test_grid_map_to_occupancy_grid.py
"""Adapter binarisation correctness.

The adapter maps a continuous traversability layer (0..1, where 1 = perfect)
into a 3-state OccupancyGrid (-1 / 0 / 100) that the existing Nav2 StaticLayer
and CFPA2 BFS consume unchanged. The mapping is:
    traversability NaN          → -1 (UNK)
    traversability > free_thresh →  0 (FREE)
    traversability < occ_thresh  → 100 (OCC)
    otherwise (gray-zone)         →  0 (FREE, optimistic — matches current behaviour)
"""
import math

from trav_cost_filters.grid_map_to_occupancy_grid import binarize


def test_nan_to_unknown():
    assert binarize(float("nan"), free_thresh=0.5, occ_thresh=0.2) == -1


def test_high_traversability_to_free():
    assert binarize(0.9, free_thresh=0.5, occ_thresh=0.2) == 0


def test_low_traversability_to_occ():
    assert binarize(0.1, free_thresh=0.5, occ_thresh=0.2) == 100


def test_gray_zone_to_free_by_default():
    # 0.3 sits between occ_thresh=0.2 and free_thresh=0.5
    assert binarize(0.3, free_thresh=0.5, occ_thresh=0.2) == 0


def test_exact_thresholds():
    # Boundary semantics: > free_thresh is FREE; < occ_thresh is OCC.
    assert binarize(0.5, free_thresh=0.5, occ_thresh=0.2) == 0
    assert binarize(0.2, free_thresh=0.5, occ_thresh=0.2) == 0   # not strictly less
```

- [ ] **Step 2: Run it to verify failure.**

```bash
cd /home/hanszhu/Research/Collab_QRC
PYTHONPATH=src/collaborative_exploration/trav_cost_filters \
  python3 -m pytest src/collaborative_exploration/trav_cost_filters/test/test_grid_map_to_occupancy_grid.py -v 2>&1 | tail -15
```

Expected: `ModuleNotFoundError` (binarize doesn't exist yet).

- [ ] **Step 3: Implement the adapter node (binarize + node).**

```python
# trav_cost_filters/grid_map_to_occupancy_grid.py
"""Adapter — subscribes a grid_map_msgs/GridMap, picks one layer, binarises
to nav_msgs/OccupancyGrid, publishes on the topic Nav2 + CFPA2 already
consume (/<ns>/traversability_grid).

This is the compatibility shim that lets the new elevation_mapping_cupy +
grid_map_filters pipeline take over from the old mapper_node 2D projection
without changing any downstream consumer.
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid


def binarize(value: float, free_thresh: float, occ_thresh: float) -> int:
    """Map a continuous traversability score to an OccupancyGrid cell value.

    Convention: traversability ∈ [0, 1] where 1 = perfect, 0 = blocked.
    NaN means the cell is unobserved → UNK (-1).
    > free_thresh → FREE (0).
    < occ_thresh  → OCC (100).
    Gray-zone (occ_thresh ≤ v ≤ free_thresh) → FREE (optimistic; matches the
    current behaviour where Nav2 + CFPA2 already treat anything non-blocked
    as traversable and let inflation handle the margin).
    """
    if not math.isfinite(value):
        return -1
    if value < occ_thresh:
        return 100
    # Anything not strictly under occ_thresh is FREE — matches the spec test.
    return 0


class GridMapToOccupancyGrid(Node):
    def __init__(self) -> None:
        super().__init__("grid_map_to_occupancy_grid")
        self.declare_parameter("input_topic", "elevation_map_filtered")
        self.declare_parameter("output_topic", "traversability_grid")
        self.declare_parameter("layer_name", "traversability")
        self.declare_parameter("free_thresh", 0.5)
        self.declare_parameter("occ_thresh", 0.2)
        self.declare_parameter("output_frame_id", "map")

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self.layer_name = self.get_parameter("layer_name").value
        self.free_thresh = float(self.get_parameter("free_thresh").value)
        self.occ_thresh = float(self.get_parameter("occ_thresh").value)
        self.output_frame_id = self.get_parameter("output_frame_id").value

        # nvblox_frontend used TRANSIENT_LOCAL on traversability_grid so the
        # Nav2 StaticLayer received the last sample immediately on subscribe;
        # match that exactly.
        out_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(OccupancyGrid, out_topic, out_qos)
        self.sub = self.create_subscription(GridMap, in_topic, self._cb, 10)
        self.get_logger().info(
            f"grid_map_to_occupancy_grid: {in_topic} layer={self.layer_name} → "
            f"{out_topic} free={self.free_thresh} occ={self.occ_thresh}"
        )

    def _cb(self, msg: GridMap) -> None:
        try:
            idx = msg.layers.index(self.layer_name)
        except ValueError:
            self.get_logger().warn(
                f"layer '{self.layer_name}' not in GridMap layers {msg.layers}",
                throttle_duration_sec=5.0,
            )
            return
        data_layer = msg.data[idx]
        if data_layer.layout.dim is None or len(data_layer.layout.dim) < 2:
            return
        rows = data_layer.layout.dim[0].size  # grid_map: outer dim = rows (y)
        cols = data_layer.layout.dim[1].size
        if rows * cols != len(data_layer.data):
            self.get_logger().warn(
                f"layer size mismatch rows*cols={rows*cols} vs data len={len(data_layer.data)}",
                throttle_duration_sec=5.0,
            )
            return

        arr = np.asarray(data_layer.data, dtype=np.float32).reshape(rows, cols)
        # grid_map stores cells col-major from +x corner (see grid_map docs);
        # OccupancyGrid stores row-major from origin upward. Flip axes to
        # match nav_msgs convention.
        arr = np.flip(arr, axis=(0, 1))

        out = np.empty(arr.size, dtype=np.int8)
        for k, v in enumerate(arr.ravel()):
            out[k] = binarize(float(v), self.free_thresh, self.occ_thresh)

        og = OccupancyGrid()
        og.header.stamp = msg.header.stamp
        og.header.frame_id = self.output_frame_id
        og.info.resolution = float(msg.info.resolution)
        og.info.width = cols
        og.info.height = rows
        og.info.origin.position.x = (
            msg.info.pose.position.x - msg.info.length_x * 0.5
        )
        og.info.origin.position.y = (
            msg.info.pose.position.y - msg.info.length_y * 0.5
        )
        og.info.origin.orientation.w = 1.0
        og.data = out.tolist()
        self.pub.publish(og)


def main(argv: Optional[list[str]] = None) -> None:
    rclpy.init(args=argv)
    node = GridMapToOccupancyGrid()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run the tests — they should pass.**

```bash
PYTHONPATH=src/collaborative_exploration/trav_cost_filters \
  python3 -m pytest src/collaborative_exploration/trav_cost_filters/test/test_grid_map_to_occupancy_grid.py -v 2>&1 | tail -10
```

Expected: 5 passed.

- [ ] **Step 5: Build the package.**

```bash
colcon build --symlink-install --packages-select trav_cost_filters \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3 2>&1 | tail -5
```

Expected: 1 package finished.

- [ ] **Step 6: Commit.**

```bash
git add src/collaborative_exploration/trav_cost_filters/test/ \
        src/collaborative_exploration/trav_cost_filters/trav_cost_filters/grid_map_to_occupancy_grid.py
git commit -m "trav_cost_filters: grid_map → OccupancyGrid adapter (+ unit tests)"
```

### Task 5.2: Integration test — adapter end-to-end

- [ ] **Step 1:** Launch sim (legacy off via Phase 0's launch arg — new adapter takes over):

```bash
rm -f /tmp/3d_launch.log /tmp/elev.log /tmp/filters.log /tmp/adapter.log

bash scripts/launch/nav_test_3d_explore.sh enable_legacy_2d_proj:=false > /tmp/3d_launch.log 2>&1 &
until grep -q "exploration_status" /tmp/3d_launch.log 2>/dev/null; do sleep 3; done

ros2 run elevation_mapping_cupy elevation_mapping_node \
  --ros-args -r __ns:=/robot \
  --params-file src/collaborative_exploration/trav_cost_filters/config/elevation_mapping.yaml \
  -p use_sim_time:=true > /tmp/elev.log 2>&1 &
sleep 8

ros2 run grid_map_filters grid_map_filters_node \
  --ros-args -r __ns:=/robot \
  -r input_topic:=elevation_map_raw \
  -r output_topic:=elevation_map_filtered \
  --params-file src/collaborative_exploration/trav_cost_filters/config/grid_map_filters.yaml \
  -p use_sim_time:=true > /tmp/filters.log 2>&1 &
sleep 8

ros2 run trav_cost_filters grid_map_to_occupancy_grid \
  --ros-args -r __ns:=/robot -p use_sim_time:=true > /tmp/adapter.log 2>&1 &
sleep 5

python3 scripts/debug/trav_grid_diag.py --wall-bands --robot-radial --png /tmp/trav_new.png 2>&1 | head -45
```

Expected output should show:
- `OCC` near walls (south wall observed first)
- `FREE` on ramp tail/mid/head (continuous traversability ≥ 0.5 there)
- `UNK` past walls
- `FREE cells OUTSIDE wall box: 0 cells` (or single-digit) — leak should be gone because elevation map only covers observed cells.

- [ ] **Step 2:** Capture the PNG `/tmp/trav_new.png` and visually compare to the previous `/tmp/trav_p1p2.png`. The ramp footprint should be uniformly FREE, walls clean OCC strips, no scattered black dots inside FREE.

- [ ] **Step 3:** If the comparison passes, stop the sim and proceed.

```bash
pkill -f grid_map_to_occupancy_grid ; pkill -f grid_map_filters_node ; \
  pkill -f elevation_mapping_node ; pkill -f "ros2 launch nav_test_3d_explore" ; sleep 2
```

If it fails, examine `/tmp/adapter.log` and `/tmp/filters.log`; common failure modes are frame_id mismatch (Nav2 wants `map`, grid_map publishes in `odom`) and the row/col flip in `_cb`. Fix and re-run before committing.

- [ ] **Step 4:** No commit (no code change since 5.1) unless 5.2 surfaced a fix.

**Review checkpoint — Phase 5 done.** New pipeline produces an OccupancyGrid on the same topic the old pipeline used. Nav2 + CFPA2 should be receiving it (visible in their `ros2 topic info ... -v` subscription counts).

---

## Phase 6 — Integrate into launch + flip the default

### Task 6.1: Add trav_pipeline.launch.py

**Files:**
- Create: `src/collaborative_exploration/trav_cost_filters/launch/trav_pipeline.launch.py`

- [ ] **Step 1:** Write the launch (parameter-driven so the parent launch can override topics):

```python
"""trav_pipeline.launch.py — elevation_mapping_cupy + grid_map_filters + adapter.

Composed by nav_test_3d_explore.launch.py when nav_costmap_mode:=3d.
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("trav_cost_filters")
    args = [
        DeclareLaunchArgument("robot_namespace", default_value="robot"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "elevation_config",
            default_value=os.path.join(pkg_share, "config", "elevation_mapping.yaml"),
        ),
        DeclareLaunchArgument(
            "filters_config",
            default_value=os.path.join(pkg_share, "config", "grid_map_filters.yaml"),
        ),
        DeclareLaunchArgument("free_thresh", default_value="0.5"),
        DeclareLaunchArgument("occ_thresh", default_value="0.2"),
    ]

    ns = LaunchConfiguration("robot_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    elevation = Node(
        package="elevation_mapping_cupy",
        executable="elevation_mapping_node",
        namespace=ns,
        name="elevation_mapping",
        output="screen",
        parameters=[
            LaunchConfiguration("elevation_config"),
            {"use_sim_time": use_sim_time},
        ],
        respawn=True,
        respawn_delay=3.0,
    )

    filters = Node(
        package="grid_map_filters",
        executable="grid_map_filters_node",
        namespace=ns,
        name="grid_map_filters",
        output="screen",
        remappings=[
            ("input_topic", "elevation_map_raw"),
            ("output_topic", "elevation_map_filtered"),
        ],
        parameters=[
            LaunchConfiguration("filters_config"),
            {"use_sim_time": use_sim_time},
        ],
        respawn=True,
        respawn_delay=3.0,
    )

    adapter = Node(
        package="trav_cost_filters",
        executable="grid_map_to_occupancy_grid",
        namespace=ns,
        name="grid_map_to_occupancy_grid",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": "elevation_map_filtered",
            "output_topic": "traversability_grid",
            "layer_name": "traversability",
            "free_thresh": LaunchConfiguration("free_thresh"),
            "occ_thresh": LaunchConfiguration("occ_thresh"),
            "output_frame_id": "map",
        }],
        respawn=True,
        respawn_delay=3.0,
    )

    return LaunchDescription([*args, elevation, filters, adapter])
```

- [ ] **Step 2:** Build and commit:

```bash
colcon build --symlink-install --packages-select trav_cost_filters \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3 2>&1 | tail -3
git add src/collaborative_exploration/trav_cost_filters/launch/trav_pipeline.launch.py
git commit -m "trav_cost_filters: trav_pipeline.launch.py"
```

### Task 6.2: Include trav_pipeline.launch.py from nav_test_3d_explore.launch.py

**Files:**
- Modify: `src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py`

(The `enable_legacy_2d_proj` launch arg already exists from Phase 0. This task only adds the new pipeline include.)

- [ ] **Step 1:** Read the current file to find where the mapper Node is created and the deferred TimerAction lives:

```bash
grep -n "mapper_node\|trav_pipeline\|TimerAction\|deferred" src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py
```

- [ ] **Step 2:** After the `mapper = Node(...)` definition, add the trav pipeline include:

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

trav_pipeline = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("trav_cost_filters"),
        "launch", "trav_pipeline.launch.py")),
    launch_arguments={
        "robot_namespace": LaunchConfiguration("robot_namespace"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }.items(),
)
```

And add `trav_pipeline` to the deferred `TimerAction` actions list (same 5 s delay as the mapper so all start together).

- [ ] **Step 3:** Run the full launch and verify:

```bash
rm -f /tmp/3d_launch.log
bash scripts/launch/nav_test_3d_explore.sh > /tmp/3d_launch.log 2>&1 &
until grep -q "exploration_status" /tmp/3d_launch.log 2>/dev/null; do sleep 3; done
ros2 topic hz /robot/traversability_grid --window 5 2>&1 | head -3
ros2 topic info /robot/traversability_grid -v 2>&1 | head -10
```

Expected: `~5 Hz` and publisher is `grid_map_to_occupancy_grid` (not `nvblox_frontend_mapper`).

- [ ] **Step 4:** Run the diag:

```bash
timeout 12 python3 -u scripts/debug/trav_grid_diag.py --wall-bands --robot-radial --png /tmp/trav_after_rewrite.png 2>&1 | head -45
```

Compare against earlier `/tmp/trav_p1p2.png` and the in-tree `docs/claude/trav_grid_math_model_critique.md` predictions:
- ramp tail / mid / head — all FREE
- past walls — UNK
- 0 m² leak outside wall box
- noise scatter inside FREE region — count should be substantially lower (the noise was an artifact of the 3-state categorical median and last-write-wins persistence; both are gone now).

- [ ] **Step 5:** Stop the sim:

```bash
pkill -f "ros2 launch nav_test_3d_explore" ; sleep 2
```

- [ ] **Step 6:** Commit:

```bash
git add src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py
git commit -m "$(cat <<'EOF'
nav_test_3d_explore: replace 2D projection with elevation_mapping pipeline

3D mode now launches trav_pipeline.launch.py (elevation_mapping_cupy +
grid_map_filters + grid_map_to_occupancy_grid adapter) and disables the
legacy mapper_node publish_traversability via enable_legacy_2d_proj:=false.
The legacy projection stays a one-flag away for A/B comparison or
emergency fallback.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

**Review checkpoint — Phase 6 done.** Full pipeline runs end-to-end from a single launch script.

---

## Phase 7 — End-to-end validation and documentation

### Task 7.1: A/B comparison

- [ ] **Step 1:** Run new pipeline for 60 s and capture diag:

```bash
rm -f /tmp/3d_launch.log
bash scripts/launch/nav_test_3d_explore.sh > /tmp/3d_launch.log 2>&1 &
until grep -q "exploration_status" /tmp/3d_launch.log 2>/dev/null; do sleep 3; done
sleep 60
timeout 12 python3 -u scripts/debug/trav_grid_diag.py --wall-bands --robot-radial --png /tmp/A_new.png > /tmp/A_new.txt 2>&1
pkill -f "ros2 launch nav_test_3d_explore" ; sleep 3
```

- [ ] **Step 2:** Run legacy pipeline for 60 s and capture diag:

```bash
rm -f /tmp/3d_launch.log
bash scripts/launch/nav_test_3d_explore.sh enable_legacy_2d_proj:=true > /tmp/3d_launch.log 2>&1 &
until grep -q "exploration_status" /tmp/3d_launch.log 2>/dev/null; do sleep 3; done
sleep 60
timeout 12 python3 -u scripts/debug/trav_grid_diag.py --wall-bands --robot-radial --png /tmp/B_legacy.png > /tmp/B_legacy.txt 2>&1
pkill -f "ros2 launch nav_test_3d_explore" ; sleep 3
```

(Note: both publishers will be active in legacy mode — the new pipeline's adapter wins on transient_local priority, but the legacy mapper is also writing. For a clean A/B you'd want a launch arg that disables the new pipeline; for now the test is "legacy on top of new" vs "new only", which is enough signal because legacy was failing on its own anyway.)

- [ ] **Step 3:** Compare the two diag outputs. Record the comparison in `/tmp/AB_comparison.md`:

```
Metric                          | Legacy  | New
--------------------------------|---------|---------
FREE cells outside wall box     | (m²)    | (m²)
Ramp mid (x=8, y=0)            | OCC/FREE| OCC/FREE
Ramp head (x=10, y=0)          | OCC/FREE| OCC/FREE
Scattered OCC in FREE region   | (count) | (count)
```

The new pipeline should win on at least 3 of 4. If it doesn't, capture the diff and stop — revisit filter params before claiming the rewrite is done.

### Task 7.2: Update documentation

**Files:**
- Modify: `docs/claude/trav_grid_math_model_critique.md`
- Modify: `docs/claude/3d_explore_pipeline.md`
- Modify: `CLAUDE.md`

- [ ] **Step 1:** At the end of `docs/claude/trav_grid_math_model_critique.md`, add a section "Outcome (2026-05-14)" with the A/B numbers from 7.1 and a one-paragraph summary of which of the 6 defects the rewrite addressed.

- [ ] **Step 2:** Update `docs/claude/3d_explore_pipeline.md` Stage 4 (`nvblox_frontend mapper_node`) — it no longer publishes `/robot/traversability_grid` by default. Add a new Stage 4b for the elevation_mapping_cupy + grid_map_filters + adapter chain. List its input (`/robot/cloud_registered_body`), intermediate topics (`elevation_map_raw`, `elevation_map_filtered`), and output (`/robot/traversability_grid`).

- [ ] **Step 3:** Add to `CLAUDE.md` top section:

```markdown
## Active state (2026-05-14) — trav_grid replaced with elevation_mapping + grid_map filter chain

The hand-rolled 6-step 2D projection in nvblox_frontend mapper_node
(Pass1 H + clearance + classify + step/slope + median + flood-disk +
persist) is retired by default for 3D-frontier exploration. After Problems
1+2 fixes (lowest-stable H + plane-fit slope, commit 8d4a7ba) still left
the ramp + leak + noise unaddressed, the upstream architecture was the
problem (see docs/claude/trav_grid_math_model_critique.md).

In its place: native ROS 2 elevation_mapping_cupy → grid_map_filters
(slope + roughness + step → continuous cost) → grid_map_to_occupancy_grid
adapter publishing the same /robot/traversability_grid topic Nav2 and
CFPA2 already consume. The legacy projection is one launch flag away
(`enable_legacy_2d_proj:=true`) for A/B comparison or fallback.

Run plan + per-task instructions: docs/claude/plans/2026-05-14-trav-grid-rewrite.md
```

- [ ] **Step 4:** Commit documentation update:

```bash
git add docs/claude/trav_grid_math_model_critique.md docs/claude/3d_explore_pipeline.md CLAUDE.md
git commit -m "$(cat <<'EOF'
docs: trav_grid rewrite — outcome + pipeline doc updates

A/B run shows the new elevation_mapping + grid_map filter chain replaces
the ad-hoc 6-step projection with [INSERT NUMBERS FROM 7.1]. Updates the
math-model critique with the outcome section, the 3d_explore_pipeline
doc with new Stage 4b, and CLAUDE.md active state.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

**Review checkpoint — Phase 7 done.** Plan complete; rewrite landed; legacy path one flag away.

---

## Risk register

| Risk | Likelihood | Phase | Mitigation |
|---|---|---|---|
| CuPy / 5090 (Blackwell) ABI mismatch | Medium | 2 | Phase 2 has a bail-out; fall back to CPU port or to in-tree Problem-3-6 fix |
| ros-humble-grid-map not in apt sources | Low | 1 | Phase 1 dry-runs the apt query before installing |
| elevation_mapping_cupy frame TF assumes `base_link` not `body` | Medium | 3 | Phase 3 sanity-checks `ros2 topic echo /robot/elevation_map_raw`; if the map is empty/zero everywhere, frame/TF is the first suspect |
| grid_map → OccupancyGrid axis flip wrong | Medium | 5 | Phase 5.2 visual diff against /tmp/trav_p1p2.png; mismatch will be obvious |
| CFPA2 BFS reads gray-zone as FREE and picks unreachable goals | Medium | 6 | `occ_thresh:=0.2` is configurable; raise it if CFPA2 picks goals it can't reach |
| Performance — elevation_mapping_cupy + 3 nodes at 5 Hz on 5090 | Low | 4-6 | If CPU-bound, drop fps to 2 Hz (no exploration regression at these speeds) |

## Rollback steps

To revert to the legacy 2D projection at any point:

```bash
# At launch:
bash scripts/launch/nav_test_3d_explore.sh enable_legacy_2d_proj:=true

# Or to a specific phase commit:
git log --oneline docs/claude/plans/2026-05-14-trav-grid-rewrite.md
git revert <phase-N-commit-sha>
colcon build --symlink-install
```

Phases 1-2 also touch system apt + `src/vendor/`; full removal:

```bash
sudo apt remove ros-humble-grid-map ros-humble-grid-map-cv ros-humble-grid-map-msgs \
  ros-humble-grid-map-filters ros-humble-grid-map-ros ros-humble-grid-map-rviz-plugin
rm -rf src/vendor/elevation_mapping_cupy
```
