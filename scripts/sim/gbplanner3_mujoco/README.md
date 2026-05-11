# GBPlanner3 ↔ Collab_QRC MuJoCo integration (Option B)

NTNU GBPlanner3 runs in its own Noetic Docker container, consuming Collab_QRC's
**existing Fast-LIO + MuJoCo** stack via `ros1_bridge`. **No Gazebo, no smb_arl.**

This is the planner-side of the architecture we want for stair-climbing Go2
exploration. (The locomotion side is SanD-Planner, added on top later.)

## Architecture

```
                  HOST (Ubuntu 22.04 + ROS 2 Humble)
┌─────────────────────────────────────────────────────────────────┐
│  nav_test_gbplanner_{demo3,darpa}.sh                            │
│       │                                                         │
│       └─► ros2 launch gbplanner_{demo3,darpa}.launch.py         │
│              │                                                  │
│              ├── MuJoCo (urban_2story_go2.xml or demo3_go2_real.xml) │
│              ├── Fast-LIO2 (publishes /robot/Odometry,           │
│              │                       /robot/cloud_registered_body)│
│              ├── CHAMP locomotion                               │
│              ├── Static TF: base_link → lidar                   │
│              │              map → world  (for gbplanner)        │
│              └── rviz2                                          │
└────────────────────────────────┬────────────────────────────────┘
                                 │
                          ros1_bridge (parameter_bridge)
                                 │
┌────────────────────────────────▼────────────────────────────────┐
│ Docker containers (started by `make launch ...`)                │
│                                                                 │
│  ros1_launch_roscore        — Noetic master                     │
│  ros2_launch_ros1_bridge    — Humble<->Noetic translation       │
│  ros1_launch_gbplanner      — gbplanner3 + Voxblox + PCI        │
│       inputs (bridged from host Humble):                        │
│         /robot/odom                  ← /robot/Odometry          │
│         /robot/lidar/points_downsampled ← /robot/cloud_registered_body│
│         /tf, /tf_static               (bidirectional)           │
│       outputs (bridged back to host Humble):                    │
│         /command/trajectory          (PCI MultiDOFJointTraj.)   │
│         /gbplanner_path              (Path)                     │
│         /gbplanner_status            (String)                   │
└─────────────────────────────────────────────────────────────────┘
```

## Files

### Configs (Go2 mission for gbplanner3)
- [`config/collab_qrc_go2/gbplanner_config.yaml`](config/collab_qrc_go2/gbplanner_config.yaml)
  RobotParams (size, slope), SensorParams (single Mid-360, 8m range, 60° vFoV),
  LocalPlannerParams (RRG bbox 10×10×3.5m incl. one-floor-up for stairs),
  ExplorationParams (information gain), GroundParams (max_inclination_rad=0.45 ≈ 26°)
- [`config/collab_qrc_go2/voxblox_config.yaml`](config/collab_qrc_go2/voxblox_config.yaml)
  TSDF 0.15m voxel, 8m ray, world frame (with static map→world)
- [`config/collab_qrc_go2/planner_control_interface_config.yaml`](config/collab_qrc_go2/planner_control_interface_config.yaml)
  PCI thresholds + Go2 dynamics (v_max=0.5 m/s)
- [`config/collab_qrc_go2/manhole_detector.yaml`](config/collab_qrc_go2/manhole_detector.yaml)
  Stub (no manholes in our scenes)
- [`config/bridge_topics_collab_qrc.yaml`](config/bridge_topics_collab_qrc.yaml)
  ros1_bridge parameter_bridge whitelist (renames included so topic names
  match gbplanner_sim.launch expectations on Noetic side)

### Compose
- [`compose/docker-compose.collab_qrc.yml`](compose/docker-compose.collab_qrc.yml)
  Replaces NTNU's `docker-compose.ugv_sim.yml`: disables Gazebo Harmonic +
  smb_arl, mounts our Go2 mission config + bridge yaml.

### Top-level entry points (in `scripts/launch/`)

Both wrappers do the same thing — start the Docker stack (Noetic gbplanner3 +
ros1_bridge), wait for `gbplanner_node`, register static TF aliases on the
host side, then `exec` `nav_test_fastlio.sh` (Collab_QRC's existing MuJoCo +
Fast-LIO launcher) with `explore:=false` to disable CFPA2 and let gbplanner3
own the exploration.

- `nav_test_gbplanner_demo3.sh` — flat-scene smoke test (24×16m demo3)
- `nav_test_gbplanner_darpa.sh` — 3D test scenes (default `urban_2story`,
  switch via `scene:=pittsburgh_mine | stairwell | vertical_shaft`)

These shells handle the conda/micromamba `cmu_env` activation, ROS sourcing,
and workspace overlay correctly via the same pattern as `nav_test_fastlio.sh`.
**No separate ROS 2 launch.py is needed on the Humble side** — we reuse
Collab_QRC's existing one with `explore:=false`.

## Quick start

```bash
# 0. Make sure UAS Docker images are built (one-time, takes ~1h):
cd ~/Research/uas_deploy/unified_autonomy_stack
./scripts/import_all_repos.sh     # if not done yet
make images                        # builds the 6 base images
make build                         # compiles all workspaces

# 1. Flat-scene wiring smoke test (demo3):
cd ~/Research/Collab_QRC
./scripts/launch/nav_test_gbplanner_demo3.sh
# wait until GUI windows show, then:
./scripts/launch/nav_test_gbplanner_demo3.sh start_mission

# 2. Real 3D test (urban_2story warehouse with stairs):
./scripts/launch/nav_test_gbplanner_darpa.sh        # default scene
# OR
./scripts/launch/nav_test_gbplanner_darpa.sh scene:=pittsburgh_mine_go2

./scripts/launch/nav_test_gbplanner_darpa.sh start_mission

# 3. Stop everything:
./scripts/launch/nav_test_gbplanner_darpa.sh stop
```

## Topic-level wiring details

### Humble → Noetic (consumed by gbplanner_sim.launch)

| Host (Humble) topic            | Container (Noetic) topic                  | Why renamed                                  |
|---|---|---|
| `/robot/Odometry`              | `/robot/odom`                              | `gbplanner_sim.launch` expects `/<ns>/odom` |
| `/robot/cloud_registered_body` | `/robot/lidar/points_downsampled`         | matches `/<ns>/lidar/points_downsampled` |
| `/robot/tf`                    | `/tf`                                      | strip namespace so Noetic TF is flat       |
| `/robot/tf_static`             | `/tf_static`                              | same                                       |

### Noetic → Humble (re-exported for Humble-side adapters)

| Container topic                   | Host topic                          | Used by                            |
|---|---|---|
| `/command/trajectory`             | `/command/trajectory`                | future SanD-Planner adapter        |
| `/pci_command_trajectory_vis`    | same                                 | rviz visualization                 |
| `/gbplanner_path`                 | same                                 | rviz visualization                 |
| `/gbplanner_status`               | same                                 | health monitoring                  |

**Voxblox TSDF/ESDF map** (`voxblox_msgs/Layer`) is **not** bridged — it's
ROS 1 only and stays inside the planner container. The planner's volumetric
state remains internal; only its waypoint outputs cross to Humble.

## What this gets us vs Option A (full Orin native install)

| Aspect | This (Option B, Docker) | Option A (Orin native catkin) |
|---|---|---|
| Where it runs | Laptop (or strong host) | Onboard Jetson Orin |
| Locomotion latency | WiFi to robot | Localhost on Orin |
| GBPlanner3 source modifiable | Mount-mode: edit host file → `make build` rebuild | Direct catkin in `~/gbplanner3_ws` |
| Setup time first-run | ~1h (image bake + repo clone + catkin build) | ~30-90 min on Orin |
| Tested in this session | ✓ (canned ugv_sim demo, 5/6 images built, gbplanner ran) | Pending (scripts ready in `scripts/real/gbplanner3/`) |

Both eventually share the same gbplanner3 binaries and Go2 mission config.

## Known caveats

1. **gbplanner3 is on dev branch `gbplanner3_test`** — UAS manifest pulls
   the dev variant. If you hit crashes, `cd workspaces/ws_gbplanner/src/exploration/gbplanner_ros && git checkout gbplanner3 && cd /workspace && catkin build`.

2. **`libmujoco.so.3.6.0 cannot open` is harmless** — it's a non-fatal
   plugin warning from mujoco_ros2_control (Humble side) trying to load
   cmu_env's MuJoCo 3.6.0 sensor plugin while linked against 3.3.1. It
   does not affect the gbplanner container.

3. **`Starting position is not clear`** — common gbplanner3 cold-start
   warning when the robot spawn is too close to walls. Move the robot 0.5m
   forward via `ros2 topic pub /robot/cmd_vel ...` to unstick, or edit
   the scene MJCF to spawn in a more open area.

4. **rviz config** — the `rviz_gbplanner_demo3.rviz` referenced in the
   launchers is a TODO. For now rviz will start with default config; add
   PointCloud2 + Path + TSDF map displays manually.

5. **SanD-Planner adapter** — `pci_to_sand_adapter` not yet wired. Once
   added, the Humble side will consume `/command/trajectory` → emit
   `/sand_planner/goal` → drive Go2 via SanD-Planner's depth-based
   diffusion planner. For now, gbplanner outputs are visualization-only.

## Next steps after smoke-test passes

1. Write `pci_to_sand_adapter` (already sketched in
   [`scripts/real/gbplanner3/`](../../real/gbplanner3/) — port to this
   Docker setup).
2. Validate Go2 in `urban_2story_go2` actually climbs stairs end-to-end
   (gbplanner3 picks 2nd-floor frontier → SanD-Planner executes).
3. Deploy same stack to real Go2 + Jetson Orin via Option A scripts.
