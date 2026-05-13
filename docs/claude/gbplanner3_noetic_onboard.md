# GBPlanner3 + Point-LIO onboard Noetic stack — pitfalls & runbook (2026-05-13)

Native ROS 1 Noetic deployment of GBPlanner3 (3D exploration planner) on the Go2 Jetson Orin, paired with Point-LIO (HKU MaRS) as the SLAM front-end. Sister document to [`noetic_fastlio_onboard.md`](noetic_fastlio_onboard.md) — that one covers the SLAM brick, this one covers everything downstream.

## Why this stack

GBPlanner3 is the canonical 3D-exploration planner for the [UAS (NTNU)](https://github.com/ntnu-arl/unified_autonomy_stack) ecosystem. It's ROS 1 Noetic only; production-grade for ground / aerial robots with Mid-360-class LiDAR. Pairing it with Point-LIO (vs the original FAST-LIO2) was prompted by **measured rate degradation in long-run real-robot data**:

| FAST-LIO2 (ops2 bag, 180s walk) | Point-LIO (no run yet, expected) |
|---|---|
| /robot/Odometry: 6.9 Hz @ start → 4.4 Hz @ end (36% degradation) | iVox replaces ikd-tree → O(1) avg query, expected ≈ flat 10 Hz |
| 43% of LiDAR frames dropped over 3 min | Decoupled IMU + LiDAR threads — odom output never blocks on LiDAR |
| 0.22 m / 23 s end-displacement with stationary robot (drift OK, but inputs starved) | TBD — expected similar drift floor + sustained Hz |

The diagnostic that fingerprinted the issue (input rates rock-steady at 10/200 Hz while output drops 7→4 Hz) is in [scripts (commit 0505ff0 era)](../../scripts/) — `check_distribution.py` style histograms over rosbag2 sqlite.

## Architecture (2026-05-13)

```text
Jetson Orin (JP 5.x, Ubuntu 20.04 Focal, ARM64)
┌─────────────────────────────────────────────────────────────────────┐
│  ROS 1 Noetic (native, single roscore — onboard_pointlio_noetic.sh) │
│                                                                     │
│   livox_ros_driver2 ─► /livox/lidar  (CustomMsg, 10 Hz)             │
│                     ─► /livox/imu    (200 Hz)                       │
│                                  │                                  │
│                                  ▼                                  │
│   Point-LIO  ─► /robot/Odometry           (~10 Hz, sustained)       │
│             ─► /robot/cloud_registered_body  (body-frame cloud)     │
│             ─► /robot/cloud_registered    (world-frame cloud)       │
│             ─► /tf, /tf_static  (map → camera_init → body → base_link) │
│                                  │  (rostopic direct, NO bridge)    │
│                                  ▼                                  │
│   gbplanner_node                                                    │
│     ├─ voxblox (BAKED IN — no separate voxblox_node)                │
│     │   ─► /gbplanner_node/{tsdf,esdf}_map_out                      │
│     │   ─► /gbplanner_node/mesh, /occupied_nodes                    │
│     └─ planner core (RRG, NBV)                                      │
│         ─► /gbplanner_path  /gbplanner_status                       │
│                                  │                                  │
│                                  ▼                                  │
│   pci_general_ros_node  ─► /pci_command_path  (PoseArray)           │
│                                  │                                  │
│                                  │  ros1_bridge (param_bridge)      │
│                                  ▼                                  │
│   /pci_command_path on ROS 2 Foxy local DDS                         │
│                                  │                                  │
└──────────────────────────────────┼──────────────────────────────────┘
                                   │ WiFi CycloneDDS (cross-version Foxy↔Humble)
                                   ▼
┌─────────────────────────────────────────────────────────────────────┐
│ Laptop (Jammy + Humble) — pci_to_sand_adapter → SanD-Planner →      │
│                            /cmd_vel → Go2 sport API                 │
└─────────────────────────────────────────────────────────────────────┘
```

Key design wins vs the older Foxy-FAST-LIO + ros1_bridge architecture (see [`scripts/real/gbplanner3/README.md`](../../scripts/real/gbplanner3/README.md)):

1. **No bridge on the SLAM stream** — voxblox subscribes directly to ROS 1 Point-LIO. The 5-10 MB/s `/cloud_registered_body` no longer crosses parameter_bridge.
2. **One roscore on Jetson** — both SLAM and gbplanner3 are ROS 1 native, sharing the same master.
3. **Voxblox is baked into gbplanner_node** (loaded as rosparams, not a separate process). Upstream did this in v3; we don't fight it.
4. **iVox replaces ikd-tree** in Point-LIO → O(1) avg query, scales to long runs.

## Workspace layout (Jetson)

```text
~/noetic_fastlio_ws/                 # SLAM (Point-LIO + FAST-LIO + livox driver)
├── src/
│   ├── FAST_LIO/        (CATKIN_IGNORE — Point-LIO is the production path)
│   ├── Point-LIO/       (patched per below)
│   └── livox_ros_driver2/

~/gbplanner3_ws/                     # planner + voxblox + pci
├── src/                 (FLATTENED — see pitfall §1)
│   ├── BehaviorTree.CPP/
│   ├── adaptive_obb_ros/
│   ├── catkin_simple/
│   ├── eigen_catkin/  eigen_checks/  gflags_catkin/  glog_catkin/
│   ├── elevation_mapping/
│   ├── gbplanner_ros/   (the planner; gbplanner_go2.launch lives here)
│   ├── grid_map/
│   ├── image_transport_plugins/  (CATKIN_IGNORE — needs libturbojpeg)
│   ├── kindr/  kindr_ros/  kindr_rviz_plugins/
│   ├── manhole_detector_ros/   (CATKIN_IGNORE — OpenCV link bug)
│   ├── mav_comm/
│   ├── message_logger/
│   ├── minkindr/  minkindr_ros/
│   ├── numpy_eigen/  pci_general/
│   ├── robot_bringup/
│   └── voxblox/
└── build/  devel/  install/   (catkin_make outputs)

~/gbplanner3_scripts/                # operator scripts (not in workspace)
├── bridge_topics.yaml               # only /pci_command_path crosses now
├── gbplanner_config.yaml            # (legacy, outdated — see pitfall §8)
├── gbplanner_go2.launch             # (also copied into gbplanner package launch/)
└── orin_launch_gbplanner.sh         # tmux: ros1_bridge + planner
```

## Quick start

```bash
# 1. (One-time) Build gbplanner3_ws — see "Pitfalls" below for the journey.
#    This whole section runs ONCE per Jetson.
#    NB: orin_install.sh in scripts/real/gbplanner3/ assumes Jetson has
#    internet. The actual deployment used a laptop-side-clone + rsync path
#    because this Jetson can't reach github.

# 2. Start Point-LIO
ssh unitree@192.168.123.18
echo 123 | sudo -S ip addr add 192.168.123.100/24 dev eth0 2>/dev/null
echo 123 | sudo -S jetson_clocks
~/noetic_fastlio_ws/scripts/onboard_pointlio_noetic.sh
# Wait for /robot/Odometry @ ~10 Hz.

# 3. Start gbplanner3 + ros1_bridge (tmux session)
bash ~/gbplanner3_scripts/orin_launch_gbplanner.sh

# 4. (Laptop) Trigger exploration mission via DDS-bridged service
ros2 service call /planner_control_interface/std_srvs/automatic_planning \
  std_srvs/srv/Empty "{}"

# Stop:
~/gbplanner3_ws/scripts/onboard_pointlio_noetic.sh stop  # Point-LIO
tmux kill-session -t gbplanner3                          # planner + bridge
```

---

## Pitfalls — the build saga (2026-05-13)

7-step build journey from "ssh unitree" to a running gbplanner_node. Each pitfall: **symptom → root cause → fix**. None of these are documented anywhere upstream.

### §1. `src/src/<category>/<pkg>` double-nested directory

**Symptom**: `catkin_make` runs `cmake` and `make` apparently fine, but **0 packages compile** (no artifacts in `devel/lib/`). `build/CMakeFiles/Makefile2` is empty of target paths. catkin's `order_packages.cmake` says `CATKIN_ORDERED_PACKAGES ""`.

**Root cause**: UAS's `.repos` manifest path keys start with `src/`. Running `vcs import --recursive src < manifest` from the workspace root produced packages under `src/src/exploration/foo/` (double nested). catkin's toplevel.cmake only `add_subdirectory`'s direct children of `src/` that have CMakeLists.txt. `src/src/` doesn't have one, so all packages were orphaned.

**Fix**: `mv src/src/* src/`. Or import differently (without `src` target). Even after that, we hit pitfall §2.

### §2. `src/COLCON_IGNORE` blocks catkin discovery too

**Symptom**: After fixing §1, `find_packages("src")` (catkin_pkg Python) still returns 0. The `package.xml` files are present but invisible.

**Root cause**: We had dropped `COLCON_IGNORE` in `ws_gbplanner/src/` on the laptop side (intent: keep our laptop's colcon out of this ROS 1 tree, mirroring `src/vendor/fast_lio_ros1/COLCON_IGNORE`). It rsync'd to the Jetson — and `catkin_pkg.find_packages` also respects `COLCON_IGNORE` as a "skip this directory" signal, not just `CATKIN_IGNORE`. The check fires at the directory level, so a single file at `src/COLCON_IGNORE` killed ALL recursion.

**Fix**: `rm src/COLCON_IGNORE`. Use `CATKIN_IGNORE` per-package only when actually needed (pitfalls §6, §7).

### §3. Eight ROS 1 apt deps missing on Jetson — no internet

**Symptom**: cmake error chain `Could NOT find <package>` for: `costmap_2d`, `octomap_msgs`, `octomap_ros`, `joy`, `twist_mux`, `geographic_msgs`, `voxel_grid`, etc.

**Root cause**: Jetson has no internet (no DNS, no ICMP egress — verified via `getent hosts packages.ros.org`). `apt install` from tsinghua mirror fails. We can't run orin_install.sh's `apt-get install` line at all.

**Fix**: **Laptop downloads `.deb`s from `packages.ros.org`, scp's them to Jetson, `dpkg -i` locally**. Iteratively: each `dpkg -i` complains about transitive deps; we fetch those too. End list (this Jetson):

| Deb | Provides |
|---|---|
| `ros-noetic-octomap-msgs` | gbplanner / voxblox dep |
| `ros-noetic-octomap` | OctoMap C++ library |
| `ros-noetic-octomap-ros` | RViz / ROS msg wrappers for octomap |
| `ros-noetic-costmap-2d` | needed by some elevation_mapping helper |
| `ros-noetic-voxel-grid` | transitive dep of costmap-2d |

`joy`, `twist-mux`, `geographic-msgs`, `interactive-marker-twist-server`, `git-lfs`, `ros-foxy-ros1-bridge` were in `orin_install.sh`'s explicit list but **not actually needed by the gbplanner3 build path** — we discovered by trial-and-error (try build, install only what cmake / link demands).

A pure-laptop replacement for the orin_install.sh apt step lives at `/tmp/dl_deps.sh` (transient — should be vendored into `scripts/real/gbplanner3/` if we expect to redo this).

### §4. `gflags_catkin` ExternalProject downloads from GitHub

**Symptom**: Build hits 8% then dies with `make[2]: *** [gflags_catkin/CMakeFiles/gflags_src.dir/build.make:91: …/gflags_src-stamp/gflags_src-download] Error 1`. The CMake target is an `ExternalProject_Add` that does `URL https://github.com/gflags/gflags/archive/v2.2.1.zip` at build time.

**Root cause**: Same as §3 — no internet. `ExternalProject` is an additional download step beyond apt.

**Fix**: **Pre-stage the zip**. CMake's ExternalProject checks the download dir before fetching; if the file is already there with matching MD5, it skips the download.

```bash
# Laptop:
curl -L https://github.com/gflags/gflags/archive/v2.2.1.zip -o gflags-v2.2.1.zip
md5sum gflags-v2.2.1.zip   # MUST equal 2d988ef0b50939fb50ada965dafce96b

# Jetson:
mkdir -p ~/gbplanner3_ws/build/gflags_catkin/gflags_src-prefix/src/
# scp the zip into that dir, name it v2.2.1.zip
```

If gbplanner3_ws or other deps add more `ExternalProject` URLs in future releases, repeat per URL. There's no global list — `grep -r "ExternalProject_Add" src/` finds them.

### §5. `image_transport_plugins` needs `libturbojpeg` system lib

**Symptom**: cmake fails with `pkg_check_modules(TurboJPEG REQUIRED libturbojpeg) — A required package was not found`.

**Root cause**: `compressed_image_transport` uses TurboJPEG for JPEG compression. We don't have `libturbojpeg0-dev` on Jetson.

**Fix**: gbplanner3 core (voxblox + gbplanner + pci_general) doesn't need image transport — that whole meta-package is for ROS image topic compression. **`touch image_transport_plugins/CATKIN_IGNORE`**. Saved fetching another deb + transitive deps.

### §6. `manhole_detector_ros` has broken OpenCV linkage

**Symptom**: 92% built, then `manhole_detector_node` link fails: `undefined reference to cv::Mat::Mat()`.

**Root cause**: `manhole_detector_ros/CMakeLists.txt` `find_package(OpenCV)` but doesn't `target_link_libraries(... ${OpenCV_LIBS})`. Upstream bug. Triggered here because system's OpenCV doesn't auto-link via CMake's implicit transitive deps the way GCC older toolchains did.

**Fix**: gbplanner3 doesn't need manhole detection. **`touch manhole_detector_ros/CATKIN_IGNORE`**. Total skipped packages: `image_transport_plugins`, `manhole_detector_ros`.

### §7. BT XML relative path depends on `config/<folder>` depth

**Symptom**: `gbplanner_node` starts, loads yaml, but **crashes 0.5 s after launch** with: `Error parsing the XML: XML_ERROR_FILE_NOT_FOUND filename=/…/gbplanner/config/ugv/go2/../../../bt_xml/lge_insp.xml`.

**Root cause**: `GbplannerRos::registerTree()` constructs sub-tree file paths as `<config_folder>/../../../bt_xml/<subtree>.xml`. The number of `..`s is **hardcoded for the upstream config layout**: `config/ugv/gzc/urban_exploration/main_tree.xml` (4 levels deep from `config/`). `../../../bt_xml/` from there resolves to `config/bt_xml/` ✓.

If `config_folder` is at 3 levels deep (e.g. `config/ugv/go2/`), `../../../bt_xml/` resolves to `gbplanner/bt_xml/` — which doesn't exist.

**Fix**: Move the Go2 config dir **one level deeper to match the 4-deep upstream convention**:

```bash
gbplanner/config/ugv/real/go2/   # ← OK (4 levels)
# not
gbplanner/config/ugv/go2/        # ← BAD (3 levels)
```

This is also the source of the long-standing `project_gbplanner3_config_bug.md` memory entry — the previous attempt placed configs at the wrong depth.

### §8. Old `gbplanner_config.yaml` was v2 schema, not v3

**Symptom**: Even with everything else correct, our [`scripts/real/gbplanner3/gbplanner_config.yaml`](../../scripts/real/gbplanner3/gbplanner_config.yaml) wouldn't fly. It declares sections like `LocalPlannerParams`, `GlobalPlannerParams`, `ExplorationParams`, `MapParams`. gbplanner3 V3 expects: `RobotParams`, `SensorParams`, `CameraAnnotationParams`, `BoundedSpaceParams`, `NoGainZones`, `RandomSamplerParams`, `PlanningParams`, **`AdaptiveObbParams`** (this last one is gbplanner3-only).

**Root cause**: That file pre-dates the v3 schema rework — it was written against the older `gbplanner_ros` v2 codebase. Many top-level sections silently ignored.

**Fix**: **Throw it out**. Use upstream `config/ugv/gzc/urban_exploration/gbplanner_config.yaml` as the template — it has all 8 sections. Patch:

- `RobotParams.size:` `[1.0, 1.0, 0.4]` → `[0.6, 0.4, 0.3]` (Go2 dimensions)
- `RobotParams.size_extension:` tighter (`[0.10, 0.10, 0.05]`)
- `RobotParams.safety_extension:` tighter (`[0.15, 0.30, 0.05]`)
- `SensorParams.sensor_list:` `["OS064", "Cam", "CamExp"]` → `["Mid360"]`
- Replace `OS064` block with `Mid360` block (max_range 8m, rotations [0, 0.2636, 0] for mount pitch, fov [2π, 104° vertical])

Likewise patch `voxblox_sim_config.yaml`:

- `world_frame: world` → `map` (matches our `map → camera_init → body → base_link` TF tree)
- `tsdf_voxel_size: 0.3` → `0.15` (Mid-360 indoor)
- `max_ray_length_m: 20` → `8.0`
- `truncation_distance: 0.9` → `0.30`

The full patched config lives at `~/gbplanner3_ws/src/gbplanner_ros/gbplanner/config/ugv/real/go2/` on the Jetson — copy back to laptop and vendor it under `scripts/real/gbplanner3/config/` if it survives a real-world test.

### §9. Our `gbplanner_go2.launch` had separate voxblox_node (wrong)

**Symptom**: pre-existing `scripts/real/gbplanner3/gbplanner_go2.launch` spawned a standalone `voxblox_ros/esdf_server` node alongside `gbplanner_node`. That worked in gbplanner v2 era. In v3, voxblox is **baked into** `gbplanner_node` directly.

**Fix**: Rewrite launch following upstream `ugv/gzc/ugv_gzc_urban_exploration.launch` pattern. Voxblox config is loaded INTO gbplanner_node via `<rosparam command="load" file="$(arg map_config_file)"/>`. No standalone voxblox node. Three nodes total: `gbplanner_node`, `pci_general_ros_node`, optional rviz.

---

## Verified state at end of session (2026-05-13)

Last confirmed working:

```text
ps -eo pid,etime,comm | grep -E "pointlio|gbplanner|pci|livox_ros|roscore"
  2932  uptime=??  roscore
  3083  uptime=??  livox_ros_drive
  3183  uptime=??  pointlio_mappin
 31517  00:25      gbplanner_node
 31518  00:25      pci_general_ros

rostopic hz /robot/Odometry        # 10.001 Hz, std 1.1 ms
rostopic list | grep gbplanner     # 10+ gbplanner / pci_command_path topics
gbplanner log: "Received the first odometry, reset the map"
gbplanner log: "[PCI]: Starting run() loop"
```

A mission service hasn't been called yet. The planner is in the "waiting for trigger" state.

## Known issues / next steps

1. **PCI defaults `world_frame_id: world`** (warning at startup: `No world_frame_id setting, set it to: world`). Our TF tree uses `map`. May need to add a rosparam in `planner_control_interface_sim_config.yaml`. Untested whether this breaks `/pci_command_path` consumers.
2. **`ros1_bridge` not yet installed on Jetson** — `bridge_topics.yaml` is ready but the bridge process isn't running. `orin_launch_gbplanner.sh` will fail at the bridge launch step until `ros-foxy-ros1-bridge` deb is installed.
3. **`/pci_command_path` consumer on laptop not yet wired** — there should be a `pci_to_sand_adapter` (or equivalent) listening on Humble side. Architecturally documented in `scripts/real/gbplanner3/README.md` but no code yet.
4. **Config files only on Jetson** — the Go2-specific yaml and launch live at `~/gbplanner3_ws/src/.../config/ugv/real/go2/` on the Jetson only. They should be vendored back into `scripts/real/gbplanner3/config/go2/` and rsync'd via a future `deploy_gbplanner_to_jetson.sh` (sister to `deploy_noetic_to_jetson.sh`).
5. **Source tree not in our git** — the 22 vendored gbplanner3 repos are at `src/vendor/gbplanner3_src/ws_gbplanner/src/` on the laptop. Decide whether to commit them as vendor (1.6 GB, including .git dirs) or just commit the manifests + a re-import script.
