# Real Robot — Unitree Go2W / Go2 Autonomy

Real-robot operation: connection modes, SLAM selection, nav backends, common failure modes. Migrated from COMP0225_LRC_stack on 2026-04-17; Go2 (no-wheel) added 2026-04-17. Sim docs live in [door_task.md](door_task.md), [nav_benchmarks.md](nav_benchmarks.md), [slam_and_scenes.md](slam_and_scenes.md); this doc is real-robot only.

**Robots supported**: Unitree Go2W (wheeled-legged) and Unitree Go2 (no-wheel, spherical feet). Same network, same DDS, same Unitree sport API — only nav tuning differs.

## Entry points

```bash
# Unified entry — works for both Go2W and Go2. Kills nothing else.
scripts/real/real_autonomy.sh \
  robot={go2w|go2} \
  connect={ethernet|webrtc} \
  slam={carto_l1|fastlio_mid360} \
  nav={cfpa2|tare|far} \
  mapper={scan|carto_binary|carto_2d} \
  oa={true|false} \
  carto_mode={2d|3d}

# Convenience shim for Go2 (one-liner → real_autonomy.sh robot=go2 "$@")
scripts/real/real_autonomy_go2.sh [same flags]

scripts/real/real_autonomy.sh stop             # kill every real-robot process

# Sub-scripts (sourced by real_autonomy.sh; also standalone):
scripts/real/connect_ethernet.sh               # Ethernet + CycloneDDS preflight
scripts/real/connect_webrtc.sh                 # WiFi + go2_ros2_sdk WebRTC
scripts/real/monitor.sh                        # topic echo + optional cmd_vel pub
scripts/real/calibrate_imu.sh                  # two-phase IMU calib → imu_calib.yaml
```

Defaults: `robot=go2w + ethernet + carto_l1 + cfpa2 + scan + oa=true`.

## Go2W vs Go2 — what actually differs

CHAMP is **sim-only**; on real, Unitree's onboard controller handles gait via sport API (`api_id=1003` with OA, `1008` raw), fed by `cmd_vel_to_sport_bridge`. That collapses the Go2W/Go2 delta to nav tuning:

| Axis | Go2W | Go2 |
|---|---|---|
| Footprint | 0.70 × 0.40 m (wheel arches) | 0.70 × 0.32 m (spherical feet) |
| Max linear speed | 0.30 m/s (rolling safe margin) | 0.60 m/s (walking sustains this) |
| Max angular | 0.90 rad/s | 1.00 rad/s |
| FAR inflate | 2 cells (0.4 m/side) | 1 cell (0.2 m/side) |
| Nav YAML (reactive) | `go2w_config/config/nav/default_nav_single_go2w.yaml` | `go2w_real_bringup/config/nav/default_nav_real_go2.yaml` |
| Nav YAML (FAR) | `go2w_config/config/nav/far_planner_real.yaml` | `go2w_real_bringup/config/nav/far_planner_real_go2.yaml` |
| Network / DDS / WebRTC / IMU / LiDAR / SLAM / CFPA2 / TARE / sport bridge | identical |

Go2 configs live in `go2w_real_bringup/` (real-robot-only), keeping sim/real nav boundaries crisp. Go2W real nav still reuses `go2w_config/` YAMLs (the same ones sim uses) because Go2W has been validated there — don't split unless we find a real/sim tuning divergence.

## Package layout

```
src/go2w/go2w_real_bringup/
├── launch/
│   ├── real_bringup_core.launch.py   shared: transform_everything, static TF,
│   │                                  carto_odom_bridge, p2ls, rear filter,
│   │                                  twist_bridge, cmd_vel mux, sport bridge
│   ├── slam.launch.py                selects carto_l1 vs fastlio_mid360
│   ├── real_single.launch.py         top-level: SLAM + core + nav (cfpa2/far)
│   └── real_single_tare.launch.py    real_single + TARE planner + waypoint mux
├── config/
│   ├── slam/                         cartographer_l1_{2d,3d}.lua, fastlio_mid360.yaml,
│   │                                  octomap_mapping.yaml
│   ├── rviz/                         autonomy.rviz, cartographer*.rviz, octomap.rviz
│   └── imu_calib.yaml                acc/gyro bias + cross-axis coupling
├── docker/                           Unitree driver container
├── docs/CONNECT.md                   physical hookup guide
└── tools/calibrate_imu.py            two-phase IMU calibration tool
```

TARE planner: `src/collaborative_exploration/go2_tare_planner_ros2/` (generated ROS2 tree; upstream ROS1 regen tooling stayed in LRC).

## Connection modes

| Mode | Transport | Robot IP | Protocol | Notes |
|---|---|---|---|---|
| **ethernet** | USB-C dongle | 192.168.123.161 | CycloneDDS | Full ROS graph, point cloud available |
| **webrtc** | WiFi (`Go2_21585`) | 192.168.12.1 | `go2_ros2_sdk` WebRTC | Close Unitree phone app first |

Env overrides for Ethernet: `GO2W_ETH_IFACE`, `GO2W_ETH_IP`, `GO2W_HOST_IP`. For WiFi: `GO2W_WIFI_SSID`, `GO2W_WIFI_PASS`, `GO2W_WIFI_ROBOT`.

## SLAM backends

Both publish `map → base_link` TF and feed downstream nav identically.

| Backend | Sensor | Topic in | Topic out | Rate |
|---|---|---|---|---|
| `carto_l1` | Unitree L1 (built-in) | `/utlidar/transformed_cloud` (from `transform_everything`) + `/utlidar/transformed_imu` | TF `map → body`, `/robot/map_prob` | 11 Hz LiDAR, 200 Hz IMU |
| `fastlio_mid360` | Livox Mid-360 | `/livox/lidar` + `/livox/imu` | TF `camera_init → body_lidar` (bridged via static TF to `base_link`) + `/cloud_registered` | 10 Hz LiDAR, 200 Hz IMU |

Cartographer + L1 is the proven path — validated in LRC. Fast-LIO + Mid-360 is wired but needs real-robot tuning; the config (`fastlio_mid360.yaml`) starts from the Livox reference. `transform_everything` is skipped when `slam=fastlio_mid360`.

### LiDAR autodetection (`slam=auto`, default)

`real_autonomy.sh` defaults to `slam=auto`, which runs a ping probe against `GO2W_MID360_IP` (default `192.168.123.120`, the Unitree EDU+ factory address when a Mid-360 is wired through the M8 port) *after* `ensure_link` succeeds.

| Probe result | Selected SLAM |
|---|---|
| Mid-360 responds within `GO2W_LIDAR_PROBE_TIMEOUT` (default 2 s) | `fastlio_mid360` |
| No response | `carto_l1` |

The banner shows `slam: fastlio_mid360 (autodetected from 192.168.123.120)` so you can tell it wasn't picked explicitly. Use `slam=carto_l1` or `slam=fastlio_mid360` to override and skip the probe.

**Test the probe without launching anything**:
```bash
./scripts/real/connect_ethernet.sh lidar
# → Detected LiDAR: mid360   (or l1)
```

**If your Mid-360 is re-IPed** (e.g. Livox factory IP `192.168.1.1XX` behind the Go2's NAT, or you moved it to a different subnet with Livox Viewer):
```bash
GO2W_MID360_IP=192.168.1.120 ./scripts/real/real_autonomy.sh
# or just:
./scripts/real/real_autonomy.sh slam=fastlio_mid360
```

**Why not the Livox UDP 0x0000 device-info query**: it's the authoritative probe, but it requires CRC-16 + CRC-32 framing per Livox protocol 2.0 — implementing that correctly in shell is fragile. Ping covers 99% of presence cases; Livox devices always respond to ICMP once the IP is configured and the ~3 s boot completes.

### Livox Mid-360 — full wiring (driver + SDK)

The Mid-360 is a UDP-only device. Three components must be in place to get a useful `/livox/lidar` topic:

1. **Livox-SDK2** (C++ SDK) — vendored in `src/vendor/Livox-SDK2/`, built and installed to the **workspace-local prefix** `install/Livox-SDK2/` (no `sudo make install` polluting `/usr/local`). Build once:
   ```bash
   cd src/vendor/Livox-SDK2 && mkdir -p build && cd build
   cmake -DCMAKE_INSTALL_PREFIX=$PWD/../../../../install/Livox-SDK2 \
         -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
   make -j && make install
   ```
   Marked `COLCON_IGNORE` so colcon doesn't try to build it.

2. **livox_ros_driver2** — vendored in `src/vendor/livox_ros_driver2/` with two small patches: (a) `find_library`/`find_path` honour `LIVOX_SDK2_PREFIX` so it locates the workspace-local SDK, and (b) `package_ROS2.xml` + `launch_ROS2/` are copied to `package.xml` + `launch/` before build. Build:
   ```bash
   LIVOX_SDK2_PREFIX=$PWD/install/Livox-SDK2 \
     colcon build --packages-select livox_ros_driver2 --symlink-install \
     --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
                  -DLIVOX_SDK2_PREFIX=$PWD/install/Livox-SDK2
   ```
   **Important**: any time you add/rebuild this driver, also rebuild `fast_lio` — Fast-LIO's CMakeLists conditionally compiles Livox CustomMsg support based on whether `livox_ros_driver2` is in the workspace at build time.

3. **MID360_config.json** — in `src/go2w/go2w_real_bringup/config/slam/`. Host IPs are hardcoded to `192.168.123.100` and lidar IP to `192.168.123.20` (see "Network layout" table). If you're on a different rig, edit the JSON.

**Network layout verified on this rig (2026-04-17)**:

| Role | IP | MAC | Notes |
|---|---|---|---|
| Laptop primary (NetworkManager) | `192.168.123.222` | dongle | auto-assigned |
| Laptop secondary (Livox host) | `192.168.123.100` | dongle | required for Livox `bind()` — `ensure_link` adds this idempotently |
| Go2 dev board bridge | `192.168.123.161` | 7e:1d:75:60:f5:89 | proxy-ARPs the whole /24 |
| Onboard Jetson | `192.168.123.18` | 48:b0:2d:f8:f3:a2 | Foxconn OUI, SSH+HTTP open |
| **Livox Mid-360** | **`192.168.123.20`** | e4:7a:2c:34:01:c1 | UDP-only, no TCP |

**Verified rates** (measured against the live robot):

| Topic | Rate | Source |
|---|---|---|
| `/livox/lidar` | 10.0 Hz | Livox driver CustomMsg |
| `/livox/imu` | 200.0 Hz | Livox driver IMU |
| `/cloud_registered` | 10.0 Hz | Fast-LIO2 registered world-frame cloud |

**Troubleshooting**:

| Symptom | Cause | Fix |
|---|---|---|
| `bind failed` from Livox driver | Laptop IP on dongle ≠ host IPs in MID360_config.json | Run `source scripts/real/connect_ethernet.sh && ensure_link` (auto-adds `.100` as secondary) |
| `livox_ros_driver2 not built. Cannot use AVIA lidar type.` in Fast-LIO | Fast-LIO was built before livox_ros_driver2 existed | `colcon build --packages-select fast_lio` |
| `Init lds lidar fail!` | Wrong lidar IP in MID360_config.json, or Mid-360 isn't booted | `connect_ethernet.sh lidar` to verify reachability |
| Rates are right but RViz shows nothing | Wrong frame_id chain; `body_lidar` must lead to `base_link` via the static TF | `ros2 run tf2_tools view_frames` and look for a gap |

### Visualization: 2D grid + 3D voxel view

Two RViz windows spawn in parallel by default (`rviz=true`, `rviz_3d=true`):

| Window | Config | View | What's shown |
|---|---|---|---|
| `rviz2_2d` | `autonomy.rviz` | TopDownOrtho | BinaryMap (nav-grade `/robot/map`), CartoProbabilityGrid (raw `/robot/map_prob` under-layer), LaserScan, PlannedPath, RobotTrajectory, FrontierCylinders, FinalGoalMarker, **RobotPoseTriangle (red)** |
| `rviz2_3d` | `3d_view.rviz` | Orbit 3D | OccupancyUnderlay (dimmed), **OctoMapVoxels** (height-coloured voxel cloud from `/robot/octomap_point_cloud_centers`), RegisteredCloud (Fast-LIO `/cloud_registered`), PlannerLaserScan, PlannedPath, RobotTrajectory, FrontierCylinders, **RobotPoseTriangle (red)** |

Both show the same red triangle for the robot pose — published to `/robot/robot_pose_marker` by the active nav planner (`reactive_nav` or `default_nav`). Shape is `TRIANGLE_LIST` sized to the Go2/Go2W footprint (0.70 × 0.35 m). Color swap from cyan to red was 2026-04-17 for better contrast against Cartographer's grey/black grid.

**3D voxel grid source**:

| SLAM backend | 3D source | How |
|---|---|---|
| `carto_l1` | Viz-only octomap_server | Subscribes to `/utlidar/transformed_cloud`, publishes `/robot/octomap_point_cloud_centers` + `/robot/octomap_binary` + `/robot/octomap_full`. `projected_map` is renamed to `/robot/octomap_projected_viz` so it can't clobber Cartographer's `/robot/map`. |
| `fastlio_mid360` | Existing mapper-gen octomap_server | The same octomap instance that produces `/robot/map` also publishes voxel centres — the 3D RViz subscribes for free. |

**Toggle the 3D window off** for headless benchmarks or when you want to focus on the 2D view:

```bash
./scripts/real/real_autonomy.sh rviz_3d=false
./scripts/real/real_autonomy.sh rviz=false rviz_3d=false   # fully headless
```

### Cartographer sub-mode

`carto_mode=2d` uses `cartographer_l1_2d.lua` (default; native 2D grid, free-space carving, optimised for flat indoor).
`carto_mode=3d` uses `cartographer_l1_3d.lua` (3D scan matching, with `cartographer_occupancy_grid_node` projecting down to 2D). Auto-forced to 2d when `mapper=carto_2d`.

## Nav backends

| Backend | Entry | What runs | Use |
|---|---|---|---|
| `cfpa2` | `real_single.launch.py` nav_backend=reactive | CFPA2 frontier picker → `reactive_nav_node` (RRT*) | Default exploration |
| `far` | `real_single.launch.py` nav_backend=far | CMU terrain analysis + FAR V-graph + local planner + path follower | Larger/cluttered spaces; uses `far_planner_real.yaml` (conservative stub; tune on robot) |
| `tare` | `real_single_tare.launch.py` | Everything in `cfpa2` + TARE planner (TSP/VRP on frontiers via or-tools) + waypoint_mux | Global-optimal exploration routes; falls back to CFPA2 if TARE goes silent >1 s |

## Mapper backends (how `/robot/map` gets populated)

| Mapper | Pipeline | When to use |
|---|---|---|
| **`carto_2d`** **(default)** | Cartographer 2D trajectory builder → `/robot/map_prob` → `probability_grid_binarizer` → `/robot/map` | Flat indoor. Free-space carving is proper, dynamic obstacles decay via Cartographer's miss probability, loop closures tighten the grid globally. |
| `carto_binary` | Cartographer 3D trajectory builder → 2D projection via `cartographer_occupancy_grid_node` → `/robot/map_prob` → binarizer → `/robot/map` | Multi-storey or non-planar scenes where 2D scan matching drifts. |
| `scan` | `simple_scan_mapper_cpp` paints scan-ray hits onto a grid directly from `/robot/scan_3d` + TF | Fallback only. No free-space carving, no decay — stale obstacles linger, dynamic clutter accumulates. Kept for runs where you want to isolate SLAM issues from Cartographer's probability pipeline. |

When `slam=fastlio_mid360`, Cartographer isn't running — the mapper branch spawns `octomap_server` instead: `/cloud_registered` → voxel grid → 2D projection → `/robot/map`. Ground-return rejection is set via `point_cloud_min_z` (0.20 m Go2W, 0.30 m Go2).

**Why `carto_2d` replaced `scan` as the default (2026-04-17)**: `simple_scan_mapper_cpp` paints ray endpoints but never erases them, so a dynamic obstacle (person walking through) becomes a permanent wall on the grid. Cartographer's 2D grid uses hit/miss probability with asymmetric weights (`hit_probability=0.60`, `miss_probability=0.45` in `cartographer_l1_2d.lua`) — rays passing through a cell erode occupancy, so moving clutter fades within seconds. Also gives proper unknown vs free vs occupied tri-state which CFPA2 frontier detection needs.

## Topic contract (after `real_bringup_core.launch.py`)

| Topic | Direction | Notes |
|---|---|---|
| `/utlidar/cloud` | robot → stack | Raw UTLidar L1 cloud |
| `/utlidar/imu` | robot → stack | Raw UTLidar L1 IMU |
| `/utlidar/transformed_cloud` | stack internal | Pitch-corrected, self-hit-filtered, body frame |
| `/utlidar/transformed_imu` | stack internal | Axis-corrected, bias-subtracted, LPF'd |
| `/robot/scan_3d_raw` | stack internal | `pointcloud_to_laserscan` output |
| `/robot/scan_3d` | stack internal | After `scan_rear_filter` removes self-rays |
| `/robot/map` | stack internal | Binary occupancy from mapper |
| `/robot/map_prob` | stack internal | Cartographer probability grid |
| `/robot/odom/nav` | stack internal | `carto_odom_bridge` Odometry from `map → base_link` TF |
| `/robot/way_point_coord` | stack internal | CFPA2 frontier goal |
| `/robot/way_point_coord_nav` | stack internal | TARE → mux output (when nav=tare) |
| `/robot/cmd_vel_stamped` | stack internal | Planner output |
| `/robot/cmd_vel_auto` | stack internal | twist_bridge unstamped |
| `/robot/cmd_vel_manual` | joystick → mux | teleop fallback |
| `/cmd_vel` | stack → robot | Mux output → `cmd_vel_to_sport_bridge` |
| `/api/sport/request` | stack → robot | Unitree sport API (1003 with OA, 1008 raw) |

## Onboard SLAM split (shipped 2026-04-30 — Fast-LIO + Livox on Jetson)

**Goal**: push Fast-LIO + Livox driver onto the Go2 Jetson (`192.168.123.18`,
Ubuntu 20.04 + ROS 2 Foxy) so the laptop is left with planning, control,
viz, and bridges only. Frees ~1 core on the laptop and unblocks loop closure
(Phase 2 — SC-PGO port pending).

**Component compatibility verified**: Livox-SDK2 supports Ubuntu 20.04 +
ARM (README §2.2), livox_ros_driver2 supports ROS 2 Foxy via `./build.sh ROS2`,
Fast-LIO 2 builds against Foxy unchanged. **No Docker needed** despite the
laptop running Humble — cross-distro DDS works for the topic types in use
(Twist, Odometry, PointCloud2, Path, Marker).

### Topology after the split

```
ONBOARD JETSON (Foxy)                   LAPTOP (Humble)
─────────────────────                   ────────────────
livox_ros_driver2_node                  Nav2 (planner / controller / behaviors)
  /livox/lidar (CustomMsg)              CFPA2 + cfpa2_to_nav2_bridge + watchdog
fastlio_mapping                         octomap_server (consumes /cloud_registered_body
  /Odometry ─────────────────DDS──►       over DDS from the Jetson)
  /cloud_registered_body ────DDS──►     path_relay, robot_pose_marker
3 static TFs (map→camera_init,          RViz × 2, supervisor_panic, joy
  body→base_link, map→odom)             cmd_vel_activity_mux + cmd_vel_to_sport_bridge
fast_lio_tf_adapter                     ◄──/cmd_vel──── (Sport API to robot)
  /robot/odom/nav ───────────DDS──►
[future] sc_pgo_node
  /robot/corrected_odom ─────DDS──►
```

DDS bridge: both hosts on `ROS_DOMAIN_ID=0`, `rmw_cyclonedds_cpp`, peers list
includes the other side. Multicast on the Ethernet NIC.

### Files added (laptop side)

| File | Role |
|---|---|
| [`scripts/real/deploy_to_jetson.sh`](../../scripts/real/deploy_to_jetson.sh) | rsync vendor packages + configs + helper scripts to `~/onboard_ws/` on the Jetson. Idempotent. |
| [`scripts/real/onboard_slam.sh`](../../scripts/real/onboard_slam.sh) | runs ON THE JETSON. Brings up Mid-360 NIC bind + Livox + Fast-LIO + 3 static TFs + `fast_lio_tf_adapter` + (optional) `sc_pgo`. Ctrl+C teardown. |
| [`scripts/real/connect_ethernet.sh:60-95`](../../scripts/real/connect_ethernet.sh) | adds Jetson `192.168.123.18` to CycloneDDS peer list when `ONBOARD_SLAM=1` env is set. |
| [`scripts/real/real_autonomy.sh`](../../scripts/real/real_autonomy.sh) | new `onboard=true|false` flag exports `ONBOARD_SLAM` + plumbs `onboard_slam:=$ONBOARD` into the launch. |
| [`real_single.launch.py`](../../src/go2w/go2w_real_bringup/launch/real_single.launch.py) | new `onboard_slam` arg. When true, **skips the slam.launch.py include** entirely — laptop stops spawning Livox + Fast-LIO + the static TFs + `fast_lio_tf_adapter`. |

### One-time onboard build (run on the Jetson via SSH)

```bash
# From laptop:
./scripts/real/connect_ethernet.sh   # ensure link
./scripts/real/deploy_to_jetson.sh   # rsync source

# Then SSH to Jetson:
ssh unitree@192.168.123.18           # password: 123
cd ~/onboard_ws

# Build Livox-SDK2 (workspace-local — no /usr/local pollution)
cd src/Livox-SDK2 && mkdir -p build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$HOME/onboard_ws/install/Livox-SDK2 \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
make -j$(nproc) && make install

# Build livox_ros_driver2 (uses its own ROS 2 build script)
cd ~/onboard_ws/src/livox_ros_driver2 && ./build.sh ROS2

# Build Fast-LIO via colcon
cd ~/onboard_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select fast_lio \
  --cmake-args -DLivox-SDK2_DIR=$HOME/onboard_ws/install/Livox-SDK2/lib/cmake/Livox-SDK2
```

### Day-to-day operation

```bash
# On Jetson (run from laptop via SSH):
ssh unitree@192.168.123.18 "cd ~/onboard_ws && ./scripts/onboard_slam.sh"

# On laptop (in another terminal):
./scripts/real/real_autonomy_go2.sh onboard=true oa=false
```

The laptop banner now reads `onboard : true (laptop skips livox+fast_lio; expects Jetson @ 192.168.123.18)`.

### Verification

After both sides come up, from the laptop:

```bash
# These were previously published locally; now they come from the Jetson over DDS:
ros2 topic hz /Odometry                  # 10 Hz expected
ros2 topic hz /cloud_registered_body     # 10 Hz expected
ros2 topic hz /robot/odom/nav            # 10 Hz from fast_lio_tf_adapter onboard

# TF chain still resolves end-to-end:
ros2 run tf2_ros tf2_echo map base_link  # non-empty translation

# Local Fast-LIO is NOT running:
pgrep -fa fastlio_mapping                # should be empty on the laptop
```

### Foxy compatibility patches (one-time, all forward-compatible to Humble)

Five source patches were needed to build + run on Foxy that aren't needed
on Humble. All are mechanical and stay valid on Humble — no rollback.

1. **`livox_ros_driver2/src/comm/pub_handler.cpp:135`** —
   `kLivoxLidarTypeMid360s` (an `s` variant) doesn't exist in our Livox-SDK2.
   Patched out: `sed -i 's@||dev_type==LivoxLidarDeviceType::kLivoxLidarTypeMid360s@@'`.

2. **`fast_lio/src/laserMapping.cpp:1124`** — uses `Trigger::Request::ConstSharedPtr`
   (a Humble-only alias). Foxy expects plain `Request::SharedPtr`. Build error
   without it: `enable_if<false, void>::type does not exist` in rclcpp's
   `AnyServiceCallback::set` overload.
   `sed -i 's@Trigger::Request::ConstSharedPtr@Trigger::Request::SharedPtr@'`.

3. **CycloneDDS XML schema** — Foxy 0.7.x:
   `<NetworkInterfaceAddress>auto</NetworkInterfaceAddress>` + `<AllowMulticast>`.
   Humble 0.10+: `<Interfaces><NetworkInterface .../></Interfaces>`. Schema
   mismatch → `unknown element` → `rmw_handle invalid` → all nodes crash.
   Branched in `onboard_slam.sh` based on `$ROS_DISTRO`. (We chose FastDDS on
   Foxy anyway — see #5; CycloneDDS segfaults in `spin()` regardless of XML.)

4. **`static_transform_publisher` arg syntax** — Humble takes `--frame-id parent
   --child-frame-id child --x ...`. Foxy is positional: `x y z qx qy qz qw parent child`
   OR `x y z yaw pitch roll parent child` (note: yaw-pitch-roll order, not
   roll-pitch-yaw). Wrong syntax → `not having the right number of arguments`.

5. **Foxy parameter parser rejects empty `-p key:=`** — quotes required:
   `-p lvx_file_path:='""'`. Without quotes: `Couldn't parse parameter override rule`.

6. **rclpy 0.9.x segfaults on this Tegra Foxy** — `fast_lio_tf_adapter.py`
   (the laptop's adapter) segfaults instantly. So does a 30-line minimal
   relay. C++ rclcpp is fine. Workaround: have Fast-LIO itself remap
   `/Odometry` → `/<ns>/odom/nav` via `-r` arg at launch (no Python relay).
   Re-enable the full adapter on Humble for SC-PGO bootstrap-from-GT logic.

### The 13-GB-per-node bloat — and the `ulimit -v` fix

The hard one. Every C++ ROS 2 node on this Tegra Foxy mmap'd a single
**14.5 GB anonymous region** at startup, then lazy-faulted ~13 GB of it
resident as 8 threads each touched their share. Five SLAM nodes ×
13 GB = OOM kill within 8 seconds. Symptoms: continual
`bad_alloc caught: std::bad_alloc` log spam, then `Out of memory: Killed
process … (fastlio_mapping)` in `dmesg`.

**Methodically ruled out**: CUDA / sanitizers / `MALLOC_ARENA_MAX` /
jemalloc preload / tcmalloc preload / FastDDS shared-memory transport /
`MALLOC_MMAP_THRESHOLD_` / FastDDS dynamic-pool XML profile / CycloneDDS
(segfaults instead of bloating) / `taskset -c 0` (still spawns 8 threads).
None reduced the 13 GB.

**The fix is one line**: `ulimit -v 1500000` (1.5 GB virtual cap) in
`onboard_slam.sh` before spawning children. The speculative `mmap` *fails*
because it can't reserve 14.5 GB inside a 1.5 GB cap; the library catches
`std::bad_alloc` (the "harmless" log spam), falls back to bounded
buffers, and the process runs in **22 MB resident**. Same observable
behavior, 500× less memory.

| Metric | Before fix | After `ulimit -v 1500000` |
|---|---|---|
| Per-node RSS | 13 GB | 22 MB |
| 5-node total | OOM | 333 MB |
| /robot/odom/nav rate (cross-host) | n/a | 26.78 Hz |
| /livox/lidar discovery | OOM before publish | discovered cross-host |

**Why this works** — Foxy's rclcpp + FastDDS pre-reserves a single huge
anonymous mapping at participant init (likely a per-thread message-pool
sized off `/proc/meminfo` total RAM). On Tegra Orin's 15 GB UMA, the
"available" calculation produces ~14.5 GB. The kernel grants the virtual
mapping happily; pages are committed lazily as threads write. Cap virtual
memory → speculative reservation fails → fallback path uses bounded
small buffers → same publishing/subscribing behavior, 500× less RAM.

This is a Foxy-specific bug; Humble doesn't do this (laptop runs same
nodes in <2 GB total). Fix is forward-compatible — `ulimit` is a no-op
on Humble.

### Phase 2 — loop closure (SC-PGO) — TODO

`src/vendor/sc_pgo/fast_lio_sam/` is ROS 1 catkin source. Port spec is in
[`PORT_TO_ROS2.md`](../../src/vendor/sc_pgo/PORT_TO_ROS2.md). ~640 LOC across
3 cpp + 4 headers, plus build files. Effort ~1-2 days focused work:
rename package `fast_lio_sam` → `sc_pgo`, swap catkin → ament_cmake, port
ros::* → rclcpp::*, replace message_filters synchronizer (ROS 2 has a
different API), drop `tf_conversions` (gone in ROS 2 — use tf2_eigen),
verify GTSAM links (`apt install libgtsam-dev` on Foxy). The `loop_closure:=true`
launch toggle already exists end-to-end; once `sc_pgo` builds + installs,
the toggle picks it up via topic-name contract documented in PORT_TO_ROS2.md.

Tighten the LC threshold during the port: radius 5.0 → 3.0 m, ICP fitness
0.3 → 0.15 (per `PORT_TO_ROS2.md` §7) — empirical for indoor demo3.

### Future: Jetson Orin Super migration

When the Orin Super arrives (JetPack 6 / Ubuntu 22.04 / native Humble):
1. Re-run `deploy_to_jetson.sh` against the new IP — script is idempotent.
2. Onboard build switches `./build.sh ROS2` → `./build.sh humble`.
3. SC-PGO port carries over unchanged (rclcpp API is identical Foxy↔Humble
   for this surface area).
4. Cross-distro DDS caveat goes away — both sides on Humble.

The deploy + onboard launcher are deliberately distro-portable.

## Common failure modes

| Symptom | Likely cause | Fix |
|---|---|---|
| `Interface ... not found` on ethernet | USB-C dongle not recognised | `ip link show`; set `GO2W_ETH_IFACE` |
| Cannot ping 192.168.123.161 | Cable/link; robot not booted | Check dongle LED; wait for boot chime |
| `carto_odom_bridge` silent | Cartographer TF not arriving; watch `ros2 topic hz /tf` | Is SLAM actually up? Check `ros2 node list` |
| Map frozen / doesn't grow | scan QoS mismatch or stale TF fallback | Check `/robot/scan_3d` hz; it must match `/utlidar/cloud` rate after pitching |
| `No Effective Points!` in Fast-LIO | Livox driver not publishing or timestamp broken | Verify `ros2 topic hz /livox/lidar` |
| Robot doesn't move | `cmd_vel_activity_mux` in manual timeout | Joystick left active? `ros2 topic echo /robot/control_source` |
| Robot moves when it shouldn't | `oa=false` + stale planner command | Send `scripts/real/real_autonomy.sh stop` then `scripts/real/monitor.sh stop` |
| WebRTC connect fails silently | Unitree phone app holding session | Close the app, try again |

## IMU calibration

`transform_everything` applies acc/gyro bias subtraction + cross-axis coupling. Calibrate on first setup and whenever the robot is dropped or the IMU is reseated:

```bash
scripts/real/calibrate_imu.sh both 30
# Move emitted imu_calib_data.yaml → src/go2w/go2w_real_bringup/config/imu_calib.yaml
colcon build --packages-select go2w_real_bringup --symlink-install
```

The calib YAML is loaded at startup via the `imu_calib_yaml` launch arg (defaults to the package-share copy). Override at launch with `imu_calib_yaml:=/path/to/other.yaml`.

## Supervisor panic override (any-button emergency)

Any button press on the Unitree BT controller latches a **5 s panic window** during which:

1. Autonomy cmd_vel is blocked at `cmd_vel_activity_mux`.
2. FAR / local planner drop `autonomyMode` (because `autonomy_enabler` zeroes `axes[2]` on the synthetic `/joy`).
3. Only real joystick Twist (`/<ns>/cmd_vel_manual` via `teleop_twist_joy`) reaches the controller.
4. When no manual twist is active, the mux publishes zero Twist — robot halts.
5. Re-pressing any button extends the window by another 5 s.
6. On expiry, state → `nominal`, auto resumes automatically.

**Why "any button"**: no dedicated e-stop key on the Unitree pad. Under stress the operator should not need to remember a specific index. The synthetic `/joy` published by `autonomy_enabler` always has `buttons=[0]*11`, so only real human presses arm the latch.

**Topics**:

| Topic | Type | Notes |
|---|---|---|
| `/<ns>/supervisor_state` | `std_msgs/String` | `"nominal"` or `"panic"` at 20 Hz |
| `/<ns>/supervisor/panic_trigger` | `std_msgs/Empty` | Programmatic trigger (keyboard, web UI, etc.) |
| `/<ns>/panic_cmd_vel` | `geometry_msgs/Twist` | Zero-twist heartbeat; used by mux as safe fallback during panic |
| `/<ns>/control_source` | `std_msgs/String` | Adds `"panic"` + `"manual_override"` values |

**Launch args** (in `safety.launch.py`):
- `panic_duration_sec` — default `5.0`

**Testing without a pad** — you can fire the trigger topic manually:
```bash
ros2 topic pub --once /robot/supervisor/panic_trigger std_msgs/msg/Empty '{}'
ros2 topic echo /robot/supervisor_state      # → "panic" for 5 s, then "nominal"
ros2 topic echo /robot/control_source        # → "panic" → "nominal"
```

## Golden rules for real robot

1. `use_sim_time: false` everywhere — the real robot drives the clock.
2. **Always start with `connect_ethernet.sh`** to validate the link before launching SLAM. Saves 30 s of Cartographer failure-logs.
3. **Any button = panic override.** Pressing *any* button on the Unitree BT pad latches a 5 s window that blocks autonomy and disarms FAR; the sticks still drive. Re-press to extend. This is orthogonal to the `manual_timeout_sec` stick-latch (0.35 s) which only passes manual twist through.
4. **Never deploy on the real robot with `oa=false` until you've dry-run with `oa=true`.** Obstacle-avoidance (api_id=1003) bounces at walls; raw sport (1008) does not.
5. **Kill everything with `real_autonomy.sh stop`** before relaunching — zombie Cartographer instances corrupt subsequent runs.

## Bug chain (2026-04-17): "map doesn't expand"

Shipping Fast-LIO + Mid-360 on real hardware took **10 layered fixes**. Symptom the operator reported: "initial frame shows a map, but it doesn't grow as the robot moves". Each fix exposed the next one. Record kept here so future Claudes don't re-hunt the same ghosts.

### Bug genealogy

| # | Bug | File | Nature | How it masked the next one |
|---|---|---|---|---|
| 1 | `base_link` double parent (`body → base_link` static + Cartographer's `map → base_link`) | `real_bringup_core.launch.py` / `slam.launch.py` | TF2 splits the tree; every `lookupTransform(map, base_link)` fails | Kept any downstream inspection from succeeding; marker, nav, everything blind |
| 2 | `body_lidar → base_link` static pointed at a dead frame (Fast-LIO publishes `camera_init → body`, not `body_lidar`) | `slam.launch.py` Fast-LIO branch | In Fast-LIO mode `base_link` dangled in an orphan subtree | Same — TF tree broken |
| 3 | `publish_pose_marker()` inside `if (!has_goal_ \|\| !last_scan_) return;` | `reactive_nav_node.cpp::tick()` | Red triangle wasn't emitted until CFPA2 produced its first goal | Looked like Bug 1/2 was worse than it was |
| 4 | Pose marker was ARROW + cyan (low contrast on grey Cartographer grid) | `reactive_nav_node.cpp::publish_pose_marker()` + `default_nav.py` | UX; marker was there but easily missed | — |
| 5 | `/robot/map` publisher VOLATILE, subscribers TRANSIENT_LOCAL | octomap_server default | ROS 2 silently dropped **every** message due to QoS mismatch | Map never reached RViz so nothing else could be diagnosed |
| 6 *(red herring)* | Thought `probability_grid_binarizer` was clobbering with stale empty grids | `real_bringup_core.launch.py` | Actually wasn't publishing in Fast-LIO mode (no `/robot/map_prob` input). Gating it off was cleanup, not a fix | — |
| 7 | `pointcloud_to_laserscan` subscribed to `/utlidar/transformed_cloud` (only exists in carto mode — `transform_everything` is off in Fast-LIO) | `real_bringup_core.launch.py` | `/robot/scan_3d` empty → reactive_nav `tick()` early-returns zero cmd_vel → **robot doesn't move autonomously** | Without robot motion, Bug 8's failure mode (points beyond 8 m from spawn) never triggered |
| 8 *(root cause of "map doesn't expand")* | octomap subscribed to `/cloud_registered` (`frame_id="camera_init"`, world frame). `TF(map ← camera_init)` is identity static → octomap's `sensor_origin = (0, 0, 0)` forever. `(point − sensor_origin).norm() > max_range=8 m` truncates every occupied endpoint once robot leaves its 8 m spawn ball | `real_single.launch.py` Fast-LIO octomap remap | — | — |
| 9 | Mid-360 is mounted tilted (+15.1° pitch, -2.1° roll measured). Identity `body → base_link` static doesn't cancel the mount → after Bug 8 is fixed, the WHOLE MAP renders tilted in RViz | `slam.launch.py` Fast-LIO branch | — | — |
| 10 | Fast-LIO2 does **not** gravity-align its world frame (`IMU_Processing.hpp:195` — `state_inout.rot = Eye3d` is commented out). `camera_init` = IMU body orientation at t=0, which IS the mount tilt. Even with Bug 9 fixed, the map frame itself stays tilted | `map → camera_init` static in `slam.launch.py` | — | — |

### Ramp-surface scatter (bonus, 2026-04-17)

On slopes, the world-frame z-band filter (`occupancy_min_z: 0.30`) can't distinguish "ramp surface at world-z > 0.30" from "wall". Ramp points leak through as phantom obstacles. Fix: enable octomap's RANSAC `filter_ground_plane: True` — it segments ground in base_link frame (which tilts with the robot), so the ramp reads as horizontal and gets dropped. Additional: `filter_speckles: True` removes isolated voxel noise from specular reflections.

### Final TF chain (Fast-LIO + Mid-360)

```
map                       GRAVITY-ALIGNED (our gravity-true frame)
 │
 │ static TF "map_to_camera_init":
 │   roll -0.036809, pitch +0.263591      ← inverse-tilt of measured body init,
 ▼                                          recovers gravity-alignment from
camera_init               Fast-LIO's non-gravity-aligned world
 │
 │ dynamic TF (Fast-LIO):  camera_init → body
 ▼                                          tracks robot motion in IMU frame
body                      IMU body (tilted by mount on level floor)
 │
 │ static TF "body_to_base_link_fastlio":
 │   roll +0.036809, pitch -0.263591      ← cancels mount tilt
 ▼
base_link                 LEVEL robot chassis frame
```

At t=0 (robot still, just-initialized Fast-LIO), `camera_init → body` is identity, so `map → base_link = identity × identity × identity = identity`. Base_link is level, map is gravity-aligned. When the robot moves or rotates, only the middle dynamic TF changes; the two statics hold the gravity-alignment throughout.

### Mid-360 calibration tool

Mount tilt is measured by reading the stationary accelerometer on the Mid-360's internal IMU. Gravity's direction in body frame gives roll/pitch of the sensor relative to vertical. Yaw is **not** observable from accel alone.

```bash
# Terminal A — get Livox driver publishing /livox/imu:
./scripts/real/real_autonomy.sh                     # or just slam.launch.py slam:=fastlio_mid360

# Terminal B — record 20 s of stationary IMU, compute tilt:
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/real/connect_ethernet.sh && setup_cyclonedds_ethernet
python3 src/go2w/go2w_real_bringup/tools/measure_mid360_tilt.py --seconds 20
```

Outputs an inlineable `static_transform_publisher` arguments block. Paste into `slam.launch.py` **both** the `body_to_base_link_fastlio` TF **and** the `map_to_camera_init` TF (same magnitude, opposite signs — they are proper inverses of each other).

### Why Carto path wasn't hit by the same bugs

- Bug 7 (scan source): carto runs `transform_everything`, `/utlidar/transformed_cloud` is alive → `pointcloud_to_laserscan` has input.
- Bug 8 (sensor origin): the carto viz-only octomap already subscribed to `/utlidar/transformed_cloud` (body frame), not a world-frame cloud → sensor_origin was correctly dynamic from day one.
- Bug 9/10 (mount tilt + Fast-LIO world): Cartographer runs `gravity-aligned scan matching` internally, and the L1 LiDAR has a known-fixed 15.1° mount angle pre-compensated by `transform_everything.py`. No mount-tilt calibration required.

Carto's "slowness" (sometimes mistaken for "map not expanding") is real but benign: `motion_filter.max_distance_meters: 0.1` in the lua + `publish_period_sec: 1.0` on the occupancy_grid_node mean updates at ~1 Hz and only when the robot has moved > 10 cm or rotated > 3°. Expected behavior, not a bug.
