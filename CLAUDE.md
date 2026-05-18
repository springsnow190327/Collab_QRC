# CLAUDE.md — Collab_QRC Index

Multi-robot autonomy with Unitree Go2W wheeled-legged quadrupeds + Go2 walking quadrupeds on ROS 2 Humble + MuJoCo (primary) / Gazebo Classic (legacy) / real-robot deployment. Active focus: **Nav2 SE2 holonomic stack tuning** for both single-robot exploration and heterogeneous dual-robot (Go2W + Go2) coordination, with CFPA2 frontier allocation.

Door task (Phase 2 dual-robot VLM coordination) and the legacy A*/default Python nav backends were removed in the 2026-05 cleanup; see [CLAUDE1.md](CLAUDE1.md) for Phase 1 VLM exploration history, Phase 2 FSM archive, archived 2026-04 operational notes, and the deletion log.

## Active state (2026-05-18 late night) — Jetson Orin Nano 8GB HIL bag-replay: fast_lio real-time confirmed, Point-LIO ROS 2 port dead-ended

**Goal**: prove the full autonomy stack (SLAM + elevation + Nav2 + CFPA2) sustains real-time on Orin Nano 8GB (`johnpork233@192.168.55.49`, JetPack 6.2.2, 6× Cortex-A78AE @ 1.5 GHz, 8 GB RAM) **as a weaker-board proxy for the real Go2's Orin NX 16GB** (8× @ 2.0 GHz, 16 GB). Pipeline must keep up under wall-clock bag-replay (`onboard_noetic_20260511_155920_ops2_ros2_raw`, 180s real Go2 walk, raw `/livox/imu` 200 Hz + `/livox/lidar` 10 Hz CustomMsg only — SLAM rebuilt from scratch, no pre-recorded Odometry / TF used).

### TL;DR

- **fast_lio (HKU FAST-LIO2 = `src/vendor/fast_lio`) is the production SLAM**. On bench Orin Nano: `/robot/Odometry` 10 Hz for 170s, RSS 192 MB constant (ikd-tree NOT unbounded with our config), CPU 9-12% single-core. RTF measured = 1.07 (sim/wall within ±5% noise). Real RTF = 1.0. Pose makes physical sense (~1 m/s walking, matches the captured Odometry baseline trajectory).
- **dfloreaa's Point-LIO ROS 2 port (`src/vendor/point_lio_ros2`) is abandoned for this hardware.** Two independent show-stoppers:
  1. **Heisenbug SIGSEGV** at IMU Init 100% (exit -11) on aarch64, ~50% reproduction rate in full-launch. Standalone gdb doesn't reproduce. Cause: startup race between Point-LIO's first publish, bag's first IMU/lidar arrival, and DDS publisher-subscriber matching window. Respawn=True works (2nd-Nth respawn succeeds because subscribers already discovered), but only buys process liveness — see #2.
  2. **SLAM divergence even when alive**: pose accelerates to (-148, -72, -33) in 6 seconds (40 m/s implied velocity, robot was walking 1 m/s). Tuned `mid360_go2_real.yaml` (`start_in_aggressive_motion: true`, `satu_acc 3→5`) reduced divergence rate ~30× but did not fix it. Same IMU+lidar data → fast_lio gives sensible (~1 m/s) trajectory.
- **Real bottleneck on Jetson is NOT SLAM. It's the Python single-core nodes on ARM**:
  - **CFPA2 single_robot** tick p95 **1376 ms vs budget 500 ms** (2.75× over). Adaptive load shedding kicked in (stride 2→8, targets 180→80, gain_r 40→27, skip 0→3). Still usable, but frontier replan responds at ~1.4s instead of 100ms.
  - **grid_map_to_occupancy_grid** publishes `/robot/traversability_grid` at **0.59 Hz** (vs ~5 Hz target). Single-core Python on ARM at 99% CPU.
  - Both will not improve on Orin NX 16GB — same single-core perf ratio (1.33× = ~1s tick at best). Long-term fix: numba JIT hot path or partial C++ rewrite.

### Origin / record bag context

The bag was recorded onboard the real Go2 (Orin NX 16GB) on 2026-05-11 with Mid-360 + IMU. SLAM stack at the time was **Noetic FAST-LIO** → ros1_bridge → ROS 2 → `rosbag record`. The recorded `/robot/Odometry` has Count=1023 over 180s = **5.7 Hz average**. This was originally cited (CLAUDE.md 2026-05-13 archive note) as "FAST-LIO2 degraded from 9 Hz to 4.4 Hz over 3-min ops2 walk: ikd-tree grew unbounded". This bench test contradicts that conclusion — see "Why bench is faster than onboard" below.

### What got it working — the 8 fixes shipped today

1. **DDS isolation via `ROS_DOMAIN_ID=42`** in `run_jetson_bag_full_load.sh`. Cross-host multicast was leaking ghost `imu→body` static-TF publishers from the desktop's HIL stack into the Jetson's TF tree, splitting it into `{map→odom}` + `{imu→body→base_link}` two unconnected trees ("Could not find a connection between odom and base_link"). Even after the desktop HIL stack was killed, the DDS discovery cache held the phantoms for tens of minutes. Domain isolation is the surgical fix.
2. **`livox_ros_driver2_msgs` minimal msgs-only package** at `src/vendor/livox_ros_driver2_msgs/` (separate dir, COLCON_IGNORE the original `src/vendor/livox_ros_driver2/`). Provides `livox_ros_driver2/msg/CustomMsg` + `CustomPoint` deserialization for bag-replay without building the SDK or driver binary on the Jetson. `package.xml` declares the package as `livox_ros_driver2` (same name → fast_lio's `find_package(livox_ros_driver2 QUIET)` finds it). Required also: rebuild fast_lio so the conditional `livox_ros_driver2_FOUND` path compiles in (a previous fast_lio binary built without it threw "Livox lidar_type selected but livox_ros_driver2 not available" at runtime).
3. **Bag-play preflight env sourcing**: launching `ros2 bag play` without `source /home/johnpork233/jetson_ws/install/setup.bash` first → `rosbag2_player` can't find `livox_ros_driver2/msg/CustomMsg` → silently drops the `/livox/lidar` topic with a single WARN line. Symptom: `/livox/imu` flows at 200 Hz but `/livox/lidar` publisher_count = 0. Fix: workspace source in every bag-play invocation.
4. **Bag-play `--clock` + `use_sim_time=true`** everywhere downstream. Without `--clock`, bag's message timestamps (recorded 2026-05-11) appear "1 week stale" to `ros::Time::now()` (wall-clock 2026-05-18) → TF buffer rejects every transform as outside the 10s extrapolation window. With `--clock`, ros2 nodes wait for `/clock` and operate in bag-time → TF buffer happy.
5. **Multi-pass preflight kill** (3 iterations × 1s gap) in `run_jetson_bag_full_load.sh`. Single-pass `pkill -9 -f` sometimes leaves stale `rosbag2_player` processes whose argv took longer than the signal delivery window. Discovered when 2 concurrent bag processes were publishing `/livox/imu` with 132s-offset timestamps causing Point-LIO "imu loop back, clear deque" rejection of every message.
6. **Point-LIO Odometry topic remap**: Point-LIO publishes Odometry on `/aft_mapped_to_init` (HKU upstream name) NOT `/Odometry` (fast_lio convention). dfloreaa's port kept the upstream name. Subscribers (`fast_lio_tf_adapter`, CFPA2, Nav2) expect `/robot/Odometry` so the launch remap is `("/aft_mapped_to_init", f"/{ROBOT_NS}/Odometry")`. Without this, `/robot/Odometry` has 0 publishers, fast_lio_tf_adapter never gets odom, CFPA2 has no robot pose, Nav2 stuck in "Robot is out of bounds of the costmap".
7. **Staggered launch startup** (`TimerAction` with 15/18/22/28/30s delays) for the heavy subscribers (elevation_mapping at +15s, grid_map_to_occupancy at +18s, Nav2 at +22s, CFPA2 at +28s, bridge at +30s). Point-LIO + 3 statics + adapter come up at t=0; bag at t=12. This is a workaround for the Point-LIO SIGSEGV race — see #1 in TL;DR. With staggered startup the survival rate rose from ~50% to maybe ~70%; with `respawn=True respawn_delay=2.0` added on top, every launch eventually succeeds within 1-2 respawns. Doesn't help with the SLAM divergence (#2) though.
8. **`grid_map_to_occupancy_grid.py` rclpy/rospy idiom fix**: replaced 3× `self.get_logger().warn_throttle(clock, ms_int, msg)` (rospy idiom — `RcutilsLogger` has no `warn_throttle`) with `self.get_logger().warn(msg, throttle_duration_sec=N)` (rclpy idiom). Without this fix the node crashed on first throttled-warning path: `AttributeError: 'RcutilsLogger' object has no attribute 'warn_throttle'`.

### Why bench (weaker Orin Nano 8GB) was faster than onboard (stronger Orin NX 16GB)

Initial measurement showed bench `/robot/Odometry` at 10 Hz vs onboard record of 5.7 Hz average. User pushed back: "弱机器不可能比强机器快 1.75×". The challenge prompted concrete experiments:

| Hypothesis for onboard 5.7 Hz | Action | Verdict |
|---|---|---|
| Bag was being throttled by fast_lio backpressure and `ros2 topic hz` is fooled by wall-clock measurements | Direct RTF measurement: sample `/clock` at two wall-clock points 20s apart. wall_dt=21.47s, sim_dt=22.98s → **RTF = 1.07** (within ±5% noise of 1.0). | **REFUTED.** Bag really is wall-clock real-time. |
| `rosbag record` running concurrently with FAST-LIO stole CPU | Bench experiment: keep fast_lio + full stack running, ADD `ros2 bag record` of 8 heavy topics (cloud_registered_body + elevation_map + all of /livox/* + tf) simultaneously. Disk write 38 MB/s sustained. | **REFUTED.** fast_lio CPU was 5.5% before, 5.3% after. Odometry rate went from 10.004 Hz to 10.329 Hz (UP). System loadavg 6.4/6 cores under combined load. |
| **`ros1_bridge` serialization between Noetic FAST-LIO and ROS 2 record path** | Inferred from bag filename `onboard_noetic_*_ros2_raw` (records via bridge). PointCloud2 ~100 KB × 10 Hz per topic crossing bridge requires full re-serialize. Single-core CPU cost is substantial. | **LIKELY culprit.** Not directly tested but the only remaining strong candidate. |
| Onboard `pcd_save_en` accidentally `true` | Default in fast_lio's mid360.yaml is `false`. We explicitly force `false` in the bench launch. Onboard launch may have set `true`. | Possible — would explain unbounded memory + degradation. Not verified. |
| Thermal throttle on real Go2 (sealed belly, sun, walking-induced friction heating) | Bench thermal snapshot under full load: CPU 54.1°C, GPU 53.7°C, SoC 52.8°C. **Bench has 30°C headroom to throttle (~85°C)**. Real Go2 belly is much hotter — even quiescent it would run 70°C+. | Plausible additional factor. Not measurable from this bench. |
| **Concurrent onboard workload (Mid-360 driver IO, motor controller, camera streams, CHAMP, etc.)** | Bench has only the SLAM+autonomy stack; real onboard has 20+ extra real-time threads competing for the same 8 cores. | Almost certainly a contributing factor. |

**Net conclusion**: fast_lio is *not algorithmically slow* on ARM. Onboard 5.7 Hz was a record/integration artifact, not a SLAM algorithm limit. The 2026-05-13 CLAUDE.md note ("FAST-LIO2 ikd-tree grew unbounded → 9→4.4 Hz") was likely the *pcd_save_en=true* configuration, not the algorithm itself.

### Files shipped

- `src/vendor/livox_ros_driver2_msgs/{CMakeLists.txt, package.xml, msg/{CustomMsg.msg, CustomPoint.msg}}` — msgs-only stub package
- `src/vendor/point_lio_ros2/` — uncommented CustomMsg path (preprocess.h, preprocess.cpp, laserMapping.cpp), added `find_package(livox_ros_driver2 REQUIRED)` + `ament_target_dependencies(... livox_ros_driver2)`, plus `<depend>livox_ros_driver2</depend>` in package.xml. Build verified on desktop + Jetson. (Kept in tree even though unused — useful reference for any future port debugging.)
- `src/vendor/point_lio_ros2/config/mid360_go2_real.yaml` — Mid-360-on-walking-Go2 tuned config (`start_in_aggressive_motion: true`, `satu_acc: 5.0`, `publish.path_en: false`). Not on the production path now but preserved.
- `scripts/real/orin_nano_bag_full_load.launch.py` — full bag-replay HIL launch: fast_lio + 3 statics + `fast_lio_tf_adapter` + elevation_mapping_cupy + grid_map_to_occupancy + Nav2 stack + cfpa2_single_robot + cfpa2_to_nav2_bridge. With staggered TimerActions and respawn options.
- `scripts/real/run_jetson_bag_full_load.sh` — runner: multi-pass preflight kill (incl. `rosbag2`, `yes`, `point_lio`, etc.), DDS shm cleanup, `ROS_DOMAIN_ID=42` isolation, `ELEVATION_MAPPING_FORCE_CUPY=1` env. Prints the bag-play one-liner the user runs in a second window.
- `scripts/real/deploy_to_orin_nano.sh` — updated vendor list: `point_lio_ros2` + `livox_ros_driver2_msgs` (replaces old `livox_ros_driver2` + `Livox-SDK2`). Build target now includes `point_lio`.
- `src/collaborative_exploration/trav_cost_filters/trav_cost_filters/grid_map_to_occupancy_grid.py` — 3× rospy→rclpy `warn_throttle` → `warn(throttle_duration_sec=)` migrations.

### Open / next

- **CFPA2 + grid_map_to_occupancy are the next optimization targets.** Single-core Python on ARM is fundamentally slow at the ~5 Hz target rates. Options: numba JIT the BFS/clustering hot paths in CFPA2, or rewrite grid_map_to_occupancy in C++ (it's a fixed-grid OccupancyGrid converter with seed-flood + threshold logic — straightforward to port).
- **Point-LIO ROS 2 port C++ fix** is non-trivial (DDS publisher init race + SLAM divergence on Mid-360 walking data) — not worth the bench effort; production stays on fast_lio. Keep the vendored copy + uncommented CustomMsg path for future reference if a different port appears upstream.
- **2026-05-13 CLAUDE.md note about fast_lio degradation** should be updated to reflect this bench test result — with `pcd_save_en=false` + `extrinsic_est_en=false` (current yaml defaults), 170s on the same ops2 bag shows no degradation. The earlier observation likely had different config.
- **5+ minute long-run test** (loop the 180s bag 2-3 times) needed to definitively rule out long-tail ikd-tree growth on bench. Started but not completed in this session.

## Active state (2026-05-18 evening) — bridge-as-obstacle root-cause hunt: it's the ingest, not the CNN

Follow-up to the morning's trav-CNN fine-tune commit (2dd8664). The exploration sim worked end-to-end and the robot spawned correctly at (0,0), but **RViz showed overhead bridges / awnings as red lethal cells** in `trav_fused`, even with the fine-tuned weights. Spent the afternoon ruling out hypotheses; final fix was upstream of the CNN.

### What was tried and what it told us

| Hypothesis | Action | Verdict |
|---|---|---|
| CNN under-trained on bridges → train more epochs | Inspected curve: val_mse plateau at ep100 (0.04). Adding epochs would not move it. | **Not it.** |
| CNN sees too narrow a context (7×7 at 0.10m = 0.7m FoV, smaller than typical bridge → patch looks like a wall step at the bridge boundary). | Added `--patch-stride` to [`build_trav_dataset.py`](scripts/training/build_trav_dataset.py): same 7×7 cells but spaced 0.20m, total FoV 1.4m. Trained `weights_ops2_wide.dat` (val_mse 0.088, val_acc 0.904 — worse than tiled 0.040/0.948). | **Not it.** Wider FoV degraded the model. The 7×7 base CNN already had enough info; smearing it over 1.4m hurt fine-detail walls. |
| CNN really is wrong on uniform high-z patches | Synthetic eval: fed CNN three patch types — floor (z=0), bridge top (uniform z=3m), wall edge (half z=0 / half z=3m). **All weights (pretrain / ops2_tiled / ops2_wide) predict 0.84+ free for both floor AND bridge, and 0.000 lethal for wall edge.** The CNN was correctly classifying bridges as free the whole time. | **CNN ≠ root cause. Stop blaming the model.** |
| `grid_map_to_occupancy_grid.elevation_cost_enabled=True` was unconditionally adding 90 cost for cells with z > 1.5m, overriding CNN's free verdict | Disabled it via [nav_test_3d_explore.launch.py:292](src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py#L292). Bridges still lethal. | **Not it** (but `elevation_cost_enabled=False` left as default since it didn't help and added unnecessary penalty). |
| Nav2 global_costmap.static_layer reads `/robot/map` (octomap 2D projection — octomap traces LiDAR hits regardless of z, so bridge tops project DOWN to occupied) instead of `/robot/traversability_grid` (CNN-fused, has bridge override) | Switched static_layer `map_topic` from `/robot_b/map` to `/robot_b/traversability_grid` in [nav2_go2_full_stack.yaml](src/go2w/go2w_config/config/nav/nav2_go2_full_stack.yaml). Bridges still lethal. | **Not it.** Nav2 was already going to do the right thing once trav_grid is right — but trav_grid was wrong upstream. |
| **elevation_mapping_cupy itself ingests bridge-top points and writes z=3-4m to the cells beneath them. CNN then sees those cells as either uniform-high (correctly classified free) or as edges (incorrectly classified lethal due to half-bridge half-no-data patches at bridge boundaries).** Simplest fix: cap `max_height_range` so bridge points never enter the map. | [elevation_mapping.yaml](src/collaborative_exploration/trav_cost_filters/config/elevation_mapping.yaml): `max_height_range: 5.0 → 1.7` (sensor sits at z≈0.3m, 1.7m above sensor = z≈2.0m world). Also lowered `ramped_height_range_b: 2.5 → 1.7` for the slant-range gate. | **✓ FIX. Verified in sim.** Bridges cleared in `trav_fused`. |

### Conclusion

The CNN was correctly classifying every patch type it could possibly see. The bug was that **bridges were entering the elevation map at all**, polluting the input regardless of how good the model got. The fix sits one level upstream from where we were looking, and is a one-line YAML change — but it took eliminating four other plausible suspects (CNN training, patch size, downstream elevation_cost, Nav2 static_layer source) to know that.

A second secondary fix shipped along the way:
- **Nav2 global_costmap static_layer** now reads `/robot_b/traversability_grid` instead of `/robot_b/map`. The cnn-fused trav layer with bridge override is strictly more correct than octomap's 2D z-collapsed projection for planning, even if the bridge issue happens to be solved upstream now.

### Other touch-ups this afternoon

- **`scripts/real/flatten_floor_height.py`** removed by user during a roll-back iteration (not on the current path anyway; we ended up with the cleaner `strip_floor_with_patchwork.py → ransac-tiled` mode for any future cleanup needs, and `scans_v4_sparse.obj` as the actual sim asset).
- **`build_trav_dataset.py --patch-stride N`** added during the hypothesis hunt and left dormant. Use the default (stride 1, 7×7 at 0.10 m = 0.70 m FoV) for production fine-tunes — the wide-context variant degraded val_mse without solving the bridge issue. CNN architecture would also have to scale dilations via `traversability_filter.py` to deploy a stride>1 model, which we did not do.
- **Simple 3D ramp demo** runs unchanged via `./scripts/launch/nav_test_3d_explore.sh` (demo_ramp.xml default) or `nav_test_3d_explore_go2.sh` (Menagerie Go2 body). With today's `max_height_range: 1.7` change, demo_ramp is unaffected (its tallest features are well under 2m) but ops2 bridges should clear.

## Active state (2026-05-18) — ops2 trav-CNN fine-tune end-to-end + spawn bug + Nav2 tightening

End-to-end: built the offline label-generation → 7×7 patch extraction → CNN fine-tune → sim-test pipeline for elevation_mapping_cupy's 120-param traversability filter. The sim now spawns at (0,0), uses fine-tuned weights by default, drives autonomous exploration across the ops2 scene, and goes ~15m down the main corridor in 90s.

### The shipped pieces

1. **Heightmap → labels → patches → CNN fine-tune pipeline** in [`scripts/training/`](scripts/training/):
   - [`build_float_heightmap.py`](scripts/training/build_float_heightmap.py) — top-down z(x,y) from mesh, float32 npz
   - [`auto_label_heightmap.py`](scripts/training/auto_label_heightmap.py) — slope+step+height-above-floor rules on real ops2 mesh (vs `synth_terrain_dataset.py` which uses synthetic toy ramps)
   - [`polish_trav_labels.py`](scripts/training/polish_trav_labels.py) — morphological speckle remove + bridge override (height > 1.2m AND slope < 20° → free)
   - [`build_trav_dataset.py`](scripts/training/build_trav_dataset.py) — 7×7 patch extraction with rich augmentation (rot×4, flip-LR/UD, gaussian noise ±0.03m × N rounds, dropout, tilt ±5°, gaussian blur). 6M patches from 60k labelled cells.
   - [`train_trav_filter.py`](scripts/training/train_trav_filter.py) — already existed; extended with **weighted MSE / BCE / focal** loss (`--lethal-weight 3.0` for FP-lethal-over-FN-lethal bias), **label smoothing 0.05** (guards against ~10% label noise), **pretrain mix 30%** every epoch (anti-catastrophic-forgetting from synth `weights_pretrain.dat`), **weight_decay 1e-4**, **early stopping**, and **periodic checkpoints** every N epochs.
   - Result: val_mse 0.04, val_acc 0.95 across all fine-tuned variants (v2 / snap / flat / ransac / tiled). Pretrain baseline was 0.117 / 0.86. Per-class on tiled dataset: trav_pred 0.92, lethal_pred 0.11. **5090: 6M patches × 150 epochs in ~30 s** (120-param network, batch 4096, GPU 50% util — kernel launch overhead, not compute-bound).

2. **5 mesh-cleanup variants explored** for the visual mesh feeding sim LiDAR. All trained equally well (CNN sees similar patch distributions), differ in **sim realism**:
   - **strip** (patchwork delete-tris) — has holes, LiDAR escapes
   - **snap** (patchwork + project ground to z=0) — flat but loses ramp tilt
   - **flat** (no patchwork, z<0.10m → 0) — simpler, catches 88% more low-z noise than patchwork (284k vs 241k verts)
   - **ransac** (patchwork + single-plane RANSAC) — preserves 0.43° real building tilt; ground std 16cm = expected tilt × 80m extent
   - **tiled** (patchwork + per-2m-tile RANSAC) — preserves multi-level floor + ramps (p5 z=−0.15m → p95 z=+0.19m, true 0.34m height range across the building). Spawn tile has std 4mm (essentially flat).
   - User reverted aggressive variants (orphan-strip dropped 79k debris tris, vertex-cluster 0.4m got us to 52k tri) because they cut too much; we ended back at original mesh (`scans_v4_sparse.obj`, 56k v / 150k tri) with `pos="0 0 0"` so the mesh's natural ground at z=−0.28m sits one Go2 height (~30cm) below the MJCF floor plane (z=0) acting as visual backdrop.

3. **THE spawn-position bug** — robot was spawning at **(25.x, -15.x)** instead of (0,0) despite keyframe qpos="0 0 0.32". Root cause at [`slam_ops2_v4_go2_real.xml`](src/go2w/go2_gazebo_sim/mujoco/slam_ops2_v4_go2_real.xml#L1213): `<body name="base_link" pos="29.51 -15.77 0.32" ...>` — the body's default `pos` attribute was hardcoded to a stale old spawn from a different scene. With a `<freejoint>`, both the body `pos` and the keyframe qpos contribute; the body pos was apparently winning during sim startup. Fixed in both MJCF variants. Verified via headless MuJoCo: `qpos[:3]=(0, 0, 0.32)`. Live sim confirmed: GT pose at t=0 was (-0.03, +0.0, +0.28), exploration drove the robot to (15.39, -9.14, 0.28) over 90s.

4. **Nav2 Go2 collision tightened** ([`nav2_go2_full_stack.yaml`](src/go2w/go2w_config/config/nav/nav2_go2_full_stack.yaml)) to fit narrower corridors: footprint 0.70×0.40m → **0.64×0.36m**, MPPI `collision_margin_distance` 0.10 → **0.05m**, local `inflation_radius` 0.30 → **0.22m**, global 0.25 → **0.20m**. Effective rejection envelope: 0.90×0.60m → **0.74×0.46m** (-16cm wide, -14cm tall).

5. **Default trav weights = fine-tuned**. [`nav_test_slam_ops2_v4_go2.sh`](scripts/launch/nav_test_slam_ops2_v4_go2.sh) now auto-loads `weights_ops2_tiled.dat` when present; falls back to `weights_pretrain.dat` only if no fine-tune available. Boot log explicitly prints which one is in use.

6. **`gt_passthrough` mode for `fast_lio_tf_adapter`** ([`scripts/runtime/fast_lio_tf_adapter.py`](scripts/runtime/fast_lio_tf_adapter.py)) — env `GT_PASSTHROUGH=1` makes the adapter subscribe to `/<ns>/odom/ground_truth` and emit TF directly from GT, bypassing Fast-LIO. Useful for isolating spawn-position issues from SLAM bootstrap timing.

7. **`elevation_cost_max_h: 1.00 → 1.5m`** ([`nav_test_3d_explore.launch.py`](src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py#L295)) — broadens the height band over which `grid_map_to_occupancy_grid` scales elevation cost into the 0–90 occupancy range.

### Other detours (kept as scripts but not on the main path)

- **Sonata (Meta self-supervised PTv3) semantic segmentation** [`scripts/real/sonata_inference.py`](scripts/real/sonata_inference.py), [`sonata_to_instances.py`](scripts/real/sonata_to_instances.py), [`sonata_visualize_instances.py`](scripts/real/sonata_visualize_instances.py). Got it running on Blackwell sm_120 by upgrading `spconv-cu120` → `spconv-cu126 2.3.8` + `cumm-cu126` (the prior version had no sm_120 cubins and silently SubMConv3d-segfaulted). Achieved 0.954 acc per-pixel on the v4 mesh. **But the ScanNet-20 taxonomy is wrong for our outdoor/semi-outdoor building scene** — patchwork picked 26.1% of verts as ground (matching truth) but Sonata mis-labelled most building structures as "table" or "bed". Dropped semantic path. Sonata's 5 instances (with z<1.5m guard) were still extracted into 120 CoACD convex-decomposition collision geoms (`ops2_inst_*` in MJCF) — kept for collision-only use.
- **Patchwork++ ground stripping** [`scripts/real/strip_floor_with_patchwork.py`](scripts/real/strip_floor_with_patchwork.py) (recreated inline after user deleted it during one iteration). Multiple modes: `strip` (delete tris, leaves holes), `snap` (project ground to z=0), `ransac` (single plane), **`ransac-tiled`** (per-tile plane, preserves multi-level floors). With z<0.05m guard, rescued 14k pillar-base verts that patchwork over-classified as ground.
- **Mesh editor GUIs** (none turned out to be the right UX): [`mesh_box_carver.py`](scripts/real/mesh_box_carver.py) 3D AABB select-and-delete, [`mesh_height_cutoff.py`](scripts/real/mesh_height_cutoff.py) 2D top-down + max-z cutoff slider, [`trav_labeler.py`](scripts/training/trav_labeler.py) 2D paint w/ z-band slider + Bresenham straight-line tool, [`trav_labeler_3d.py`](scripts/training/trav_labeler_3d.py) Open3D shift-click point picker, [`trav_threshold_tuner.py`](scripts/training/trav_threshold_tuner.py) live-slider rule tuner. User preferred automated path over manual labeling.
- **`flatten_floor_height.py`** [`scripts/real/flatten_floor_height.py`](scripts/real/flatten_floor_height.py) — height-only snap (no patchwork): every vert with z<0.10m → z=0. Catches 88% more low-z noise than patchwork but loses any real curb/step <10cm.

### Open problem — overhead bridges treated as lethal at runtime

The trav grid (visible in RViz `/robot/traversability_grid`) marks overhead bridges/awnings as **red lethal**, NOT free as we want for "robot walks under". Why: at offline label time, `polish_trav_labels.py` applies a `bridge_height_m=1.2m AND slope<20°` rule that flips high+flat cells to FREE. The CNN learns this from labels. **At runtime**, the elevation_mapping_cupy CNN sees a 7×7 patch from the live heightmap (max-z per cell from LiDAR) — but there is no equivalent height-based override in the *output* pipeline. The CNN's per-patch output gets multiplied with the `ramp_safe` analytical fallback ([grid_map_filters.yaml](src/collaborative_exploration/trav_cost_filters/config/grid_map_filters.yaml)) which is meant for ramp recovery, not bridge clearance.

Two ways to fix:
1. Add a post-CNN filter in `grid_map_filters.yaml`: `height_above_floor > 1.5m AND slope < 20° → trav_fused := max(trav_fused, 0.95)`. Same logic as offline polish, applied to the live grid before publish.
2. Train the CNN with patches that include FLOOR underneath a bridge (i.e. patches that show "z=4m here, but z=0 just 1m laterally" → label=trav). Currently our patches show only the top-down max-z, so a bridge patch looks like a tall obstacle.

(1) is the quick fix; (2) is the principled fix but needs a different patch builder.

## Active state (2026-05-16) — real-world walk → MuJoCo collidable scene (Go2 explores SLAM mesh autonomously)

End-to-end: offline replay of a real-robot bag → static-only mesh → Go2 spawned and trotting inside it under CFPA2 + Nav2, with only foot-floor contacts.

**Pipeline** (all scripts in [`scripts/real/erasor/`](scripts/real/erasor/)):
1. **`offline_slam.sh tag=ops2`** — Fast-LIO 2 on raw `/livox/lidar` + `/livox/imu` from a Noetic bag. No loop closure: SC-A-LOAM's ScanContext gets fooled by glass-window reflections into false LC, which warped the map worse than drift. Verified by building the full SC-A-LOAM + ERASOR Docker stack ([`scripts/real/erasor/Dockerfile`](scripts/real/erasor/Dockerfile)) and seeing LC-corrected output corrupted. Output: 25.8M points → [`bags/meshes/ops2_final/scans.obj`](bags/meshes/ops2_final/scans.obj).
2. **`density_filter.py --min-pts 8`** — temporal-consistency filter via points-per-voxel count. Static surfaces accumulate 60-700+ points/voxel across all keyframe contributions; pedestrian trails + glass-reflection outliers only 1-5 points/voxel. Threshold at 8 keeps 70.3%, removes the rest. Decoupled from mesh reconstruction (the user's key insight) — Poisson on already-clean cloud doesn't hallucinate over trail-shaped gaps.
3. **`pcd_to_mesh.py --method poisson --voxel 0.02 --depth 11 --density-pct 10`** + quadric decimation to 500K triangles + `cluster_connected_triangles` cull (keep ≥ 2000 tri components) → 240k v / 473k f.
4. **RANSAC ground alignment** in bottom-5%-z subset (initial bottom-30% picked up walls and gave 2.5° residual tilt; tight bottom-5% gives **0.19°**). Mesh ground forced to z=0 ±0.04m.

**5 MuJoCo integration fixes** (each was a blocker for autonomy):
1. **Whole-mesh convex hull pushed robot through floor.** MuJoCo collides `<geom type="mesh">` via its **convex hull** by default. An 80×32×10 m scene-wide hull pushed the body box to z=-0.078 (8cm below floor) regardless of spawn z. Fix: [`tile_mesh.py`](scripts/real/erasor/tile_mesh.py) splits mesh into 8×4 XY grid (22 non-empty tiles, max 42K tri/tile). Each tile's hull approximates only its local geometry → robot stands at proper z=0.236 m.
2. **CHAMP can't stand a robot whose initial joints are at 0** (legs straight down). With nominal joints `(hip=0, thigh=0, calf=0)` and body spawned at z=0.60, legs hit floor extended; CHAMP commands the fold-to-standing pose but motors can't lift body weight against tucked-leg geometry. Fix: `<keyframe>` block with `qpos` initialized to `(thigh=0.9, calf=-1.8)` per leg — robot spawns already in trotting-ready pose at z=0.32. nq=**19** for Go2 (no foot joints in MuJoCo despite ROS-side `*_foot_joint` names from URDF parsing).
3. **`initial_pose_guard.py` re-pins robot to (0, 0, 0.38) for 14 s after spawn** via `/gazebo/set_entity_state` calls. In MuJoCo mode that service doesn't exist so the calls no-op, BUT the script still consumes the `spawn_x/spawn_y/spawn_z` launch args; passing them via the wrapper overrides the (0, 0, 0.38) default at the param-store level (no functional override needed for MuJoCo, but matters for future Gazebo runs).
4. **Wrong launcher** — `nav_test_mujoco_fastlio.launch.py` doesn't include the elevation_mapping + filter_chain + grid_map_to_occupancy_grid trio, so CFPA2 spins for hours warning `Waiting for map topic from: robot`. Only [`nav_test_3d_explore.launch.py`](src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py) has them. New wrapper [`scripts/launch/nav_test_slam_ops2_go2.sh`](scripts/launch/nav_test_slam_ops2_go2.sh) calls 3d_explore.
5. **Spawn point selection** — the bag's first Fast-LIO odom is at (0, 0, 0) in `camera_init`, after alignment that's a poor xy spot (mesh has walls/clutter within 2 m). Sweep over the mesh bbox finds (5.12, -8.76) as best: 148 ground verts in 1 m radius, **0 mesh verts inside robot body volume** within 0.8 m. Encoded as the `<keyframe>` qpos x/y.

**Verified end state** ([`slam_ops2_go2.xml`](src/go2w/go2_gazebo_sim/mujoco/slam_ops2_go2.xml) + [`scripts/launch/nav_test_slam_ops2_go2.sh`](scripts/launch/nav_test_slam_ops2_go2.sh)):
- Standing: base_link z = **0.236-0.241 m**.
- Contacts: only `floor|*_foot_collision` pairs (no body collision).
- Trotting: foot contact state `[F, F, F, T]` cycling — 1-2 feet on ground at a time.
- Motion: position trace **(4.68, -9.02) → (3.07, -9.70) → (4.02, -10.02)** in 12 s — actively exploring.
- Stack: trav_grid 1.7 Hz, cmd_vel 20 Hz, way_point_coord driving to (17.55, -13.45).

## Active state (2026-05-15) — ETH elevation mapping + CNN traversability + ramp_safe fusion live; Go2W still tips at ramp edges (open)

Single-session end-to-end stand-up of the ETH RSL elevation_mapping_cupy stack with the CNN traversability filter actually running, fused with the analytical chain, and consumed by Nav2 in 2D. Robot autonomously explores demo_ramp via CFPA2 → Nav2 with no scripted ramp helper. One remaining open issue: Go2W tips when the planner routes it near the ramp foot transition / platform cliff edge — see [docs/claude/ramp_tipover_open_problem.md](docs/claude/ramp_tipover_open_problem.md).

### The win

1. **Pure-cupy ETH CNN backend on Blackwell sm_120.** torch 2.7.1 ships sm_50..sm_90 cubins only — `torch.cat` fails with `CUDA_ERROR_NO_BINARY_FOR_GPU` on the 5090, which is why a prior Phase-3.3 patch had *disabled* the CNN at [`elevation_mapping.py:408-417`](src/vendor/elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping.py#L408-L417) and left layer 3 at its init value 1.0 (so `traversability` was a no-op). Added [`get_filter_cupy()`](src/vendor/elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_cupy/traversability_filter.py) — 65 lines of pure cupy reimplementing the 3-branch dilated 3×3 conv stack + 1×1 fusion + `exp(-|·|)`. Float32 match vs numpy reference < 1e-7. Runtime backend selector at [`elevation_mapping.py:145-163`](src/vendor/elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping.py#L145-L163) picks the cupy filter when `cudaDeviceProperties.major >= 10`. CNN populates the `traversability` layer at ~5 Hz; on demo_ramp it correctly marks wall TOPS lethal (not just the 1-cell rim), catching ~450 more wall cells than the analytical chain.
2. **CNN ↔ analytical fusion in the filter chain** ([`grid_map_filters.yaml`](src/collaborative_exploration/trav_cost_filters/config/grid_map_filters.yaml)). CNN over-rejects ramps (training was on mostly flat ground). Built a soft analytical rescue: `slope_margin = clamp((0.5236−slope)/0.5236, 0, 1)`, `step_margin = clamp((0.06−step_residual)/0.06, 0, 1)`, `ramp_safe = clamp(slope_margin·step_margin·100, 0, 1)` (gain 100 + clamp turns the product into a near-binary mask). Then `trav_fused = max(traversability, ramp_safe) = 0.5·(a + b + |a−b|)` (EigenLab has no element-wise max). Live numbers: ramp cells CNN_lethal 91 → fused_lethal 9 (out of 1012 eligible); wall cells preserved (1834 → 1834); flat ground cleaned (452 → 0). Nav2 reads `trav_fused` from `/robot/traversability_grid`; raw CNN and analytical `trav_eth` remain as parallel comparison layers in RViz.
3. **2D CFPA2 on the fused trav grid.** With `trav_fused` producing a clean 2D OccupancyGrid, CFPA2's BFS-on-OccupancyGrid (`ig_dimension: 2d`, `planning_map_topic_suffix: /traversability_grid`) is sufficient — no need for the 3D voxel-cluster IG path. Removed `ramp_ascent_goal_node`, `ramp_cmd_vel_assist_node`, and `frontier_3d_test_node` from the launch. Pure CFPA2 → Nav2 autonomy.
4. **Height-based extra cost.** Added optional `elevation_cost_enabled` to [`grid_map_to_occupancy_grid.py`](src/collaborative_exploration/trav_cost_filters/trav_cost_filters/grid_map_to_occupancy_grid.py): cell cost mixed via max() with `clamp((elev − h_min)/(h_max − h_min) · v_max, 0, v_max)`. With `h_min=0.05, h_max=1.00, v_max=90`, live values on demo_ramp: flat ground mean cost 0.0, ramp_foot 0.2, ramp_mid 34, ramp_top 82. Planner now prefers flat-ground routes when reaching the same frontier doesn't require climbing.
5. **Mid-360 sim densification + real-robot mount calibration.** Replaced the uniform-grid raycast in `mujoco_ros2_control`'s lidar plugin with Livox-SDK official Risley non-repetitive scan-pattern replay (`scan_patterns/mid360.csv`, 800k samples = 4 s, 20k rays/frame + σ=2 cm Gaussian range noise). Calibrated Mid-360 mount (roll=−2.11°, pitch=+15.10°) in every sim TF source — 18 MJCFs, 2 xacros, the `go2w.urdf` snapshot, `pointlio_gazebo*.yaml` extrinsics, gbplanner3 sim static TF publisher, and the gbplanner3 demo3 launcher. Sim point cloud now matches real-robot density and the elevation map renders without the spurious 2° tilt that the previous 13°-pitch-only guess introduced.

### Debugging journey (this session)

The CNN re-enable + fusion + autonomy hookup happened in a tight feedback loop where each fix exposed the next layer's problem:

1. **CNN was off.** First reading of `elevation_mapping.py:408-417` revealed the Phase-3.3 commit had *deliberately commented out* the filter call. `use_chainer: false` was misleading — switching it to `true` wouldn't help because `chainer` isn't even installed. Wrote the cupy backend instead.
2. **RViz default display was reading the wrong layer.** Initial GridMap display had `Color Layer: traversability` but elevation_mapping_cupy's `traversability` layer was all-NaN until the CNN re-enable landed. The actually-computed analytical chain was `trav_eth`. After re-enable both layers had real data — RViz now shows `trav_fused` by default with raw CNN and `trav_eth` as togglable comparisons.
3. **`step_cost` topology bug (analytical chain).** For a wall TOP cell, slope ≈ 0, step_height in 3×3 window ≈ 0 (window all on wall top), so step_cost = 0 → wall top renders as FREE with only a 1-cell lethal rim. Added a max-filter dilation (`maxOfFinites(step_cost)` in 7-cell window) plus a separate `wall_seed = step_height − 0.25` gated detector that decouples wall-vs-ramp signal. Walls now fully red across thickness; ramps untouched. This was needed before CNN came online because CNN itself catches the same case — but the analytical chain is the fallback / comparison layer, so it's worth fixing.
4. **CNN fixes walls but rejects ramps.** First fused-layer screenshot: walls solid red ✓, ramp glowed red too. The CNN was trained on mostly-flat data → sustained slope reads as step-like. Rather than retrain (multi-day infra build), added the `ramp_safe` analytical rescue that `max()`-fuses on top of CNN.
5. **`ramp_safe` initially under-shot.** First version: `ramp_safe = slope_margin · step_margin`, no gain. On a 14° ramp `slope_margin ≈ 0.53` → ramp_safe ≈ 0.53 → trav_fused ≈ 0.53 → yellow in RViz. Added `· 100` then clamp(0, 1) to saturate to near-binary {0, 1} mask.
6. **`filters::FilterChain` silently dropped filters.** First fusion-yaml had filters named `filter9b, filter9c, filter9d, ...` and `filter23a, filter23b, ...`. The chain loaded only `filter1..filterN` contiguous; everything past the first non-integer suffix was silently ignored — observed in the log as `out layers=19` (missing `ramp_safe`, `trav_fused`) without any FATAL. Renumbered all filters to contiguous integers. Memory note: [feedback_filter_chain_integer_keys.md](docs/claude/memory/feedback_filter_chain_integer_keys.md).
7. **EigenLab has no `>` operator.** First attempt at the indicator mask used `(ramp_safe > 0.0) .* 1.0`; loaded as `Unknown variable 'ramp_safe > 0.0'.` and crashed the filter_chain_runner. Replaced with clamped-margin product + gain.
8. **Robot stuck at `no_frontiers` despite obvious unknown territory.** With everything working, CFPA2 reported `no_frontiers` and the robot sat at (1.4, 0.75). Live BFS analysis: 1690 raw frontier cells reachable from the robot's position. The status came from CFPA2's *post-extract* filters:
   - `cfpa2_frontier_obstacle_clearance_m: 0.35` (4-cell radius from any lethal) — the 2 m ramp corridor has walls at y=±1 m → centroid is always within 0.35 m of a side wall
   - `cfpa2_frontier_min_unknown_cells: 20` over `cfpa2_frontier_unknown_check_radius_m: 0.40` (4-cell radius) — the unknown-check window picks up corridor walls so the "live unknown" count collapses to ~0
   - **`cfpa2_max_goal_distance_m: 2.5`** — the load-bearing filter. After the robot maps everything within 2.5 m to free, the next reachable frontier ring is 3–6 m out. `_goal_too_far` drops all of them. Targets list becomes empty → `no_frontiers` reported (not `no_reachable`, because the empty check fires before utility evaluation).
9. **Scene-specific overlay rather than global relax.** Rather than touching the base `cfpa2_single_robot.yaml` (open-scene safe defaults), added a `cfpa2_config_overlay` launch arg to `nav_test_mujoco_fastlio.launch.py` and a [`cfpa2_single_robot_demo_ramp.yaml`](src/collaborative_exploration/cfpa2_collaborative_autonomy/config/cfpa2_single_robot_demo_ramp.yaml) overlay: clearance 0.20, min_unknown 5, unknown_check_radius 0.30, max_goal_distance 15.0. Verified loaded via `ros2 param get`.
10. **Threshold/cost tuning to stop Go2W tipping (partial — see open problem).** Iterated free/lethal: 0.30/0.15 → 0.22/0.10 (too permissive, robot grazed ramp foot) → 0.55/0.25 → 0.60/0.30 (final). Added `elevation_cost_enabled` height penalty (h_min=0.05, h_max=1.00, v_max=90). Bumped Nav2 InflationLayer to `inflation_radius: 0.60, cost_scaling_factor: 2.5`. Robot still tips occasionally — the ramp foot is *legitimately* traversable per every sensor; tipping is a dynamic-stability failure our 2.5D cost layer can't observe. Open problem document lists 5 candidate next steps from "tighten ramp_safe envelope" to "train CNN on tilted terrain".



The ad-hoc 6-step 2D traversability projection in `nvblox_frontend/mapper_node.cpp` has been replaced with the full ETH RSL pipeline. All 7 phases of [plans/2026-05-14-trav-grid-rewrite.md](docs/claude/plans/2026-05-14-trav-grid-rewrite.md) are committed. Phase 7 (A/B validation run) is the remaining step.

**Pipeline (activated when `nav_costmap_mode:=3d`, the default):**
1. `elevation_mapping_cupy` → `/robot/elevation_map_raw` (3 layers: elevation, variance, traversability)
2. `filter_chain_runner` (trav_cost_filters) → `/robot/elevation_map_filtered` (12 layers: adds normal_vectors_{x,y,z}, slope, slope_cost, roughness, roughness_cost, step_height, step_cost, traversability overwritten)
3. `grid_map_to_occupancy_grid` (trav_cost_filters) → `/robot/traversability_grid` (OccupancyGrid, Nav2 cost convention: 0=free if trav≥0.7, 100=lethal if trav<0.3)

**Key bugs fixed during integration (all in one session):**
- `/**:` wildcard required in YAML (not bare `filter_chain_runner:`) so params load under `/robot` namespace
- `ThresholdFilter` expects `layer:` not `condition_layer:` + `output_layer:`
- EigenLab `max(scalar, matrix)` invalid → use multiplicative traversability: `(1-slope_cost)*(1-roughness_cost)*(1-step_cost)`
- `ament_cmake_python` installed egg-info lacks `entry_points.txt` → Python executable installed via `install(FILES ... RENAME ...)` with execute permissions instead of `console_scripts` stub

**Smoke test verified:** 12 layers in `elevation_map_filtered` at 5 Hz, OccupancyGrid at ~9 Hz, against live demo_ramp sim.

## Active state (2026-05-14) — 3D frontier exploration unstuck: 6 compounding bugs

Day-long debug pass on `nav_test_3d_explore.sh` (demo_ramp + nvblox_frontend + CFPA2 3D IG). Robot kept getting stuck on the same goal forever. Six independent bugs compounded; full details + verification log in [docs/claude/3d_frontier_debugging.md](docs/claude/3d_frontier_debugging.md).

- **Ring-frontier centroid bug (the load-bearing one).** `centroid_world = (xs.mean(), ys.mean(), zs.mean())` of frontier voxels gives a point at the GEOMETRIC CENTRE of the frontier voxel set. For any incremental exploration the frontier voxels form a SHELL surrounding the robot's carved-FREE region → mean lands AT the robot → goal = current pose → no motion, ever. Fix in [`frontier_3d.py:193-225`](src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/frontier_3d.py#L193-L225): when `robot_xy` is supplied, pick the frontier voxel with max `‖voxel − robot‖²` instead. Always returns a real boundary point in the direction of largest unexplored extent. Without this, the system literally cannot explore regardless of every other fix below — they are all preliminary cleanup.
- **Why mean-centroid is broken in general.** Mean approximates "where the frontier is" only when the frontier is sharply pointed (e.g. a one-direction hallway). For compact / wraparound FREE regions — the default of every exploration session — it collapses to self-position. Any frontier-based explorer using mean centroid will deadlock the moment the frontier becomes annular.
- **Octomap-style trav_grid projection** ([`mapper_node.cpp`](src/collaborative_exploration/nvblox_frontend/src/mapper_node.cpp)). Dropped ~200 lines of polar fan-fill + persistent ray_covered + 1-cell dilation. Now: query nvblox's `occupancy_layer` per column directly. nvblox already does proper 3D Bayesian raycasting; behind-wall voxels stay log_odds=0 → projection gives UNK, no leak. Slope/step filter on highest-occupied-z still gates ramp-vs-cliff (Octomap proper doesn't do that — that's the value-add over plain Octomap).
- **trav_grid is now world-fixed and persistent.** 40 m × 40 m grid, origin locked on first odom, `cls_persist_` member retains FREE/OCC across frames; new-frame UNK does NOT overwrite a prior FREE/OCC. Behind-the-robot map persists, no rolling-window memory loss.
- **3D cluster z-band filter [-0.2, 1.5] m** ([`frontier_3d.py`](src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/frontier_3d.py)). Air voxels above ramp / under ceiling are permanently UNKNOWN (no Mid-360 rays go straight up + return) — they inflated the cluster IG forever. Filtering UNK to robot-actionable z makes volume actually shrink as exploration progresses.
- **Mid-360 geometric blind disk** (3 m). V-FOV starts at -7° → ground first visible at `0.4 / tan(7°) ≈ 3.25 m`. Forced-FREE 3 m disk around robot in `publish_traversability` (preserves OCC verdicts).
- **Dead-frontier filter skipped in 3D mode.** 2D-mode check (require N live UNK neighbours around goal) is incompatible with 3D-mode goals which land in FREE by construction.
- **ClusterTracker** ([`cluster_tracker.py`](src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cluster_tracker.py)). Cross-frame cluster identity via world-coord AABB overlap (voxel-index AABBs drift as robot-centric voxels_3d grid origin moves — must convert to world). Tracks per-cluster volume trajectory + attempt count from blacklist events; `non_actionable` flag after N attempts with no shrink → CFPA2 stops chasing dead-end clusters.
- **MuJoCo Mid-360 sim was undersampled.** Hardcoded default `1000 × 20` rays = 2.95° vertical resolution → walls show 1-3 voxels tall in nvblox at 1 m. Bumped via env vars `MUJOCO_LIDAR_HZ/VT_SAMPLES = 1024/96` in [`nav_test_3d_explore.sh`](scripts/launch/nav_test_3d_explore.sh) → cloud points 4880 → 23000 per scan, walls fill all 10 z-layers correctly. The yaml `mujoco_sensor_bridge.yaml::vt_samples` is for a *different* Python node that isn't used by the current C++ plugin; editing it has no effect.

## Active state (2026-05-13) — Point-LIO + gbplanner3 onboard, full stack up

- **Point-LIO replacing FAST-LIO2 as production SLAM** ([docs/claude/gbplanner3_noetic_onboard.md](docs/claude/gbplanner3_noetic_onboard.md)).
  Measured 36% rate degradation on FAST-LIO2 over a 3-min ops2 walk (9 Hz start → 4.4 Hz end while inputs steady at 10/200 Hz lidar/IMU). Root cause: ikd-tree grew unbounded → main-loop iterations stalled, lidar frames dropped in callback queue. Switched to Point-LIO (HKU MaRS 2023, [`src/vendor/point_lio_ros1/`](src/vendor/point_lio_ros1/)): iVox replaces ikd-tree (O(1) avg query), decoupled IMU+LiDAR threads, IMU keeps publishing odom when LiDAR stalls. Verified flat 10.001 Hz `/robot/Odometry` (std 1.1 ms) after `jetson_clocks` + MAXN power mode. Patches mirror our FAST-LIO ROS1 set: livox_ros_driver rename → driver2, advertise paths relative, plus a custom NodeHandle split so `/robot/Odometry` doesn't end up under `/robot/laserMapping/Odometry` (Point-LIO uses `nh("~")` for both params + topics; we add `pub_nh` for publishers).
  **→ Update (2026-05-18 late night HIL bench)**: The 36% degradation claim does *not* reproduce on the bench (Orin Nano 8GB, ROS 2 Humble `fast_lio` = `src/vendor/fast_lio`) with `pcd_save_en: false` + `extrinsic_est_en: false`. 170s of the same `onboard_noetic_20260511_*` ops2 bag → flat 10 Hz, RSS 192 MB constant, RTF=1.07. The original observation was on the **ROS 1 Noetic** `point_lio_ros1` / `FAST_LIO` stack with ros1_bridge in the data path; the bench is native ROS 2 with no bridge hop. The fast_lio degradation may have been a config (pcd_save) or bridge-induced artifact, not an algorithm limit. Also: dfloreaa's **`point_lio_ros2`** ROS 2 port (different package from `point_lio_ros1`) is not usable — see "Active state (2026-05-18 late night)" entry for the SIGSEGV race + SLAM divergence details.
- **gbplanner3 stack built + running on Jetson** (PID 31517 gbplanner_node, PID 31518 pci_general_ros_node, uptime 25 s clean). All 22 UAS vendor repos imported via laptop-side `vcs import` + rsync (Jetson has no GitHub access). 7-step pitfall journey documented in [gbplanner3_noetic_onboard.md](docs/claude/gbplanner3_noetic_onboard.md): src/src/ flattening, `COLCON_IGNORE` blocks catkin_pkg, 5 apt debs scp-installed, gflags ExternalProject pre-stage, `image_transport_plugins` + `manhole_detector_ros` CATKIN_IGNOREs, BT XML path depth (`config/ugv/real/go2/` 4-deep for `../../../bt_xml/` to resolve correctly), v2 → v3 config schema rewrite.
- **gbplanner3 ↔ Point-LIO topic interface**: voxblox baked INSIDE `gbplanner_node` (no separate voxblox_node), subscribes to `/robot/Odometry` + `/robot/cloud_registered_body` natively (no ros1_bridge for SLAM stream — that was the whole point of moving to Noetic). Only `/pci_command_path` (< 1 KB/s) crosses to ROS 2.
- **Updated [scripts/real/gbplanner3/](scripts/real/gbplanner3/)**: `bridge_topics.yaml` trimmed to PoseArray-only (no more 2to1 SLAM topics), `gbplanner_go2.launch` rewritten to match upstream v3 pattern (voxblox-in-gbplanner-node, no robot_config.yaml), `orin_launch_gbplanner.sh` no longer assumes Foxy Fast-LIO; preflights Point-LIO presence.

## Active state (2026-05-11) — Noetic FAST-LIO2 onboard (gbplanner3 prep)

- **Native ROS 1 FAST-LIO2 brought up on the Jetson** in a separate catkin ws (`~/noetic_fastlio_ws/`), parallel to the existing Foxy `~/onboard_ws/`. Eliminates ros1_bridge bandwidth bottleneck for the gbplanner3 voxblox path (heavy `/robot/cloud_registered_body` stream now stays in ROS 1 natively; only the tiny `/pci_command_path` PoseArray still crosses to ROS 2).
- New scripts: [`deploy_noetic_to_jetson.sh`](scripts/real/deploy_noetic_to_jetson.sh) (laptop rsync), [`onboard_fastlio_noetic.sh`](scripts/real/onboard_fastlio_noetic.sh) (Jetson launcher), [`onboard_record_noetic.sh`](scripts/real/onboard_record_noetic.sh) (`rosbag record`, nohup-protected), [`stream_cloud_live.sh`](scripts/real/stream_cloud_live.sh) (Open3D live viewer over ssh binary pipe — replaces X-forwarded RViz which renders black on Jammy + Ogre).
- 11 patches against HKU upstream `FAST_LIO` + `livox_ros_driver2` (livox driver rename, Mid-360s enum, absolute topic paths, launch param-override bug, ...). **Full pitfall list and tuning notes:** [docs/claude/noetic_fastlio_onboard.md](docs/claude/noetic_fastlio_onboard.md).
- Tuning ported from Foxy real-robot yaml: `point_filter_num=1`, `filter_size_*=0.10`, `extrinsic_est_en=true`, `pcd_save_en=false`. Effective rate ~8 Hz on Orin (was 10 Hz with default 0.50 voxel — extra CPU buys ICP correspondence density for sparse outdoor scenes).

## Active state (2026-05-10) — CFPA2 policy + Go2W wheel-skid fix

- **CFPA2 stable-challenger goal override** ([commit 0505ff0](src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_coordinator_node.py)).
  `_apply_goal_policy` previously held a goal as long as the robot was making progress, never re-evaluating utility under the current map → robot kept committing to the older frontier even when a much better one appeared mid-flight. New override (gated by 3-tick streak + 2s lock-age + 1.20× score-improvement vs re-evaluated `utility(last)`) lets a freshly-seen high-IG frontier preempt without re-introducing zigzag from cluster centroid jitter. Bypasses `goal_lock_sec` since the streak gate already provides anti-thrash. Set `cfpa2_challenger_streak_required=0` to disable.

- **CFPA2 multiplicative overlap penalty for joint allocator** (same commit).
  Old `joint = a + b - λ·overlap` with λ=1.0 deducted ≤ 1.0 from IG-dominated sums in the hundreds-thousands → both robots routinely chose the same frontier region. New `joint = (a + b) × (1 − λ·overlap)` makes λ a "max % deduction when fully overlapping" (default 0.5 = up to 50% off). Scale-invariant w.r.t. IG-box / `cfpa2_w_ig` changes. Also tightened `cfpa2_sigma_overlap_m: 0 → 4` (was 2×sensor_range fallback ≈ 7m, too gradual). Live test: same-room (500+500, 2m apart) joint=520 vs different-rooms (400+350, 12m apart) joint=735 → splits decisively.

- **Go2W wheel-skid bug — router subscribed to wrong joint_states topic** ([commit 5d0e01b](src/go2w/go2w_control/scripts/go2w_hybrid_cmd_router.py#L81)).
  In legged mode the router is supposed to mirror actual wheel ω back as setpoint so kv=5 actuator brake torque stays at 0 (true freewheel). Live `ros2 topic echo` showed it publishing `[0,0,0,0]` indefinitely → `kv·(0 − ω_actual)` ≈ 13 N·m brake force per wheel under CHAMP gait → wheels skidded. Two root causes:
  1. Router default `wheel_state_topic = /mujoco_sim/joint_states` (absolute) had publisher_count=0 in single-robot sim (controller_manager runs in `/robot/`). Subscribed to dead topic → `_latest_wheel_vels` stayed `[0,0,0,0]`. Default changed to relative `joint_states` (works for single); mixed/dual launches override back to `/mujoco_sim/joint_states` explicitly.
  2. `nav_test_mujoco_fastlio.launch.py` was spawning a duplicate `go2w_hybrid_cmd_router` (the included `single_go2w_mujoco_cfpa2.launch.py` already starts one). Both shared name+ns → CHAMP saw 2× messages. Removed.

  Bonus dual-Go2W fix: per-namespace `wheel_joint_names` override (b_*-prefixed for robot_b) so robot_b's router doesn't read robot_a's wheel ω from the shared `/mujoco_sim/joint_states`.

## Active state (2026-05-05) — SE2-only direction + observability + lidar_range

- **SE2 holonomic is now the canonical and only navigation profile we tune for.**
  After 2026-05-02 validated `se2_holonomic` (SmacPlannerLattice + diff primitives + no-strafe MPPI) as the operator-preferred behavior, all subsequent tuning effort (frontier reachability, costmap inflation, MPPI critics, exploration metrics, stop conditions) targets THIS profile and no other. The legacy `holonomic_profile=off` (diff-drive Reeds-Shepp) and `omni_2d` (SmacPlanner2D + MPPI Omni) remain in [`real_single.launch.py`](src/go2w/go2w_real_bringup/launch/real_single.launch.py) and [`real_autonomy.sh`](scripts/real/real_autonomy.sh) as **escape hatches only** — not maintained for daily nav and not recipients of new tuning. New operators / sessions should default to SE2.

- **Streamlined entry: [`scripts/real/real_autonomy_se2.sh`](scripts/real/real_autonomy_se2.sh).**
  Thin wrapper over `real_autonomy.sh` that bakes in `holonomic_profile=se2_holonomic` and forwards a curated subset of flags (`robot, slam, oa, execute, lidar_range, manual, record`). Daily ops should call this; profile-selection complexity stays out of operator memory. The full surface (off / omni_2d / etc.) remains accessible via `real_autonomy.sh` for ad-hoc comparison runs.

- **`lidar_range` launch param (default 8.0 m).**
  Threaded through `real_autonomy.sh → real_single.launch.py → real_bringup_core.launch.py`; controls octomap_server raytrace + `pointcloud_to_laserscan` `range_max` simultaneously. Lower (e.g. 4.0 m) for cluttered indoor scenes where far-range obstacles introduce noise; raise for open spaces. Independent of Fast-LIO's `det_range: 100.0` (SLAM odometry quality, not perception).

- **Exploration observability + explicit stop condition shipped.**
  [`exploration_metrics_logger.py`](src/go2w/go2w_observability/scripts/exploration_metrics_logger.py) is now the central event aggregator. Adds:
  1. **Structured event log** to stdout + `$ROS_LOG_SESSION_DIR/exploration_events_*.log` — one line per significant event with absolute timestamp + Δ from previous event. Sources: `/<ns>/exploration_status`, `/<ns>/goal_pose`, `/<ns>/behavior_tree_log` (plan timing via BT-internal timestamps, dedupes recovery-branch `ComputePathToPose` duplicates), `/<ns>/recovery_event`, `/cmd_vel`.
  2. **30 s rolling summary** — `plan_ms[p50/p95/max]`, `plan_ok=N/M (P%)`, `goals[done/abort_p/abort_c]`, `ttm_avg`, `coverage Δ%`, `status`.
  3. **Stop trigger** — publishes latched `/<ns>/exploration_complete` (with reason) + cancels `navigate_to_pose` action + zero-cmd_vel pulse, when either:
     - `consec_no_reachable_threshold=3` ticks of CFPA2's `no_reachable` status, or
     - coverage Δ < `coverage_stagnant_threshold_pct=0.5%` over `coverage_stagnant_window_sec=30s`.

  CFPA2 ([`cfpa2_single_robot_node.py`](src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_single_robot_node.py)) gained a `_status_pub` on `/<ns>/exploration_status` (states: `searching | executing | no_reachable | no_frontiers | paused`) and a subscriber to `/<ns>/exploration_complete` that latches a `_paused` flag. `verbose_logs: false` (default) suppresses the per-tick `_log_no_goal_debug` / `_maybe_log_summary` spam — only state-change INFO lines remain. [`stuck_watchdog.py`](scripts/runtime/stuck_watchdog.py) publishes `/<ns>/recovery_event` (`stuck_detected | backup_started | backup_done | backup_aborted | backup_unavailable`) so recovery activity threads through the same event stream.

- **CFPA2 ↔ Nav2 threshold alignment fix.**
  Discovered a 0.40–0.60 m dead band where Nav2's `xy_goal_tolerance: 0.40` declared SUCCEEDED but CFPA2's `switch_min_dist: 0.60` still considered the goal "in flight" — same goal got republished, bridge filtered it as unchanged, robot sat idle. Fix: [`cfpa2_single_robot.yaml`](src/collaborative_exploration/cfpa2_collaborative_autonomy/config/cfpa2_single_robot.yaml) `switch_min_dist: 0.60 → 0.45` and added `reached_blacklist_dist: 0.45` so CFPA2's notion of "reached" sits above Nav2's. Long-term fix (action-status listener for ground-truth SUCCEEDED/ABORTED) is documented but not yet implemented.

- **CFPA2 reachability uses an inflation-blind BFS** ([`cfpa2_coordinator_node.py:1208`](src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_coordinator_node.py#L1208)).
  `_distance_transform` walks 4-connected through cells where `_is_free()` is True (i.e. `0 ≤ v < occ_thresh` AND `v != -1`). It does NOT account for inflation, footprint, or kinematics — so it can mark a frontier reachable that Nav2 then cannot path through. For now this is documented as a known gap; the planned fix is to feed CFPA2 from `/{ns}/global_costmap/costmap` (inflated) instead of `/{ns}/map` (raw octomap projection).

## Active state (2026-05-02) — archived

- Detailed timeline moved to [`CLAUDE1.md`](CLAUDE1.md#archive-2026-05-02--go2w-real-nav2-profile-split--no-crab-se2-tuning).
- Headline: 3-profile runtime matrix (`off` / `omni_2d` / `se2_holonomic`) for Go2W real Nav2; finding that `SmacPlanner2D` is XY-only (one angle bin in Node2D); final SE2 tuning forces no-strafe execution (`MPPI motion_model=DiffDrive`, lattice diff primitives, forward+yaw bias). Sim parity added via [`nav2_se2_holonomic_overlay_sim.yaml`](src/go2w/go2w_config/config/nav/nav2_se2_holonomic_overlay_sim.yaml). 2026-05-05 superseded this with "SE2-only" — the older `off` / `omni_2d` profiles are escape hatches now.

## Active state (2026-04-30) — archived

- Detailed timeline moved to [`CLAUDE1.md`](CLAUDE1.md#archive-2026-04-30--onboard-slam-split-fast-lio--livox-on-jetson).
- Headline: onboard SLAM split succeeded (Jetson Foxy + laptop Humble via cross-host DDS); the 13 GB-per-node memory blow-up was resolved with `ulimit -v`; compatibility notes documented for future Humble migration.
- Canonical runbook remains [docs/claude/real_robot.md](docs/claude/real_robot.md#onboard-slam-split-shipped-2026-04-30--fast-lio--livox-on-jetson).

## Active state (2026-04-29) — archived

- Detailed timeline moved to [`CLAUDE1.md`](CLAUDE1.md#archive-2026-04-29--dual-robot-nav2-mppi-migration).
- Headline: dual-robot stack migrated to Nav2 MPPI/SmacHybrid; fixed wheel brake bug in vendored `mujoco_ros2_control`; added `stuck_watchdog`; unified sim/real TF+odom path through `fast_lio_tf_adapter`.
- Deep narrative and rationale remain in [docs/claude/nav2_mppi_journey.md](docs/claude/nav2_mppi_journey.md).

## Active state (2026-04-26) — archived

- Detailed timeline moved to [`CLAUDE1.md`](CLAUDE1.md#archive-2026-04-26--dual-robot-far--3-tier-safety-stack).
- Headline: A* retired for dual-robot runs in favor of FAR + safety stack (`pivot-lock`, `path_safety_filter`, `cmd_vel_safety_shield`), with documented deadlock tradeoffs and TF remap pitfalls.
- Consolidated context remains in [docs/claude/nav_benchmarks.md](docs/claude/nav_benchmarks.md) and [docs/claude/debug_notes.md](docs/claude/debug_notes.md).

## Skill-API detail docs

| Topic | Doc |
|---|---|
| Nav stack benchmarking (Phase 5), config A, iteration logs | [docs/claude/nav_benchmarks.md](docs/claude/nav_benchmarks.md) |
| Fast-LIO2 / Cartographer A/B, LiDAR options, demo scenes | [docs/claude/slam_and_scenes.md](docs/claude/slam_and_scenes.md) |
| Cross-cutting debugging gotchas (QoS, zombies, MuJoCo quirks) | [docs/claude/debug_notes.md](docs/claude/debug_notes.md) |
| **Gazebo vs MuJoCo — why stack works in Gazebo, MuJoCo matches real life** | [docs/claude/sim_comparison.md](docs/claude/sim_comparison.md) |
| **Real Go2W / Go2 — connect modes, SLAM A/B, Mid-360 calib, 10-layer bug chain** | [docs/claude/real_robot.md](docs/claude/real_robot.md) |
| **Go2 (non-W) integration — Menagerie MJCF, CHAMP shipped, real CMU TARE → localPlanner (FAR bypassed), RL scaffold in place** | [docs/claude/go2_integration.md](docs/claude/go2_integration.md) |
| **Dual-robot FAR safety stack (2026-04-26): pivot-lock + path_safety_filter + cmd_vel_safety_shield. Why A\* was retired for dual.** | [CLAUDE1.md archive](CLAUDE1.md#archive-2026-04-26--dual-robot-far--3-tier-safety-stack) |
| **Nav2 MPPI migration journey (2026-04-29): A\* → FAR → MPPI, brake bug, freewheel, stuck_watchdog, TF/SLAM chain. Two real-robot blockers.** | [docs/claude/nav2_mppi_journey.md](docs/claude/nav2_mppi_journey.md) |
| **Onboard SLAM split (2026-04-30): Fast-LIO + Livox onto Go2 Jetson Foxy. 5 source patches, the 13-GB-per-node bloat, the `ulimit -v` fix.** | [CLAUDE1.md archive](CLAUDE1.md#archive-2026-04-30--onboard-slam-split-fast-lio--livox-on-jetson) + [docs/claude/real_robot.md](docs/claude/real_robot.md#onboard-slam-split-shipped-2026-04-30--fast-lio--livox-on-jetson) |
| **Noetic FAST-LIO2 onboard (2026-05-11): ROS 1 native build of HKU FAST-LIO + Livox driver on Jetson for gbplanner3. 11 patches, conda-poison guard, X11-free Open3D streaming.** | [docs/claude/noetic_fastlio_onboard.md](docs/claude/noetic_fastlio_onboard.md) |
| **Point-LIO + gbplanner3 onboard (2026-05-13): switched SLAM from FAST-LIO2 → Point-LIO (iVox vs ikd-tree fixes 36% rate decay), full gbplanner3 stack built on Jetson, 8 build pitfalls (src/src/, COLCON_IGNORE, gflags ExternalProject, BT XML path depth, v2→v3 config schema, ...).** | [docs/claude/gbplanner3_noetic_onboard.md](docs/claude/gbplanner3_noetic_onboard.md) |
| **nvblox 3D frontier exploration on demo_ramp (2026-05-13): 8 bug fixes wiring nvblox_frontend mapper → Nav2 StaticLayer → CFPA2 3D IG. Includes ground-clutter filter, ramp-vs-stairs discretization fix, RewrittenYaml topic-rewrite gotcha. Open analysis: current "3D" only changes IG, not frontier search; proposes voxel→ground projection.** | [docs/claude/nvblox_3d_frontier.md](docs/claude/nvblox_3d_frontier.md) |
| **GBPlanner3 / OmniPlanner sim-side integration (2026-05-13): ros1_bridge gotchas (latched /tf_static drop), voxblox + elevation_mapping config alignment with NTNU UAS UGV ref, host-pipe spam filter, the unsolved voxblox-doesn't-populate wall, sim-only vs. Jetson-portable artifact split.** | [docs/claude/gbplanner3_integration.md](docs/claude/gbplanner3_integration.md) |
| **3D frontier exploration unstuck (2026-05-14): 6 compounding bugs in `nav_test_3d_explore.sh`. The load-bearing one: mean-of-frontier-voxels centroid for a SHELL-shaped frontier lands at the ROBOT, so the goal = current pose → no motion ever. Plus octomap-style projection (drop fan-fill, query nvblox 3D state direct), world-fixed 40m trav_grid with persistent cls, z-band UNK filter, geometric blind disk, ClusterTracker non-actionable, MuJoCo lidar sim density.** | [docs/claude/3d_frontier_debugging.md](docs/claude/3d_frontier_debugging.md) |
| **ETH trav pipeline (2026-05-14): elevation_mapping_cupy + grid_map_filters + OccupancyGrid adapter replacing the broken 6-step 2D projection. 7-phase rewrite plan, 4 YAML/param gotchas discovered during integration, smoke-tested at 12 layers + 9 Hz.** | [docs/claude/plans/2026-05-14-trav-grid-rewrite.md](docs/claude/plans/2026-05-14-trav-grid-rewrite.md) + [docs/claude/eth_elevation_mapping_design.md](docs/claude/eth_elevation_mapping_design.md) |
| **Open problem (2026-05-15): Go2W tips at ramp foot / platform cliff. All static-cost mitigations (trav thresholds, height-cost layer, inflation halo, ramp_safe envelope) applied; tipping is dynamic-stability failure unobservable from 2.5D. 5 candidate next steps listed.** | [docs/claude/ramp_tipover_open_problem.md](docs/claude/ramp_tipover_open_problem.md) |
| **Jetson Orin Nano HIL bag-replay (2026-05-18 late night): full autonomy stack stress-tested on weaker bench board (6×A78AE @ 1.5 GHz, 8 GB) as proxy for real Go2's Orin NX 16GB. fast_lio sustains 10 Hz / RSS 192 MB / CPU 9-12% on 170s real-walk bag, RTF=1.07 (real-time confirmed). Point-LIO ROS 2 port abandoned: SIGSEGV race at IMU Init 100% (~50% rate) + SLAM divergence on Mid-360 walking data. 8 fixes shipped: DDS isolation via ROS_DOMAIN_ID=42, livox_ros_driver2_msgs stub package, bag-play workspace-source preflight, `--clock`+`use_sim_time=true`, multi-pass kill, Point-LIO `/aft_mapped_to_init` remap, staggered TimerActions + respawn, rclpy `warn_throttle` migration. Real bottlenecks identified: **CFPA2 Python tick p95 1.4s** + **grid_map_to_occupancy 0.59 Hz** — single-core Python on ARM. Refuted hypotheses: concurrent record overhead (38 MB/s disk write didn't dent fast_lio), fast_lio degradation (no ikd-tree growth observed in 170s with `pcd_save_en=false`). Likely culprit for onboard 5.7 Hz baseline: ros1_bridge serialization overhead.** | This file's "Active state (2026-05-18 late night)" entry |
| **Go2W real Nav2 profile split (2026-05-02): `off` / `omni_2d` / `se2_holonomic`, SmacPlanner2D XY-only finding, no-crab final SE2 tuning.** | This file's "Active state (2026-05-02)" entry |

## Scripts layout

```
scripts/
├── launch/    user-invoked entry points (nav_test_*, vlm_demo)
├── bench/     multi-trial PASS-criterion runners + session_reporter
├── runtime/   ROS 2 nodes started by launch files (policy, checkers, supervisors)
├── debug/     observe-a-running-sim tools (far_monitor, vlm_debug_web, …)
├── ops/       one-shot ops & dev utilities (reset_vgraph, test_lidar, sync_to_main)
├── real/      real-robot only (unchanged grouping)
└── common_logging.sh
```

## Quick launch

```bash
# Single-robot Go2W nav smoke test (Nav2 MPPI + SE2 by default)
./scripts/launch/nav_test_fastlio.sh robot:=go2w gui:=true rviz:=true
./scripts/launch/nav_test_fastlio.sh robot:=go2w gui:=false           # headless

# Nav benchmark (FAR baseline) — 5-trial / 10-trial
./scripts/bench/benchmark_far_nav.sh
NUM_TRIALS=10 DURATION_SEC=120 OUT_DIR=/tmp/cfgA_10 ./scripts/bench/benchmark_far_nav.sh
./scripts/bench/benchmark_fastlio.sh                                  # Fast-LIO + MID-360

# Demo2 / LRC maze
./scripts/launch/nav_test_demo2.sh gui:=false
./scripts/launch/nav_test_lrc_maze.sh

# VLM exploration demo (Phase 1) — uses nav2_mppi
./scripts/launch/vlm_demo_mujoco.sh

# Heterogeneous dual (Go2W + Go2 + demo3_mixed + CFPA2 coord, nav2_mppi)
./scripts/launch/nav_test_demo3_mixed.sh gui:=true rviz:=true
./scripts/launch/nav_test_demo3_mixed.sh nav_backend_a:=far nav_backend_b:=far  # both FAR

# Go2 (no-wheel) sim — CHAMP locomotion, demo1 12×8 m / demo3 24×16 m
./scripts/launch/nav_test_go2.sh gui:=true rviz:=true                 # walk + FAR smoke
./scripts/launch/nav_test_go2_demo3.sh gui:=true rviz:=true           # larger scene
./scripts/bench/benchmark_go2.sh                                      # 5-trial PASS check
./scripts/bench/benchmark_go2_demo3.sh                                # same on demo3
./scripts/launch/nav_test_go2_tare_real.sh gui:=true rviz:=true       # CMU TARE → localPlanner
./scripts/bench/benchmark_go2_tare.sh                                 # 10-trial TARE benchmark
./scripts/launch/nav_test_go2.sh rl_policy:=true                      # RL (experimental, see go2_integration.md)

# Real robot (Go2W) — RECOMMENDED entry as of 2026-05-05:
#   SE2 holonomic baked in, oa=false (sport API direct), curated flag surface.
./scripts/real/real_autonomy_se2.sh                                   # default Go2W SE2
./scripts/real/real_autonomy_se2.sh slam=fastlio_mid360                # Mid-360 + Fast-LIO
./scripts/real/real_autonomy_se2.sh slam=fastlio_mid360 lidar_range=4.0
./scripts/real/real_autonomy_se2.sh stop                              # kill everything

# Legacy multi-profile real launcher (escape hatches for ad-hoc comparison):
./scripts/real/real_autonomy.sh                                       # Cartographer + L1 default
./scripts/real/real_autonomy.sh slam=fastlio_mid360
./scripts/real/real_autonomy.sh oa=false holonomic_profile=off        # diff-drive Reeds-Shepp
./scripts/real/real_autonomy.sh oa=false holonomic_profile=omni_2d    # SmacPlanner2D + MPPI Omni
./scripts/real/real_autonomy.sh slam=fastlio_mid360 nav=far
./scripts/real/real_autonomy_go2.sh                                   # Go2 (no wheel), nav2_mppi
./scripts/real/real_autonomy_go2.sh slam=fastlio_mid360 nav=far
# Real CMU TARE → localPlanner direct (FAR unwired, watchdog armed).
# oa=false is REQUIRED — default (oa=true) routes Move to /api/obstacles_avoid/request
# which needs manual mode pre-arm; oa=false sends to /api/sport/request (api_id=1008).
./scripts/real/real_autonomy.sh robot=go2 slam=fastlio_mid360 nav=tare_real oa=false
./scripts/real/real_autonomy.sh stop                                  # kill everything real-robot
```

Debug dashboard:
- VLM exploration: <http://localhost:8501> (auto-starts)

## Build

```bash
micromamba activate cmu_env
source /opt/ros/humble/setup.bash

# Full build
touch src/mtare_ros1_ws/COLCON_IGNORE
colcon build --symlink-install --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3

# Incremental
colcon build --symlink-install --packages-select <pkg>
```

YAML + Python changes are instant via symlink-install; C++ requires rebuild.

## Repo layout

```
src/
  go2w/                             Go2W platform packages
    go2_gazebo_sim/                   MJCF/world + launch files
    mujoco_sensor_bridge/             MuJoCo sensor nodes
    go2w_control/                     Hybrid cmd_vel router (legged + wheel mux)
    go2w_nav/                         Safety utilities (collision_monitor, cmd_vel_safety_shield, …)
    go2w_perception/ go2w_config/     QoS bridge, robot_self_filter, configs, sub-launches
    unitree_go2w_ros2/                Unitree ROS 2 integration
  collaborative_exploration/
    cfpa2_collaborative_autonomy/     CFPA2 frontier allocator (single + dual joint allocator)
    dynamic_scene_filter/             Dynamic obstacle filter (peer body + temporal voxel)
    slam_backend_adapters/            Fast-LIO ↔ Nav2 adapters
  vendor/
    fast_lio/                         Fast-LIO2 SLAM
    autonomy_stack_go2/               CMU stack (FAR, terrain analysis)
    mujoco_ros2_control/              DFKI MuJoCo HW interface
    far_planner/                      FAR global planner
  vlm_explorer/                     VLM-in-the-loop exploration (Phase 1)
scripts/                            Launch scripts, benchmark runners, utilities
config/                             DDS config (fastdds_no_shm.xml)
docs/claude/                        Detailed skill-API docs (this index)
```

## Nav backends

Switchable via `nav_backend:=` at launch time. Two production backends now (legacy `astar` / `default` / `reactive` / `mppi_nav_node` were retired in commit 5c46a51).

| Backend | Planner | Note |
|---|---|---|
| `nav2_mppi` | `nav2_planner` (SmacPlannerHybrid REEDS_SHEPP / SmacPlannerLattice for SE2) + `nav2_controller` (MPPIController) + `nav2_behaviors` + `nav2_bt_navigator` + `nav2_lifecycle_manager` | **Default for sim and real.** Per-platform yaml: [`nav2_go2w_full_stack.yaml`](src/go2w/go2w_config/config/nav/nav2_go2w_full_stack.yaml) for Go2W, [`nav2_go2_full_stack.yaml`](src/go2w/go2w_config/config/nav/nav2_go2_full_stack.yaml) for Go2. Real Go2W supports overlay profile selection via `holonomic_profile`: `off` (diff-drive Reeds-Shepp, escape hatch), `omni_2d` (`SmacPlanner2D` + MPPI Omni, escape hatch), `se2_holonomic` (`SmacPlannerLattice` + forward/pivot MPPI, no strafe — **canonical**). Outer-loop `stuck_watchdog` per robot. CFPA2 `way_point` is bridged to `goal_pose` via `cfpa2_to_nav2_bridge`. |
| `far` | CMU autonomy stack | Terrain analysis + FAR V-graph + path follower (see nav_benchmarks.md). Used for benchmarking comparison and Go2 TARE exploration. |

## Golden rules (must-follow across all work)

1. **Always `use_sim_time: true`** for all nodes in MuJoCo or Gazebo. Mixed time domains corrupt maps.
2. **Never use stale TF fallback.** Drop the scan on TF failure; don't use `tf2::TimePointZero`.
3. **Each scan painted exactly once.** Clear `last_scan_` after processing in mappers.
4. **Dual-robot TF must be namespaced.** Remap `/tf` → `/{ns}/tf` for all nodes.
5. **DDS config matters.** `config/fastdds_no_shm.xml` disables shared memory for reliability. Real robot uses CycloneDDS.
6. **Verify with `ros2 topic hz`** after changing sensor rates. Xacro changes require model re-spawn.
7. **Kill zombie MuJoCo before re-launch** (see [debug_notes.md](docs/claude/debug_notes.md)).
8. **Benchmark PASS criterion** = `completed ∧ coverage≥90% ∧ contacts==0 ∧ ¬tipped`. 5 trials is too small for reliability claims; use ≥10.
9. **Supervisor panic = any-button override (real robot).** Any press on the Unitree BT pad latches a 5 s window: auto `cmd_vel` blocked, FAR disarmed, sticks drive directly. See [docs/claude/real_robot.md](docs/claude/real_robot.md#supervisor-panic-override-any-button-emergency).
10. **Any node doing TF lookup in dual-robot setup MUST have `tf_remaps`.** Without `("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")`, the node's TF buffer subscribes the global `/tf` (empty in our namespaced setup) and every lookup silently fails — no error log, just stale data or `passthrough_no_tf` status. Discovered the hard way 2026-04-26: path_safety_filter + cmd_vel_safety_shield were inert for hours because of this. **Verify lookups work via `ros2 run tf2_ros tf2_echo map base_link --ros-args -r /tf:=/{ns}/tf -r /tf_static:=/{ns}/tf_static`**.
11. **CMU's `vehicle` frame is NOT in our SLAM tree by default.** CMU only publishes `sensor → vehicle` (static); `sensor` itself isn't connected to `map` (we have `map → sensor_at_scan`, not `sensor`). So `lookup_transform(map, vehicle)` fails. The mixed launch bridges this with a `base_link → vehicle` static publisher per namespace; nodes reading paths in vehicle frame should also accept a `base_frame_fallback` parameter.
12. **Multi-layer safety stacks deadlock easily.** CFPA2 pivot-lock (refuses goal change) + cmd_vel shield (kills ω) + path_safety_filter (rejects path) can all latch a robot in place if held goal demands rotation it can't execute. Always provide a max-hold/timeout escape valve (e.g. `pivot_lock_max_hold_sec`) on each stateful safety layer. Verify each layer's status topic, NOT just the absence of motion — a robot frozen by 3 layers stacked looks identical to "stuck planner" in /nav_status alone.
13. **`peer_pose_stale_sec` must be generous in sim.** sim_time and wall-clock-stamped messages can drift several seconds during startup; a strict 0.3s threshold rejects fresh peer poses → self_filter publishes scans untouched → peer body becomes a permanent imprint in /map. Default 5.0s in dual-robot launches; tighten on real robot only after verifying timestamp alignment.
14. **MPPI's effective rejection radius = `robot_radius` + `collision_margin_distance`.** With `consider_footprint: false` (the default-because-old-comment-said-it-throws), MPPI treats the robot as a circle and adds margin on top. demo3_mixed has 0.425 m corridors; with `robot_radius=0.40 + margin=0.20 = 0.60 m` MPPI permanently rejected forward motion. **Always set `consider_footprint: true` + a polygon `footprint:` on both costmaps**, then drop `collision_margin_distance` to 0.03 m. The "throws at configure" comment was a yaml-parse bug, not a humble-1.1.20 platform bug.
15. **Footprint changes ripple through CFPA2 frontier filters** ([cfpa2_coordinator.yaml](src/collaborative_exploration/cfpa2_collaborative_autonomy/config/cfpa2_coordinator.yaml)). When you tighten footprint, you typically need to *loosen* `cfpa2_frontier_obstacle_clearance_m`, `cfpa2_frontier_unknown_check_radius_m`, and `cfpa2_frontier_min_cluster_area_m2` proportionally, otherwise late-stage exploration falsely declares "complete" with significant unknown remaining. CFPA2 has **no parameter callback** (`add_on_set_parameters_callback` is not registered), so `ros2 param set` only updates the param store; cached `self.cfpa2_*` values stay frozen. Edit yaml + restart the node.
16. **Sim/real share the same TF + odom data path via `fast_lio_tf_adapter`.** This was *not* the case before 2026-04-29: sim relied on `mujoco_odom_bridge` writing GT directly to TF (bypassing the EKF chain), and real had nothing to take over because CHAMP's `state_estimation_node` outputs `7.8e+34` NaN that starves the EKF. Now [`scripts/runtime/fast_lio_tf_adapter.py`](scripts/runtime/fast_lio_tf_adapter.py) is the single owner of `odom → base_link` TF and `/<ns>/odom/nav` topic, sourced from Fast-LIO's `/<ns>/Odometry`. **`mujoco_odom_bridge.publish_tf` must stay `false`** (or it competes with the adapter on the same TF link). Real-bot drops in unchanged; only `bootstrap_from_gt:=false` differs (no GT to align to).
17. **Don't blame the architecture before grepping the upstream.** Half a day was lost mis-locating the wheel brake bug at the router / MPPI / footprint level when the root cause was a 2-line bug in vendored `mujoco_ros2_control` (the VELOCITY actuator branch never updated `last_command`, so once the controller commanded 0 once, ctrl was never refreshed and stayed at the previous setpoint). When data clearly shows a chain breaking and "everyone says they're publishing the right thing", stop tuning higher layers and read the lowest layer's source.
18. **Outer-loop stuck recovery is needed because MPPI/DWB/FAR rarely *report* failure.** They emit (v ≈ 0, ω ≈ 0) and self-report happy. Nav2's BT recovery only fires on a controller-reported failure → never triggers in this scenario. [`scripts/runtime/stuck_watchdog.py`](scripts/runtime/stuck_watchdog.py) is the watchdog: 10 s no-motion + active goal → Nav2 BackUp action → republish goal. Per-namespace, real-robot-compatible. Caveat: BackUp itself collision-checks `simulate_ahead_time × backup_speed` of clearance (≈ 0.20 m), so a robot wedged between two walls still fails recovery — last-resort raw-cmd_vel pulse not yet wired.
19. **`SmacPlanner2D` is XY-only; do not expect it to solve heading-dependent footprint-fit maneuvers.** If the requirement is "choose body orientation to pass anisotropic narrow geometry", use an SE2 planner (`SmacPlannerHybrid` / `SmacPlannerLattice`) and tune controller execution policy separately.
20. **Missing file errors under `install/.../share/<pkg>/...` usually mean stale install artifacts, not bad launch paths.** New YAML under `src/` is invisible to `get_package_share_directory()` until that package is rebuilt (`colcon build --symlink-install --packages-select <pkg>`). This exact failure happened with `nav2_go2w_real_omni_overlay.yaml` on 2026-05-01.
21. **Don't hardcode absolute topic paths to controller_manager-namespaced state sources.** JointStateBroadcaster (and most `controller_manager`-loaded broadcasters) publish to `<cm_ns>/joint_states`, where `cm_ns` varies per launch: single-robot sim runs `cm_ns=/robot/`, mixed/dual sims run `cm_ns=/mujoco_sim/`, real robots run yet another. A node hardcoding `wheel_state_topic=/mujoco_sim/joint_states` worked silently in mixed/dual but had `publisher_count=0` in single → router published `[0,0,0,0]` wheel commands → kv=5 actuator brake-locked the wheels under CHAMP gait → wheel-skid bug, hidden for months (2026-05-10 fix). Use a relative default that picks up the per-namespace topic, and override in launches whose `cm_ns` is elsewhere. **Verify with `ros2 topic info <topic> -v` that publisher_count > 0 BEFORE assuming the node is wired.**

## Communication style

- Fast and direct. Short questions expect immediate, precise answers.
- Show work but don't narrate it. Run commands, make changes, report what happened.
- When told "still not working", don't repeat the same fix — go deeper.
- Respect the maintainer's hypotheses. When they say "I AM certain it's due to X", treat as strong signal; validate or disprove with evidence, not conjecture.

## Environment

- Python 3.10 (micromamba `cmu_env`)
- ROS 2 Humble (`/opt/ros/humble/`)
- Build: colcon + ament_cmake / ament_python
- DDS: FastDDS (sim), CycloneDDS (real robot)
- Sim: MuJoCo 3.6.0 (pip) + DFKI mujoco_ros2_control
- VLM API: xAI Grok-4-1-fast-non-reasoning, key in `.env.xai`
