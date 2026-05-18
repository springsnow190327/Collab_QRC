# Orin Nano HIL Runbook

Hardware-in-the-loop test: desktop plays the "real world" (MuJoCo + sensor pubs),
Orin Nano runs the autonomy stack (SLAM → trav → CFPA2 → Nav2 → cmd_vel), cmd_vel
loops back to MuJoCo. If this works on a Nano 8GB, the real Go2's Orin NX 16GB is
guaranteed to handle it.

## Topology

```
┌────────────────────────────────────────────┐         ┌──────────────────────────┐
│  Desktop (5090, Ubuntu 22.04, Humble)      │         │  Orin Nano Super (8GB)   │
│                                            │         │  JetPack 6.2.2 / Humble  │
│  MuJoCo scene (ops2)                       │         │                          │
│   ├─ mujoco_sensor_bridge                  │  enp10s0 ──── direct cable ─── enP8p1s0
│   │   ├─ /livox/lidar  (PointCloud2 10 Hz) │         │  192.168.55.49           │
│   │   ├─ /livox/imu    (Imu 200 Hz)        │         │                          │
│   │   └─ /robot/joint_states               │ 192.168.55.1                       │
│   │                                        │         │                          │
│   └─ subscribes /cmd_vel → CHAMP/wheel     │         │  Point-LIO               │
│                                            │ ◄────── │  → /robot/Odometry       │
│                                            │         │  → /robot/cloud_registered_body
│  RViz (optional)                           │         │                          │
│                                            │         │  elevation_mapping_cupy  │
│                                            │         │  + filter_chain          │
│                                            │         │  + grid_map_to_occupancy │
│                                            │         │                          │
│                                            │         │  Nav2 MPPI + CFPA2       │
│                                            │ /cmd_vel│  → /cmd_vel              │
│                                            │ ◄────── │                          │
└────────────────────────────────────────────┘         └──────────────────────────┘
        (DDS: CycloneDDS, peer list on both ends, no multicast)
```

**Why peer-list CycloneDDS**: this is a point-to-point link with no multicast
forwarder. Same pattern cheatsheet §8 Option 1 prescribes for Tailscale.

## Identity

| Side | What | Address |
|---|---|---|
| Desktop | enp10s0 (motherboard NIC) | 192.168.55.1/24 |
| Desktop | hostname | `hanszhu` |
| Orin Nano | enP8p1s0 (built-in NIC) | 192.168.55.49/24 (DHCP) |
| Orin Nano | user / pwd | `johnpork233` / `233` |
| Orin Nano | hostname | `ubuntu` (default — change later if you want) |
| Orin Nano | workspace | `/home/johnpork233/jetson_ws` |

## Phase 0 — Jetson first-time bring-up (one-shot, ≈ 15 min)

```bash
# From desktop
JETSON_PASS=233 ssh johnpork233@192.168.55.49 'bash -s' < scripts/real/orin_nano_setup.sh
```

What it does:
1. `nvpmodel -m 0 && jetson_clocks` → MAXN Super, 25W (default 7W halves compute)
2. Swap check (Orin Nano 8GB → 3.8 GB swap already configured by JetPack 6.2)
3. `~/.bashrc` gets CUDA paths
4. apt: ros-humble-grid-map-*, nav2-mppi, nav2-smac, tf-transformations
5. pip: `torch torchvision` from `pypi.jetson-ai-lab.dev/jp6/cu126`
6. pip: `cupy-cuda12x`
7. pip: `jetson-stats` (`jtop` dashboard)
8. Smoke: `torch.cuda.is_available()` and `cupy.cuda.runtime` round-trip a 512×512 matmul

**PASS criterion**: smoke test prints `cupy mm: OK` (torch is optional — see below).

**Verified 2026-05-18 on Orin Nano Super 8GB**:
- CuPy 14.0.1 + numpy 2.2.6 + scipy 1.15.3 all play together (system scipy at
  /usr/lib was numpy-1-built → ABI break; pip --user shadow fixes it).
- `get_filter_cupy` 3-branch dilated CNN on 120×120 elevation: **2.60 ms / 384 Hz**.
  Output: (1, 1, 114, 114). Throughput is kernel-launch-bound, not compute-bound.

**PyTorch is OPTIONAL**: the trav-filter selector at
[elevation_mapping.py:158-167](../../src/vendor/elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping.py#L158-L167)
checks an `ELEVATION_MAPPING_FORCE_CUPY=1` env var that bypasses the torch
path entirely. Add this to every elevation_mapping_cupy launch on the Jetson:
```bash
export ELEVATION_MAPPING_FORCE_CUPY=1
```

**If PyTorch install fails**: `pypi.jetson-ai-lab.dev` DNS sometimes doesn't
resolve (observed on this network 2026-05-18 — apt + pypi.org work, only
jetson-ai-lab.dev fails). Two paths:
- Skip torch: `SKIP_TORCH=1` env to the setup script. The cupy backend is
  feature-complete for our pipeline.
- Or install from NVIDIA's official wheels:
  https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/

## Phase 1 — DDS setup (5 min, both sides)

### Desktop side

```bash
# Static IP on enp10s0 (idempotent)
sudo ip addr add 192.168.55.1/24 dev enp10s0 2>/dev/null || true
sudo ip link set enp10s0 up

# Run dnsmasq in the background OR convert Jetson to static (see below)
sudo dnsmasq --interface=enp10s0 --bind-interfaces \
  --dhcp-range=192.168.55.10,192.168.55.100,1h \
  --pid-file=/tmp/dnsmasq-orin.pid --log-dhcp \
  --conf-file=/dev/null &

# CycloneDDS config — see scripts/real/cyclonedds_orin_nano.xml (desktop variant)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/scripts/real/cyclonedds_orin_nano_desktop.xml
```

### Jetson side

```bash
# (after first SSH-in)
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file:///home/johnpork233/jetson_ws/config/cyclonedds_orin_nano.xml' >> ~/.bashrc
```

### Verify

```bash
# Desktop terminal A
ros2 run demo_nodes_cpp talker

# Jetson terminal (over SSH)
ros2 run demo_nodes_cpp listener
# Should print "I heard: ..." within 2-3 seconds
```

**PASS criterion**: listener prints messages within 5 sec of talker starting.

**Common failures**:
- `tailscale0` interface picked over `enp10s0` in CycloneDDS — fix the
  `<NetworkInterface name="enp10s0">` line.
- ROS_DOMAIN_ID mismatch (default 0 on both).
- Firewall: `sudo ufw status` should be inactive or allow port 7400+.

## Phase 2 — Sync code + build (15-30 min)

```bash
# From desktop
JETSON_PASS=233 ./scripts/real/deploy_to_orin_nano.sh

# Then build remotely
JETSON_PASS=233 ./scripts/real/deploy_to_orin_nano.sh build
```

Packages built (`--packages-up-to`):
- `fast_lio` + `livox_ros_driver2` + `Livox-SDK2` (SLAM)
- `elevation_mapping_cupy` + `elevation_map_msgs` + `sensor_processing`
- `cfpa2_collaborative_autonomy`
- `trav_cost_filters`
- `slam_backend_adapters`

**PASS criterion**: `colcon build` finishes with 0 errors. Warnings about
non-fatal dep mismatches are OK.

**Expected build time on Orin Nano**: 15-25 min wall. `elevation_mapping_cupy`
and `fast_lio` are the longest (C++ Eigen heavy).

## Phase 3 — SLAM-only HIL (5 min run, gate decision)

**Goal**: prove the Jetson can do Point-LIO at sustained 10 Hz fed by desktop
sensors, with no rate decay over 5 min.

### Desktop

```bash
# Launch MuJoCo + sensor publishers ONLY (no autonomy)
./scripts/launch/nav_test_slam_ops2_v4_go2.sh \
  slam_backend=external \
  autonomy=false \
  rviz=true
```

The `slam_backend=external` flag (TBD — may need to add) tells the launcher to
skip its own Fast-LIO and just publish raw `/livox/lidar` + `/livox/imu`.
**Alternative if that flag doesn't exist**: launch only the MuJoCo + sensor
bridge subset directly.

### Jetson

```bash
ssh johnpork233@192.168.55.49
cd ~/jetson_ws
source install/setup.bash
ros2 launch fast_lio mapping_mid360.launch.py
```

### Verify (desktop)

```bash
# Rate should be 10 Hz, std < 5 ms
ros2 topic hz /robot/Odometry --window 300

# CPU/GPU on Jetson — open jtop in another SSH
ssh johnpork233@192.168.55.49 jtop
```

**PASS criteria**:
- `/robot/Odometry` rate ≥ 9.5 Hz, std < 10 ms, after 5 min of running.
- Jetson RAM used < 3 GB.
- Jetson GPU util < 30% (Point-LIO is mostly CPU).

**FAIL gate — STOP and investigate**:
- Rate decays from 10 → 7 → 4 Hz over 5 min ⇒ ikd-tree / iVox growth issue
  (same bug we hit on real Go2 Jetson; might need Point-LIO instead of FAST-LIO
  if iVox isn't the active backend).
- RAM climbs past 5 GB ⇒ memory leak or pointcloud accumulation.
- Network packet loss ⇒ DDS misconfig; check `ros2 topic hz` from the other
  side to verify cross-host transit.

## Phase 4 — Add trav pipeline (5-10 min)

Build on Phase 3. Add to the Jetson:
- `elevation_mapping_cupy` (raycast 2.5D heightmap)
- `filter_chain_runner` (analytical slope/step + CNN fusion)
- `grid_map_to_occupancy_grid` (publish `/robot/traversability_grid`)

```bash
# Jetson — second SSH
cd ~/jetson_ws && source install/setup.bash
ros2 launch elevation_mapping_cupy elevation_mapping_cupy.launch.py \
  config_file:=$HOME/jetson_ws/config/nav/elevation_mapping_cupy.yaml
```

**PASS criteria**:
- `/robot/elevation_map_filtered` published at ≥ 4 Hz with 12 layers
- `/robot/traversability_grid` (OccupancyGrid) published at ≥ 4 Hz
- Jetson GPU mem < 2 GB (jtop), util ~30-50%
- Total RAM used < 5 GB

**FAIL gate**:
- CNN CUDA error → likely PyTorch wheel mismatch; verify `torch.cuda.is_available()`
- map resolution causes < 2 Hz → drop resolution from 0.05 → 0.10 m in
  `elevation_mapping_cupy.yaml`

## Phase 5 — Full closed loop (10-30 min run)

Add to the Jetson:
- `cfpa2_single_robot_node`
- `cfpa2_to_nav2_bridge`
- Nav2 stack: `controller_server` (MPPI), `planner_server` (SmacLattice),
  `behavior_server`, `bt_navigator`, `lifecycle_manager`

```bash
# Jetson
ros2 launch cfpa2_collaborative_autonomy single_go2w_mujoco_cfpa2.launch.py \
  use_sim_time:=true
```

Desktop needs to subscribe `/cmd_vel` and feed it into MuJoCo's CHAMP / wheel
router. This wiring exists in the existing launchers — confirm by checking
`mujoco_sim` subscriber count on `/cmd_vel`:

```bash
ros2 topic info /cmd_vel -v
# Should show subscription_count >= 1 from mujoco side
```

**PASS criteria**:
- Robot moves from spawn (0, 0) toward an unexplored frontier within 60 s
- No OOM on Jetson (RAM ≤ 7 GB, swap < 500 MB used)
- cmd_vel rate ≥ 15 Hz (target 20 Hz)
- Nav2 control loop running, no `controller_server` ABORTED loops

**FAIL gate**:
- OOM → drop Nav2 footprint check, or move CFPA2 back to desktop
- cmd_vel < 10 Hz → MPPI param `motion_model` may need tuning down
- /cmd_vel never publishes → goal_pose never received; check CFPA2's
  `/exploration_status` topic

## Memory budget reality check

| Component | Expected RAM | GPU RAM |
|---|---|---|
| OS + ROS daemon + DDS overhead | 800 MB | — |
| Point-LIO (iVox) | 1.5 GB | — |
| elevation_mapping_cupy | 500 MB | 500 MB |
| filter_chain_runner + adapter | 200 MB | — |
| Nav2 (MPPI + costmaps) | 700 MB | — |
| CFPA2 | 300 MB | — |
| **Total** | **≈ 4 GB** | **≈ 500 MB** |
| Headroom on 8 GB | 4 GB | 7.5 GB |

If actuals come in significantly above this, suspect:
- Point-LIO iVox growth (cap region of interest)
- Costmap resolution too fine (drop from 0.05 → 0.10)
- Pointcloud not downsampled in fast_lio config

## Rollback / abort

```bash
# Kill everything on Jetson
ssh johnpork233@192.168.55.49 'pkill -f ros2; pkill -f component_container'

# Kill on desktop
pkill -f ros2; pkill -f mujoco

# Stop DHCP server on desktop
sudo kill $(cat /tmp/dnsmasq-orin.pid) 2>/dev/null

# Network teardown (only if you want to reset)
sudo ip addr flush dev enp10s0
```

## Open questions / known unknowns

1. **Point-LIO vs FAST-LIO2 for HIL**: the Go2's real Orin runs Point-LIO over
   Noetic for the gbplanner3 path. For HIL on this Orin Nano with Humble, we
   default to FAST-LIO2 (already in `src/vendor/fast_lio`, ROS 2). If we see
   rate decay like the real robot did, switch to a Humble-port of Point-LIO.
2. **CNN weight transfer**: the trained `weights_ops2_tiled.dat` lives at the
   desktop. We need to either ship it via deploy script (add to rsync list) or
   let elevation_mapping_cupy fall back to `weights_pretrain.dat`. Verify by
   grepping the launched yaml for `weight_file`.
3. **Sim/real footprint param**: the Jetson copy of `nav2_go2_full_stack.yaml`
   has the 0.64×0.36 m footprint baked in (from the 2026-05-18 corridor fix).
   No tweak needed.
