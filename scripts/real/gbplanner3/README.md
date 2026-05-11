# GBPlanner3 native deployment on Jetson Orin (JP 5.x)

3D-native exploration planner running natively on the robot's Orin, feeding 3D
point-goals to laptop-side SanD-Planner over CycloneDDS WiFi.

## Architecture

```
                 Jetson Orin (JP 5.x, Ubuntu 20.04 Focal, ARM64)
┌──────────────────────────────────────────────────────────────────────┐
│  ROS 2 Foxy (native — already running)                               │
│    Mid-360 driver + Fast-LIO2 + fast_lio_tf_adapter                  │
│      └─► /robot/cloud_registered_body  (Mid-360 cloud)               │
│      └─► /robot/Odometry              (Fast-LIO pose)                │
│      └─► /tf, /tf_static              (map→base_link chain)          │
│                          │                                           │
│                          │ ros1_bridge (parameter_bridge, native)    │
│                          ▼                                           │
│  ROS 1 Noetic (native — installed via apt alongside Foxy)            │
│    voxblox_node ─► TSDF/ESDF                                         │
│         └─► gbplanner_node ─► PCI ─► /pci_command_path (PoseArray)   │
│                          │                                           │
│                          │ ros1_bridge (back to Foxy)                │
│                          ▼                                           │
│  ROS 2 Foxy republishes /pci_command_path                            │
└────────────────────────────────┬─────────────────────────────────────┘
                                 │
                          WiFi (CycloneDDS, cross-version Foxy↔Humble)
                                 │
┌────────────────────────────────▼─────────────────────────────────────┐
│ Laptop (Ubuntu 22.04 Jammy + ROS 2 Humble, RTX 4090)                 │
│   pci_to_sand_adapter ─► /sand_planner/goal (PointStamped in base_link)│
│   SanD-Planner ◄── RealSense D435 depth (USB-tethered to Go2)        │
│       └─► /cmd_vel ─► Go2 sport API                                  │
└──────────────────────────────────────────────────────────────────────┘
```

## Why this architecture

1. **No Docker on Orin** — JP 5.x = Focal = Noetic + Foxy both apt-installable
   natively. Docker is unnecessary overhead.
2. **gbplanner colocated with SLAM** — Fast-LIO → Voxblox → gbplanner all
   localhost loopback on Orin. Saves 30-100ms WiFi DDS jitter on point cloud.
3. **Only PoseArray crosses WiFi** — `/pci_command_path` is < 1 KB/s, robust
   to WiFi degradation. Point clouds (5-10 MB/s) stay onboard.
4. **SanD-Planner stays on laptop** — its diffusion + critic need RTX-class
   GPU at 8-10 Hz; Orin AGX GPU manages ~3-4 Hz at best.
5. **Foxy↔Humble DDS interop already proven** in Collab_QRC (see CLAUDE.md
   golden rule 16 + onboard SLAM split). PoseArray uses stable message defs.

## Files in this directory

| File | Purpose |
|---|---|
| `orin_install.sh` | One-shot installer for Orin. Adds Noetic apt repo, installs ros1_bridge, clones gbplanner3 source via vcstool, runs catkin build. ~30-90 min depending on Orin variant. |
| `orin_launch_gbplanner.sh` | Starts roscore + voxblox + gbplanner + ros1_bridge in a tmux session. |
| `bridge_topics.yaml` | Topic whitelist for ros1_bridge `parameter_bridge` — what crosses Foxy↔Noetic. |
| `gbplanner_go2.launch` | ROS 1 launch file that wires voxblox + gbplanner + PCI. Drop into `$GBPLANNER_WS/src/exploration/gbplanner_ros/gbplanner/launch/`. |
| `gbplanner_config.yaml` | Go2-specific gbplanner parameters: footprint, `max_inclination=26°` (stair-climb knob), Mid-360 FoV, batch sampling enabled. |

## Quick start (Orin)

```bash
# On Orin, after ssh-in:
scp -r laptop:Collab_QRC/scripts/real/gbplanner3 ~/gbplanner3_scripts
cd ~/gbplanner3_scripts
chmod +x *.sh

# 30-90 min build, then launches in tmux:
./orin_install.sh
./orin_launch_gbplanner.sh

# Then on laptop, trigger mission via Humble (DDS-bridged):
ros2 service call /planner_control_interface/std_srvs/automatic_planning std_srvs/srv/Empty "{}"
```

## Verification steps (each phase has a clean check)

### After `orin_install.sh`

```bash
source /opt/ros/noetic/setup.bash
source ~/gbplanner3_ws/devel/setup.bash
rospack find gbplanner          # should print path under devel/
rospack find voxblox_ros        # should print path
ros2 pkg list | grep ros1_bridge   # foxy ros1_bridge available
```

### After `orin_launch_gbplanner.sh`

```bash
# On Orin Noetic side:
source /opt/ros/noetic/setup.bash
rostopic hz /voxblox_node/tsdf_map_out          # should be ~1 Hz
rosnode info /gbplanner_node                    # check subscriptions are connected

# On Orin Foxy side (bridged to Humble laptop via DDS):
source /opt/ros/foxy/setup.bash
ros2 topic hz /pci_command_path                 # should publish after mission start

# On laptop Humble side:
ros2 topic echo /pci_command_path --once         # should arrive over WiFi
```

## Tuning knobs (where to start)

| Symptom | Knob | File |
|---|---|---|
| Robot refuses to climb stairs | `max_inclination: 0.45 → 0.61` (rad) | gbplanner_config.yaml RobotParams |
| RRG empty on multi-floor scene | `bounding_box_size: [..,..,3.5] → [..,..,5.0]` | gbplanner_config.yaml LocalPlannerParams |
| Voxblox OOM on Orin NX 16GB | `voxel_size: 0.15 → 0.20` | both gbplanner_go2.launch and gbplanner_config.yaml MapParams |
| Frequent loiter / no exploration goal | `exp_gain_threshold: 50.0 → 20.0` | gbplanner_config.yaml ExplorationParams |
| `/pci_command_path` not crossing WiFi | Check ROS_DOMAIN_ID identical on Orin Foxy + laptop Humble | env on both hosts |
| PoseArray drops on WiFi | Switch to FastDDS or pin to 5GHz | CycloneDDS profile |

## Known risks (read before going live)

1. **`gbplanner3_test` branch (not `gbplanner3`)** — UAS manifest pulls the
   dev branch. May be less stable than `gbplanner3`. If you hit weird crashes,
   `cd ~/gbplanner3_ws/src/exploration/gbplanner_ros && git checkout gbplanner3`
   and rebuild.

2. **Foxy↔Humble PoseArray ABI** — `geometry_msgs/PoseArray` is stable across
   distros, but if you hit `wire format mismatch`, switch to FastDDS RMW on
   both sides: `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.

3. **Orin NX/Nano memory** — Voxblox at 0.15 m / 8 m range eats 3-5 GB for a
   moderate indoor scene. AGX 32GB safe; NX 16GB tight; Nano 8GB you must drop
   to 0.2-0.25 m voxel.

4. **`max_inclination` vs CHAMP gait** — default 0.45 rad (≈26°) is what
   CHAMP can reliably do without dedicated stair-climb policy. If you crank to
   35° make sure RL stair policy is active or robot will fall.

5. **TF frames** — `gbplanner_go2.launch` defaults `world_frame: map`,
   `sensor_frame: mid360_link`. Your real Fast-LIO setup may use different
   names — check with `ros2 run tf2_tools view_frames` on Orin Foxy before
   running.
