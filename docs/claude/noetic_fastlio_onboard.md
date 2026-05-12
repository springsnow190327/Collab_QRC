# Onboard Noetic FAST-LIO2 — pitfalls & runbook (2026-05-11)

ROS 1 Noetic build of HKU-MARS FAST-LIO2 + Livox Mid-360 driver running natively on the Go2's Jetson Orin (Ubuntu 20.04 Focal, ARM64, JetPack 5.x). Lives in a SEPARATE catkin workspace from the existing Foxy stack.

## Why this exists

The earlier "onboard SLAM split" (2026-04-30, see [real_robot.md](real_robot.md#onboard-slam-split-shipped-2026-04-30--fast-lio--livox-on-jetson)) put **ROS 2 Foxy** FAST-LIO on the Jetson. That works for the laptop's Nav2 stack, but feeding gbplanner3 (Voxblox + planner, ROS 1 only) requires `ros1_bridge` to translate `/cloud_registered_body` from ROS 2 → ROS 1 on the same Jetson — at ~5-10 MB/s the bridge becomes the bottleneck and noisy under DDS load.

Going **native ROS 1 Noetic for FAST-LIO** means voxblox subscribes locally with no bridge for the heavy SLAM stream; only the tiny `/pci_command_path` PoseArray (< 1 KB/s) still needs to cross to ROS 2 for the laptop.

```
Before (Foxy + ros1_bridge):                After (native Noetic):
─────────────────────────────              ──────────────────────
Mid-360 → ROS 2 livox_ros_driver2          Mid-360 → ROS 1 livox_ros_driver2
        → ROS 2 fastlio                            → ROS 1 fastlio
        → ros1_bridge (PointCloud2 5MB/s)          (no bridge for SLAM stream)
        → ROS 1 voxblox + gbplanner                → ROS 1 voxblox + gbplanner
        → ros1_bridge (PoseArray <1KB/s)           → ros1_bridge (PoseArray <1KB/s)
        → ROS 2 laptop                             → ROS 2 laptop
```

## Quick reference

### Scripts (all in [`scripts/real/`](../../scripts/real/))

| Side | Script | Purpose |
|---|---|---|
| laptop | `deploy_noetic_to_jetson.sh` | rsync `src/vendor/{fast_lio_ros1,livox_ros_driver2}` + Jetson scripts → `unitree@192.168.123.18:~/noetic_fastlio_ws/` |
| Jetson | `onboard_fastlio_noetic.sh` | start roscore + livox driver + fastlio + 3 static TFs (ns=robot) |
| Jetson | `onboard_record_noetic.sh` | record `/livox/*` + `/robot/*` SLAM outputs to `~/bags/` as ROS 1 `.bag` (split 1 GB chunks, nohup-protected against ssh drop) |
| laptop | `rviz_view_onboard_fastlio.sh` | `ssh -Y` + `LIBGL_ALWAYS_SOFTWARE=1` → RViz on Jetson, display on laptop (**flaky over USB-C dongle, see X11 caveat below**) |
| laptop | `fetch_and_plot_cloud.sh` | snapshot N frames of `/robot/cloud_registered` → rsync to laptop → static Open3D viewer |
| laptop | `stream_cloud_live.sh` | **live** point cloud stream Jetson → laptop Open3D over ssh binary pipe (no X11) |

### One-time setup (on a fresh Jetson)

```bash
# 1. On laptop: clone HKU FAST-LIO2 (master == FAST-LIO2 despite repo name)
git clone --recursive https://github.com/hku-mars/FAST_LIO.git \
  src/vendor/fast_lio_ros1
touch src/vendor/fast_lio_ros1/COLCON_IGNORE   # keep laptop colcon off it

# 2. Apply local patches (see "Pitfalls" §3, §4, §6)
# 3. Deploy
JETSON_PASS=123 ./scripts/real/deploy_noetic_to_jetson.sh

# 4. On Jetson — build (conda deactivate FIRST, see pitfall §7)
ssh unitree@192.168.123.18
conda deactivate
cd ~/noetic_fastlio_ws/src/livox_ros_driver2 && ./build.sh ROS1
# (./build.sh ROS1 actually builds the entire ws via catkin_make)
```

### Daily run

```bash
# Jetson: bind Mid-360 NIC + launch stack
# (NIC bind isn't persistent across Jetson reboot — see pitfall §10)
echo 123 | sudo -S ip addr add 192.168.123.100/24 dev eth0
~/noetic_fastlio_ws/scripts/onboard_fastlio_noetic.sh

# Jetson: record while walking (Ctrl+C to finalize bag)
~/noetic_fastlio_ws/scripts/onboard_record_noetic.sh tag=corridor_run1

# Laptop: visualize live (no X11 — Open3D pulls binary frames via ssh)
JETSON_PASS=123 ./scripts/real/stream_cloud_live.sh
```

---

## Pitfalls — the 10-chain bug saga (2026-05-11)

Most of these were silent failures (wrong defaults, broken caches, hidden upstream-incompatibility). Each is documented as: **symptom → root cause → fix**.

### §1. Jetson has no internet — can't `git clone` on-device

**Symptom**: `curl github.com` times out from Jetson (robot LAN is offline).

**Root cause**: The Go2's onboard 192.168.123.x subnet has no route to the outside world. Laptop has WiFi.

**Fix**: Clone on the **laptop**, rsync into `~/noetic_fastlio_ws/src/` via [`deploy_noetic_to_jetson.sh`](../../scripts/real/deploy_noetic_to_jetson.sh).

### §2. "FAST-LIO2" repo doesn't exist — master IS FAST-LIO2

**Symptom**: Searching GitHub for "FAST-LIO2" returns nothing canonical.

**Root cause**: HKU evolved `hku-mars/FAST_LIO` in place. v1.0 tag = FAST-LIO (original); `master` HEAD (`7cc4175 Support MARSIM simulator`) = FAST-LIO 2 (T-RO 2022). The repo wasn't renamed.

**Identification**: `grep ikd_Tree src/laserMapping.cpp` — incremental KD-Tree is the v2 hallmark (v1 used standard KD-tree).

### §3. HKU FAST-LIO depends on the OLD `livox_ros_driver` (no `2`)

**Symptom**: `find_package` fails with "Could not find a package configuration file provided by 'livox_ros_driver'".

**Root cause**: FAST-LIO master predates Mid-360. It was authored when livox_ros_driver (1) was the only driver — for Avia, Horizon, Mid-40. Mid-360 needs `livox_ros_driver2` (different package with same-named `CustomMsg`).

**Fix**: sed-replace `livox_ros_driver` → `livox_ros_driver2` in 5 files (8 occurrences). Applied to our vendored copy [`src/vendor/fast_lio_ros1/`](../../src/vendor/fast_lio_ros1/):

```
CMakeLists.txt:54
package.xml:28, 39
src/preprocess.h:8, 94, 110
src/preprocess.cpp:44, 92
src/laserMapping.cpp:59, 302
```

The two drivers' `CustomMsg` is identical bit-for-bit (Livox kept message def stable), so namespace rename is the only change needed.

### §4. `livox_ros_driver2` source references Mid-360**s** enum the installed SDK doesn't have

**Symptom**: Build fails at 87% with:
```
src/comm/pub_handler.cpp:135:103: error:
  'kLivoxLidarTypeMid360s' is not a member of 'LivoxLidarDeviceType'
```

**Root cause**: Our Jetson's `~/onboard_ws/install/Livox-SDK2/` is from the original Foxy deployment. Newer livox_ros_driver2 source added support for the **Mid-360s** (newer variant), which requires a newer Livox-SDK2 enum. We don't have that hardware.

**Fix**: Drop the `||dev_type==...Mid360s` clause in [`src/vendor/livox_ros_driver2/src/comm/pub_handler.cpp:135`](../../src/vendor/livox_ros_driver2/src/comm/pub_handler.cpp#L135). Effectively makes the driver Mid-360-only, which is what we have anyway.

### §5. `MID360_config.json` ships with upstream's wrong network

**Symptom**: livox driver logs `bind failed → Create detection socket failed → Failed to init livox lidar sdk`. No `/livox/lidar` publisher appears.

**Root cause**: HKU's upstream `MID360_config.json` has `host_net_info: 192.168.1.5` and `lidar_configs: 192.168.1.12` — those are Livox SDK defaults that don't match the Go2 rig (host on `192.168.123.100`, Mid-360 on `192.168.123.20`).

**Fix**: Overlay the customized config from the Foxy ws:

```bash
# On Jetson:
cp ~/onboard_ws/config/slam/MID360_config.json \
   ~/noetic_fastlio_ws/src/livox_ros_driver2/config/MID360_config.json
```

The customized JSON is part of [`src/go2w/go2w_real_bringup/config/slam/MID360_config.json`](../../src/go2w/go2w_real_bringup/config/slam/MID360_config.json). Long term, the deploy script should copy this for us — TODO.

### §6. FAST-LIO publishes to ABSOLUTE topic paths — `ROS_NAMESPACE` is ignored

**Symptom**: Even with `ROS_NAMESPACE=robot roslaunch fast_lio mapping_mid360.launch`, outputs land at `/Odometry`, `/cloud_registered_body`, etc. — NOT `/robot/...`.

**Root cause**: HKU's [`src/laserMapping.cpp:849-860`](../../src/vendor/fast_lio_ros1/src/laserMapping.cpp#L849) advertises with leading-slash strings like `nh.advertise<...>("/Odometry", ...)`. Absolute paths bypass node namespace.

**Fix**: Drop the leading `/` in 6 advertise calls (still in [src/laserMapping.cpp](../../src/vendor/fast_lio_ros1/src/laserMapping.cpp)). The `/livox/lidar` and `/livox/imu` SUBSCRIPTIONS stay absolute (livox driver doesn't run under our namespace).

After patch:
- `/livox/lidar`, `/livox/imu` — sensor topics, global
- `/robot/Odometry`, `/robot/cloud_registered{,_body}`, `/robot/path` — SLAM outputs, namespaced for gbplanner3's [`bridge_topics.yaml`](../../scripts/real/gbplanner3/bridge_topics.yaml)

### §7. Miniconda's `(base)` python contaminates ROS C++ node link path

**Symptom**: `roslaunch` parses the launch file fine, prints PARAMETERS, then immediately exits with `"No processes to monitor"`. `~/.ros/log/<run>/laserMapping-1.log` is **empty** — node died before `main()`.

**Root cause**: This Jetson's `.bashrc` auto-runs `conda activate base`, prepending `/home/unitree/miniconda3/bin` to PATH. The C++ binaries (`fastlio_mapping`, `livox_ros_driver2_node`) are built against system `libstdc++` / `libpython3.8`, but their dynamic linker picks up miniconda's mismatched libraries first → SIGSEGV during pre-`main()` static init.

**Fix**: All Jetson-side scripts now scrub conda env at startup. From [`onboard_fastlio_noetic.sh`](../../scripts/real/onboard_fastlio_noetic.sh):

```bash
if [[ -n "${CONDA_PREFIX:-}" ]] || echo "$PATH" | grep -q miniconda; then
  type conda 2>/dev/null | head -1 | grep -q function && conda deactivate 2>/dev/null || true
  export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE '(miniconda|conda)' | tr '\n' ':')"
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL CONDA_PYTHON_EXE
fi
unset PYTHONPATH PYTHONHOME
```

`catkin_make` from an interactive shell needs the same — first build failure was `CMake Error ... try installing python3-empy` because conda's python3 didn't have `em` module. Long-term fix: edit `.bashrc` to gate conda activation behind `if [ -z "$ROS_DISTRO" ]; then ...`. Not done yet.

### §8. HKU's launch file `<param>` overrides yaml `<rosparam load>`

**Symptom**: After editing `config/mid360.yaml` to set `point_filter_num: 1`, `rosparam get /robot/point_filter_num` still returns `3`.

**Root cause**: ROS 1 launch loads params in source order. Upstream `launch/mapping_mid360.launch` has:

```xml
<rosparam command="load" file="$(find fast_lio)/config/mid360.yaml" />
<param name="point_filter_num" type="int" value="3"/>   <!-- overrides yaml -->
<param name="filter_size_surf" type="double" value="0.5" />
...
```

The 7 inline `<param>` tags silently overrode the yaml.

**Fix**: Deleted the redundant `<param>` lines in [`launch/mapping_mid360.launch`](../../src/vendor/fast_lio_ros1/launch/mapping_mid360.launch); yaml is now the single source of truth.

### §9. RViz over `ssh -X` renders black on modern Ubuntu

**Symptom**: RViz launches fine via `ssh -X`, subscriptions show `Status: Ok`, but viewport is fully black. Enabling Grid display CRASHES RViz.

**Root cause chain**:
1. SSH `-X` (untrusted) blocks GL extensions under XSecurity → Ogre crashes on Grid/large primitives.
2. SSH `-Y` (trusted) doesn't crash, but Mesa direct rendering still ships GL commands the laptop's X server silently drops (Jammy ships with indirect GLX disabled).
3. `LIBGL_ALWAYS_INDIRECT=1` → no error, no pixels (X server refuses indirect GLX).
4. `LIBGL_ALWAYS_SOFTWARE=1` → CPU rendering on Jetson, `XPutImage` blits framebuffer pixels via X11. Works but slow.

**Pragmatic fix**: Stop fighting X11. We shipped two non-X paths:

- [`fetch_and_plot_cloud.sh`](../../scripts/real/fetch_and_plot_cloud.sh) — snapshot N frames, rsync NPZ, local Open3D viewer. Static.
- [`stream_cloud_live.sh`](../../scripts/real/stream_cloud_live.sh) — Jetson rospy → binary framing → ssh stdin pipe → laptop Open3D `Visualizer.update_geometry()`. Live, ~10 fps over USB-C.

For posterity [`rviz_view_onboard_fastlio.sh`](../../scripts/real/rviz_view_onboard_fastlio.sh) still exists with `-Y` + `LIBGL_ALWAYS_SOFTWARE=1` but is documented as flaky.

The nuclear fallback (if you really need RViz interactivity): **NoMachine** is already installed on the Jetson (`~/nomachine.sh`). Install NoMachine client on laptop, get the full Jetson desktop, RViz runs natively. Bypasses X11 entirely.

### §10. Mid-360 NIC bind doesn't persist across Jetson reboot

**Symptom**: After Jetson reboot (e.g. dog auto-restarts after a fall), `onboard_fastlio_noetic.sh` fails on first run with `bind failed` — even though IP was on eth0 yesterday.

**Root cause**: `ip addr add 192.168.123.100/24 dev eth0` is a runtime modification; it's gone on reboot.

**Fix (manual, current)**: launcher tries `sudo -n ip addr add`; if sudo cache is stale, manual pre-cache is needed:

```bash
echo 123 | sudo -S ip addr add 192.168.123.100/24 dev eth0
~/noetic_fastlio_ws/scripts/onboard_fastlio_noetic.sh
```

**Fix (TODO)**: Persistent netplan or systemd unit. Suggested netplan stanza:

```yaml
network:
  ethernets:
    eth0:
      addresses: [192.168.123.18/24, 192.168.123.100/24]
```

Not yet shipped — Unitree's image uses NetworkManager / netplan in a non-obvious way, needs care.

### §11. rosbag dies on SSH drop; verification logic false-negatived

**Symptom 1**: User starts `onboard_record_noetic.sh` in an interactive ssh session, ssh drops mid-recording, rosbag dies via SIGHUP. `.bag.active` file is left orphaned.

**Fix**: `nohup rosbag record ... & disown` — rosbag now ignores SIGHUP. ssh can drop without taking the recording down. Resilience verified: 3.7 GB bag in 4 chunks survived a network drop.

**Recovery for orphaned `.bag.active`**: `mv x.bag.active x.bag && rosbag reindex x.bag`. The data is intact; just the index needs rebuilding.

**Symptom 2**: Recorder's "Verifying N topics" check reported all 8 topics as "no publisher — will record empty", even when topics WERE healthy. Recording still worked; the check was misleading.

**Root cause**: Original grep was `grep -qE "Publishers: *(\* |None)"`. This matched the same line as "Publishers:" + literal star — but RViz publishes "Publishers: " on one line and " * /node" on the NEXT line, so the regex never matched the publisher-present case.

**Fix**: Replaced with explicit `if rostopic_info | grep -q "^Publishers: None"; then missing; elif grep -q "^ \* /"; then present; else unknown; fi`.

---

## Tuning notes

Current `config/mid360.yaml` is **ported from the Foxy real-robot config** ([`src/go2w/go2w_real_bringup/config/slam/fastlio_mid360.yaml`](../../src/go2w/go2w_real_bringup/config/slam/fastlio_mid360.yaml)). Key values vs HKU upstream defaults:

| Key | HKU default | Our value | Why |
|---|---|---|---|
| `point_filter_num` | 3 | **1** | Sparse outdoor Mid-360 returns (~24k pts) need every point for ICP correspondences |
| `filter_size_surf` | 0.5 | **0.10** | Same; aggressive voxel filtering caused 23 km drift on outdoor bag |
| `filter_size_map` | 0.5 | **0.10** | Same |
| `preprocess.blind` | 0.5 | **0.20** | Keep near-field band for outdoor sparse scenes |
| `mapping.extrinsic_est_en` | false | **true** | `extrinsic_T` is factory value, not measured per unit — let EKF refine |
| `publish.path_en` | false | **true** | gbplanner3 visualizes `/robot/path` |
| `pcd_save.pcd_save_en` | true | **false** | SD wear during long runs |

CPU cost: p50 ~125 ms / frame on Orin → effective rate ~8 Hz (vs 10 Hz nominal). Acceptable for SLAM-only workload; would saturate if we also ran voxblox @ 0.10 m TSDF on the same Orin.

---

## TODO

1. **`deploy_noetic_to_jetson.sh` should overlay `MID360_config.json`** — currently the upstream `192.168.1.x` config gets deployed; user must manually `cp` the Foxy version. Easy fix.
2. **Persistent NIC bind** — netplan stanza for `192.168.123.100/24` as a secondary on eth0.
3. **`.bashrc` conda guard** — wrap conda init in `if [ -z "$ROS_DISTRO" ]`, so source'ing `/opt/ros/noetic/setup.bash` first prevents conda activation.
4. **Wire `stream_cloud_live.sh` into a daily-ops launcher** — currently a standalone script; could be a flag on `rviz_view_onboard_fastlio.sh`.
5. **gbplanner3 launch integration** — voxblox on Jetson subscribes to native `/robot/cloud_registered_body`; need to update its launch to skip the ros1_bridge for SLAM topics (only PoseArray crosses).
