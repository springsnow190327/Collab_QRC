# GBPlanner3 stack — host-side setup notes

Real-world setup recipe from first install on `hz-XPS-15-9530` (2026-05-11).
Use alongside [`README.md`](README.md). The README assumes the UAS Docker
images already exist; this file covers getting there from a clean machine.

## Prerequisites checklist

```bash
# 1. ros2 humble + colcon + Collab_QRC built (you already have this)
# 2. vcstool — needed by UAS import script
pip install vcstool
# 3. docker + nvidia driver + nvidia-smi working
```

## One-time host fixes (apply before any UAS build)

### 1. Move Docker data off the root partition

UAS images total ~80 GB. On a small root partition (`hz-XPS-15-9530` had a
98 GB `/`) this WILL fill the disk mid-build. Docker 29 uses the
`containerd-snapshotter` storage backend, so **both** Docker's data-root
AND containerd's data-root must be moved — setting `data-root` alone is
not enough.

```bash
# move docker
sudo bash -c 'systemctl stop docker docker.socket containerd && \
  mv /var/lib/docker /home/docker && \
  printf "{\n  \"data-root\": \"/home/docker\"\n}\n" > /etc/docker/daemon.json && \
  systemctl start docker'

# move containerd (Docker 29 stores image layers here, not in data-root)
sudo bash -c 'systemctl stop docker docker.socket containerd && \
  mv /var/lib/containerd /home/containerd && \
  sed -i "s|^#root = \"/var/lib/containerd\"|root = \"/home/containerd\"|" /etc/containerd/config.toml && \
  systemctl start containerd docker'

# verify
docker info | grep -E "Docker Root Dir|driver-type"
# expect: Docker Root Dir: /home/docker, driver-type: io.containerd.snapshotter.v1
```

### 2. Install nvidia-container-toolkit

`ros1_launch_gbplanner` is declared with `--gpus all` (rviz hw accel). On
machines without this toolkit, `docker compose up` fails with
`could not select device driver "nvidia" with capabilities: [[gpu]]`.

```bash
sudo bash -c '
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed "s#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g" \
    > /etc/apt/sources.list.d/nvidia-container-toolkit.list
  apt-get update -qq && apt-get install -y nvidia-container-toolkit
  nvidia-ctk runtime configure --runtime=docker
  systemctl restart docker
'
# verify
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

## Clone UAS + import sub-repos

```bash
git clone https://github.com/ntnu-arl/unified_autonomy_stack.git ~/Research/uas_deploy/unified_autonomy_stack
cd ~/Research/uas_deploy/unified_autonomy_stack

# import_all_repos.sh has a bug — it imports robot_bringup before
# creating the workspaces/ dir. Pre-create it.
mkdir -p workspaces
./scripts/import_all_repos.sh
# ws_ros_gst_bridge will fail (missing branch); harmless for gbplanner3.
```

## Build Docker images — strategic subset

`make images` builds ALL 12 targets (~80 GB, ~45 min). For gbplanner3 demo
you only need 5:

```
ros1_base   ros2_base   ros1_gbplanner   ros1-bridge-builder   ros2_ros1_bridge
```

The other 7 (`ros2_rl`, `ros2_cuda`, `ros2_nmpc`, `ros2_cbf`, `ros2_vlm`,
`cuda_pytorch`, `ros2_sim`) are for UAV NMPC / RL / VLM / Gazebo work and
are not needed.

Easiest path: run `make images`, watch for `unified_autonomy:ros2_ros1_bridge`
to appear in `docker images`, then Ctrl-C (buildkit caches won't be wasted).
Or build the subset explicitly:

```bash
cd ~/Research/uas_deploy/unified_autonomy_stack
docker buildx bake --allow=network.host \
  ros1_base ros2_base ros1_gbplanner ros1-bridge-builder ros2_ros1_bridge
```

**Note on intermittent failures**: the `ros1-bridge-builder` image runs a
chain of `git clone` calls against GitHub during build. These fail flakily
on IPv6-misconfigured hosts (Connection timed out after 134 s, then a
later retry passes). Just re-run `make images` — buildkit cache resumes
where it left off.

## Compile workspaces

Out of ~13 `make build-*` targets, only two matter:

```bash
cd ~/Research/uas_deploy/unified_autonomy_stack
make build-ros1_bridge   # fast — extracts a pre-baked .tgz
make build-gbplanner     # catkin build of ws_gbplanner inside ros1_gbplanner image (~3 min, 62 packages)
```

## Launch

```bash
cd ~/Collab_QRC
./scripts/launch/nav_test_gbplanner_demo3.sh         # full start
./scripts/launch/nav_test_gbplanner_demo3.sh start_mission  # trigger automatic_planning
./scripts/launch/nav_test_gbplanner_demo3.sh stop    # tear down
```

## Known issue — `collab_qrc_go2` config incomplete

[`config/collab_qrc_go2/gbplanner_config.yaml`](config/collab_qrc_go2/gbplanner_config.yaml)
(added in commit `454d6e4`) is missing RRG-required sections
(`BoundedSpaceParams`, `RandomSamplerParams`, `PlanningParams`, ...).
`gbplanner_node` shuts down 0.5 s after start with
`Could not load all required parameters. Shutdown ROS node.` and PCI
loops waiting for `planner_server` forever.

**Working templates to merge from** (both have all 8 required sections):
- `workspaces/ws_gbplanner/src/exploration/gbplanner_ros/gbplanner/config/ugv/gzc/urban_exploration/gbplanner_config.yaml` — NTNU original UGV
- `workspaces/robot_bringup/config/ros1/gbplanner/ugv_sim/gbplanner_config.yaml` — UAS-derived UGV (already tuned for ground robot)

Copy `ugv_sim` as the base, override `RobotParams.size` to Go2 dims
(0.6×0.3×0.4), tune `SensorParams` to Mid-360 (60° vFoV, 40 m range),
keep the rest as-is.
