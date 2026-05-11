#!/usr/bin/env bash
# nav_test_swarm_lio2_mixed.sh — sim Go2W (robot_a) + Go2 (robot_b) +
# CHAMP + Swarm-LIO2 (ROS1 in docker, drone_id=1/2) + Nav2 MPPI SE2
# holonomic + CFPA2 frontier coordination + multirobot_map_merge.
#
# Cousin of nav_test_swarm_lio2_dual.sh — heterogeneous fleet. The launch
# brings up the docker compose stack itself (bridge.yaml already exposes
# /robot_{a,b}/swarm_lio2_raw/*) and tears it down on Ctrl+C.
#
# Prerequisites (one-time):
#   1. ROS1 SLAM source in external/ — see scripts/setup/fetch_slam_backends.sh
#   2. Build the docker image once:
#        cd docker/ros1_hybrid_slam && docker compose build
#   3. Build the catkin workspace inside the container:
#        cd docker/ros1_hybrid_slam && docker compose run --rm ros1_hybrid_slam build
#
# Usage:
#   ./scripts/launch/nav_test_swarm_lio2_mixed.sh                                 # gui:=false
#   ./scripts/launch/nav_test_swarm_lio2_mixed.sh gui:=true rviz:=true
#   ./scripts/launch/nav_test_swarm_lio2_mixed.sh holonomic_profile_a:=off        # SmacHybrid baseline
#   ./scripts/launch/nav_test_swarm_lio2_mixed.sh nav_backend_a:=far nav_backend_b:=far
#   ./scripts/launch/nav_test_swarm_lio2_mixed.sh debug:=true                     # focus stdout on nav
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"

safe_source() { set +u; source "$1"; set -u; }

if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  safe_source "${HOME}/miniforge3/etc/profile.d/conda.sh"
  conda activate cmu_env 2>/dev/null || true
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate cmu_env 2>/dev/null || true
fi

safe_source "${ROS2_SETUP_BASH}"
safe_source "${WS_DIR}/install/setup.bash"

export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"

if ! docker images --format '{{.Repository}}' | grep -q '^ros1_hybrid_slam-ros1_hybrid_slam$'; then
  cat >&2 <<EOF
=========================================================================
WARNING: docker image 'ros1_hybrid_slam-ros1_hybrid_slam' not found.
The launch will trigger a rebuild during startup, which can take ~5 min
and may race the MuJoCo bringup. Build it now:

  cd ${WS_DIR}/docker/ros1_hybrid_slam && docker compose build

Then verify the catkin workspace exists:

  docker compose run --rm ros1_hybrid_slam build

Continuing in 5s — Ctrl+C to abort.
=========================================================================
EOF
  sleep 5
fi

echo "=== MuJoCo Mixed Nav Test (Swarm-LIO2 hybrid + Nav2 SE2 + CFPA2) ==="
echo ""

exec ros2 launch go2_gazebo_sim nav_test_mujoco_swarm_lio2_mixed.launch.py "$@"
