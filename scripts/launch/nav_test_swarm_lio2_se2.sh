#!/usr/bin/env bash
# nav_test_swarm_lio2_se2.sh — sim Go2 + CHAMP + Swarm-LIO2 (ROS1 in docker)
# + Nav2 SE2 holonomic + CFPA2 frontier exploration.
#
# Companion to nav_test_go2.sh, but Fast-LIO2 is replaced by the dockerized
# ROS1 Swarm-LIO2 stack. Validates the SLAM swap before going to real
# hardware. The launch starts the docker compose stack itself and tears it
# down on Ctrl+C; you do NOT need to `docker compose up` manually.
#
# Prerequisites (one-time):
#   1. ROS1 SLAM source in external/ — see scripts/setup/fetch_slam_backends.sh
#   2. Build the docker image once:
#        cd docker/ros1_hybrid_slam && docker compose build
#   3. Build the catkin workspace inside the container:
#        cd docker/ros1_hybrid_slam && docker compose run --rm ros1_hybrid_slam build
#
# Usage:
#   ./scripts/launch/nav_test_swarm_lio2_se2.sh                 # gui:=true
#   ./scripts/launch/nav_test_swarm_lio2_se2.sh gui:=false      # headless
#   ./scripts/launch/nav_test_swarm_lio2_se2.sh rviz:=true
#   ./scripts/launch/nav_test_swarm_lio2_se2.sh explore:=false  # manual goals only
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

# Pre-flight: confirm the docker image was built; if not, hint at the build
# step rather than letting `docker compose up` rebuild it during the launch
# (which delays MuJoCo startup and racing nodes time out).
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

echo "=== MuJoCo Nav Test (Swarm-LIO2 hybrid + Nav2 SE2 + CFPA2) ==="
echo ""

exec ros2 launch go2_gazebo_sim nav_test_mujoco_swarm_lio2_se2.launch.py "$@"
