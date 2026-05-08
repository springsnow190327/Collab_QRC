#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=scripts/setup/backend_check_common.sh
source "${WS_DIR}/scripts/setup/backend_check_common.sh"

SRC_DIR="${ERASOR_SOURCE_DIR:-${WS_DIR}/external/ERASOR}"
DEPLOYMENT_MODE="${DEPLOYMENT_MODE:-}"
REQUESTED_STRATEGY=""

for arg in "$@"; do
  case "${arg}" in
    --host)
      REQUESTED_STRATEGY="host_noetic_catkin"
      ;;
    --docker)
      REQUESTED_STRATEGY="docker_catkin"
      ;;
    --help|-h)
      cat <<'EOF'
Usage: scripts/setup/check_erasor.sh [--host|--docker] [deployment_mode]

Prints ERASOR cleanup backend availability JSON. ERASOR is checked only as an
asynchronous map cleanup backend, not as an odometry loop component.
EOF
      exit 0
      ;;
    *)
      DEPLOYMENT_MODE="${arg}"
      ;;
  esac
done

if [[ -z "${DEPLOYMENT_MODE}" ]]; then
  case "${REQUESTED_STRATEGY}" in
    host_noetic_catkin) DEPLOYMENT_MODE="real_hybrid_ros1_slam_ros2_nav" ;;
    *) DEPLOYMENT_MODE="sim_hybrid_ros1_slam_ros2_nav" ;;
  esac
fi

STRATEGY="$(backend_check_resolve_strategy "${REQUESTED_STRATEGY}" "${DEPLOYMENT_MODE}")"
WORKSPACE_PATH="${ROS1_HYBRID_WS:-${WS_DIR}/.local_deps/ros1_hybrid_slam_ws}"

backend_check_emit \
  "erasor" \
  "ERASOR" \
  "${STRATEGY}" \
  "${DEPLOYMENT_MODE}" \
  "${SRC_DIR}" \
  "${SRC_DIR}/package.xml" \
  "${SRC_DIR}/launch/run_erasor.launch" \
  "ros-noetic-pcl-ros, ros-noetic-tf, Eigen, OpenMP, offline PCD map artifacts" \
  "${WORKSPACE_PATH}"
