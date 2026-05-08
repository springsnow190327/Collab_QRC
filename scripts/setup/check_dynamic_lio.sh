#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=scripts/setup/backend_check_common.sh
source "${WS_DIR}/scripts/setup/backend_check_common.sh"

SRC_DIR="${DYNAMIC_LIO_SOURCE_DIR:-${WS_DIR}/external/dynamic_lio}"
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
Usage: scripts/setup/check_dynamic_lio.sh [--host|--docker] [deployment_mode]

Prints Dynamic-LIO filtering backend availability JSON. Dynamic-LIO is never
treated as a primary odometry source by this project.
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
  "dynamic_lio" \
  "Dynamic-LIO" \
  "${STRATEGY}" \
  "${DEPLOYMENT_MODE}" \
  "${SRC_DIR}" \
  "${SRC_DIR}/sr_lio/package.xml" \
  "${SRC_DIR}/sr_lio/launch/lio_urban_nav.launch" \
  "ros-noetic-pcl-ros, ros-noetic-tf, ros-noetic-cv-bridge, GTSAM/SC-PGO optional, Livox/Ouster input driver" \
  "${WORKSPACE_PATH}"
