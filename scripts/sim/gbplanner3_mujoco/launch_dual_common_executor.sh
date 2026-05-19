#!/usr/bin/env bash
# Start/stop dual GBPlanner3 for the mixed common-executor benchmark.
#
# This launches two isolated ROS1 masters through the Unified Autonomy Stack:
#   robot_a master: localhost:11311 -> /robot_a/command/trajectory
#   robot_b master: localhost:11312 -> /robot_b/command/trajectory
#
# The Collab_QRC ROS2 launch handles MuJoCo/Fast-LIO/Nav2 and subscribes to
# those namespaced trajectory topics through gbplanner_to_waypoint_adapter.py.
set -u -o pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
UAS_REPO_ROOT="${UAS_REPO_ROOT:-$HOME/Research/uas_deploy/unified_autonomy_stack}"
COLLAB_QRC_ROOT="${COLLAB_QRC_ROOT:-$WS_DIR}"
OVERLAY_COMPOSE="${COLLAB_QRC_ROOT}/scripts/sim/gbplanner3_mujoco/compose/docker-compose.collab_qrc_dual.yml"
LOG_PATH="${GBPLANNER3_DUAL_LOG_PATH:-/tmp/gbplanner3_dual_common_executor.log}"

stop_stack() {
  if [[ -d "${UAS_REPO_ROOT}" && -f "${OVERLAY_COMPOSE}" ]]; then
    (
      cd "${UAS_REPO_ROOT}" &&
      UAS_REPO_ROOT="${UAS_REPO_ROOT}" \
      COLLAB_QRC_ROOT="${COLLAB_QRC_ROOT}" \
      docker compose -f "${OVERLAY_COMPOSE}" --profile launch down --remove-orphans
    ) >/tmp/gbplanner3_dual_stop.log 2>&1 || true
  fi
}

case "${1:-run}" in
  stop)
    stop_stack
    exit 0
    ;;
  run)
    ;;
  *)
    echo "Usage: $0 [run|stop]" >&2
    exit 2
    ;;
esac

[[ -d "${UAS_REPO_ROOT}" ]] || {
  echo "ERROR: UAS_REPO_ROOT not found: ${UAS_REPO_ROOT}" >&2
  echo "Set UAS_REPO_ROOT to unified_autonomy_stack." >&2
  exit 1
}
[[ -f "${OVERLAY_COMPOSE}" ]] || {
  echo "ERROR: compose overlay not found: ${OVERLAY_COMPOSE}" >&2
  exit 1
}
command -v docker >/dev/null 2>&1 || {
  echo "ERROR: docker command not found" >&2
  exit 1
}

child=""
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "${child}" ]]; then
    kill -TERM "${child}" 2>/dev/null || true
    wait "${child}" 2>/dev/null || true
  fi
  stop_stack
}
terminate() {
  cleanup
  exit 0
}
trap cleanup EXIT
trap terminate INT TERM

stop_stack
echo "Starting dual GBPlanner3 common-executor stack"
echo "  UAS_REPO_ROOT   : ${UAS_REPO_ROOT}"
echo "  COLLAB_QRC_ROOT : ${COLLAB_QRC_ROOT}"
echo "  compose         : ${OVERLAY_COMPOSE}"
echo "  log             : ${LOG_PATH}"

cd "${UAS_REPO_ROOT}"
UAS_REPO_ROOT="${UAS_REPO_ROOT}" \
COLLAB_QRC_ROOT="${COLLAB_QRC_ROOT}" \
DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
bash -c "make launch DOCKER_COMPOSE_FILE='${OVERLAY_COMPOSE}' 2>&1 | grep --line-buffered -v \"No 'elevation' layer in map\"" \
  >"${LOG_PATH}" 2>&1 &
child=$!

wait "${child}"
child=""
