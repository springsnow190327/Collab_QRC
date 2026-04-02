#!/usr/bin/env bash
# VLM-in-the-loop single-robot exploration demo.
#
# Usage:
#   ./vlm_demo.sh                                     # defaults (xAI/Grok, FAR nav, carto_2d)
#   ./vlm_demo.sh nav_execution_backend:=reactive      # override nav backend
#
# VLM history logs are saved to ~/.ros/log/vlm_history/<run_timestamp>/
# A live debug viewer starts automatically at http://localhost:8501
# It waits for the coordinator to create the log dir, then auto-refreshes every 2s.
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"
XAI_ENV_FILE="${XAI_ENV_FILE:-${WS_DIR}/src/exploration/cfpa2-rh-physics-exploration/.env.xai}"

safe_source() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  safe_source "${HOME}/miniforge3/etc/profile.d/conda.sh"
  conda activate cmu_env
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate cmu_env
fi

safe_source "${ROS2_SETUP_BASH}"
safe_source "${WS_DIR}/install/setup.bash"
safe_source "${WS_DIR}/scripts/common_logging.sh"

if [[ -f "${XAI_ENV_FILE}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${XAI_ENV_FILE}"
  set +a
fi

setup_run_logging "vlm_demo"

LAUNCH_JOB_PID=""
LAUNCH_JOB_PGID=""
SELF_PGID="$(ps -o pgid= $$ | tr -d ' ')"

terminate_matching_processes() {
  local pattern="$1"
  local signal="$2"
  local pid
  for pid in $(pgrep -f "$pattern" 2>/dev/null || true); do
    [[ "$pid" == "$$" ]] && continue
    [[ "$pid" == "$PPID" ]] && continue
    kill "-${signal}" "$pid" 2>/dev/null || true
  done
}

any_matching_processes() {
  local pattern="$1"
  local pid
  for pid in $(pgrep -f "$pattern" 2>/dev/null || true); do
    [[ "$pid" == "$$" ]] && continue
    [[ "$pid" == "$PPID" ]] && continue
    return 0
  done
  return 1
}

cleanup_stale_processes() {
  local signal="$1"
  local -a patterns=(
    'ros2 launch vlm_explorer single_vlm_gazebo.launch.py'
    'ros2 launch vlm_explorer single_vlm_gazebo_far.launch.py'
    'ros2 launch go2_gazebo_sim single_go2w_gazebo_cfpa2.launch.py'
    'vlm_history_viewer.py'
    "${WS_DIR}/install/"
    '/opt/ros/humble/lib/cartographer_ros/cartographer_node'
    '/opt/ros/humble/lib/rviz2/rviz2 .*single_vlm_gazebo.*\\.rviz'
    '/opt/ros/humble/lib/tf2_ros/static_transform_publisher .*(__ns:=/robot|/robot/tf|/robot/tf_static)'
    '/go2_gazebo_sim/lib/go2_gazebo_sim/waypoint_mux.py'
    '/vlm_explorer/lib/vlm_explorer/vlm_coordinator_node'
    '/vlm_explorer/lib/vlm_explorer/artifact_detector_node'
    '/vlm_explorer/lib/vlm_explorer/interaction_tool_node'
    '/install/far_planner/lib/far_planner/far_planner'
    '/install/local_planner/lib/local_planner/localPlanner'
    '/install/local_planner/lib/local_planner/pathFollower'
    '/install/terrain_analysis/lib/terrain_analysis/terrainAnalysis'
    '/install/terrain_analysis_ext/lib/terrain_analysis_ext/terrainAnalysisExt'
    '/install/go2w_control/lib/go2w_control/slam_startup_gate.py'
    '/install/go2w_control/lib/go2w_control/slam_health_guard.py'
    '/install/go2w_control/lib/go2w_control/waypoint_readiness_gate.py'
    '[g]zserver'
    '(^|/)gzclient( |$)'
    '(^|/)gazebo( |$)'
  )
  local pattern
  for pattern in "${patterns[@]}"; do
    terminate_matching_processes "$pattern" "$signal"
  done
}

cleanup_targets_remaining() {
  local -a patterns=(
    'ros2 launch vlm_explorer single_vlm_gazebo.launch.py'
    'ros2 launch vlm_explorer single_vlm_gazebo_far.launch.py'
    'ros2 launch go2_gazebo_sim single_go2w_gazebo_cfpa2.launch.py'
    'vlm_history_viewer.py'
    "${WS_DIR}/install/"
    '/install/far_planner/lib/far_planner/far_planner'
    '/install/local_planner/lib/local_planner/localPlanner'
    '/install/local_planner/lib/local_planner/pathFollower'
    '/install/terrain_analysis/lib/terrain_analysis/terrainAnalysis'
    '/install/terrain_analysis_ext/lib/terrain_analysis_ext/terrainAnalysisExt'
    '/install/go2w_control/lib/go2w_control/slam_startup_gate.py'
    '/install/go2w_control/lib/go2w_control/slam_health_guard.py'
    '/install/go2w_control/lib/go2w_control/waypoint_readiness_gate.py'
    '[g]zserver'
    '(^|/)gzclient( |$)'
    '(^|/)gazebo( |$)'
  )
  local pattern
  for pattern in "${patterns[@]}"; do
    if any_matching_processes "$pattern"; then
      return 0
    fi
  done
  return 1
}

terminate_launch_job() {
  local signal="$1"
  if [[ -n "${LAUNCH_JOB_PGID}" ]] && [[ "${LAUNCH_JOB_PGID}" != "${SELF_PGID}" ]]; then
    kill "-${signal}" "--" "-${LAUNCH_JOB_PGID}" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_JOB_PID}" ]] && kill -0 "${LAUNCH_JOB_PID}" 2>/dev/null; then
    pkill "-${signal}" -P "${LAUNCH_JOB_PID}" 2>/dev/null || true
    kill "-${signal}" "${LAUNCH_JOB_PID}" 2>/dev/null || true
  fi
}

cleanup_on_exit() {
  local exit_code=$?
  local attempt
  trap - EXIT INT TERM
  terminate_launch_job INT
  sleep 1
  terminate_launch_job TERM
  for _ in 1 2 3; do
    cleanup_stale_processes TERM
    sleep 1
  done
  cleanup_stale_processes KILL
  for attempt in 1 2 3 4 5; do
    if ! cleanup_targets_remaining; then
      break
    fi
    sleep 1
    cleanup_stale_processes KILL
  done
  exit "${exit_code}"
}

trap cleanup_on_exit EXIT INT TERM

cleanup_stale_processes TERM
sleep 1
cleanup_stale_processes KILL
sleep 1

if [[ -z "${GAZEBO_MASTER_URI:-}" ]]; then
  pick_port() {
    local port
    for port in "$@"; do
      if ! ss -H -ltn "sport = :${port}" 2>/dev/null | grep -q .; then
        echo "${port}"
        return 0
      fi
    done
    return 1
  }
  GAZEBO_PORT="$(pick_port 11345 11346 11347 11348 11349 11350)" || {
    echo "ERROR: no free Gazebo master port found in 11345-11350" >&2
    exit 1
  }
  export GAZEBO_MASTER_URI="http://127.0.0.1:${GAZEBO_PORT}"
fi

if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -f "${WS_DIR}/config/fastdds_no_shm.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"
fi

VLM_PROVIDER="${VLM_PROVIDER:-auto}"
ARTIFACT_DETECTION_MODE="${ARTIFACT_DETECTION_MODE:-placeholder}"
SLAM_SOURCE="cartographer"
NAV_EXECUTION_BACKEND="${NAV_EXECUTION_BACKEND:-reactive}"
MAP_BACKEND="carto_2d"
FLORENCE2_ENABLED="${FLORENCE2_ENABLED:-false}"
MISSION_PROMPT="${MISSION_PROMPT:-find any small unusual objects on the floor}"

# CLI overrides for nav backend only (SLAM and map are fixed to cartographer + carto_2d).
FORWARD_ARGS=()
for arg in "$@"; do
  case "$arg" in
    nav_execution_backend:=*)
      NAV_EXECUTION_BACKEND="${arg#nav_execution_backend:=}"
      ;;
    *)
      FORWARD_ARGS+=("$arg")
      ;;
  esac
done

NAV_EXECUTION_BACKEND="${NAV_EXECUTION_BACKEND,,}"
case "${NAV_EXECUTION_BACKEND}" in
  reactive|rrt_star|far_rrt_star|far) ;;
  *)
    echo "ERROR: unsupported NAV_EXECUTION_BACKEND='${NAV_EXECUTION_BACKEND}' (expected reactive|rrt_star|far_rrt_star|far)" >&2
    exit 2
    ;;
esac

if [[ -z "${VLM_MODEL:-}" ]]; then
  if [[ "${VLM_PROVIDER}" == "xai" || ( "${VLM_PROVIDER}" == "auto" && -n "${XAI_API_KEY:-}" ) ]]; then
    VLM_MODEL="${XAI_VLM_MODEL:-grok-2-vision-latest}"
  else
    VLM_MODEL="gpt-4o-mini"
  fi
fi

echo "GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  echo "Using FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
fi
echo "VLM_PROVIDER=${VLM_PROVIDER}"
echo "VLM_MODEL=${VLM_MODEL}"
echo "ARTIFACT_DETECTION_MODE=${ARTIFACT_DETECTION_MODE}"
echo "SLAM_SOURCE=${SLAM_SOURCE}"
echo "NAV_EXECUTION_BACKEND=${NAV_EXECUTION_BACKEND}"
echo "MAP_BACKEND=${MAP_BACKEND}"
echo "FLORENCE2_ENABLED=${FLORENCE2_ENABLED}"
echo "MISSION_PROMPT=${MISSION_PROMPT}"

# Default all nodes to WARN; CFPA2 and nav nodes override to INFO
# via ros_arguments in their launch files.
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=WARN

# Start VLM history web viewer in background (http://localhost:8501)
python3 "${WS_DIR}/src/vlm_explorer/scripts/vlm_history_viewer.py" --port 8501 &
VIEWER_PID=$!
echo "VLM history viewer: http://localhost:8501 (pid=${VIEWER_PID})"

run_pretty_logged ros2 launch vlm_explorer single_vlm_gazebo_far.launch.py \
  vlm_model:="${VLM_MODEL}" \
  slam_source:="${SLAM_SOURCE}" \
  nav_execution_backend:="${NAV_EXECUTION_BACKEND}" \
  map_backend:="${MAP_BACKEND}" \
  florence2_enabled:="${FLORENCE2_ENABLED}" \
  florence2_goal_prompt:="small red block" \
  mission_prompt:="${MISSION_PROMPT}" \
  gui:=true \
  rviz:=true \
  "${FORWARD_ARGS[@]}" &
LAUNCH_JOB_PID=$!
LAUNCH_JOB_PGID="$(ps -o pgid= "${LAUNCH_JOB_PID}" | tr -d ' ' || true)"
wait "${LAUNCH_JOB_PID}"
