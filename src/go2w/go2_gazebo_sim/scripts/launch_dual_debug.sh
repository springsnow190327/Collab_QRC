#!/usr/bin/env bash

# End-to-end dual-robot diagnostic launcher for go2_gazebo_sim.
# Focus: deadlocks caused by obstacle/scan oversensitivity and topic/bridge faults.

set -u
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
LOG_DIR="${LOG_DIR:-/tmp/go2_dual_debug_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "${LOG_DIR}"

GUI="${GUI:-true}"
RVIZ="${RVIZ:-false}"
CLEANUP_STALE="${CLEANUP_STALE:-true}"
CMU_ENV_NAME="${CMU_ENV_NAME:-cmu_env}"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"

LAUNCH_PKG="go2_gazebo_sim"
LAUNCH_FILE="two_go2_t_world_coordinated_autonomy.launch.py"

declare -a EXTRA_ARGS=("$@")

LAUNCH_PID=""

print_header() {
  echo
  echo "================================================================"
  echo "$1"
  echo "================================================================"
}

safe_source() {
  local file="$1"
  if [[ -f "${file}" ]]; then
    # ROS setup scripts may reference unset vars; temporarily relax nounset.
    local had_u=0
    if [[ $- == *u* ]]; then
      had_u=1
      set +u
    fi
    # shellcheck disable=SC1090
    source "${file}"
    local rc=$?
    if [[ ${had_u} -eq 1 ]]; then
      set -u
    fi
    return ${rc}
  fi
  return 1
}

check_pkg() {
  local pkg="$1"
  if ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
    echo "[OK] package '${pkg}' found"
  else
    echo "[MISS] package '${pkg}' NOT found"
  fi
}

wait_for_topic_once() {
  local topic="$1"
  local timeout_s="${2:-3}"
  if timeout "${timeout_s}" ros2 topic echo --once "${topic}" >/dev/null 2>&1; then
    echo "[OK] ${topic}"
    return 0
  fi
  echo "[MISS] ${topic}"
  return 1
}

show_param() {
  local node="$1"
  local param="$2"
  local val
  val="$(ros2 param get "${node}" "${param}" 2>/dev/null | sed -n 's/^.*: //p' | tail -n 1)"
  if [[ -n "${val}" ]]; then
    echo "  ${node} :: ${param} = ${val}"
  else
    echo "  ${node} :: ${param} = <unavailable>"
  fi
}

show_nav_snapshot() {
  local ns="$1"
  local raw
  raw="$(timeout 3 ros2 topic echo --once "/${ns}/nav_status" 2>/dev/null | sed -n 's/^data: //p' | head -n 1 | sed "s/^'//; s/'$//")"
  if [[ -z "${raw}" ]]; then
    echo "[${ns}] nav_status: <missing>"
    return
  fi

  python3 - "${ns}" "${raw}" <<'PY'
import json
import sys

ns = sys.argv[1]
raw = sys.argv[2]
try:
    d = json.loads(raw)
except Exception:
    print(f"[{ns}] nav_status: unparsable -> {raw}")
    raise SystemExit(0)

mode = d.get("mode", "?")
steer = d.get("steer", "?")
min_front = d.get("min_front", "?")
blocked = d.get("blocked_sec", "?")
dist = d.get("dist_goal", "?")
ext_stop = d.get("ext_stop", "?")
goal = d.get("goal", None)
target = d.get("target", None)
cmd = d.get("cmd", None)
plan = d.get("plan_wps", None)
esc = d.get("escape", None)
zero_reason = d.get("zero_reason", None)
scan_age = d.get("scan_age_sec", None)
stop_age = d.get("stop_age_sec", None)
stop_msgs = d.get("stop_msgs", None)

print(
    f"[{ns}] mode={mode} steer={steer} dist_goal={dist} "
    f"min_front={min_front} blocked_sec={blocked} ext_stop={ext_stop} "
    f"plan_wps={plan} goal={goal} target={target} cmd={cmd} "
    f"zero_reason={zero_reason} scan_age={scan_age} stop_age={stop_age} "
    f"stop_msgs={stop_msgs} escape={esc}"
)
PY
}

show_stop_snapshot() {
  local ns="$1"
  local line
  line="$(timeout 2 ros2 topic echo --once "/${ns}/stop" 2>/dev/null | sed -n 's/^data: //p' | head -n 1)"
  if [[ -n "${line}" ]]; then
    echo "[${ns}] stop=${line}"
  else
    echo "[${ns}] stop=<missing>"
  fi
}

show_topic_hz() {
  local topic="$1"
  local line
  line="$(timeout 4 ros2 topic hz "${topic}" 2>/dev/null | tail -n 1)"
  if [[ -n "${line}" ]]; then
    echo "  ${topic} :: ${line}"
  else
    echo "  ${topic} :: <no rate sample>"
  fi
}

cleanup() {
  print_header "Cleanup"
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "Stopping launch process ${LAUNCH_PID}"
    kill "${LAUNCH_PID}" 2>/dev/null || true
    sleep 1
    kill -9 "${LAUNCH_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

print_header "Environment Setup"
echo "WS_ROOT=${WS_ROOT}"
echo "LOG_DIR=${LOG_DIR}"
echo "GUI=${GUI} RVIZ=${RVIZ} CLEANUP_STALE=${CLEANUP_STALE}"
echo "CMU_ENV_NAME=${CMU_ENV_NAME}"

# Force execution on the CMU conda env.
CONDA_SH_PATH="${CONDA_SH_PATH:-${HOME}/miniforge3/etc/profile.d/conda.sh}"
if [[ -f "${CONDA_SH_PATH}" ]]; then
  # shellcheck disable=SC1091
  source "${CONDA_SH_PATH}"
  if conda env list | awk '{print $1}' | grep -Fxq "${CMU_ENV_NAME}"; then
    conda activate "${CMU_ENV_NAME}"
  else
    echo "[FATAL] Conda env '${CMU_ENV_NAME}' not found."
    exit 1
  fi
else
  echo "[FATAL] conda.sh not found at ${CONDA_SH_PATH}"
  exit 1
fi

echo "[OK] active conda env: ${CONDA_DEFAULT_ENV:-<none>}"
if [[ "${CONDA_DEFAULT_ENV:-}" != "${CMU_ENV_NAME}" ]]; then
  echo "[FATAL] Failed to activate '${CMU_ENV_NAME}'. Current env: ${CONDA_DEFAULT_ENV:-<none>}"
  exit 1
fi

if ! safe_source "${ROS2_SETUP_BASH}"; then
  echo "[FATAL] Cannot source ${ROS2_SETUP_BASH}"
  exit 1
fi

if safe_source "${WS_ROOT}/install/setup.bash"; then
  echo "[OK] sourced ${WS_ROOT}/install/setup.bash"
else
  echo "[WARN] ${WS_ROOT}/install/setup.bash missing. Build first if packages are unresolved."
fi

if ! python - <<'PY'
import numpy
print("numpy", numpy.__version__)
PY
then
  echo "[FATAL] numpy is unavailable in '${CMU_ENV_NAME}'."
  exit 1
fi

print_header "Layer 1: Package/Environment Sanity"
for pkg in \
  go2_gazebo_sim champ_gazebo gazebo_ros go2_description go2_config champ_base \
  pointcloud_to_laserscan controller_manager robot_localization; do
  check_pkg "${pkg}"
done

echo
echo "Optional SLAM packages:"
for pkg in fast_lio point_lio_unilidar; do
  check_pkg "${pkg}"
done

print_header "Launch Dual-Robot Stack"
echo "Command:"
echo "  ros2 launch ${LAUNCH_PKG} ${LAUNCH_FILE} gui:=${GUI} rviz:=${RVIZ} cleanup_stale:=${CLEANUP_STALE} ${EXTRA_ARGS[*]}"

(
  ros2 launch "${LAUNCH_PKG}" "${LAUNCH_FILE}" \
    gui:="${GUI}" \
    rviz:="${RVIZ}" \
    cleanup_stale:="${CLEANUP_STALE}" \
    "${EXTRA_ARGS[@]}"
) > >(tee "${LOG_DIR}/launch.stdout.log") 2> >(tee "${LOG_DIR}/launch.stderr.log" >&2) &
LAUNCH_PID=$!

echo "Launch PID: ${LAUNCH_PID}"
echo "Waiting for startup (30s)..."
sleep 30

print_header "Layer 2: Bridge/Topic Wiring Checks"
for t in \
  /robot_a/registered_scan \
  /robot_a/registered_scan_reliable \
  /robot_a/scan_3d \
  /robot_b/registered_scan \
  /robot_b/registered_scan_reliable \
  /robot_b/scan_3d \
  /robot_a/cmd_vel_stamped \
  /robot_a/cmd_vel \
  /robot_b/cmd_vel_stamped \
  /robot_b/cmd_vel; do
  wait_for_topic_once "${t}" 3
done

print_header "Layer 3: Controller/Stop Sensitivity Parameters"
for ns in robot_a robot_b; do
  echo "[${ns}] default_nav tunables"
  show_param "/${ns}/default_nav" "obstacle_slow_dist"
  show_param "/${ns}/default_nav" "obstacle_stop_dist"
  show_param "/${ns}/default_nav" "wall_scan_trigger_dist"
  show_param "/${ns}/default_nav" "wall_scan_blocked_sec"
  show_param "/${ns}/default_nav" "blocked_replan_sec"
  show_param "/${ns}/default_nav" "blocked_progress_epsilon"
  show_param "/${ns}/default_nav" "planner_enabled"
  echo "[${ns}] wall_collision_checker tunables"
  show_param "/${ns}/wall_collision_checker" "safety_dist"
  show_param "/${ns}/wall_collision_checker" "check_angle_deg"
  show_param "/${ns}/wall_collision_checker" "min_valid_range"
  show_param "/${ns}/wall_collision_checker" "min_close_points"
done

print_header "Layer 4: SLAM Path Presence (Optional in Dual Launch)"
for t in /Odometry /aft_mapped_to_init /slam/odom; do
  wait_for_topic_once "${t}" 2 || true
done

print_header "Layer 5: Global Frontier Planner Health"
for t in \
  /robot_a/map \
  /robot_b/map \
  /robot_a/frontier_markers \
  /robot_b/frontier_markers \
  /robot_a/way_point_raw \
  /robot_b/way_point_raw \
  /robot_a/way_point_coord \
  /robot_b/way_point_coord; do
  wait_for_topic_once "${t}" 3
done

print_header "Layer 6: Local Planner Decision Loop (Blocked Diagnosis)"
echo "Sampling nav decisions for 120s. Logs will be saved in ${LOG_DIR}/decision_loop.log"

{
  for i in $(seq 1 60); do
    echo "----- sample ${i} @ $(date +%H:%M:%S) -----"
    show_nav_snapshot "robot_a"
    show_nav_snapshot "robot_b"
    show_stop_snapshot "robot_a"
    show_stop_snapshot "robot_b"
    sleep 2
  done
} | tee "${LOG_DIR}/decision_loop.log"

print_header "Topic Rate Snapshot"
for t in \
  /robot_a/scan_3d \
  /robot_b/scan_3d \
  /robot_a/odom/ground_truth \
  /robot_b/odom/ground_truth \
  /robot_a/nav_status \
  /robot_b/nav_status \
  /robot_a/way_point_coord \
  /robot_b/way_point_coord; do
  show_topic_hz "${t}"
done | tee "${LOG_DIR}/topic_rates.log"

print_header "Done"
echo "Artifacts:"
echo "  ${LOG_DIR}/launch.stdout.log"
echo "  ${LOG_DIR}/launch.stderr.log"
echo "  ${LOG_DIR}/decision_loop.log"
echo "  ${LOG_DIR}/topic_rates.log"
echo
echo "If blocked behavior persists, grep for mode=wall_scan / blocked_sec growth:"
echo "  rg 'mode=|blocked_sec=|ext_stop=' ${LOG_DIR}/decision_loop.log"
echo
echo "Press Ctrl-C to stop launch, or leave it running for manual inspection."

wait "${LAUNCH_PID}"
