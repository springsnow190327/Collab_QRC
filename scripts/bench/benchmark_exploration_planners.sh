#!/usr/bin/env bash
# Benchmark CFPA2, GBPlanner2/3, and MTARE/TARE under the same Nav2 MPPI executor.
#
# Default matrix:
#   envs     : demo3_mixed + generated rooms/corridors mazes
#   planners : cfpa2 gbplanner3 mtare
#   trials   : 10
#   duration : 600 sim-seconds
#
# External planner hooks:
#   GBPLANNER2_EXTERNAL_CMD  long-running command that publishes
#                            /robot_a/command/trajectory and /robot_b/command/trajectory
#   GBPLANNER3_EXTERNAL_CMD  long-running command that publishes
#                            /robot_a/command/trajectory and /robot_b/command/trajectory
#   MTARE_EXTERNAL_CMD       long-running command that publishes PointStamped
#                            waypoints on MTARE_WAYPOINT_TOPIC_A/B
#
# Without those hooks, cfpa2 runs normally, gbplanner2/gbplanner3 start the
# built-in dual UAS/Docker wrapper + adapters, and mtare uses the vendored TARE
# planner if it is built, otherwise the local autonomous fallback.
set -u -o pipefail

NUM_TRIALS="${NUM_TRIALS:-10}"
DURATION_SEC="${DURATION_SEC:-600}"
SESSION_TIME_SOURCE="${SESSION_TIME_SOURCE:-sim}"
OUT_DIR="${OUT_DIR:-/tmp/exploration_bench/$(date +%Y%m%d_%H%M%S)}"
PLANNERS="${PLANNERS:-cfpa2 gbplanner3 mtare}"
SCENE_FILTER="${SCENE_FILTER:-}"
SCENE_AREA_M2="${SCENE_AREA_M2:-384.0}"
GUI="${GUI:-false}"
RVIZ="${RVIZ:-false}"
EXTRA_ARGS="${EXTRA_ARGS:-}"
GBPLANNER3_EXTERNAL_CMD="${GBPLANNER3_EXTERNAL_CMD:-}"
GBPLANNER2_EXTERNAL_CMD="${GBPLANNER2_EXTERNAL_CMD:-}"
MTARE_EXTERNAL_CMD="${MTARE_EXTERNAL_CMD:-}"
MTARE_WAYPOINT_TOPIC_A="${MTARE_WAYPOINT_TOPIC_A:-/robot_a/mtare/way_point}"
MTARE_WAYPOINT_TOPIC_B="${MTARE_WAYPOINT_TOPIC_B:-/robot_b/mtare/way_point}"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
UAS_REPO_ROOT="${UAS_REPO_ROOT:-$HOME/Research/uas_deploy/unified_autonomy_stack}"
GBPLANNER_COMPOSE_FILE="${GBPLANNER_COMPOSE_FILE:-${GBPLANNER3_COMPOSE_FILE:-${WS_DIR}/scripts/sim/gbplanner3_mujoco/compose/docker-compose.collab_qrc_dual.yml}}"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"
MJK_DIR="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco"
GENERATED_DIR="${MJK_DIR}/generated"

safe_source() { set +u; source "$1"; set -u; }

if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  safe_source "${HOME}/miniforge3/etc/profile.d/conda.sh"
  conda activate cmu_env
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate cmu_env
fi

safe_source "${ROS2_SETUP_BASH}"
safe_source "${WS_DIR}/install/setup.bash"
export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"

if [[ "${SESSION_TIME_SOURCE}" != "sim" && "${SESSION_TIME_SOURCE}" != "wall" ]]; then
  echo "ERROR: SESSION_TIME_SOURCE must be 'sim' or 'wall' (got '${SESSION_TIME_SOURCE}')" >&2
  exit 1
fi

mkdir -p "${OUT_DIR}" "${GENERATED_DIR}"
python3 "${WS_DIR}/scripts/bench/generate_exploration_mazes.py" \
  --output-dir "${GENERATED_DIR}" >/tmp/exploration_bench_generate_mazes.log

SCENES=(
  "demo3_mixed:${MJK_DIR}/demo3_mixed.xml"
  "maze_rooms_seed101:${GENERATED_DIR}/demo3_mixed_rooms_seed101.xml"
  "maze_corridors_seed202:${GENERATED_DIR}/demo3_mixed_corridors_seed202.xml"
)

if [[ -n "${SCENE_FILTER}" ]]; then
  FILTERED_SCENES=()
  for scene_entry in "${SCENES[@]}"; do
    env_name="${scene_entry%%:*}"
    if [[ " ${SCENE_FILTER} " == *" ${env_name} "* ]]; then
      FILTERED_SCENES+=("${scene_entry}")
    fi
  done
  if [[ "${#FILTERED_SCENES[@]}" -eq 0 ]]; then
    echo "ERROR: SCENE_FILTER matched no scenes: ${SCENE_FILTER}" >&2
    exit 1
  fi
  SCENES=("${FILTERED_SCENES[@]}")
fi

cleanup_procs() {
  if [[ -f "${GBPLANNER_COMPOSE_FILE}" ]] && command -v docker >/dev/null 2>&1; then
    UAS_REPO_ROOT="${UAS_REPO_ROOT}" \
    COLLAB_QRC_ROOT="${WS_DIR}" \
    docker compose -f "${GBPLANNER_COMPOSE_FILE}" --profile launch down --remove-orphans >/tmp/exploration_bench_gbplanner_down.log 2>&1 || true
  fi
  pkill -f 'ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio_mixed' 2>/dev/null || true
  pkill -f 'mujoco_ros2_control/mujoco_ros2_control' 2>/dev/null || true
  pkill -f 'cfpa2_coordinator_node' 2>/dev/null || true
  pkill -f 'gbplanner_to_waypoint_adapter.py' 2>/dev/null || true
  pkill -f 'external_point_to_waypoint_coord_adapter.py' 2>/dev/null || true
  pkill -f 'tare_planner_node' 2>/dev/null || true
  pkill -f 'tare_waypoint_watchdog.py' 2>/dev/null || true
  pkill -f 'fastlio_mapping' 2>/dev/null || true
  pkill -f 'session_reporter.py' 2>/dev/null || true
  pkill -f 'exploration_metrics_logger.py' 2>/dev/null || true
  pkill -f 'controller_server' 2>/dev/null || true
  pkill -f 'planner_server' 2>/dev/null || true
  pkill -f 'bt_navigator' 2>/dev/null || true
  sleep 1
  pkill -9 -f 'mujoco_ros2_control/mujoco_ros2_control' 2>/dev/null || true
  pkill -9 -f 'fastlio_mapping' 2>/dev/null || true
  rm -f /dev/shm/sem.fastrtps_* /dev/shm/sem.fastdds_* /dev/shm/fastrtps_* /dev/shm/fastdds_* 2>/dev/null || true
  sleep 2
}

trap 'echo "[benchmark] interrupted, cleaning up"; cleanup_procs; exit 130' INT TERM

cat >"${OUT_DIR}/benchmark_config.json" <<EOF
{
  "num_trials": ${NUM_TRIALS},
  "duration_sec": ${DURATION_SEC},
  "session_time_source": "${SESSION_TIME_SOURCE}",
  "planners": "$(printf '%s' "${PLANNERS}")",
  "scene_filter": "$(printf '%s' "${SCENE_FILTER}")",
  "scene_area_m2": ${SCENE_AREA_M2},
  "gui": "${GUI}",
  "rviz": "${RVIZ}",
  "extra_args": "${EXTRA_ARGS}",
  "gbplanner2_external_cmd": "$(printf '%s' "${GBPLANNER2_EXTERNAL_CMD}")",
  "gbplanner3_external_cmd": "$(printf '%s' "${GBPLANNER3_EXTERNAL_CMD}")",
  "mtare_external_cmd": "$(printf '%s' "${MTARE_EXTERNAL_CMD}")"
}
EOF

echo "================================================================"
echo "  Exploration planner benchmark"
echo "  workspace    : ${WS_DIR}"
echo "  out dir      : ${OUT_DIR}"
echo "  trials       : ${NUM_TRIALS}"
if [[ "${SESSION_TIME_SOURCE}" == "sim" ]]; then
echo "  duration/run : ${DURATION_SEC} sim-s"
else
echo "  duration/run : ${DURATION_SEC} wall-s"
fi
echo "  time source  : ${SESSION_TIME_SOURCE}"
echo "  planners     : ${PLANNERS}"
if [[ -n "${SCENE_FILTER}" ]]; then
echo "  scene filter : ${SCENE_FILTER}"
fi
echo "  scenes       : ${#SCENES[@]}"
echo "================================================================"

if [[ " ${PLANNERS} " == *" gbplanner3 "* && -z "${GBPLANNER3_EXTERNAL_CMD}" ]]; then
  echo "  gbplanner3  : using built-in dual UAS/Docker wrapper"
fi
if [[ " ${PLANNERS} " == *" gbplanner2 "* && -z "${GBPLANNER2_EXTERNAL_CMD}" ]]; then
  echo "  gbplanner2  : using built-in dual UAS/Docker wrapper"
fi

if [[ " ${PLANNERS} " == *" mtare "* && -z "${MTARE_EXTERNAL_CMD}" ]]; then
  echo "  mtare       : using vendor TARE/MTARE if built, otherwise local autonomous fallback"
fi

for scene_entry in "${SCENES[@]}"; do
  env_name="${scene_entry%%:*}"
  scene_path="${scene_entry#*:}"
  if [[ ! -f "${scene_path}" ]]; then
    echo "ERROR: missing scene ${scene_path}" >&2
    exit 1
  fi

  for planner in ${PLANNERS}; do
    for trial in $(seq -w 1 "${NUM_TRIALS}"); do
      trial_dir="${OUT_DIR}/${env_name}/${planner}/trial_${trial}"
      mkdir -p "${trial_dir}"
      trial_log="${trial_dir}/launch.log"
      cat >"${trial_dir}/launch_args.txt" <<EOF
exploration_planner:=${planner}
mujoco_model_path:=${scene_path}
scene_area_m2:=${SCENE_AREA_M2}
session_duration_sec:=${DURATION_SEC}
session_time_source:=${SESSION_TIME_SOURCE}
session_output_dir:=${trial_dir}
metrics_output_dir:=${trial_dir}
experiment_name:=${env_name}_${planner}_trial_${trial}
nav_backend_a:=nav2_mppi
nav_backend_b:=nav2_mppi
holonomic_profile_a:=se2_holonomic
holonomic_profile_b:=se2_holonomic
record_nav_bags:=false
EOF

      echo
      echo "---------- ${env_name} / ${planner} / trial_${trial} ----------"
      echo "  scene: ${scene_path}"
      echo "  dir  : ${trial_dir}"

      cleanup_procs
      if [[ "${planner}" == "gbplanner2" && -z "${GBPLANNER2_EXTERNAL_CMD}" ]] || \
         [[ "${planner}" == "gbplanner3" && -z "${GBPLANNER3_EXTERNAL_CMD}" ]]; then
        echo "  preparing ${planner} UAS workspace before starting sim"
        if ! UAS_REPO_ROOT="${UAS_REPO_ROOT}" \
          "${WS_DIR}/scripts/sim/gbplanner3_mujoco/prepare_gbplanner_ref.sh" "${planner}" \
          >"${trial_dir}/gbplanner_prepare.log" 2>&1; then
          echo "  ERROR: ${planner} prepare failed; tail:"
          tail -40 "${trial_dir}/gbplanner_prepare.log" | sed 's/^/    /'
          echo "prepare_failed" >"${trial_dir}/exit_code.txt"
          continue
        fi
      fi
      outer_timeout=$((DURATION_SEC * 3 + 180))
      optional_launch_args=()
      if [[ -n "${GBPLANNER3_EXTERNAL_CMD}" ]]; then
        optional_launch_args+=("gbplanner3_external_cmd:=${GBPLANNER3_EXTERNAL_CMD}")
      fi
      if [[ -n "${GBPLANNER2_EXTERNAL_CMD}" ]]; then
        optional_launch_args+=("gbplanner2_external_cmd:=${GBPLANNER2_EXTERNAL_CMD}")
      fi
      if [[ -n "${MTARE_EXTERNAL_CMD}" ]]; then
        optional_launch_args+=("mtare_external_cmd:=${MTARE_EXTERNAL_CMD}")
      fi
      set +e
      timeout --signal=SIGTERM --kill-after=15 "${outer_timeout}" \
        "${WS_DIR}/scripts/launch/nav_test_demo3_mixed.sh" \
          gui:="${GUI}" \
          rviz:="${RVIZ}" \
          exploration_planner:="${planner}" \
          mujoco_model_path:="${scene_path}" \
          scene_area_m2:="${SCENE_AREA_M2}" \
          session_duration_sec:="${DURATION_SEC}" \
          session_time_source:="${SESSION_TIME_SOURCE}" \
          session_output_dir:="${trial_dir}" \
          metrics_output_dir:="${trial_dir}" \
          experiment_name:="${env_name}_${planner}_trial_${trial}" \
          nav_backend_a:=nav2_mppi \
          nav_backend_b:=nav2_mppi \
          holonomic_profile_a:=se2_holonomic \
          holonomic_profile_b:=se2_holonomic \
          record_nav_bags:=false \
          mtare_waypoint_topic_a:="${MTARE_WAYPOINT_TOPIC_A}" \
          mtare_waypoint_topic_b:="${MTARE_WAYPOINT_TOPIC_B}" \
          "${optional_launch_args[@]}" \
          ${EXTRA_ARGS} \
          >"${trial_log}" 2>&1
      rc=$?
      set -u
      echo "${rc}" >"${trial_dir}/exit_code.txt"
      echo "  exit: ${rc}"
      if [[ ! -f "${trial_dir}/robot_a.json" || ! -f "${trial_dir}/robot_b.json" ]]; then
        echo "  WARN: session JSON missing; tail of log:"
        tail -20 "${trial_log}" | sed 's/^/    /'
      fi
    done
  done
done

cleanup_procs

python3 "${WS_DIR}/scripts/bench/aggregate_exploration_benchmark.py" "${OUT_DIR}" || true
echo "Benchmark output: ${OUT_DIR}"
