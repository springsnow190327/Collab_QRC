#!/usr/bin/env bash

setup_run_logging() {
  local run_name="${1:-run}"
  local base_dir="${ROS_LOG_ROOT:-${ROS_LOG_DIR:-/tmp/ros_logs}}"
  local ts

  ts="$(date +%Y%m%d_%H%M%S)"
  export ROS_LOG_ROOT="${base_dir}"
  export ROS_LOG_SESSION_DIR="${ROS_LOG_ROOT}/sessions/${run_name}_${ts}"
  export ROS_LOG_DIR="${ROS_LOG_SESSION_DIR}/ros"
  mkdir -p "${ROS_LOG_DIR}" "${ROS_LOG_SESSION_DIR}/stages"
  ln -sfn "${ROS_LOG_SESSION_DIR}" "${ROS_LOG_ROOT}/latest_${run_name}"

  export PYTHONUNBUFFERED=1
  export RCUTILS_COLORIZED_OUTPUT="${RCUTILS_COLORIZED_OUTPUT:-1}"
  export RCUTILS_CONSOLE_OUTPUT_FORMAT="${RCUTILS_CONSOLE_OUTPUT_FORMAT:-[{severity}] [{name}]: {message}}"

  cat <<EOF
============================================================
  Run:        ${run_name}
  Session:    ${ROS_LOG_SESSION_DIR}
  ROS logs:   ${ROS_LOG_DIR}
  Console:    ${ROS_LOG_SESSION_DIR}/console.log
  Pipeline:   ${ROS_LOG_SESSION_DIR}/pipeline.log
  Stages:     ${ROS_LOG_SESSION_DIR}/stages/
============================================================
EOF
}

run_pretty_logged() {
  if [[ -z "${ROS_LOG_SESSION_DIR:-}" ]]; then
    setup_run_logging "run"
  fi

  local console_log="${ROS_LOG_SESSION_DIR}/console.log"
  local command_log="${ROS_LOG_SESSION_DIR}/command.sh"
  local helper_dir
  local formatter
  helper_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  formatter="${helper_dir}/pretty_ros_log.py"

  printf '#!/usr/bin/env bash\n' > "${command_log}"
  printf '%q ' "$@" >> "${command_log}"
  printf '\n' >> "${command_log}"
  chmod +x "${command_log}"

  stdbuf -oL -eL "$@" 2>&1 \
    | tee "${console_log}" \
    | python3 "${formatter}"

  local statuses=("${PIPESTATUS[@]}")
  return "${statuses[0]}"
}
