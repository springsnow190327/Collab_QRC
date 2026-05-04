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

# ── Auto-record helper for real-robot runs ──────────────────────────
# setup_rosbag_recording <robot> <slam> <nav> <mode> [ns]
#   mode: essential | full
#   ns:   robot namespace used to template topic names (default: robot)
# Spawns `ros2 bag record` in the background. Sets BAG_PID + BAG_DIR for
# the caller's cleanup hook. Caller must SIGINT the bag before SIGKILLing
# the rest of the stack — SIGTERM/SIGKILL leaves an unfinalized bag.
#
# Output dir: ${BAG_DIR_ROOT:-$REPO_ROOT/bags}/realrun_<robot>_<slam>_<nav>_<stamp>/
# Storage:    MCAP, 2 GB max per file (rosbag2 splits at the cap).
setup_rosbag_recording() {
  local robot="${1:?robot required}"
  local slam="${2:?slam required}"
  local nav="${3:?nav required}"
  local mode="${4:-essential}"
  local ns="${5:-robot}"

  local repo_root="${REPO_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." &>/dev/null && pwd)}"
  local out_root="${BAG_DIR_ROOT:-$repo_root/bags}"
  mkdir -p "$out_root"

  local stamp; stamp="$(date +%Y%m%d_%H%M%S)"
  local bag_dir="$out_root/realrun_${robot}_${slam}_${nav}_${stamp}"

  # rosbag2 creates the dir itself; we hand it a non-existing path.
  # Manifest gets written immediately afterwards.
  local manifest_pending="${out_root}/.${stamp}.manifest"
  {
    echo "robot=$robot"
    echo "slam=$slam"
    echo "nav=$nav"
    echo "mode=$mode"
    echo "ns=$ns"
    echo "git_sha=$(git -C "$repo_root" rev-parse HEAD 2>/dev/null || echo unknown)"
    echo "git_branch=$(git -C "$repo_root" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
    echo "git_dirty=$(git -C "$repo_root" diff --quiet 2>/dev/null && echo no || echo yes)"
    echo "hostname=$(hostname)"
    echo "user=$(whoami)"
    echo "date=$(date -Iseconds)"
    echo "args=$*"
  } > "$manifest_pending"

  local topics; topics=( $(_bag_topics_for "$mode" "$ns") )

  # 2 GB = 2147483648 bytes. rosbag2 rolls into <bag>_1.mcap, _2.mcap, …
  ros2 bag record \
      --output "$bag_dir" \
      --storage mcap \
      --max-bag-size 2147483648 \
      "${topics[@]}" \
      >"${out_root}/.${stamp}.record.log" 2>&1 &
  BAG_PID=$!
  BAG_DIR="$bag_dir"

  # Give rosbag2 a moment to create the directory before we move the
  # manifest into it. If it never appears (e.g. ros2 missing), keep the
  # manifest at the root so the user can still see what was attempted.
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    [[ -d "$bag_dir" ]] && break
    sleep 0.3
  done
  if [[ -d "$bag_dir" ]]; then
    mv -f "$manifest_pending" "$bag_dir/manifest.txt"
    mv -f "${out_root}/.${stamp}.record.log" "$bag_dir/record.log" 2>/dev/null || true
  fi

  echo "[record] $bag_dir (pid=$BAG_PID, mode=$mode, topics=${#topics[@]})"
}

# Internal: prints the topic list for a given mode + namespace, one per line.
_bag_topics_for() {
  local mode="$1"
  local ns="$2"

  # Essential set — nav/control/state. ~50 KB/s on the real robot.
  local essential=(
    # cmd_vel family (every variant the mux/shield touches)
    "/${ns}/cmd_vel"
    "/${ns}/cmd_vel_auto"
    "/${ns}/cmd_vel_manual"
    "/${ns}/cmd_vel_stamped"
    "/${ns}/cmd_vel_legged"
    "/${ns}/control_source"

    # nav state
    "/${ns}/odom/nav"
    "/Odometry"
    "/${ns}/goal_pose"
    "/${ns}/way_point_coord"

    # planner outputs
    "/${ns}/plan"
    "/${ns}/local_plan"
    "/${ns}/transformed_global_plan"
    "/${ns}/unsmoothed_plan"
    "/${ns}/local_path"
    "/${ns}/path"
    "/${ns}/planned_path"

    # costmaps + map
    "/${ns}/global_costmap/costmap"
    "/${ns}/global_costmap/costmap_updates"
    "/${ns}/local_costmap/costmap"
    "/${ns}/local_costmap/costmap_updates"
    "/${ns}/local_costmap/published_footprint"
    "/${ns}/global_costmap/published_footprint"
    "/${ns}/map"
    "/${ns}/map_prob"
    "/${ns}/map_updates"
    "/${ns}/map_prob_updates"

    # exploration
    "/${ns}/frontier_markers"
    "/${ns}/frontier_cylinders"
    "/${ns}/frontier_replan"
    "/${ns}/cfpa2_status"
    "/${ns}/free_cells_vis_array"
    "/${ns}/final_goal_marker"
    "/${ns}/final_goal_marker_array"

    # BT + lifecycle
    "/${ns}/behavior_tree_log"
    "/${ns}/bt_navigator/transition_event"
    "/${ns}/controller_server/transition_event"
    "/${ns}/planner_server/transition_event"
    "/${ns}/behavior_server/transition_event"

    # input
    "/joy"
    "/${ns}/joy"

    # TF (both global and namespaced — adapter writes to /<ns>/tf)
    "/tf"
    "/tf_static"
    "/${ns}/tf"
    "/${ns}/tf_static"

    # IMU (small, useful for retrospective tilt analysis)
    "/livox/imu"
  )

  # Full set — adds raw point clouds + octomap. ~5 MB/s on the real robot.
  local full=(
    "${essential[@]}"
    "/livox/lidar"
    "/cloud_registered"
    "/cloud_registered_body"
    "/cloud_effected"
    "/Laser_map"
    "/${ns}/octomap_binary"
    "/${ns}/octomap_full"
    "/${ns}/octomap_point_cloud_centers"
  )

  case "$mode" in
    full)      printf '%s\n' "${full[@]}" ;;
    essential) printf '%s\n' "${essential[@]}" ;;
    *)         echo "ERROR: bag mode must be essential|full (got '$mode')" >&2; return 1 ;;
  esac
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
