#!/usr/bin/env bash
# Demo3 scene (24×16 m flat) + GBPlanner3 (Docker Noetic) + Collab_QRC MuJoCo.
#
# Architecture:
#   Host Humble:
#     MuJoCo (demo3_go2_real.xml — Menagerie body) + Fast-LIO + CHAMP   (via nav_test_fastlio.sh,
#                                                    with explore:=false)
#     + static_transform_publishers: base_link→lidar, world→map
#                              │
#                              │  /robot/Odometry, /robot/cloud_registered_body, /robot/tf
#                              ▼
#   Docker Noetic:
#     roscore + ros1_bridge + gbplanner3 + PCI
#
# Usage:
#   ./scripts/launch/nav_test_gbplanner_demo3.sh                 # full start
#   ./scripts/launch/nav_test_gbplanner_demo3.sh stop            # kill everything
#   ./scripts/launch/nav_test_gbplanner_demo3.sh start_mission   # trigger automatic_planning
#
# Prereqs: UAS Docker images built. See scripts/sim/gbplanner3_mujoco/README.md.

set -u -o pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
UAS_REPO_ROOT="${UAS_REPO_ROOT:-$HOME/Research/uas_deploy/unified_autonomy_stack}"
COLLAB_QRC_ROOT="${COLLAB_QRC_ROOT:-$WS_DIR}"
OVERLAY_COMPOSE="${COLLAB_QRC_ROOT}/scripts/sim/gbplanner3_mujoco/compose/docker-compose.collab_qrc.yml"
SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/demo3_go2_real.xml"
SCENE_AREA_M2="${SCENE_AREA_M2:-384}"

# ===== Subcommands =====
case "${1:-}" in
  stop)
    echo "==> Stopping gbplanner3 containers..."
    cd "$UAS_REPO_ROOT"
    UAS_REPO_ROOT="$UAS_REPO_ROOT" COLLAB_QRC_ROOT="$COLLAB_QRC_ROOT" \
      make stop DOCKER_COMPOSE_FILE="$OVERLAY_COMPOSE" 2>&1 | tail -10
    echo "==> Killing host static_transform_publishers..."
    pkill -f "static_transform_publisher.*base_to_lidar_alias_gbplanner" 2>/dev/null || true
    pkill -f "static_transform_publisher.*world_to_map_tf_gbplanner" 2>/dev/null || true
    pkill -f "topic_tools.relay.*tf" 2>/dev/null || true
    source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"
    exit 0
    ;;
  start_mission)
    echo "==> Triggering /planner_control_interface/std_srvs/automatic_planning ..."
    GBP=$(docker ps --format "{{.Names}}" | grep gbplanner | head -1)
    [[ -z "$GBP" ]] && { echo "ERROR: no gbplanner container running"; exit 1; }
    docker exec -t "$GBP" bash -lc '
      source /opt/ros/noetic/setup.bash
      rosservice call /planner_control_interface/std_srvs/automatic_planning "{}"
    '
    exit 0
    ;;
esac

# ===== Full start =====
[[ -f "$SCENE" ]] || { echo "ERROR: scene not found: $SCENE"; exit 1; }

echo "==> [1/4] Preflight kill of any prior sim..."
source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"
# Also stop pre-existing UAS containers
if docker ps --format "{{.Names}}" | grep -q unified_autonomy_stack; then
  echo "    Stopping pre-existing UAS containers..."
  cd "$UAS_REPO_ROOT" && \
    UAS_REPO_ROOT="$UAS_REPO_ROOT" COLLAB_QRC_ROOT="$COLLAB_QRC_ROOT" \
    make stop DOCKER_COMPOSE_FILE="$OVERLAY_COMPOSE" 2>/dev/null | tail -3
fi

echo ""
echo "==> [2/4] Start Docker stack (Noetic gbplanner3 + ros1_bridge)..."
cd "$UAS_REPO_ROOT"
nohup env \
  UAS_REPO_ROOT="$UAS_REPO_ROOT" \
  COLLAB_QRC_ROOT="$COLLAB_QRC_ROOT" \
  DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  make launch DOCKER_COMPOSE_FILE="$OVERLAY_COMPOSE" \
  > /tmp/gbplanner3_demo3_uas.log 2>&1 &
disown
echo "    UAS log: /tmp/gbplanner3_demo3_uas.log"

echo "==> Waiting up to 60s for gbplanner_node in Noetic..."
for i in $(seq 1 60); do
  if docker ps --format "{{.Names}}" | grep -q gbplanner; then
    GBP=$(docker ps --format "{{.Names}}" | grep gbplanner | head -1)
    if docker exec "$GBP" bash -lc 'source /opt/ros/noetic/setup.bash && rosnode list 2>/dev/null | grep -q gbplanner_node' 2>/dev/null; then
      echo "    ✓ gbplanner_node up"
      break
    fi
  fi
  sleep 1
  [[ $i -eq 60 ]] && { echo "    ✗ TIMEOUT (no gbplanner_node)"; exit 1; }
done

# ===== Start static TF publishers BEFORE nav_test_fastlio.sh exec
# (the script will exec, replacing the shell — so backgrounds must be set up now)
echo ""
echo "==> [3/4] Source env + start static TF aliases (base_link→lidar, world→map)..."
safe_source() { set +u; source "$1"; set -u; }
if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  safe_source "${HOME}/miniforge3/etc/profile.d/conda.sh"
  conda activate cmu_env
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate cmu_env
fi
safe_source /opt/ros/humble/setup.bash
safe_source "${WS_DIR}/install/setup.bash"

# CRITICAL fix: mujoco_ros2_control loads MuJoCo sensor plugins from cmu_env's
# site-packages/mujoco/plugin/, but those plugins are linked against
# libmujoco.so.3.6.0 which cmu_env's conda activation does NOT add to
# LD_LIBRARY_PATH. Without this export, libsensor.so fails to dlopen →
# MuJoCo can't bind <sensor> blocks in the MJCF → no IMU / LiDAR / pose
# topics published → Fast-LIO has no input → "TF disconnect (odom ↔
# base_link)" downstream.
if [[ -n "${CONDA_PREFIX:-}" ]] && [[ -d "${CONDA_PREFIX}/lib/python3.10/site-packages/mujoco" ]]; then
  export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib/python3.10/site-packages/mujoco:${LD_LIBRARY_PATH:-}"
  echo "    added MuJoCo plugin lib path: ${CONDA_PREFIX}/lib/python3.10/site-packages/mujoco"
fi

# base_link → lidar (gbplanner expects "lidar" sensor frame; our MJCF has "mid360_link")
# Quat from euler (0, 0.2269, 0) for the Mid-360 pitch: w=cos(0.11345), y=sin(0.11345)
#   approximately (w=0.99357, y=0.11321) → xyzw=[0, 0.113, 0, 0.994]
nohup ros2 run tf2_ros static_transform_publisher \
  --x 0.16143 --y 0.0 --z 0.12262 \
  --qx 0.0 --qy 0.11321 --qz 0.0 --qw 0.99357 \
  --frame-id base_link --child-frame-id lidar \
  --ros-args -r __node:=base_to_lidar_alias_gbplanner \
  -r /tf:=/robot/tf -r /tf_static:=/robot/tf_static \
  > /tmp/gbplanner3_tf_lidar.log 2>&1 &
disown

# world → map (gbplanner / voxblox use "world"; Collab_QRC uses "map")
nohup ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
  --frame-id world --child-frame-id map \
  --ros-args -r __node:=world_to_map_tf_gbplanner \
  -r /tf:=/robot/tf -r /tf_static:=/robot/tf_static \
  > /tmp/gbplanner3_tf_world_map.log 2>&1 &
disown
echo "    static TF nodes started in background"

# NOTE on TF: Humble side stays namespaced (/robot/tf, /robot/tf_static —
# per CLAUDE.md golden rule 10 — multi-robot ready). dynamic_bridge will
# bridge /robot/tf → /robot/tf to the Noetic container. The Noetic-side
# relay /robot/tf → /tf is launched INSIDE the gbplanner container (see
# docker-compose.collab_qrc.yml) so gbplanner's default tf2 lookups work
# without polluting Humble's global /tf.

# ===== Launch Collab_QRC nav stack with CFPA2 disabled =====
# PREFLIGHT_KILL=0 disables nav_test_fastlio.sh's own preflight_kill (we
# already ran it in step [1]). Otherwise a second run hits pipefail in
# _preflight_stop_ros2_daemon when no daemon is alive — combined with the
# script's set -euo pipefail this aborts nav_test_fastlio.sh silently.
echo ""
echo "==> [4/4] Start MuJoCo + Fast-LIO via nav_test_fastlio.sh (explore:=false)..."
cd "$WS_DIR"
exec env PREFLIGHT_KILL=0 "${WS_DIR}/scripts/launch/nav_test_fastlio.sh" \
  "mujoco_model_path:=${SCENE}" \
  "scene_area_m2:=${SCENE_AREA_M2}" \
  "has_wheels:=false" \
  "two_way_drive:=false" \
  "explore:=false" \
  "$@"
