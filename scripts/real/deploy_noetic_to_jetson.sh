#!/usr/bin/env bash
# ⚠️ SUPERSEDED (2026-05-20) by the consolidated jetson_ws/ workflow.
# This script targets the OLD ~/noetic_fastlio_ws/ split workspace and rsyncs
# from src/vendor/{point_lio_ros1,trav_pipeline_ros1,elevation_mapping_cupy_ros1,
# fast_lio_ros1} — directories that were REMOVED when jetson_ws/ became the
# canonical deployment snapshot. For the current full-autonomy onboard stack on
# the Go2 Orin NX (/home/unitree/autonomous_exploration_zhu/), use jetson_ws/ —
# see jetson_ws/README.md for deploy + build + run. Kept here for historical
# reference of the FAST-LIO/Point-LIO bring-up sequence only.
#
# deploy_noetic_to_jetson.sh — rsync ROS 1 FAST-LIO2 + livox_ros_driver2 to the
# Go2 Jetson into a SEPARATE Noetic catkin workspace (~/noetic_fastlio_ws/).
#
# Sister to deploy_to_jetson.sh (which targets the Foxy ~/onboard_ws/). Kept as
# a parallel script because:
#   - Mixing ROS 1 catkin + ROS 2 colcon trees in one workspace is fragile
#     (build/, devel/, install/ semantics differ; sourcing order matters).
#   - livox_ros_driver2 needs to be built TWICE on the Jetson (./build.sh ROS1
#     for Noetic vs ROS2 for Foxy); a SECOND copy avoids cross-flavour build
#     artifact conflicts in the same source dir.
#
# What ships:
#   - src/vendor/fast_lio_ros1/         → noetic_fastlio_ws/src/FAST_LIO/
#     (HKU-MARS/FAST_LIO master = FAST-LIO2; package name is "fast_lio" — the
#      dir rename is purely organisational.)
#   - src/vendor/livox_ros_driver2/     → noetic_fastlio_ws/src/livox_ros_driver2/
#   - scripts/real/onboard_fastlio_noetic.sh → noetic_fastlio_ws/scripts/
#
# What stays on the laptop / in the existing Foxy ws:
#   - Livox-SDK2 — already built under ~/onboard_ws/install/Livox-SDK2/; the
#     Noetic livox_ros_driver2 build re-uses it via Livox_SDK_DIR.
#   - Nav2 / CFPA2 / RViz on the laptop side.
#
# Usage:
#   ./deploy_noetic_to_jetson.sh                             # default 192.168.123.18
#   ./deploy_noetic_to_jetson.sh user=unitree pass=123 host=192.168.123.18
#   ./deploy_noetic_to_jetson.sh dry                         # show diff only
#   ./deploy_noetic_to_jetson.sh ws=/home/unitree/noetic_fastlio_ws

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/../.." &> /dev/null && pwd )"

# ── Defaults ─────────────────────────────────────────────────────────
JETSON_USER="unitree"
JETSON_PASS="${JETSON_PASS:-}"
JETSON_HOST="${GO2W_JETSON_IP:-192.168.123.18}"
JETSON_WS="/home/unitree/noetic_fastlio_ws"
FOXY_WS="/home/unitree/onboard_ws"   # Livox-SDK2 install lives here (re-used)
DRY_RUN="false"

for arg in "$@"; do
  case "$arg" in
    user=*)    JETSON_USER="${arg#user=}" ;;
    pass=*)    JETSON_PASS="${arg#pass=}" ;;
    host=*)    JETSON_HOST="${arg#host=}" ;;
    ws=*)      JETSON_WS="${arg#ws=}" ;;
    foxy_ws=*) FOXY_WS="${arg#foxy_ws=}" ;;
    dry|dry_run|dryrun) DRY_RUN="true" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

case "$DRY_RUN" in true|false) ;; *) echo "ERROR: dry must be true|false" >&2; exit 1 ;; esac

# ── Tools ────────────────────────────────────────────────────────────
command -v rsync &>/dev/null || { echo "ERROR: rsync not installed" >&2; exit 1; }
command -v sshpass &>/dev/null || { echo "ERROR: sshpass not installed (apt install sshpass)" >&2; exit 1; }
if [[ -z "$JETSON_PASS" ]]; then
  echo "ERROR: JETSON_PASS not set." >&2
  echo "  Either: export JETSON_PASS=<robot-password>" >&2
  echo "  Or:     pass it on the cmdline: pass=<password> $0 ..." >&2
  exit 1
fi

SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=5 -o BatchMode=no)
SSH="sshpass -p $JETSON_PASS ssh ${SSH_OPTS[*]} ${JETSON_USER}@${JETSON_HOST}"
RSYNC_RSH="sshpass -p $JETSON_PASS ssh ${SSH_OPTS[*]}"

# ── Reachability ─────────────────────────────────────────────────────
echo ""
echo "################################################"
echo "  Deploying Noetic FAST-LIO2 tree to Jetson"
echo "    target   : ${JETSON_USER}@${JETSON_HOST}"
echo "    ws       : ${JETSON_WS}"
echo "    foxy ws  : ${FOXY_WS}  (Livox-SDK2 re-used from here)"
echo "    dry_run  : $DRY_RUN"
echo "################################################"
echo ""

if ! ping -c 2 -W 2 "$JETSON_HOST" &>/dev/null; then
  echo "ERROR: Cannot reach $JETSON_HOST. Check Ethernet (./connect_ethernet.sh first)." >&2
  exit 1
fi

if ! sshpass -p "$JETSON_PASS" ssh "${SSH_OPTS[@]}" "${JETSON_USER}@${JETSON_HOST}" "echo OK" &>/dev/null; then
  echo "ERROR: SSH auth to ${JETSON_USER}@${JETSON_HOST} failed (wrong password?)." >&2
  exit 1
fi
echo "  SSH + ping OK"

# Sanity-check that the Foxy Livox-SDK2 install exists — the Noetic
# livox_ros_driver2 ROS1 build will fail without it.
if ! sshpass -p "$JETSON_PASS" ssh "${SSH_OPTS[@]}" "${JETSON_USER}@${JETSON_HOST}" \
       "test -d ${FOXY_WS}/install/Livox-SDK2/include" &>/dev/null; then
  echo "WARN: ${FOXY_WS}/install/Livox-SDK2/ not found on Jetson." >&2
  echo "      Run ./deploy_to_jetson.sh first (it ships Livox-SDK2 source and"  >&2
  echo "      its build is documented in the Foxy deploy 'Next steps')."         >&2
  echo "      Continuing — but the Noetic livox_ros_driver2 build WILL fail."    >&2
fi

# ── Ensure remote workspace exists ───────────────────────────────────
echo "  ensuring ${JETSON_WS}/{src,scripts} exist..."
$SSH "mkdir -p ${JETSON_WS}/{src,scripts}"

# ── Common rsync options ─────────────────────────────────────────────
RSYNC_OPTS=(
  -avz --update
  --exclude=build/ --exclude=devel/ --exclude=install/ --exclude=log/
  --exclude=.git/ --exclude=__pycache__/ --exclude='*.pyc'
  --exclude=.colcon/ --exclude=COLCON_IGNORE
)
[[ "$DRY_RUN" == "true" ]] && RSYNC_OPTS+=(--dry-run)

# Helper: rsync a tree from repo to the Jetson workspace.
#   $1 = source path under REPO_ROOT
#   $2 = destination path under JETSON_WS
rsync_to_jetson() {
  local src="$REPO_ROOT/$1"
  local dst="${JETSON_USER}@${JETSON_HOST}:${JETSON_WS}/$2"
  if [[ ! -e "$src" ]]; then
    echo "  SKIP (missing locally): $src"
    return 0
  fi
  echo "  → $1  →  $2"
  rsync "${RSYNC_OPTS[@]}" -e "$RSYNC_RSH" "$src" "$dst"
}

# ── FAST-LIO2 ROS 1 ──────────────────────────────────────────────────
# Dir is renamed FAST_LIO on the Jetson side for clarity (the package.xml
# name "fast_lio" is unchanged; rospack still finds it).
echo ""
echo "── rsyncing fast_lio_ros1 → src/FAST_LIO ──"
SRC="$REPO_ROOT/src/vendor/fast_lio_ros1/"
DST="${JETSON_USER}@${JETSON_HOST}:${JETSON_WS}/src/FAST_LIO/"
if [[ ! -d "$REPO_ROOT/src/vendor/fast_lio_ros1" ]]; then
  echo "ERROR: src/vendor/fast_lio_ros1 missing locally." >&2
  echo "       Run: git clone --recursive https://github.com/hku-mars/FAST_LIO.git \\" >&2
  echo "                       src/vendor/fast_lio_ros1" >&2
  exit 1
fi
# Trailing-slash semantics: copy contents of fast_lio_ros1/ INTO FAST_LIO/.
rsync "${RSYNC_OPTS[@]}" -e "$RSYNC_RSH" "$SRC" "$DST"

# ── Point-LIO ROS 1 (optional) ───────────────────────────────────────
# Point-LIO is FAST-LIO2's successor (HKU MaRS 2023): iVox replaces ikd-tree
# → O(1) avg query, no long-run degradation; decoupled IMU+LiDAR threads
# keep /Odometry publishing at IMU rate even when LiDAR processing lags.
# Shipped as a drop-in alternative — same livox_ros_driver2, same /<ns>/topic
# layout. User picks fast_lio vs point_lio at launch time.
if [[ -d "$REPO_ROOT/src/vendor/point_lio_ros1" ]]; then
  echo ""
  echo "── rsyncing point_lio_ros1 → src/Point-LIO ──"
  SRC="$REPO_ROOT/src/vendor/point_lio_ros1/"
  DST="${JETSON_USER}@${JETSON_HOST}:${JETSON_WS}/src/Point-LIO/"
  rsync "${RSYNC_OPTS[@]}" -e "$RSYNC_RSH" "$SRC" "$DST"
else
  echo "  (point_lio_ros1 not present locally — skipping)"
fi

# ── livox_ros_driver2 (separate copy for ROS 1 build) ────────────────
echo ""
echo "── rsyncing livox_ros_driver2 → src/livox_ros_driver2 ──"
rsync_to_jetson  src/vendor/livox_ros_driver2  src/

# ── Onboard launcher + recorder ──────────────────────────────────────
echo ""
echo "── rsyncing Jetson-side scripts → scripts/ ──"
for s in onboard_fastlio_noetic.sh onboard_pointlio_noetic.sh onboard_record_noetic.sh; do
  if [[ -f "$REPO_ROOT/scripts/real/$s" ]]; then
    rsync_to_jetson  "scripts/real/$s"  scripts/
    $SSH "chmod +x ${JETSON_WS}/scripts/$s" 2>/dev/null || true
  fi
done

# ── gbplanner3 configs (place at 4-deep path to satisfy BT XML resolver) ─────
# Config live in scripts/real/gbplanner3/config/go2/ on the laptop.
# Destination on Jetson: ~/gbplanner3_ws/src/gbplanner_ros/gbplanner/config/ugv/real/go2/
# The BT XML files (main_tree.xml, …) are already in the upstream source there;
# we only sync our patched YAML configs — never main_tree.xml (that stays upstream).
GBPLANNER_WS_JETSON="/home/${JETSON_USER}/gbplanner3_ws"
GBPLANNER_CONFIG_SRC="$REPO_ROOT/scripts/real/gbplanner3/config/go2"
GBPLANNER_CONFIG_DST="${JETSON_USER}@${JETSON_HOST}:${GBPLANNER_WS_JETSON}/src/gbplanner_ros/gbplanner/config/ugv/real/go2/"
if [[ -d "$GBPLANNER_CONFIG_SRC" ]]; then
  echo ""
  echo "── rsyncing gbplanner3 configs → gbplanner3_ws/…/config/ugv/real/go2/ ──"
  $SSH "mkdir -p ${GBPLANNER_WS_JETSON}/src/gbplanner_ros/gbplanner/config/ugv/real/go2"
  rsync "${RSYNC_OPTS[@]}" -e "$RSYNC_RSH" \
    --include="*.yaml" --exclude="*" \
    "$GBPLANNER_CONFIG_SRC/" "$GBPLANNER_CONFIG_DST"
fi

# Also sync gbplanner3 launch file and launcher script (to ~/gbplanner3_scripts/)
GBPLANNER_SCRIPTS_DST="${JETSON_USER}@${JETSON_HOST}:/home/${JETSON_USER}/gbplanner3_scripts/"
if [[ -d "$REPO_ROOT/scripts/real/gbplanner3" ]]; then
  echo ""
  echo "── rsyncing gbplanner3 scripts → ~/gbplanner3_scripts/ ──"
  $SSH "mkdir -p /home/${JETSON_USER}/gbplanner3_scripts"
  for f in orin_launch_gbplanner.sh bridge_topics.yaml; do
    SRC="$REPO_ROOT/scripts/real/gbplanner3/$f"
    [[ -f "$SRC" ]] || continue
    rsync "${RSYNC_OPTS[@]}" -e "$RSYNC_RSH" "$SRC" "$GBPLANNER_SCRIPTS_DST"
  done
  # Also sync the launch file into the gbplanner3 workspace
  LAUNCH_DST="${JETSON_USER}@${JETSON_HOST}:${GBPLANNER_WS_JETSON}/src/gbplanner_ros/gbplanner/launch/gbplanner_go2.launch"
  LAUNCH_SRC="$REPO_ROOT/scripts/real/gbplanner3/gbplanner_go2.launch"
  [[ -f "$LAUNCH_SRC" ]] && \
    rsync "${RSYNC_OPTS[@]}" -e "$RSYNC_RSH" "$LAUNCH_SRC" "$LAUNCH_DST"
fi

# ── elevation_mapping_cupy ROS 1 (upstream main branch) ──────────────
# The upstream leggedrobotics/elevation_mapping_cupy MAIN branch is the ROS 1
# version (catkin/rospy).  Since the Jetson has no internet, clone on the
# laptop first, then rsync.  The repo lives at a sibling dir alongside
# fast_lio_ros1 in src/vendor/.
echo ""
echo "── elevation_mapping_cupy ROS 1 ──"
EMCUPY_LOCAL="$REPO_ROOT/src/vendor/elevation_mapping_cupy_ros1"
if [[ ! -d "$EMCUPY_LOCAL/elevation_mapping_cupy" ]]; then
  echo ""
  echo "  ERROR: elevation_mapping_cupy ROS 1 source not found locally." >&2
  echo "         Clone it first (on the laptop):" >&2
  echo ""  >&2
  echo "    git clone https://github.com/leggedrobotics/elevation_mapping_cupy.git \\" >&2
  echo "        src/vendor/elevation_mapping_cupy_ros1" >&2
  echo "    touch src/vendor/elevation_mapping_cupy_ros1/COLCON_IGNORE" >&2
  echo ""  >&2
  echo "  Then re-run this deploy script." >&2
  echo "  Skipping elevation_mapping_cupy rsync for now." >&2
else
  # Rsync the elevation_mapping_cupy package (main node + Python package).
  echo "  → elevation_mapping_cupy_ros1/elevation_mapping_cupy  →  src/elevation_mapping_cupy"
  rsync "${RSYNC_OPTS[@]}" -e "$RSYNC_RSH" \
    "$EMCUPY_LOCAL/elevation_mapping_cupy/" \
    "${JETSON_USER}@${JETSON_HOST}:${JETSON_WS}/src/elevation_mapping_cupy/"

  # Rsync elevation_map_msgs (ROS 1 message definitions needed by the build).
  if [[ -d "$EMCUPY_LOCAL/elevation_map_msgs" ]]; then
    echo "  → elevation_mapping_cupy_ros1/elevation_map_msgs  →  src/elevation_map_msgs"
    rsync "${RSYNC_OPTS[@]}" -e "$RSYNC_RSH" \
      "$EMCUPY_LOCAL/elevation_map_msgs/" \
      "${JETSON_USER}@${JETSON_HOST}:${JETSON_WS}/src/elevation_map_msgs/"
  fi

  # Replace the upstream CMakeLists.txt with our Python-only version that
  # skips the pybind11_catkin C++ dependency (not in standard Noetic apt).
  PATCH_CMAKE="$REPO_ROOT/src/vendor/trav_pipeline_ros1/patches/elevation_mapping_cupy_CMakeLists.txt"
  if [[ -f "$PATCH_CMAKE" ]]; then
    echo "  Applying Python-only CMakeLists.txt patch..."
    rsync "${RSYNC_OPTS[@]}" -e "$RSYNC_RSH" \
      "$PATCH_CMAKE" \
      "${JETSON_USER}@${JETSON_HOST}:${JETSON_WS}/src/elevation_mapping_cupy/CMakeLists.txt"
  fi
fi

# ── trav_pipeline_ros1 (our catkin package) ───────────────────────────
echo ""
echo "── rsyncing trav_pipeline_ros1 → src/trav_pipeline_ros1 ──"
rsync_to_jetson  src/vendor/trav_pipeline_ros1  src/

# ── Traversability onboard launcher + bench scripts ──────────────────
echo ""
echo "── rsyncing onboard_elevation_mapping_noetic.sh + slam_bench_noetic.sh → scripts/ ──"
for _s in onboard_elevation_mapping_noetic.sh slam_bench_noetic.sh \
           onboard_fastlio_noetic.sh onboard_pointlio_noetic.sh \
           onboard_record_noetic.sh; do
  if [[ -f "$REPO_ROOT/scripts/real/$_s" ]]; then
    rsync_to_jetson  "scripts/real/$_s"  scripts/
    $SSH "chmod +x ${JETSON_WS}/scripts/$_s" 2>/dev/null || true
  fi
done

# ── Copy trained weights if present ──────────────────────────────────
# Weights live in the ROS 2 package on the laptop; copy to both the
# upstream pkg config/ (fallback path) and our pkg config/ (preferred path).
echo ""
echo "── weights ──"
WEIGHTS_SRC_DIR="$REPO_ROOT/src/vendor/elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_cupy"
WEIGHTS_DST_TRAV="${JETSON_USER}@${JETSON_HOST}:${JETSON_WS}/src/trav_pipeline_ros1/config/"
WEIGHTS_DST_UPSTREAM="${JETSON_USER}@${JETSON_HOST}:${JETSON_WS}/src/elevation_mapping_cupy/config/core/"
for wf in weights_pretrain.dat weights_ops2_tiled.dat weights_ops2_wide.dat; do
  if [[ -f "$WEIGHTS_SRC_DIR/$wf" ]]; then
    echo "  → $wf"
    rsync "${RSYNC_OPTS[@]}" -e "$RSYNC_RSH" "$WEIGHTS_SRC_DIR/$wf" "$WEIGHTS_DST_TRAV"
    rsync "${RSYNC_OPTS[@]}" -e "$RSYNC_RSH" "$WEIGHTS_SRC_DIR/$wf" "$WEIGHTS_DST_UPSTREAM"
  fi
done

# ── Done ─────────────────────────────────────────────────────────────
echo ""
echo "################################################"
if [[ "$DRY_RUN" == "true" ]]; then
  echo "  DRY RUN complete — no files were transferred."
else
  echo "  Sync complete."
  echo ""
  echo "  ── First-time Jetson setup (one-time, ~15 min) ─────────────────"
  echo "  ssh ${JETSON_USER}@${JETSON_HOST}"
  echo ""
  echo "  # 1. ROS + grid_map apt deps (run once):"
  echo "  sudo apt install -y \\"
  echo "    ros-noetic-grid-map-ros ros-noetic-grid-map-msgs ros-noetic-grid-map-filters \\"
  echo "    ros-noetic-pcl-ros ros-noetic-ros-numpy python3-scipy python3-numpy"
  echo ""
  echo "  # 2. cupy for Orin (CUDA 12.x = JetPack 6; use cupy-cuda11x for JetPack 5):"
  echo "  pip3 install cupy-cuda12x"
  echo ""
  echo "  # 3. PyTorch for Orin — follow NVIDIA JetPack instructions:"
  echo "  #    https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/"
  echo "  #    (torch wheels are NOT on PyPI for arm64; use the NVIDIA index URL)"
  echo "  #    Example for JetPack 6 / Python 3.10:"
  echo "  #    pip3 install torch --index-url https://developer.download.nvidia.com/compute/redist/jp/v60/"
  echo ""
  echo "  # 4. ros_numpy Python package (if ros-noetic-ros-numpy is unavailable):"
  echo "  pip3 install ros_numpy"
  echo ""
  echo "  ── Build steps (on the Jetson — ~10-20 min total) ─────────────"
  echo "  cd ${JETSON_WS}"
  echo "  source /opt/ros/noetic/setup.bash"
  echo ""
  echo "  # 1. Build livox_ros_driver2 (ROS1 flavor)"
  echo "  cd src/livox_ros_driver2"
  echo "  ./build.sh ROS1"
  echo ""
  echo "  # 2. Build everything (FAST-LIO / Point-LIO + elevation_mapping_cupy + trav_pipeline_ros1)"
  echo "  cd ${JETSON_WS}"
  echo "  catkin_make -DCMAKE_BUILD_TYPE=Release -j4"
  echo ""
  echo "  # 3. Source workspace"
  echo "  source ${JETSON_WS}/devel/setup.bash"
  echo ""
  echo "  ── Daily run ───────────────────────────────────────────────────"
  echo "  # Terminal 1 — SLAM (already running from SLAM launcher):"
  echo "  ~/noetic_fastlio_ws/scripts/onboard_pointlio_noetic.sh"
  echo ""
  echo "  # Terminal 2 — Traversability pipeline:"
  echo "  ~/noetic_fastlio_ws/scripts/onboard_elevation_mapping_noetic.sh"
  echo ""
  echo "  # On laptop — add to bridge_topics.yaml so ros1_bridge forwards:"
  echo "  #   /robot/traversability_grid  (nav_msgs/OccupancyGrid)"
fi
echo "################################################"
