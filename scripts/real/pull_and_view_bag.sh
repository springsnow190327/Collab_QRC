#!/usr/bin/env bash
# pull_and_view_bag.sh — laptop-side post-record workflow:
#   1. rsync the latest (or tag-matched) Noetic record bag back from Jetson
#   2. Optionally convert ROS 1 .bag → ROS 2 mcap (for octomap_server / rviz2)
#   3. Open with Foxglove Studio
#
# Usage:
#   ./pull_and_view_bag.sh                    # pull latest tag, open Foxglove
#   ./pull_and_view_bag.sh tag=run1           # pull onboard_noetic_*run1*
#   ./pull_and_view_bag.sh tag=run1 mcap      # also convert .bag → ROS 2 mcap dir
#   ./pull_and_view_bag.sh skip_pull          # skip rsync, just open most recent local
#   ./pull_and_view_bag.sh no_view            # rsync only, don't launch Foxglove

set -eo pipefail

JETSON_USER="unitree"
JETSON_HOST="${GO2W_JETSON_IP:-192.168.123.18}"
JETSON_PASS="${JETSON_PASS:-123}"
REMOTE_BAG_DIR="/home/unitree/bags"
LOCAL_BAG_DIR="${LOCAL_BAG_DIR:-$HOME/Collab_QRC/bags}"
TAG=""
DO_MCAP="false"
DO_PULL="true"
DO_VIEW="true"

for arg in "$@"; do
  case "$arg" in
    tag=*)    TAG="${arg#tag=}" ;;
    host=*)   JETSON_HOST="${arg#host=}" ;;
    user=*)   JETSON_USER="${arg#user=}" ;;
    pass=*)   JETSON_PASS="${arg#pass=}" ;;
    dst=*)    LOCAL_BAG_DIR="${arg#dst=}" ;;
    mcap)     DO_MCAP="true" ;;
    skip_pull) DO_PULL="false" ;;
    no_view)  DO_VIEW="false" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

mkdir -p "$LOCAL_BAG_DIR"

# Pattern to match — empty tag matches all onboard_noetic_*.bag.
if [[ -n "$TAG" ]]; then
  REMOTE_GLOB="${REMOTE_BAG_DIR}/onboard_noetic_*${TAG}*"
  LOCAL_PATTERN="onboard_noetic_*${TAG}*"
else
  REMOTE_GLOB="${REMOTE_BAG_DIR}/onboard_noetic_*"
  LOCAL_PATTERN="onboard_noetic_*"
fi

# ── 1. rsync ─────────────────────────────────────────────────────────
if [[ "$DO_PULL" == "true" ]]; then
  echo ""
  echo "[1/3] rsync ${JETSON_USER}@${JETSON_HOST}:${REMOTE_GLOB}"
  echo "          → ${LOCAL_BAG_DIR}/"
  command -v sshpass &>/dev/null \
    || { echo "ERROR: sshpass not installed" >&2; exit 1; }
  if ! ping -c 1 -W 2 "$JETSON_HOST" &>/dev/null; then
    echo "ERROR: Jetson ${JETSON_HOST} unreachable" >&2
    exit 1
  fi
  sshpass -p "$JETSON_PASS" rsync -avz --progress \
    -e "sshpass -p $JETSON_PASS ssh -o StrictHostKeyChecking=accept-new" \
    "${JETSON_USER}@${JETSON_HOST}:${REMOTE_GLOB}" \
    "$LOCAL_BAG_DIR/" 2>&1 | tail -8
else
  echo "[1/3] skip_pull — using existing local files"
fi

# ── 2. Pick the most-recent first-chunk .bag (or all chunks for a tag) ──
LATEST_BAG=$(ls -t "${LOCAL_BAG_DIR}"/${LOCAL_PATTERN}*_0_0.bag 2>/dev/null | head -1)
if [[ -z "$LATEST_BAG" ]]; then
  # Fallback: maybe single-chunk recording (no _0_0 split).
  LATEST_BAG=$(ls -t "${LOCAL_BAG_DIR}"/${LOCAL_PATTERN}*.bag 2>/dev/null \
    | grep -v '\.orig\.bag$' | head -1)
fi
if [[ -z "$LATEST_BAG" ]]; then
  echo "ERROR: no .bag found in $LOCAL_BAG_DIR matching $LOCAL_PATTERN" >&2
  exit 1
fi
echo ""
echo "    latest: $LATEST_BAG ($(du -h "$LATEST_BAG" | cut -f1))"

# ── 2b. Show all chunks for context if it's a split recording ────────
BASE="${LATEST_BAG%_0_0.bag}"
if [[ -f "${BASE}_0_0.bag" ]]; then
  echo "    all chunks of this run:"
  ls -lh ${BASE}_0_*.bag 2>/dev/null | awk '{print "      " $NF "  " $5}'
fi

# ── 3. Optional mcap conversion ──────────────────────────────────────
if [[ "$DO_MCAP" == "true" ]]; then
  echo ""
  echo "[2/3] converting ROS 1 bag → ROS 2 mcap..."
  command -v rosbags-convert &>/dev/null \
    || { echo "ERROR: rosbags-convert missing. Run:  pip install --user rosbags" >&2; exit 1; }
  MCAP_DST="${BASE}_ros2"
  if [[ -d "$MCAP_DST" ]]; then
    echo "    $MCAP_DST already exists — removing for fresh convert"
    rm -rf "$MCAP_DST"
  fi
  # Pass ALL chunks so the converted ROS 2 bag is the full timeline,
  # not just the first 1 GB segment.
  CHUNKS=( ${BASE}_0_*.bag )
  [[ ${#CHUNKS[@]} -eq 0 ]] && CHUNKS=( "$LATEST_BAG" )
  echo "    converting ${#CHUNKS[@]} chunk(s)..."
  rosbags-convert --src "${CHUNKS[@]}" --dst "$MCAP_DST" 2>&1 | tail -5
  echo "    ✓ $MCAP_DST/"
  echo ""
  echo "    Replay + octomap_server + rviz2 (3 terminals):"
  echo "      T1: ros2 bag play $MCAP_DST --clock"
  echo "      T2: ros2 run octomap_server octomap_server_node --ros-args \\"
  echo "            -p frame_id:=camera_init -p resolution:=0.10 \\"
  echo "            -p sensor_model.max_range:=8.0 -p use_sim_time:=true \\"
  echo "            -r cloud_in:=/robot/cloud_registered_body"
  echo "      T3: rviz2  (Fixed Frame camera_init, add OccupancyGrid /projected_map)"
else
  echo "[2/3] mcap conversion skipped (pass 'mcap' to enable)"
fi

# ── 4. Open Foxglove Studio ──────────────────────────────────────────
if [[ "$DO_VIEW" == "true" ]]; then
  echo ""
  echo "[3/3] opening Foxglove Studio..."
  if ! command -v foxglove-studio &>/dev/null; then
    echo "    foxglove-studio not on PATH; try: snap install foxglove-studio" >&2
    echo "    Or open manually:  $LATEST_BAG"
  else
    # `&` so this script exits and Foxglove keeps running independently.
    foxglove-studio "$LATEST_BAG" >/dev/null 2>&1 &
    disown
    echo "    ✓ Foxglove launching with $(basename "$LATEST_BAG")"
    echo ""
    echo "    Suggested panels (add via top-bar):"
    echo "      • 3D — Fixed Frame=camera_init, add topic /robot/cloud_registered_body"
    echo "      • Raw Messages — /robot/Odometry  (numeric pose track)"
    echo "      • Plot — /robot/Odometry/pose/pose/position/x  (drift over time)"
  fi
else
  echo "[3/3] view skipped (no_view)"
fi
echo ""
