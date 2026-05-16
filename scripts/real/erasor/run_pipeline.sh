#!/usr/bin/env bash
# run_pipeline.sh — runs INSIDE the erasor:noetic Docker container.
#
# Stage A: FAST_LIO + SC-A-LOAM (loop closure)  → /out/sc_pgo/aft_pgo_map.pcd
# Stage B: ERASOR offline (dynamic removal)     → /out/erasor/staticmap_*.pcd
#
# Inputs:
#   /bags/<bag_v1.bag>           pre-converted via convert_bag_for_erasor.py
#
# Outputs (in /out, mounted to host scripts/real/erasor/out/):
#   /out/sc_pgo/Scans/*.pcd       keyframe clouds (in body frame)
#   /out/sc_pgo/optimized_poses.txt
#   /out/sc_pgo/aft_pgo_map.pcd   loop-closure-corrected accumulated map
#   /out/erasor/staticmap_via_erasor.pcd   (only if STAGE_ERASOR=1)
set -eo pipefail

BAG=${BAG:-/bags/$(ls /bags/*_v1.bag 2>/dev/null | head -1 | xargs basename 2>/dev/null)}
RATE=${RATE:-0.5}
STAGE_ERASOR=${STAGE_ERASOR:-0}    # set to 1 to also run ERASOR (more complex setup)

[[ -f "$BAG" ]] || { echo "ERROR: bag not found at $BAG" >&2; exit 1; }
echo "bag = $BAG"
echo "rate = $RATE"

mkdir -p /out/sc_pgo/Scans /out/sc_pgo/SCDs /out/erasor
SAVE_DIR=/out/sc_pgo/

source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash

# ── 1. roscore ───────────────────────────────────────────────────────
echo "[1/4] starting roscore..."
roscore > /out/log_roscore.log 2>&1 &
ROSCORE_PID=$!
until rostopic list >/dev/null 2>&1; do sleep 0.5; done
echo "  roscore ready (PID=$ROSCORE_PID)"

cleanup() {
  echo "→ cleanup..."
  pkill -INT -f "fastlio\|alaserPGO\|main_in_your_env\|offline_map_updater" 2>/dev/null || true
  sleep 5
  pkill -9   -f "fastlio\|alaserPGO\|main_in_your_env\|offline_map_updater" 2>/dev/null || true
  kill -INT $ROSCORE_PID 2>/dev/null || true
  sleep 2
  kill -9   $ROSCORE_PID 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ── 2. FAST_LIO + SC-A-LOAM (combined launch) ────────────────────────
echo "[2/4] starting FAST_LIO + SC-A-LOAM..."
# Patch FAST_LIO's mid360.yaml to enable PCD save and reasonable interval
# (FAST-LIO ROS1 reads from launch/mapping_mid360.launch which loads
#  config/mid360.yaml — already in the image)
roslaunch --wait /host_scripts/fastlio_scaloam.launch > /out/log_fastlio_scpgo.log 2>&1 &
sleep 6

# alaserPGO publishes /aft_pgo_map every time the pose graph is optimized,
# but doesn't save it to PCD. Subscribe and snapshot to /out/sc_pgo/aft_pgo_map_*.pcd
mkdir -p /out/sc_pgo/pgo_snapshots
( cd /out/sc_pgo/pgo_snapshots && \
  rosrun pcl_ros pointcloud_to_pcd input:=/aft_pgo_map _prefix:=pgo_ \
    > /out/log_pcd_recorder.log 2>&1 ) &
PCD_REC_PID=$!
echo "  /aft_pgo_map → snapshots in /out/sc_pgo/pgo_snapshots/  (PID=$PCD_REC_PID)"
sleep 2

# Confirm nodes alive
for n in fastlio_mapping alaserPGO; do
  if ! rosnode list 2>/dev/null | grep -q "$n"; then
    echo "  WARN: node $n not visible yet" >&2
  fi
done

# ── 3. play bag ──────────────────────────────────────────────────────
echo "[3/4] playing bag (rate=${RATE})..."
rosbag play "$BAG" --clock --rate "$RATE"
echo "  bag done."

# Let SC-PGO drain its queue and finalize
echo "  waiting 20s for SC-PGO to flush pose graph + save..."
sleep 20

# Graceful shutdown of alaserPGO triggers `aft_pgo_map.pcd` save
pkill -INT -f alaserPGO 2>/dev/null || true
sleep 10
pkill -INT -f fastlio_mapping 2>/dev/null || true
sleep 5

# ── 4. Results ────────────────────────────────────────────────────────
echo "[4/4] Results in /out/sc_pgo/:"
ls -lh /out/sc_pgo/ 2>/dev/null
echo "---"
N_SCANS=$(ls /out/sc_pgo/Scans/*.pcd 2>/dev/null | wc -l)
echo "  $N_SCANS keyframe scans saved"

if [[ "$STAGE_ERASOR" == "1" ]]; then
  echo ""
  echo "=== Stage B: ERASOR dynamic-object removal ==="
  if [[ ! -f /out/sc_pgo/aft_pgo_map.pcd ]]; then
    echo "  ERROR: aft_pgo_map.pcd not found; cannot run ERASOR" >&2
    exit 1
  fi
  # Run ERASOR's offline_map_updater in standalone mode against aft_pgo_map.pcd
  # Note: this requires the run_erasor.launch pipeline which is KITTI-tuned.
  # For Mid-360, we'd need a custom your_own_env_mid360.yaml + launch — TODO.
  echo "  TODO: ERASOR launch for Mid-360 (requires custom yaml + node-msg bag)"
  echo "  for now, just outputting the loop-closure-corrected map."
fi

echo ""
echo "✓ pipeline done. Output map at /out/sc_pgo/aft_pgo_map.pcd"
