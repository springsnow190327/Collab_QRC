#!/usr/bin/env bash
# explore_autorun.sh — one-command headless ops2 exploration run with automated
# diagnosis + trajectory tracking. The debugging harness for the "robot must
# traverse x = +35 → -35" goal.
#
# What it does:
#   1. Launches the desktop standalone ops2 sim (nav_test_slam_ops2_v4_go2.sh)
#      headless (gui=false rviz=false), teeing full stdout to a timestamped log.
#      That launch already starts CFPA2 + Nav2 + stuck_watchdog + stuck_diagnoser
#      (auto wired). stuck_diagnoser writes verdict JSONL on every stuck event.
#   2. Starts trajectory_monitor (tracks x-extent vs ±35 target, path length,
#      correlates CFPA2 status + stuck verdicts).
#   3. Loops printing a compact heartbeat (trajectory extent + recent diagnoses)
#      until DURATION elapses, the ±35 goal is met, or the sim dies.
#   4. On exit: tears everything down, prints the trajectory summary + a verdict
#      histogram from the diagnosis log.
#
# Usage:
#   ./scripts/debug/explore_autorun.sh                 # 1200 s default
#   DURATION=600 ./scripts/debug/explore_autorun.sh    # shorter
#   ./scripts/debug/explore_autorun.sh -- gui:=true rviz:=true   # pass-through
#
# Logs land in   logs/explore_autorun/<ts>/   (sim.log, monitor.log,
# stuck_diagnosis_robot.jsonl, trajectory_robot.json).
set -u -o pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
NS="${ROBOT_NS:-robot}"
DURATION="${DURATION:-1200}"
TS="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${WS_DIR}/logs/explore_autorun/${TS}"
mkdir -p "${LOG_DIR}"

# Anything after `--` is forwarded to the sim launch script.
PASSTHRU=()
if [[ "${1:-}" == "--" ]]; then shift; PASSTHRU=("$@"); fi

SIM_LOG="${LOG_DIR}/sim.log"
MON_LOG="${LOG_DIR}/monitor.log"
DIAG_LOG="${LOG_DIR}/stuck_diagnosis_${NS}.jsonl"
TRAJ_JSON="${LOG_DIR}/trajectory_${NS}.json"

echo "════════════════════════════════════════════════════════════════"
echo "  explore_autorun  ns=${NS}  duration=${DURATION}s"
echo "  log dir: ${LOG_DIR}"
echo "  sim   : nav_test_slam_ops2_v4_go2.sh gui:=false rviz:=false ${PASSTHRU[*]:-}"
echo "════════════════════════════════════════════════════════════════"

# Route the diagnoser's log into our run dir so everything is co-located.
export STUCK_DIAGNOSER_LOG="${DIAG_LOG}"
export STUCK_DIAGNOSER=1

# ── env for the side processes (monitor) ────────────────────────────────
source_env() {
  set +u
  if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
    source "${HOME}/miniforge3/etc/profile.d/conda.sh"; conda activate cmu_env
  elif command -v micromamba >/dev/null 2>&1; then
    eval "$(micromamba shell hook -s bash)"; micromamba activate cmu_env
  fi
  source /opt/ros/humble/setup.bash
  source "${WS_DIR}/install/setup.bash"
  export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"
  set -u
}

CHILD_PIDS=()
cleanup() {
  echo ""
  echo "── tearing down ──"
  for pid in "${CHILD_PIDS[@]}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 3
  # let trajectory_monitor flush its summary, then hard-kill the sim tree
  bash "${WS_DIR}/scripts/launch/_preflight_kill.sh" >/dev/null 2>&1 || true
  pkill -f "nav_test_3d_explore" 2>/dev/null || true
  pkill -f "trajectory_monitor.py" 2>/dev/null || true
  echo "── final report ──"
  if [[ -f "${TRAJ_JSON}" ]]; then
    python3 - "$TRAJ_JSON" "$DIAG_LOG" <<'PY'
import json, sys, collections
traj = json.load(open(sys.argv[1]))
print(f"  x range : [{traj.get('x_min')}, {traj.get('x_max')}]  "
      f"target [{traj.get('target_xmin')},{traj.get('target_xmax')}] "
      f"tol {traj.get('tol')}  → GOAL_MET={traj.get('goal_met')}")
print(f"  path len: {traj.get('path_len_m')} m   elapsed {traj.get('elapsed_s')} s")
try:
    verds = collections.Counter()
    for line in open(sys.argv[2]):
        verds[json.loads(line).get('verdict','?')] += 1
    print(f"  stuck verdicts: {dict(verds)}")
except FileNotFoundError:
    print("  stuck verdicts: (no diagnosis log)")
PY
  else
    echo "  (no trajectory summary written — sim may not have produced odom)"
  fi
  echo "  logs: ${LOG_DIR}"
}
trap cleanup EXIT INT TERM

# ── 1. launch sim (its own script sources conda+ros+ws) ─────────────────
# GUI + RViz ON by default (operator requirement: each launch must be visible).
# Set GUI=false / RVIZ=false to override for fully-headless CI batch runs.
GUI="${GUI:-true}"; RVIZ="${RVIZ:-true}"
echo "[autorun] launching sim (gui=${GUI} rviz=${RVIZ}) → ${SIM_LOG}"
bash "${WS_DIR}/scripts/launch/nav_test_slam_ops2_v4_go2.sh" \
  "gui:=${GUI}" "rviz:=${RVIZ}" "${PASSTHRU[@]}" >"${SIM_LOG}" 2>&1 &
SIM_PID=$!
CHILD_PIDS+=("$SIM_PID")

# ── 2. wait for the stack to come up (CFPA2 startup_delay is ~24 s) ──────
echo "[autorun] waiting 45 s for stack warmup..."
sleep 45

# ── 3. start trajectory monitor ─────────────────────────────────────────
echo "[autorun] starting trajectory_monitor → ${MON_LOG}"
( source_env
  python3 -u "${WS_DIR}/scripts/debug/trajectory_monitor.py" \
    --ns "${NS}" --report-sec 20 \
    --target-xmax 35 --target-xmin -35 --tol 0.10 \
    --summary-file "${TRAJ_JSON}" \
    --ros-args -p use_sim_time:=true ) >"${MON_LOG}" 2>&1 &
MON_PID=$!
CHILD_PIDS+=("$MON_PID")

# ── 4. heartbeat loop ───────────────────────────────────────────────────
START=$(date +%s)
while true; do
  sleep 30
  NOW=$(date +%s); EL=$((NOW - START))
  if ! kill -0 "$SIM_PID" 2>/dev/null; then
    echo "[autorun] sim process exited (after ${EL}s) — see ${SIM_LOG}"
    break
  fi
  # heartbeat: last monitor line + recent verdicts
  echo "── [autorun ${EL}s] ──"
  tail -n 1 "${MON_LOG}" 2>/dev/null | sed 's/^/  monitor: /' || true
  if [[ -f "${DIAG_LOG}" ]]; then
    echo "  recent verdicts: $(tail -n 5 "${DIAG_LOG}" 2>/dev/null | python3 -c 'import sys,json; print([json.loads(l)["verdict"] for l in sys.stdin if l.strip()])' 2>/dev/null || echo '[]')"
  fi
  # goal-met early exit
  if [[ -f "${TRAJ_JSON}" ]] && python3 -c "import json,sys; sys.exit(0 if json.load(open('${TRAJ_JSON}')).get('goal_met') else 1)" 2>/dev/null; then
    echo "[autorun] ✅ GOAL MET (trajectory spans ±35 within tol) at ${EL}s"
    break
  fi
  if (( EL >= DURATION )); then
    echo "[autorun] duration ${DURATION}s reached"
    break
  fi
done
