#!/usr/bin/env bash
# Wrap any ros2 launch / benchmark with a side-car trav_corpus_collector.
#
# Starts trav_corpus_collector.py in the background, waits a short stabilise
# window (so the elevation_map node has time to come up), then exec's the
# wrapped command. On SIGTERM / SIGINT / normal exit, sends SIGTERM to the
# collector so it flushes cleanly to .npz.
#
# Required env:
#   CORPUS_SCENE       MJCF stem (e.g. "demo3_mixed")
#   CORPUS_OUTPUT      output .npz path
#
# Optional env:
#   CORPUS_NS          robot namespace (default "robot")
#   CORPUS_TRIAL_ID    trial identifier string (default derived from CORPUS_OUTPUT)
#   CORPUS_LABEL_DIR   directory containing <scene>_gtlabel.npy
#                       (default: src/go2w/go2_gazebo_sim/mujoco/)
#   CORPUS_PATCHES_PER_FRAME  default 50
#   CORPUS_MAX_PATCHES        default 200000
#   CORPUS_WARMUP_SEC         seconds to wait before collector starts (default 5)
#
# Usage:
#   CORPUS_SCENE=demo3_mixed \
#   CORPUS_OUTPUT=/tmp/trav_corpus/demo3_mixed_t000.npz \
#   ./scripts/bench/run_with_corpus.sh ros2 launch <pkg> <launch> ...
set -u -o pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
COLLECTOR="${WS_DIR}/scripts/bench/trav_corpus_collector.py"

: "${CORPUS_SCENE:?need CORPUS_SCENE env var}"
: "${CORPUS_OUTPUT:?need CORPUS_OUTPUT env var}"

CORPUS_NS="${CORPUS_NS:-robot}"
CORPUS_LABEL_DIR="${CORPUS_LABEL_DIR:-${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco}"
CORPUS_PATCHES_PER_FRAME="${CORPUS_PATCHES_PER_FRAME:-50}"
CORPUS_MAX_PATCHES="${CORPUS_MAX_PATCHES:-200000}"
CORPUS_WARMUP_SEC="${CORPUS_WARMUP_SEC:-5}"
CORPUS_TRIAL_ID="${CORPUS_TRIAL_ID:-$(basename "${CORPUS_OUTPUT}" .npz)}"

STATIC_LABEL="${CORPUS_LABEL_DIR}/${CORPUS_SCENE}_gtlabel.npy"
if [[ ! -f "${STATIC_LABEL}" ]]; then
  echo "[run_with_corpus] static label not found: ${STATIC_LABEL}" >&2
  echo "[run_with_corpus] run scripts/training/mujoco_static_label_map.py first" >&2
  exit 2
fi

mkdir -p "$(dirname "${CORPUS_OUTPUT}")"

echo "[run_with_corpus] scene=${CORPUS_SCENE} ns=${CORPUS_NS}"
echo "[run_with_corpus] static_label=${STATIC_LABEL}"
echo "[run_with_corpus] output=${CORPUS_OUTPUT}"
echo "[run_with_corpus] warmup=${CORPUS_WARMUP_SEC}s patches/frame=${CORPUS_PATCHES_PER_FRAME}"

# Launch the collector in background after a warmup so the elevation map
# pipeline has had time to come up. Without warmup the collector logs
# "layer 'elevation' not in GridMap" repeatedly.
(
  sleep "${CORPUS_WARMUP_SEC}"
  exec python3 -u "${COLLECTOR}" \
    --namespace "${CORPUS_NS}" \
    --scene "${CORPUS_SCENE}" \
    --static-label "${STATIC_LABEL}" \
    --output "${CORPUS_OUTPUT}" \
    --trial-id "${CORPUS_TRIAL_ID}" \
    --patches-per-frame "${CORPUS_PATCHES_PER_FRAME}" \
    --max-patches "${CORPUS_MAX_PATCHES}"
) &
COLLECTOR_PID=$!

cleanup() {
  if kill -0 "${COLLECTOR_PID}" 2>/dev/null; then
    echo "[run_with_corpus] sending SIGTERM to collector (pid=${COLLECTOR_PID}) for clean flush"
    kill -TERM "${COLLECTOR_PID}" 2>/dev/null || true
    # Give it up to 10s to flush.
    for _ in $(seq 1 20); do
      kill -0 "${COLLECTOR_PID}" 2>/dev/null || break
      sleep 0.5
    done
    kill -KILL "${COLLECTOR_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[run_with_corpus] launching: $*"
"$@"
rc=$?

cleanup
exit "${rc}"
