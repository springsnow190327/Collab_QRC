#!/usr/bin/env bash
# fetch_jetson_logs.sh — rsync Jetson HIL logs back to desktop for offline
# debugging.
#
# Logs live on Jetson at ~/jetson_ws/logs/jetson_hil_<timestamp>.log
# (timestamped per run, rotated to keep last 30 by run_jetson_hil.sh).
#
# This script pulls them into <repo>/logs/jetson/ on desktop so they can be
# grepped, kept across sessions, and committed-if-interesting.
#
# Usage:
#   JETSON_PASS=233 ./scripts/real/fetch_jetson_logs.sh           # default
#   ./scripts/real/fetch_jetson_logs.sh latest                    # only the
#                                                                 # current run
#   ./scripts/real/fetch_jetson_logs.sh tail                      # follow
#                                                                 # latest.log
#   ./scripts/real/fetch_jetson_logs.sh host=192.168.55.49 pass=233
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/../.." &> /dev/null && pwd )"

JETSON_USER="${ORIN_USER:-johnpork233}"
JETSON_PASS="${JETSON_PASS:-}"
JETSON_HOST="${ORIN_IP:-192.168.55.49}"
JETSON_WS="${ORIN_WS:-/home/${JETSON_USER}/jetson_ws}"

MODE="all"
for arg in "$@"; do
  case "$arg" in
    user=*)  JETSON_USER="${arg#user=}" ;;
    pass=*)  JETSON_PASS="${arg#pass=}" ;;
    host=*)  JETSON_HOST="${arg#host=}" ;;
    ws=*)    JETSON_WS="${arg#ws=}" ;;
    latest)  MODE="latest" ;;
    tail)    MODE="tail" ;;
    all)     MODE="all" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

command -v rsync   &>/dev/null || { echo "ERROR: rsync missing" >&2; exit 1; }
command -v sshpass &>/dev/null || { echo "ERROR: sshpass missing (apt install sshpass)" >&2; exit 1; }
[[ -n "$JETSON_PASS" ]] || { echo "ERROR: set JETSON_PASS env or pass=... arg" >&2; exit 1; }

SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=5 -o BatchMode=no)
RSYNC_RSH="sshpass -p $JETSON_PASS ssh ${SSH_OPTS[*]}"

LOCAL_DIR="${REPO_ROOT}/logs/jetson"
mkdir -p "${LOCAL_DIR}"

case "$MODE" in
  tail)
    echo "tailing ${JETSON_HOST}:${JETSON_WS}/logs/latest.log (Ctrl-C to stop)"
    exec sshpass -p "$JETSON_PASS" ssh "${SSH_OPTS[@]}" "${JETSON_USER}@${JETSON_HOST}" \
      "tail -f ${JETSON_WS}/logs/latest.log"
    ;;
  latest)
    echo "fetching latest log only..."
    rsync -avz -e "$RSYNC_RSH" --update \
      "${JETSON_USER}@${JETSON_HOST}:${JETSON_WS}/logs/latest.log" \
      "${LOCAL_DIR}/latest.log" 2>&1 | tail -3
    # Resolve symlink to actual filename for traceability
    rsync -avz -e "$RSYNC_RSH" --update -L \
      "${JETSON_USER}@${JETSON_HOST}:${JETSON_WS}/logs/latest.log" \
      "${LOCAL_DIR}/latest_resolved.log" 2>&1 | tail -3
    echo "→ ${LOCAL_DIR}/latest.log"
    ;;
  all)
    echo "fetching all timestamped logs from ${JETSON_HOST}..."
    rsync -avz -e "$RSYNC_RSH" --update \
      --include='jetson_hil_*.log' --exclude='*' \
      "${JETSON_USER}@${JETSON_HOST}:${JETSON_WS}/logs/" \
      "${LOCAL_DIR}/" 2>&1 | tail -8
    # Always update the latest symlink resolution too
    rsync -avz -e "$RSYNC_RSH" --update -L \
      "${JETSON_USER}@${JETSON_HOST}:${JETSON_WS}/logs/latest.log" \
      "${LOCAL_DIR}/latest.log" 2>&1 | tail -2
    ls -lt "${LOCAL_DIR}/"jetson_hil_*.log 2>/dev/null | head -5
    echo "→ ${LOCAL_DIR}/  (`ls "${LOCAL_DIR}"/jetson_hil_*.log 2>/dev/null | wc -l` files)"
    ;;
esac
