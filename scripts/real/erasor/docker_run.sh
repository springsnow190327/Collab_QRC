#!/usr/bin/env bash
# docker_run.sh — start the erasor:noetic container with bags + output mounted
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/../../.." &> /dev/null && pwd )"

OUT_DIR="$REPO_ROOT/scripts/real/erasor/out"
mkdir -p "$OUT_DIR"

xhost +local:docker >/dev/null 2>&1 || true

docker run --rm -it \
  --net=host \
  --ipc=host \
  -e DISPLAY="${DISPLAY:-:0}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$REPO_ROOT/bags":/bags:ro \
  -v "$OUT_DIR":/out:rw \
  -v "$SCRIPT_DIR":/host_scripts:ro \
  --name erasor_noetic \
  erasor:noetic \
  bash
