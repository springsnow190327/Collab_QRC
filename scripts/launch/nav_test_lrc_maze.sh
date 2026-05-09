#!/usr/bin/env bash
# Launch nav test with the LRC slope maze.
# Usage:
#   ./scripts/nav_test_lrc_maze.sh                        # default (GUI + RViz)
#   ./scripts/nav_test_lrc_maze.sh gui:=false              # headless
set -euo pipefail

# Kill any stale nav/sim processes from a prior launch (see _preflight_kill.sh).
source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MAZE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/lrc_maze_go2w.xml"

exec "${WS_DIR}/scripts/nav_test_mujoco.sh" "mujoco_model_path:=${MAZE}" "$@"
