#!/usr/bin/env bash
# Nav test on Go2 inside NTNU SubT "Urban 2 Story" industrial warehouse.
#
# Scene:  40m × 40m × 21m two-story building with internal metal stairs
#         (auto-converted from subt_cave_sim DAE via scripts/sim/sdf_to_mjcf/).
#
# Use case: validate 3D-native exploration (gbplanner3 / future planner) with
#           stair-climbing.  The building has explicit upper-level geometry
#           you can only reach via interior stairs.
#
# Usage:
#   ./scripts/launch/nav_test_urban_2story.sh                       # headless
#   ./scripts/launch/nav_test_urban_2story.sh gui:=true rviz:=true  # with viz
#   ./scripts/launch/nav_test_urban_2story.sh gui:=true nav_backend:=nav2_mppi
#
# Notes:
#   - Spawn pose (5, -15, -0.9) is on the ground floor near south wall.
#     Adjust by editing urban_2story_go2.xml <body name="base_link"> pos.
#   - scene_area_m2 is set to 1600 (40×40) so coverage metrics are sensible.
#   - has_wheels:=false, two_way_drive:=false — Go2 walks (no wheels) and
#     CHAMP doesn't have a validated reverse-walking gait.

set -u -o pipefail

source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/urban_2story_go2.xml"

if [[ ! -f "$SCENE" ]]; then
  echo "ERROR: scene not found: $SCENE"
  echo "  Did you run the SDF→MJCF converter? See scripts/sim/sdf_to_mjcf/README.md"
  exit 1
fi

exec "${WS_DIR}/scripts/launch/nav_test_fastlio.sh" \
  "mujoco_model_path:=${SCENE}" \
  "scene_area_m2:=1600" \
  "has_wheels:=false" \
  "two_way_drive:=false" \
  "$@"
