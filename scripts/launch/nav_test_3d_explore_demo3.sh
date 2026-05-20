#!/usr/bin/env bash
# demo3 scene (24x16 m, 384 m^2, 4 quadrants) + 3D-frontier ETH trav pipeline.
#
# Wires the existing nav_test_3d_explore.launch.py against demo3.xml instead of
# demo_ramp.xml. Pipeline:
#   MuJoCo + Fast-LIO -> elevation_mapping_cupy -> filter_chain_runner ->
#   grid_map_to_occupancy_grid -> /robot/traversability_grid -> Nav2 + CFPA2.
#
# CFPA2 runs in ig_dimension=2d but reads the fused trav grid (planning_map_topic
# _suffix=/traversability_grid). nvblox 3D voxel mapper stays OFF (default).
#
# Spawn (4, 2) matches the demo3.xml NW-quadrant keyframe.
#
# Usage:
#   ./scripts/launch/nav_test_3d_explore_demo3.sh                          # GUI + RViz
#   ./scripts/launch/nav_test_3d_explore_demo3.sh gui:=false rviz:=false   # headless
#   ./scripts/launch/nav_test_3d_explore_demo3.sh enable_nvblox_mapper:=true
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/demo3.xml"

# Default to the pure-C++ CFPA2 binary. The Python entry point's
# install(FILES ... RENAME ...) drops the .py without package context, so
# `from .cfpa2_coordinator_node import ...` fails with ImportError. The C++
# port has no such issue and matches the production path in CLAUDE.md
# (2026-05-19 hexagonal-isolation entry).
exec "${WS_DIR}/scripts/launch/nav_test_3d_explore.sh" \
  "mujoco_model_path:=${SCENE}" \
  "spawn_x:=4.0" \
  "spawn_y:=2.0" \
  "spawn_yaw:=0.0" \
  "cfpa2_executable_suffix:=_cpp" \
  "$@"
