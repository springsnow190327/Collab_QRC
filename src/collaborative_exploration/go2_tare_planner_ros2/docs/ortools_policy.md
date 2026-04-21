# OR-Tools Provider Policy (`go2_tare_planner_ros2`)

## CMake option

- `TARE_ORTOOLS_PROVIDER=auto|system|vendored` (default: `auto`)

## Provider behavior

- `auto`
  - Prefer system OR-Tools (`find_package(ortools)`).
  - Fallback to vendored tree at `generated/tare_planner/or-tools` when available.
  - If neither is present, continue in stub mode (no OR-Tools link), with warning.

- `system`
  - Require `find_package(ortools)` success, otherwise fail configure.

- `vendored`
  - Require vendored tree with `include/` and `lib/` under `generated/tare_planner/or-tools`.
  - Enforce x86_64/amd64 architecture policy for vendored mode.

## Notes

- This package currently scaffolds exact-backend integration and validates provider policy.
- Full TARE algorithm linkage from generated sources is tracked separately in phased rollout.
