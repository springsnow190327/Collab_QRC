# go2_tare_planner_ros2

Copy-first scaffold for `tare_ros2_exact` backend.

## Workflow

1. Refresh upstream mirror and generate ROS2 tree:

```bash
python3 tools/generate_ros2_tare.py --refresh-upstream
```

2. Verify deterministic regeneration:

```bash
bash tools/check_regen.sh
```

## Layout

- `upstream/tare_planner`: immutable mirror of ROS1 source tree
- `generated/tare_planner`: mechanical transform output used by ROS2 integration
- `UPSTREAM_MANIFEST.json`: deterministic checksum manifest for upstream mirror
