# Dynamic Object Filtering

## Purpose

The dynamic filter prevents short-lived moving objects from polluting
long-term team-SLAM keyframes and map-sharing products. It does not replace
Fast-LIO and does not use semantic human detection.

## Runtime Interfaces

Inputs per robot:

- `/robot_*/cloud_registered_body`
- `/robot_*/Odometry` or `/robot_*/corrected_odom`

Outputs:

- `/robot_*/cloud_static`
- `/robot_*/cloud_dynamic`
- `/robot_*/dynamic_voxel_markers`
- `/robot_*/dynamic_obstacle_mask`
- `/team_slam/static_keyframe_clouds`
- `/team_slam/dynamic_filter_metrics`

## Classification Logic

Each voxel stores first/last observation time, hit count, centroid, velocity,
static score, and dynamic score. Repeated low-velocity observations become
static. Short-lived or fast-moving observations become dynamic and decay after
`dynamic_obstacle_ttl_sec`.

Fast-LIO still consumes raw cloud. Loop closure uses static cloud when
`use_dynamic_filter:=true`.

## Validation

Run:

```bash
bash scripts/bench/run_dynamic_filter_validation.sh
```

Pass means the moving proxy is filtered and decays while repeated static points
remain available to Scan Context and ICP.
