# ROS 1 SC-PGO bridge

This container path runs the vendored ROS 1 `fast_lio_sam_node` natively and
uses `ros1_bridge` to exchange only standard message topics with the ROS 2
Humble sim graph.

Start it before or alongside a ROS 2 launch using `loop_closure:=true`:

```bash
bash scripts/launch/scpgo_ros1_bridge.sh
```

The bridge intentionally does not carry ROS 1 `/tf` or SC-PGO's original
`/corrected_odom`; upstream publishes that topic as a `PointCloud2`
visualization. Instead, it bridges `/<robot>/sc_pgo/pose_stamped`, and the
ROS 2 launch starts `scripts/runtime/scpgo_pose_to_odom_adapter.py` to publish
`/<robot>/corrected_odom` as `nav_msgs/Odometry`.

If your Docker registry has a Humble-specific bridge image, override:

```bash
ROS1_BRIDGE_IMAGE=ros:humble-ros1-bridge \
ROS2_DISTRO_IN_BRIDGE=humble \
bash scripts/launch/scpgo_ros1_bridge.sh
```
