#!/usr/bin/env python3
"""cloud_stamp_rewriter — restamp Fast-LIO body cloud to current ROS time.

Why this exists: voxblox (used internally by gbplanner3 / OmniPlanner) calls
`tf_listener_.canTransform(to_frame, from_frame, cloud_stamp)` per cloud and
**drops the cloud silently if canTransform returns false** at that exact
timestamp (voxblox_ros/src/transformer.cc:103). When the cloud crosses
ros1_bridge from Humble → Noetic, the bridge adds variable latency on /tf vs
the cloud topic, so `cloud_stamp` regularly lands in a window where the TF
chain isn't fully populated yet on the Noetic side. Result on Collab_QRC:
voxblox queue grows to 10, "Input pointcloud queue getting too long!"
fires, every cloud gets dropped, TSDF stays empty, gbplanner spams
"No 'elevation' layer in map" at multi-kHz, no /command/trajectory is
ever produced.

This node:
  - subscribes to /robot/cloud_registered_body (bridged from Humble)
  - rewrites header.stamp to rospy.Time.now()
  - optionally removes near-body self returns before they enter voxblox
  - republishes to /rmf/lidar/points_downsampled (the topic voxblox + the
    elevation_mapping_lidar node already consume)

It REPLACES the existing topic_tools relay for this topic in
docker-compose.collab_qrc.yml. The relay for /robot/cloud_registered_body
must be removed when this node is enabled, otherwise both publish to the
same topic and voxblox sees an inconsistent stream.

Cost: loses temporal alignment between cloud and pose for moving robots.
Acceptable for our use case: robot is stationary at mission start, and
voxblox uses ros::Time(0) semantics via this rewrite (latest TF).
"""
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from typing import Dict, Tuple


def _filtered_cloud(
    msg: PointCloud2,
    *,
    radius: float,
    box: Tuple[float, float, float, float, float, float],
) -> Tuple[PointCloud2, Dict[str, float]]:
    """Drop points in the robot body envelope.

    GBPlanner3's voxblox map assumes the robot's own body returns have already
    been filtered. Fast-LIO's body cloud can still contain close-range body/leg
    points in MuJoCo; if those enter voxblox, GBPlanner3 rejects the current
    pose as occupied before it can sample any autonomous path.
    """
    box_min_x, box_max_x, box_min_y, box_max_y, box_min_z, box_max_z = box
    radius2 = radius * radius
    names = [field.name for field in msg.fields]
    try:
        ix = names.index("x")
        iy = names.index("y")
        iz = names.index("z")
    except ValueError:
        return msg, {"input": -1.0, "kept": -1.0, "dropped": -1.0}

    kept = []
    total = 0
    dropped = 0
    kept_min_x = kept_min_y = kept_min_z = float("inf")
    kept_max_x = kept_max_y = kept_max_z = float("-inf")
    min_r = float("inf")
    for point in pc2.read_points(msg, field_names=names, skip_nans=True):
        total += 1
        x = float(point[ix])
        y = float(point[iy])
        z = float(point[iz])
        in_box = (
            box_min_x <= x <= box_max_x
            and box_min_y <= y <= box_max_y
            and box_min_z <= z <= box_max_z
        )
        in_radius = (x * x + y * y + z * z) <= radius2
        if in_box or in_radius:
            dropped += 1
            continue
        kept.append(point)
        kept_min_x = min(kept_min_x, x)
        kept_min_y = min(kept_min_y, y)
        kept_min_z = min(kept_min_z, z)
        kept_max_x = max(kept_max_x, x)
        kept_max_y = max(kept_max_y, y)
        kept_max_z = max(kept_max_z, z)
        min_r = min(min_r, (x * x + y * y + z * z) ** 0.5)

    out = pc2.create_cloud(msg.header, msg.fields, kept)
    out.is_dense = True
    if kept:
        stats = {
            "input": float(total),
            "kept": float(len(kept)),
            "dropped": float(dropped),
            "min_x": kept_min_x,
            "max_x": kept_max_x,
            "min_y": kept_min_y,
            "max_y": kept_max_y,
            "min_z": kept_min_z,
            "max_z": kept_max_z,
            "min_r": min_r,
        }
    else:
        stats = {
            "input": float(total),
            "kept": 0.0,
            "dropped": float(dropped),
            "min_x": 0.0,
            "max_x": 0.0,
            "min_y": 0.0,
            "max_y": 0.0,
            "min_z": 0.0,
            "max_z": 0.0,
            "min_r": 0.0,
        }
    return out, stats


def _freespace_cloud(
    msg: PointCloud2,
    *,
    stride: int,
    samples_per_ray: int,
    endpoint_margin_m: float,
    min_range_m: float,
    max_range_m: float,
) -> Tuple[PointCloud2, int]:
    names = [field.name for field in msg.fields]
    try:
        ix = names.index("x")
        iy = names.index("y")
        iz = names.index("z")
    except ValueError:
        return pc2.create_cloud_xyz32(msg.header, []), 0

    stride = max(1, stride)
    samples_per_ray = max(1, samples_per_ray)
    points = []
    for idx, point in enumerate(pc2.read_points(msg, field_names=names, skip_nans=True)):
        if idx % stride:
            continue
        x = float(point[ix])
        y = float(point[iy])
        z = float(point[iz])
        dist = (x * x + y * y + z * z) ** 0.5
        usable = min(dist - endpoint_margin_m, max_range_m)
        if usable <= min_range_m:
            continue
        for sample_idx in range(1, samples_per_ray + 1):
            sample_dist = usable * sample_idx / float(samples_per_ray + 1)
            if sample_dist < min_range_m:
                continue
            scale = sample_dist / dist
            points.append((x * scale, y * scale, z * scale))
    return pc2.create_cloud_xyz32(msg.header, points), len(points)


def main() -> None:
    rospy.init_node("cloud_stamp_rewriter", anonymous=False)
    src_topic = rospy.get_param("~source_topic", "/robot/cloud_registered_body")
    dst_topic = rospy.get_param("~dest_topic",   "/rmf/lidar/points_downsampled")
    freespace_topic = rospy.get_param("~freespace_topic", "/freespace_pointcloud")
    max_rate_hz = float(rospy.get_param("~max_rate_hz", 5.0))
    self_filter_enable = bool(rospy.get_param("~self_filter_enable", True))
    log_stats = bool(rospy.get_param("~log_stats", True))
    stats_period_sec = float(rospy.get_param("~stats_period_sec", 10.0))
    freespace_enable = bool(rospy.get_param("~freespace_enable", True))
    freespace_stride = int(rospy.get_param("~freespace_stride", 6))
    freespace_samples = int(rospy.get_param("~freespace_samples_per_ray", 2))
    freespace_endpoint_margin_m = float(rospy.get_param("~freespace_endpoint_margin_m", 0.45))
    freespace_min_range_m = float(rospy.get_param("~freespace_min_range_m", 0.30))
    freespace_max_range_m = float(rospy.get_param("~freespace_max_range_m", 8.0))
    self_filter_radius_m = float(rospy.get_param("~self_filter_radius_m", 0.65))
    self_filter_box = (
        float(rospy.get_param("~self_filter_min_x", -0.45)),
        float(rospy.get_param("~self_filter_max_x", 0.45)),
        float(rospy.get_param("~self_filter_min_y", -0.35)),
        float(rospy.get_param("~self_filter_max_y", 0.35)),
        float(rospy.get_param("~self_filter_min_z", -0.35)),
        float(rospy.get_param("~self_filter_max_z", 0.45)),
    )
    pub = rospy.Publisher(dst_topic, PointCloud2, queue_size=5)
    free_pub = rospy.Publisher(freespace_topic, PointCloud2, queue_size=5) if freespace_enable else None
    last_pub = rospy.Time(0)
    last_stats_log = rospy.Time(0)

    def cb(msg: PointCloud2) -> None:
        nonlocal last_pub, last_stats_log
        now = rospy.Time.now()
        if max_rate_hz > 0.0 and last_pub != rospy.Time(0):
            if (now - last_pub).to_sec() < (1.0 / max_rate_hz):
                return
        last_pub = now
        msg.header.stamp = rospy.Time.now()
        stats = None
        if self_filter_enable:
            msg, stats = _filtered_cloud(
                msg,
                radius=self_filter_radius_m,
                box=self_filter_box,
            )
            msg.header.stamp = rospy.Time.now()
        free_count = 0
        if free_pub is not None:
            free_msg, free_count = _freespace_cloud(
                msg,
                stride=freespace_stride,
                samples_per_ray=freespace_samples,
                endpoint_margin_m=freespace_endpoint_margin_m,
                min_range_m=freespace_min_range_m,
                max_range_m=freespace_max_range_m,
            )
            free_msg.header.stamp = msg.header.stamp
            free_pub.publish(free_msg)
        if log_stats and stats is not None:
            if last_stats_log == rospy.Time(0) or (now - last_stats_log).to_sec() >= stats_period_sec:
                last_stats_log = now
                rospy.loginfo(
                    "cloud_stamp_rewriter stats: frame=%s input=%d kept=%d dropped=%d "
                    "free=%d kept_xyz=[%.2f,%.2f]x[%.2f,%.2f]x[%.2f,%.2f] min_r=%.2f",
                    msg.header.frame_id,
                    int(stats["input"]),
                    int(stats["kept"]),
                    int(stats["dropped"]),
                    free_count,
                    stats["min_x"],
                    stats["max_x"],
                    stats["min_y"],
                    stats["max_y"],
                    stats["min_z"],
                    stats["max_z"],
                    stats["min_r"],
                )
        pub.publish(msg)

    rospy.Subscriber(src_topic, PointCloud2, cb, queue_size=1)
    rospy.loginfo(
        "cloud_stamp_rewriter: %s -> %s (header.stamp <- ros::Time::now(), max_rate=%.1f Hz, self_filter=%s radius=%.2f, freespace=%s -> %s)",
        src_topic,
        dst_topic,
        max_rate_hz,
        self_filter_enable,
        self_filter_radius_m,
        freespace_enable,
        freespace_topic,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
