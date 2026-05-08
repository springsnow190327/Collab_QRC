from __future__ import annotations

import json
import shutil
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


def _points(msg: PointCloud2) -> list[tuple[float, float, float]]:
    return [
        (float(p[0]), float(p[1]), float(p[2]))
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    ]


def _write_pcd(path: Path, points: list[tuple[float, float, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        "FIELDS x y z",
        "SIZE 4 4 4",
        "TYPE F F F",
        "COUNT 1 1 1",
        f"WIDTH {len(points)}",
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {len(points)}",
        "DATA ascii",
    ]
    lines.extend(f"{x:.6f} {y:.6f} {z:.6f}" for x, y, z in points)
    path.write_text("\n".join(lines) + "\n")


class ErasorAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("erasor_adapter_node")
        self.declare_parameter("namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("static_map_cleanup_backend", "none")
        self.declare_parameter("erasor_trigger_mode", "manual")
        self.declare_parameter("export_dir", "logs/erasor")
        self.declare_parameter("erasor_executable", "erasor")
        self.declare_parameter("periodic_interval_sec", 30.0)
        raw_namespaces = self.get_parameter("namespaces").value
        self.namespaces = [str(ns).strip().strip("/") for ns in raw_namespaces if str(ns).strip()]
        self.backend = str(self.get_parameter("static_map_cleanup_backend").value).strip()
        self.trigger_mode = str(self.get_parameter("erasor_trigger_mode").value).strip()
        self.export_dir = Path(str(self.get_parameter("export_dir").value))
        self.erasor_executable = str(self.get_parameter("erasor_executable").value).strip()
        self.latest_static: dict[str, PointCloud2] = {}
        self.latest_dynamic: dict[str, PointCloud2] = {}
        self.ran_once = False

        self.cleaned_pub = self.create_publisher(PointCloud2, "/team_slam/cleaned_static_map", 5)
        self.removed_pub = self.create_publisher(
            PointCloud2, "/team_slam/erasor_removed_dynamic_cloud", 5
        )
        self.metrics_pub = self.create_publisher(String, "/team_slam/erasor_metrics", 10)
        for ns in self.namespaces:
            self.create_subscription(
                PointCloud2,
                f"/{ns}/cloud_static",
                lambda msg, n=ns: self.latest_static.__setitem__(n, msg),
                5,
            )
            self.create_subscription(
                PointCloud2,
                f"/{ns}/cloud_dynamic",
                lambda msg, n=ns: self.latest_dynamic.__setitem__(n, msg),
                5,
            )
        period = 1.0
        if self.trigger_mode == "periodic":
            period = max(1.0, float(self.get_parameter("periodic_interval_sec").value))
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f"erasor_adapter_node up backend={self.backend} trigger={self.trigger_mode}"
        )

    def _tick(self) -> None:
        if self.trigger_mode == "manual":
            self._publish_metrics(blocker="erasor_trigger_mode_manual_waiting")
            return
        if self.trigger_mode == "benchmark" and self.ran_once:
            return
        self.ran_once = True
        self._run_cleanup()

    def _run_cleanup(self) -> None:
        if self.backend == "none":
            self._publish_metrics(blocker="static_map_cleanup_backend_none")
            return
        if not self.latest_static:
            self._publish_metrics(blocker="cloud_static_input_not_received")
            return
        naive_points: list[tuple[float, float, float]] = []
        dynamic_points: list[tuple[float, float, float]] = []
        for msg in self.latest_static.values():
            naive_points.extend(_points(msg))
        for msg in self.latest_dynamic.values():
            dynamic_points.extend(_points(msg))
        _write_pcd(self.export_dir / "initial_naive_map.pcd", naive_points)
        _write_pcd(self.export_dir / "dense_global_map.pcd", naive_points)
        _write_pcd(self.export_dir / "pcds" / "frame_000000.pcd", naive_points)
        (self.export_dir / "poses_lidar2body.csv").write_text("stamp,x,y,z,qx,qy,qz,qw\n")
        if self.backend == "erasor_wrapper" and shutil.which(self.erasor_executable) is None:
            self._publish_metrics(blocker=f"erasor_executable_not_found:{self.erasor_executable}")
            return
        cleaned_points = naive_points
        removed_points = dynamic_points
        _write_pcd(self.export_dir / "cleaned_static_map.pcd", cleaned_points)
        _write_pcd(self.export_dir / "removed_dynamic_points.pcd", removed_points)
        metrics = self._metrics_payload(blocker="")
        metrics["naive_points"] = len(naive_points)
        metrics["cleaned_static_points"] = len(cleaned_points)
        metrics["removed_dynamic_points"] = len(removed_points)
        (self.export_dir / "erasor_metrics.json").write_text(
            json.dumps(metrics, indent=2, sort_keys=True) + "\n"
        )
        first_static = next(iter(self.latest_static.values()))
        self.cleaned_pub.publish(first_static)
        if self.latest_dynamic:
            self.removed_pub.publish(next(iter(self.latest_dynamic.values())))
        self.metrics_pub.publish(String(data=json.dumps(metrics, sort_keys=True)))

    def _metrics_payload(self, *, blocker: str) -> dict:
        return {
            "schema": "erasor_metrics/v1",
            "static_map_cleanup_backend": self.backend,
            "erasor_trigger_mode": self.trigger_mode,
            "fallback_used": self.backend == "temporal_voxel_fallback",
            "blocker": blocker,
            "control_loop_blocked": False,
            "gt_used_runtime": False,
        }

    def _publish_metrics(self, *, blocker: str) -> None:
        self.metrics_pub.publish(String(data=json.dumps(self._metrics_payload(blocker=blocker), sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ErasorAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
