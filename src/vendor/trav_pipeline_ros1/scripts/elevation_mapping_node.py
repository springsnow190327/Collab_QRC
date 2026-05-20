#!/usr/bin/env python3
"""Real-robot entry point for elevation_mapping_cupy with ROS namespace support.

Fixes two issues in the upstream ElevationMapWrapper that break namespaced
deployment on the Jetson:

1. Publisher topics: upstream uses /{node_name}/{key} (absolute path) which
   ignores the ROS namespace set by <node ns="robot" ...>.  We replace with
   relative topic names so elevation_map_raw lands at /robot/elevation_map_raw.

2. Config loading: upstream hardcodes config_type="sim" and calls
   `rosparam delete /elevation_mapping` + `rosparam load ... elevation_mapping`
   ignoring the node's actual namespace.  We load config from a ~config_file
   param set by the launch file and reload into the correct node namespace.

Launch-file params consumed here (all optional, sensible defaults):
  ~config_file  path to the real-robot params YAML (default: upstream sim cfg)
  ~weight_file  path to .dat weights file (default: pkg/config/core/weights.dat)
"""

import os
import numpy as np
import rospy
import rospkg
from functools import partial

import tf2_ros
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout as MAL
from std_msgs.msg import MultiArrayDimension as MAD

from elevation_mapping_cupy.elevation_mapping import ElevationMap
from elevation_mapping_cupy.parameter import Parameter
from elevation_mapping_cupy.elevation_mapping_ros import ElevationMapWrapper


class RealElevationMapWrapper(ElevationMapWrapper):
    """ElevationMapWrapper with namespace-safe publishers and external config.

    Overrides __init__ rather than calling super() because the upstream __init__
    hard-codes weight_file and config loading before we can intercept them.
    """

    def __init__(self):
        rospack = rospkg.RosPack()
        self.root = rospack.get_path("elevation_mapping_cupy")
        self.node_name = "elevation_mapping"
        self._last_t = None

        # Step 1 — init ROS node (must precede any rospy.get_param calls).
        rospy.init_node(self.node_name, anonymous=False)
        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer)

        # Step 2 — load config into the correct namespace and read into self.
        self.get_ros_params()

        # Step 3 — build Parameter + ElevationMap.
        # weight_file comes from a launch-file param so we can swap weights
        # without re-flashing the Jetson.
        custom_weight = rospy.get_param("~weight_file", "")
        weight_file = custom_weight or os.path.join(self.root, "config/core/weights.dat")
        plugin_cfg = os.path.join(self.root, "config/core/plugin_config.yaml")
        self.param = Parameter(
            use_chainer=False,
            weight_file=weight_file,
            plugin_config_file=plugin_cfg,
        )
        self.param.subscriber_cfg = self.subscribers
        self.param.update()

        rospy.loginfo(f"[elevation_mapping] weights: {weight_file}")

        self._pointcloud_process_counter = 0
        self._image_process_counter = 0
        self._map = ElevationMap(self.param)
        # cell_n includes a 1-cell border on each side
        inner = self._map.cell_n - 2
        self._map_data = np.zeros((inner, inner), dtype=np.float32)
        self._map_q = None
        self._map_t = None

        self.register_subscribers()
        self.register_publishers()
        self.register_timers()

    # ------------------------------------------------------------------
    # Config loading override
    # ------------------------------------------------------------------

    def get_ros_params(self):
        """Load config from ~config_file param or fall back to upstream logic."""
        config_file = rospy.get_param("~config_file", "")
        # rospy.get_name() returns the fully-namespaced node name, e.g.
        # /robot/elevation_mapping — safe to use as rosparam namespace.
        node_ns = rospy.get_name()

        if config_file:
            # Wipe any stale params and load our real-robot YAML.
            os.system(f"rosparam delete {node_ns}")
            os.system(f"rosparam load {config_file} {node_ns}")
        else:
            # Fall back to upstream per-type configs (sim / real inside pkg).
            typ = rospy.get_param("~config_type", "sim")
            for suffix in ["_parameters", "_sensor_parameter", "_plugin_config"]:
                f = os.path.join(self.root, f"config/{typ}{suffix}.yaml")
                if os.path.exists(f):
                    os.system(f"rosparam load {f} {node_ns}")

        # Read all parameters (mirror of upstream ElevationMapWrapper.get_ros_params).
        self.subscribers = rospy.get_param("~subscribers")
        self.publishers = rospy.get_param("~publishers")
        self.initialize_frame_id = rospy.get_param("~initialize_frame_id", "base")
        self.initialize_tf_offset = rospy.get_param("~initialize_tf_offset", 0.0)
        self.pose_topic = rospy.get_param("~pose_topic", "pose")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.corrected_map_frame = rospy.get_param("~corrected_map_frame", "corrected_map")
        self.initialize_method = rospy.get_param("~initialize_method", "cubic")
        self.position_lowpass_alpha = rospy.get_param("~position_lowpass_alpha", 0.2)
        self.orientation_lowpass_alpha = rospy.get_param("~orientation_lowpass_alpha", 0.2)
        self.recordable_fps = rospy.get_param("~recordable_fps", 3.0)
        self.update_variance_fps = rospy.get_param("~update_variance_fps", 1.0)
        self.time_interval = rospy.get_param("~time_interval", 0.1)
        self.update_pose_fps = rospy.get_param("~update_pose_fps", 10.0)
        self.initialize_tf_grid_size = rospy.get_param("~initialize_tf_grid_size", 0.5)
        self.map_acquire_fps = rospy.get_param("~map_acquire_fps", 5.0)
        self.publish_statistics_fps = rospy.get_param("~publish_statistics_fps", 1.0)
        self.enable_pointcloud_publishing = rospy.get_param(
            "~enable_pointcloud_publishing", False
        )
        self.enable_normal_arrow_publishing = rospy.get_param(
            "~enable_normal_arrow_publishing", False
        )
        self.enable_drift_corrected_TF_publishing = rospy.get_param(
            "~enable_drift_corrected_TF_publishing", False
        )
        self.use_initializer_at_start = rospy.get_param("~use_initializer_at_start", False)

    # ------------------------------------------------------------------
    # Publisher override: relative topic names for namespace compatibility
    # ------------------------------------------------------------------

    def register_publishers(self):
        """Publish to relative names so the node works under any ROS namespace.

        Upstream uses f'/{self.node_name}/{key}' (absolute), which resolves to
        /elevation_mapping/elevation_map_raw regardless of ns=robot.
        Using a relative name 'elevation_map_raw' resolves to the node's
        namespace: /robot/elevation_map_raw when ns=robot.
        """
        self._publishers = {}
        self._publishers_timers = []
        for k, v in self.publishers.items():
            self._publishers[k] = rospy.Publisher(k, GridMap, queue_size=10)
            self._publishers_timers.append(
                rospy.Timer(
                    rospy.Duration(1.0 / v["fps"]),
                    partial(self.publish_map, k),
                )
            )


if __name__ == "__main__":
    wrapper = RealElevationMapWrapper()
    while not rospy.is_shutdown():
        rospy.spin()
