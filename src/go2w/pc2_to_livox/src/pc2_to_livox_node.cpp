// pc2_to_livox_node.cpp — sensor_msgs/PointCloud2 → livox_ros_driver2/CustomMsg.
//
// For the Jetson HIL benches: MuJoCo's lidar sim publishes a PointCloud2
// (PointXYZI), but the onboard Livox-native SLAM (Point-LIO / FAST-LIO2,
// lidar_type=1) wants the raw Mid-360 CustomMsg stream. This node repackages
// each cloud so ros1_bridge can forward CustomMsg to the Jetson, making the
// Jetson see exactly the raw-sensor interface it would on the real robot.
//
// Field mapping (CustomPoint):
//   x,y,z          ← PointCloud2 x,y,z (float32)
//   reflectivity   ← intensity (clamped 0..255; 0 if no intensity field)
//   offset_time    ← synthesized per-point time within the sweep (see below)
//   tag            ← 0
//   line           ← 0 (single-return; Mid-360 has 1 effective scan line in
//                       the non-repetitive pattern as far as Point-LIO cares)
//
// offset_time: the real Mid-360 stamps each point's offset from the frame's
// timebase. MuJoCo gives no per-point time, so we synthesize a monotonic span
// across the cloud. Matching pointcloud_adapter.py: a SHORT 100 µs total span
// (not a 10 ms Velodyne sweep) — the Mid-360 Risley pattern + Point-LIO's
// deskew are happiest with a small span; a large synthetic span makes the
// deskewer over-correct under rotation. Points are assigned offset_time
// linearly 0..span_us across the cloud order.

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"

namespace
{
// Find a named field's byte offset + datatype in a PointCloud2; returns -1 off.
int field_offset(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  for (const auto & f : msg.fields) {
    if (f.name == name) return static_cast<int>(f.offset);
  }
  return -1;
}
}  // namespace

class Pc2ToLivox : public rclcpp::Node
{
public:
  Pc2ToLivox()
  : Node("pc2_to_livox")
  {
    in_topic_ = declare_parameter<std::string>("input_topic", "registered_scan");
    out_topic_ = declare_parameter<std::string>("output_topic", "/livox/lidar");
    // Total synthetic per-frame time span, microseconds. 100 µs matches
    // pointcloud_adapter.py (small span → no deskew over-correction).
    span_us_ = declare_parameter<double>("offset_time_span_us", 100.0);
    // Override the output frame_id (Livox driver uses "body" for the Mid-360
    // body frame). Empty → keep the input cloud's frame_id.
    frame_id_ = declare_parameter<std::string>("frame_id", "body");

    pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(out_topic_, 10);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      in_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_cloud(*msg); });

    RCLCPP_INFO(get_logger(), "pc2_to_livox: %s (PointCloud2) -> %s (CustomMsg), span=%.0fus, frame=%s",
      in_topic_.c_str(), out_topic_.c_str(), span_us_,
      frame_id_.empty() ? "(input)" : frame_id_.c_str());
  }

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2 & in)
  {
    const int x_off = field_offset(in, "x");
    const int y_off = field_offset(in, "y");
    const int z_off = field_offset(in, "z");
    if (x_off < 0 || y_off < 0 || z_off < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "input cloud missing x/y/z fields — skipping");
      return;
    }
    const int i_off = field_offset(in, "intensity");  // optional

    const size_t n = static_cast<size_t>(in.width) * static_cast<size_t>(in.height);
    const size_t step = in.point_step;
    const uint8_t * base = in.data.data();

    livox_ros_driver2::msg::CustomMsg out;
    out.header = in.header;
    if (!frame_id_.empty()) out.header.frame_id = frame_id_;
    // timebase: nanoseconds of the frame stamp (Livox convention is ns since
    // epoch of the first point). Use the header stamp.
    out.timebase = static_cast<uint64_t>(in.header.stamp.sec) * 1000000000ULL
      + static_cast<uint64_t>(in.header.stamp.nanosec);
    out.lidar_id = 0;
    out.rsvd = {0, 0, 0};
    out.points.reserve(n);

    const double span = span_us_;
    const double inv_n = (n > 1) ? 1.0 / static_cast<double>(n - 1) : 0.0;

    size_t emitted = 0;
    for (size_t k = 0; k < n; ++k) {
      const uint8_t * p = base + k * step;
      float fx, fy, fz;
      std::memcpy(&fx, p + x_off, 4);
      std::memcpy(&fy, p + y_off, 4);
      std::memcpy(&fz, p + z_off, 4);
      // Drop NaN/inf points (MuJoCo no-return rays).
      if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(fz)) continue;

      livox_ros_driver2::msg::CustomPoint cp;
      cp.x = fx;
      cp.y = fy;
      cp.z = fz;
      if (i_off >= 0) {
        float fi;
        std::memcpy(&fi, p + i_off, 4);
        cp.reflectivity = static_cast<uint8_t>(std::clamp(fi, 0.0f, 255.0f));
      } else {
        cp.reflectivity = 0;
      }
      cp.tag = 0;
      cp.line = 0;
      cp.offset_time = static_cast<uint32_t>(
        static_cast<double>(k) * inv_n * span * 1000.0);  // span_us → ns
      out.points.push_back(cp);
      ++emitted;
    }
    out.point_num = static_cast<uint32_t>(emitted);
    pub_->publish(out);
  }

  std::string in_topic_, out_topic_, frame_id_;
  double span_us_{100.0};
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pc2ToLivox>());
  rclcpp::shutdown();
  return 0;
}
