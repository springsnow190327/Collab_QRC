/**
 * scan_rear_filter.cpp — Blank out self-hit readings in the rear half of a LaserScan.
 *
 * The Go2W's legs/body appear as persistent obstacles behind the LiDAR.
 * Any reading in the rear hemisphere (|angle| > π/2) closer than
 * rear_blank_radius is set to +inf so downstream nodes ignore it.
 *
 * Subscribes: scan_in   (sensor_msgs/LaserScan)
 * Publishes:  scan_out  (sensor_msgs/LaserScan)
 */

#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ScanRearFilter : public rclcpp::Node {
public:
  ScanRearFilter() : Node("scan_rear_filter") {
    this->declare_parameter("rear_blank_radius", 0.45);
    rear_blank_radius_ = this->get_parameter("rear_blank_radius").as_double();

    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan_out", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan_in", rclcpp::SensorDataQoS(),
        std::bind(&ScanRearFilter::scan_cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Rear filter: blanking <%.2fm in rear half (|angle|>90°)",
                rear_blank_radius_);
  }

private:
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto out = *msg;  // copy
    const size_t n = out.ranges.size();
    const float inf = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < n; ++i) {
      float angle = out.angle_min + static_cast<float>(i) * out.angle_increment;
      // Rear half: |angle| > π/2
      if (std::fabs(angle) > M_PI_2 && out.ranges[i] < rear_blank_radius_) {
        out.ranges[i] = inf;
      }
    }
    pub_->publish(out);
  }

  double rear_blank_radius_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanRearFilter>());
  rclcpp::shutdown();
  return 0;
}
