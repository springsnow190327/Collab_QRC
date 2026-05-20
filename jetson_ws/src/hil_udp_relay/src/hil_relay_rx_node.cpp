// hil_relay_rx_node.cpp (ROS 1 Noetic) — DOWN-link receiver on the Orin NX.
//
// Binds the sensor UDP ports the laptop (ROS 2) sends to, reassembles +
// deserializes, and republishes on the ROS 1 side so the native Noetic autonomy
// stack (FAST-LIO / Point-LIO / nav) consumes them as if from a real driver.
// Replaces the broken ros1_bridge 2to1 path.
//
//   UDP lidar_port --> /livox/lidar (livox_ros_driver2/CustomMsg)
//   UDP imu_port   --> /livox/imu   (sensor_msgs/Imu)
//
// The two ports each run a UdpReceiver on a dedicated recv thread (poll loop).
// roscpp publishers are thread-safe, so we publish directly from the recv thread.
// Wire format + LE/fragmentation assumptions: include/hil_udp_relay/udp_protocol.hpp.

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include "hil_udp_relay/udp_protocol.hpp"
#include "hil_udp_relay/udp_transport.hpp"
#include "livox_ros_driver2/CustomMsg.h"
#include "livox_ros_driver2/CustomPoint.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

namespace hil_udp_relay {

class HilRelayRxNode {
 public:
  HilRelayRxNode() : nh_("~") {
    int lidar_port, imu_port;
    nh_.param("lidar_port", lidar_port, 9001);
    nh_.param("imu_port", imu_port, 9002);

    lidar_pub_ = nh_.advertise<livox_ros_driver2::CustomMsg>("/livox/lidar", 10);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/livox/imu", 100);

    if (lidar_rx_.open(static_cast<uint16_t>(lidar_port),
                       [this](uint16_t t, const std::vector<uint8_t>& p) {
                         onPayload(t, p);
                       })) {
      spawn(lidar_rx_);
    } else {
      ROS_FATAL("lidar UDP bind failed on port %d", lidar_port);
    }
    if (imu_rx_.open(static_cast<uint16_t>(imu_port),
                     [this](uint16_t t, const std::vector<uint8_t>& p) {
                       onPayload(t, p);
                     })) {
      spawn(imu_rx_);
    } else {
      ROS_FATAL("imu UDP bind failed on port %d", imu_port);
    }

    ROS_INFO("HIL RX up (NX): /livox/lidar<-%d, /livox/imu<-%d", lidar_port, imu_port);
  }

  ~HilRelayRxNode() {
    running_ = false;
    for (auto& t : threads_)
      if (t.joinable()) t.join();
  }

 private:
  void spawn(UdpReceiver& rx) {
    threads_.emplace_back([this, &rx]() {
      while (running_ && ros::ok()) rx.poll_once(50);
    });
  }

  void onPayload(uint16_t msg_type, const std::vector<uint8_t>& payload) {
    Reader r(payload.data(), payload.size());
    switch (msg_type) {
      case MSG_LIDAR_CUSTOM: {
        CustomMsg m;
        if (!deserialize_custommsg(r, m)) return;
        livox_ros_driver2::CustomMsg out;
        out.header.stamp = ros::Time(m.header.stamp_sec, m.header.stamp_nsec);
        out.header.frame_id = m.header.frame_id;
        out.timebase = m.timebase;
        out.point_num = m.point_num;
        out.lidar_id = m.lidar_id;
        out.rsvd[0] = m.rsvd[0];
        out.rsvd[1] = m.rsvd[1];
        out.rsvd[2] = m.rsvd[2];
        out.points.resize(m.points.size());
        for (size_t i = 0; i < m.points.size(); ++i) {
          const auto& s = m.points[i];
          auto& d = out.points[i];
          d.offset_time = s.offset_time;
          d.x = s.x;
          d.y = s.y;
          d.z = s.z;
          d.reflectivity = s.reflectivity;
          d.tag = s.tag;
          d.line = s.line;
        }
        lidar_pub_.publish(out);
        break;
      }
      case MSG_IMU: {
        Imu m;
        if (!deserialize_imu(r, m)) return;
        sensor_msgs::Imu out;
        out.header.stamp = ros::Time(m.header.stamp_sec, m.header.stamp_nsec);
        out.header.frame_id = m.header.frame_id;
        out.orientation.x = m.orientation[0];
        out.orientation.y = m.orientation[1];
        out.orientation.z = m.orientation[2];
        out.orientation.w = m.orientation[3];
        out.angular_velocity.x = m.angular_velocity[0];
        out.angular_velocity.y = m.angular_velocity[1];
        out.angular_velocity.z = m.angular_velocity[2];
        out.linear_acceleration.x = m.linear_acceleration[0];
        out.linear_acceleration.y = m.linear_acceleration[1];
        out.linear_acceleration.z = m.linear_acceleration[2];
        imu_pub_.publish(out);
        break;
      }
      default:
        break;
    }
  }

  ros::NodeHandle nh_;
  ros::Publisher lidar_pub_;
  ros::Publisher imu_pub_;
  std::atomic<bool> running_{true};
  std::vector<std::thread> threads_;
  UdpReceiver lidar_rx_;
  UdpReceiver imu_rx_;
};

}  // namespace hil_udp_relay

int main(int argc, char** argv) {
  ros::init(argc, argv, "hil_relay_rx");
  hil_udp_relay::HilRelayRxNode node;
  ros::spin();
  return 0;
}
