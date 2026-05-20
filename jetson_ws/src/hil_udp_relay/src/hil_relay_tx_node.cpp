// hil_relay_tx_node.cpp (ROS 1 Noetic) — UP-link transmitter on the Orin NX.
//
// Subscribes the autonomy outputs on the ROS 1 side and ships them over UDP back
// to the laptop (ROS 2). Replaces the broken ros1_bridge 1to2 path.
//
//   /robot/cmd_vel             (geometry_msgs/Twist)        --> UDP cmd_vel_port   PRIMARY
//   /robot/Odometry            (nav_msgs/Odometry)          --> UDP odom_port      viz
//   /robot/traversability_grid (nav_msgs/OccupancyGrid)     --> UDP trav_port      viz
//
// Each callback converts the native ROS 1 msg to a POD struct (udp_protocol.hpp),
// serializes it, and hands it to a UdpSender which fragments + emits datagrams.
// Wire format + LE/fragmentation assumptions: include/hil_udp_relay/udp_protocol.hpp.

#include <string>

#include "geometry_msgs/Twist.h"
#include "hil_udp_relay/udp_protocol.hpp"
#include "hil_udp_relay/udp_transport.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"

namespace hil_udp_relay {

class HilRelayTxNode {
 public:
  HilRelayTxNode() : nh_("~") {
    nh_.param<std::string>("laptop_ip", laptop_ip_, "192.168.123.100");
    int cmd_vel_port, odom_port, trav_port;
    nh_.param("cmd_vel_port", cmd_vel_port, 9003);
    nh_.param("odom_port", odom_port, 9004);
    nh_.param("trav_port", trav_port, 9005);
    nh_.param("enable_viz", enable_viz_, true);

    if (!cmd_vel_tx_.open(laptop_ip_, static_cast<uint16_t>(cmd_vel_port))) {
      ROS_FATAL("cmd_vel UDP sender open failed to %s:%d", laptop_ip_.c_str(),
                cmd_vel_port);
    }
    cmd_vel_sub_ = global_nh_.subscribe("/robot/cmd_vel", 10,
                                        &HilRelayTxNode::onTwist, this);

    if (enable_viz_) {
      if (!odom_tx_.open(laptop_ip_, static_cast<uint16_t>(odom_port))) {
        ROS_WARN("odom UDP sender open failed to %s:%d", laptop_ip_.c_str(),
                 odom_port);
      }
      if (!trav_tx_.open(laptop_ip_, static_cast<uint16_t>(trav_port))) {
        ROS_WARN("trav UDP sender open failed to %s:%d", laptop_ip_.c_str(),
                 trav_port);
      }
      odom_sub_ = global_nh_.subscribe("/robot/Odometry", 10,
                                       &HilRelayTxNode::onOdom, this);
      trav_sub_ = global_nh_.subscribe("/robot/traversability_grid", 2,
                                       &HilRelayTxNode::onTrav, this);
    }

    ROS_INFO("HIL TX up (NX): cmd_vel->%s:%d (viz=%s)", laptop_ip_.c_str(),
             cmd_vel_port, enable_viz_ ? "on" : "off");
  }

 private:
  void onTwist(const geometry_msgs::Twist::ConstPtr& msg) {
    Twist m;
    m.linear[0] = msg->linear.x;
    m.linear[1] = msg->linear.y;
    m.linear[2] = msg->linear.z;
    m.angular[0] = msg->angular.x;
    m.angular[1] = msg->angular.y;
    m.angular[2] = msg->angular.z;
    std::vector<uint8_t> payload;
    Writer w(payload);
    serialize_twist(w, m);
    cmd_vel_tx_.send(MSG_TWIST, payload);
  }

  void onOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    Odometry m;
    m.header.stamp_sec = msg->header.stamp.sec;
    m.header.stamp_nsec = msg->header.stamp.nsec;
    m.header.frame_id = msg->header.frame_id;
    m.child_frame_id = msg->child_frame_id;
    m.position[0] = msg->pose.pose.position.x;
    m.position[1] = msg->pose.pose.position.y;
    m.position[2] = msg->pose.pose.position.z;
    m.orientation[0] = msg->pose.pose.orientation.x;
    m.orientation[1] = msg->pose.pose.orientation.y;
    m.orientation[2] = msg->pose.pose.orientation.z;
    m.orientation[3] = msg->pose.pose.orientation.w;
    m.twist.linear[0] = msg->twist.twist.linear.x;
    m.twist.linear[1] = msg->twist.twist.linear.y;
    m.twist.linear[2] = msg->twist.twist.linear.z;
    m.twist.angular[0] = msg->twist.twist.angular.x;
    m.twist.angular[1] = msg->twist.twist.angular.y;
    m.twist.angular[2] = msg->twist.twist.angular.z;
    std::vector<uint8_t> payload;
    Writer w(payload);
    serialize_odom(w, m);
    odom_tx_.send(MSG_ODOM, payload);
  }

  void onTrav(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    OccupancyGrid m;
    m.header.stamp_sec = msg->header.stamp.sec;
    m.header.stamp_nsec = msg->header.stamp.nsec;
    m.header.frame_id = msg->header.frame_id;
    m.info.resolution = msg->info.resolution;
    m.info.width = msg->info.width;
    m.info.height = msg->info.height;
    m.info.origin_position[0] = msg->info.origin.position.x;
    m.info.origin_position[1] = msg->info.origin.position.y;
    m.info.origin_position[2] = msg->info.origin.position.z;
    m.info.origin_orientation[0] = msg->info.origin.orientation.x;
    m.info.origin_orientation[1] = msg->info.origin.orientation.y;
    m.info.origin_orientation[2] = msg->info.origin.orientation.z;
    m.info.origin_orientation[3] = msg->info.origin.orientation.w;
    m.data.assign(msg->data.begin(), msg->data.end());
    std::vector<uint8_t> payload;
    Writer w(payload);
    serialize_occgrid(w, m);
    trav_tx_.send(MSG_OCCGRID, payload);
  }

  ros::NodeHandle nh_;         // private (~) for params
  ros::NodeHandle global_nh_;  // global for absolute topic names
  std::string laptop_ip_;
  bool enable_viz_ = true;

  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber trav_sub_;

  UdpSender cmd_vel_tx_;
  UdpSender odom_tx_;
  UdpSender trav_tx_;
};

}  // namespace hil_udp_relay

int main(int argc, char** argv) {
  ros::init(argc, argv, "hil_relay_tx");
  hil_udp_relay::HilRelayTxNode node;
  ros::spin();
  return 0;
}
