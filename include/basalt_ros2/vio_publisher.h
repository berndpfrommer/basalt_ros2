/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once
#include <basalt/utils/imu_types.h>

#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace basalt_ros2 {
class VIOPublisher {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VIOPublisher(const std::shared_ptr<rclcpp::Node>& node);

  void publish(const basalt::PoseVelBiasState::Ptr& data);

 private:
  // ----- variables --
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadCaster_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pub_;
  std::array<double, 36> cov_;
  nav_msgs::msg::Odometry msg_;
  bool extraTF_{false};
  Eigen::Vector3d T_extra_;    // extra translation:T_world_basaltworld
  Eigen::Quaterniond q_extra_;  // extra quaternion: q_world_basaltworld
};
}  // namespace basalt_ros2
