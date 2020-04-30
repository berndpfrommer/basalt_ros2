/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <basalt_vio_ros2_msgs/msg/optical_flow_result.hpp>

#include <basalt/optical_flow/optical_flow.h>

#include <rclcpp/rclcpp.hpp>

#include <tbb/concurrent_queue.h>

#include <memory>

namespace basalt_ros2 {
class OpticalFlowSubscriber {
 public:
  typedef basalt_vio_ros2_msgs::msg::OpticalFlowResult OpticalFlowMsg;
  typedef std::shared_ptr<const OpticalFlowMsg> OpticalFlowMsgConstPtr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OpticalFlowSubscriber(const std::shared_ptr<rclcpp::Node>& node,
                        const std::string& topic);

  void setQueue(
      tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr>* q) {
    queue_ = q;
  }
  ~OpticalFlowSubscriber(){};

 private:
  void callback(const OpticalFlowMsgConstPtr msg);
  // ----- variables --
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<OpticalFlowMsg>::SharedPtr sub_;
  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr>* queue_{
      nullptr};

  long int max_q_{0};
};
}  // namespace basalt_ros2
