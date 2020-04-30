/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <basalt/optical_flow/optical_flow.h>
#include <basalt_vio_ros2_msgs/msg/optical_flow_result.hpp>

#include <tbb/concurrent_queue.h>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <thread>

namespace basalt_ros2 {
class OpticalFlowPublisher {
 public:
  OpticalFlowPublisher(const std::shared_ptr<rclcpp::Node>& node,
                       const std::string& topic);

  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr>*
  getInputQueue() {
    return (&inputQueue_);
  }
  void start();

 private:
  void processingThread();
  // ----- variables --
  std::shared_ptr<rclcpp::Node> node_;
  std::string topic_;
  std::shared_ptr<
      rclcpp::Publisher<basalt_vio_ros2_msgs::msg::OpticalFlowResult>>
      pub_;
  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr> inputQueue_;
  std::thread thread_;
};
}  // namespace basalt_ros2
