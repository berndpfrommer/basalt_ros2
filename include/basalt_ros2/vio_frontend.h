/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once
#include <basalt_ros2/image_subscriber.h>
#include <basalt_ros2/optical_flow_publisher.h>

#include <basalt/optical_flow/optical_flow.h>
#include <basalt/calibration/calibration.hpp>

#include <tbb/concurrent_queue.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

namespace basalt_ros2 {
class VIOFrontEnd {
 public:
  VIOFrontEnd(const std::shared_ptr<rclcpp::Node>& node,
              const basalt::Calibration<double>& calib);

  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr>**
  getOpticalFlowQueue() {
    return (&opticalFlowOut_);
  }
  void setOpticalFlowQueue(
      tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr>* q) {
    opticalFlowOut_ = q;
  }
  void start();

 private:
  void loadCalibration(const std::string& calib_path);
  // ----- variables --
  std::shared_ptr<rclcpp::Node> node_;
  basalt::OpticalFlowBase::Ptr opticalFlow_;
  basalt::Calibration<double> calibration_;
  std::shared_ptr<ImageSubscriber> imageSub_;
  std::shared_ptr<OpticalFlowPublisher> opticalFlowPub_;
  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr>*
      opticalFlowOut_ = nullptr;
};
}  // namespace basalt_ros2
