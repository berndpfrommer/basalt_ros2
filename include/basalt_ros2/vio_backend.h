/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once
#include <basalt_ros2/imu_subscriber.h>
#include <basalt_ros2/optical_flow_subscriber.h>
#include <basalt_ros2/vio_publisher.h>

#include <basalt/optical_flow/optical_flow.h>
#include <basalt/vi_estimator/vio_estimator.h>
#include <basalt/calibration/calibration.hpp>

#include <tbb/concurrent_queue.h>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <thread>
#include <vector>

namespace basalt_ros2 {
class VIOBackEnd {
 public:
  typedef tbb::concurrent_bounded_queue<basalt::OpticalFlowInput::Ptr>
      OpticalFlowInputQueue;
  typedef tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr>
      OpticalFlowResultQueue;
  typedef std::shared_ptr<OpticalFlowInputQueue> OpticalFlowInputQueuePtr;
  VIOBackEnd(const std::shared_ptr<rclcpp::Node>& node,
             const basalt::Calibration<double>& calib,
             const basalt::VioConfig& config,
             const std::vector<std::string>& imu_topics,
             OpticalFlowResultQueue** opt_flow_queue);

  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr>*
  getOpticalFlowQueue() {
    return (&vio_->vision_data_queue);
  }

  void start();

 private:
  void publishingThread();
  // ----- variables --
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<IMUSubscriber> imuSub_;
  std::shared_ptr<OpticalFlowSubscriber> flowSub_;
  basalt::VioEstimatorBase::Ptr vio_;
  std::shared_ptr<basalt_ros2::VIOPublisher> publisher_;
  tbb::concurrent_bounded_queue<basalt::PoseVelBiasState::Ptr> outputQueue_;
  std::thread thread_;
};
}  // namespace basalt_ros2
