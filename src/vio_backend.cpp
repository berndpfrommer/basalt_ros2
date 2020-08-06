/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros2/vio_backend.h>

template <typename T>
static T get_param(const std::shared_ptr<rclcpp::Node>& node,
                   const std::string& key, const T& def) {
  return (node->has_parameter(key) ? node->get_parameter(key).get_value<T>()
                                   : node->declare_parameter<T>(key, def));
}

namespace basalt_ros2 {
VIOBackEnd::VIOBackEnd(const std::shared_ptr<rclcpp::Node>& node,
                       const basalt::Calibration<double>& calib,
                       const basalt::VioConfig& config,
                       const std::vector<std::string>& imu_topics,
                       OpticalFlowResultQueue** opt_flow_queue)
    : node_(node) {
  const auto logger = node_->get_logger();
  RCLCPP_INFO_STREAM(
      logger, "VIO debug: " << (config.vio_debug ? " TRUE" : " FALSE"));
  RCLCPP_INFO_STREAM(logger, "VIO debug bad data: "
                     << (config.vio_debug_bad_data));

  // create VIO object
  vio_ = basalt::VioEstimatorFactory::getVioEstimator(
    config, calib, basalt::constants::g, true);
  vio_->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  // if no external queue is provided, subscribe to
  // optical flow topic and pipe output into vio
  if (opt_flow_queue) {
    *opt_flow_queue = &vio_->vision_data_queue;
  } else {
    flowSub_ = std::make_shared<OpticalFlowSubscriber>(node_, "optical_flow");
    flowSub_->setQueue(&vio_->vision_data_queue);
  }

  // subscribe to IMU and pipe into the vio
  imuSub_ = std::make_shared<IMUSubscriber>(node_, imu_topics);
  imuSub_->setQueue(&(vio_->imu_data_queue));

  // create publisher for odom
  publisher_ = std::make_shared<basalt_ros2::VIOPublisher>(node_);

  // connect the VIO output to our own queue for publishing
  vio_->out_state_queue = &outputQueue_;
}

void VIOBackEnd::start() {
  thread_ = std::thread(&VIOBackEnd::publishingThread, this);
  RCLCPP_INFO(node_->get_logger(), "backend started!");
}

void VIOBackEnd::publishingThread() {
  basalt::PoseVelBiasState::Ptr data;
  for (;;) {
    outputQueue_.pop(data);
    if (!data.get()) {
      RCLCPP_INFO(node_->get_logger(), "exiting!");
      break;
    }
    publisher_->publish(data);
  }
}

}  // namespace basalt_ros2
