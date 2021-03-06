/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros2/vio_frontend.h>

#include <fstream>
#include <ios>
#include <stdexcept>
#include <thread>

namespace basalt_ros2 {
VIOFrontEnd::VIOFrontEnd(const std::shared_ptr<rclcpp::Node>& node,
                         const basalt::Calibration<double>& calib,
                         const basalt::VioConfig& config)
  : node_(node), calibration_(calib), config_(config) {}

void VIOFrontEnd::start() {
  imageSub_ =
      std::make_shared<ImageSubscriber>(node_, "left_image", "right_image");

  // create optical flow
  opticalFlow_ =
      basalt::OpticalFlowFactory::getOpticalFlow(config_, calibration_);

  // connect image sub output queue to optical flow input queue
  imageSub_->setQueue(&(opticalFlow_->input_queue));
  if (opticalFlowOut_) {
    opticalFlow_->output_queue = opticalFlowOut_;
  } else {
    opticalFlowPub_ =
        std::make_shared<OpticalFlowPublisher>(node_, "optical_flow");
    opticalFlowPub_->start();
    opticalFlow_->output_queue = opticalFlowPub_->getInputQueue();
  }
}

}  // namespace basalt_ros2
