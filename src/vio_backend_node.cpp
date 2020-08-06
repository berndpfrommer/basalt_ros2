/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros2/vio_backend.h>
#include <basalt_ros2/utils.h>

#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <memory>
#include <thread>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("vio");
  basalt::Calibration<double> calib;
  basalt::VioConfig vioConfig;
  basalt_ros2::load_calib_and_config(node, &calib, &vioConfig);

  const std::vector<std::string> imu_topics = {"imu"};
  basalt_ros2::VIOBackEnd::OpticalFlowResultQueue **q = NULL;
  const auto backEnd =
    std::make_shared<basalt_ros2::VIOBackEnd>(node, calib, vioConfig,
                                              imu_topics, q);

  backEnd->start();

  RCLCPP_INFO(node->get_logger(), "vio_backend_node started up!");
  // actually run the node
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
