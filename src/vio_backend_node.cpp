/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros2/vio_backend.h>

#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <memory>
#include <thread>

// needed for loading calibration via cereal
#include <basalt/serialization/headers_serialization.h>

static void load_calib(basalt::Calibration<double> *calib,
                       const std::string &calib_path,
                       const std::shared_ptr<rclcpp::Node> &node) {
  if (calib_path.empty()) {
    RCLCPP_ERROR(node->get_logger(), "no calib file specified!");
    throw std::invalid_argument("no calibration file specified");
  }
  std::ifstream os(calib_path, std::ios::binary);
  if (os.is_open()) {
    cereal::JSONInputArchive archive(os);
    archive(*calib);
    RCLCPP_INFO_STREAM(node->get_logger(), "loaded calibration file with "
                                               << calib->intrinsics.size()
                                               << " cameras");
  } else {
    throw std::ios_base::failure("could not find calibration file: " +
                                 calib_path);
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("vio");
  //
  // load calibration
  //
  basalt::Calibration<double> calib;
  const std::string calibFile = node->declare_parameter("calibration_file", "");
  load_calib(&calib, calibFile, node);

  const std::vector<std::string> imu_topics = {"imu"};
  basalt_ros2::VIOBackEnd::OpticalFlowResultQueue **q = NULL;
  const auto backEnd =
      std::make_shared<basalt_ros2::VIOBackEnd>(node, calib, imu_topics, q);

  backEnd->start();

  RCLCPP_INFO(node->get_logger(), "vio_backend_node started up!");
  // actually run the node
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
