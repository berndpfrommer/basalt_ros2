/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros2/utils.h>

// needed for loading calibration via cereal
#include <basalt/serialization/headers_serialization.h>

#include <fstream>

namespace basalt_ros2 {
void load_calib(const std::shared_ptr<rclcpp::Node> &node,
                basalt::Calibration<double> *calib,
                const std::string &calib_path) {
  if (calib_path.empty()) {
    RCLCPP_ERROR(node->get_logger(), "no calib file specified!");
    throw std::invalid_argument("no calibration file specified");
  }
  std::ifstream os(calib_path, std::ios::binary);
  if (os.is_open()) {
    cereal::JSONInputArchive archive(os);
    archive(*calib);
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "loaded calibration file with "
                       << calib->intrinsics.size()
                       << " cameras");
  } else {
    RCLCPP_ERROR_STREAM(node->get_logger(),
                        "could not load calibration file " << calib_path);
    throw std::ios_base::failure("could not find calibration file: " +
                                 calib_path);
  }
}

void load_calib_and_config(const std::shared_ptr<rclcpp::Node> &node,
                           basalt::Calibration<double> *calib,
                           basalt::VioConfig *config) {
  const std::string calib_path =
    node->declare_parameter("calibration_file", "");
  load_calib(node, calib, calib_path);
  // load config
  config->optical_flow_skip_frames = 1;
  const std::string vioConfigFile =
    node->declare_parameter("vio_config_file", "");
  if (!vioConfigFile.empty()) {
    config->load(vioConfigFile);
  }
  // overwrite debug flags if requested
  if (!node->has_parameter("debug_vio")) {
    config->vio_debug = node->declare_parameter("debug_vio", false);
  } else {
    node->get_parameter("debug_vio", config->vio_debug);
  }

  if (!node->has_parameter("debug_bad_data")) {
    config->vio_debug_bad_data =
      node->declare_parameter("debug_bad_data", false);
  } else {
    node->get_parameter("debug_bad_data",  config->vio_debug_bad_data);
  }
}
}  // namespace basalt_ros1
