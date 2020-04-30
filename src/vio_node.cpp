/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros2/vio_backend.h>
#include <basalt_ros2/vio_frontend.h>

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

#define SHORT_WIRE_BACK_AND_FRONT_END
//#define TEST_DATA_DROPPING

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // make ROS nodes corresponding to objects
  std::shared_ptr<rclcpp::Node> frontEndNode =
      rclcpp::Node::make_shared("vio_frontend");
#ifdef TEST_DATA_DROPPING
  std::shared_ptr<rclcpp::Node> backEndNode = frontEndNode;
#else
  std::shared_ptr<rclcpp::Node> backEndNode =
      rclcpp::Node::make_shared("vio_backend");
  rclcpp::executors::SingleThreadedExecutor exec;
#endif
  //
  // load calibration
  //
  basalt::Calibration<double> calib;
  const std::string calibFile =
      frontEndNode->declare_parameter("calibration_file", "");
  load_calib(&calib, calibFile, frontEndNode);

  // make actual front and back end objects
  const auto frontEnd =
      std::make_shared<basalt_ros2::VIOFrontEnd>(frontEndNode, calib);
  std::vector<std::string> imu_topics = {"gyro", "accel"};
#ifdef SHORT_WIRE_BACK_AND_FRONT_END
  // link front end to backend, by-passing the ROS messaging
  basalt_ros2::VIOBackEnd::OpticalFlowResultQueue **q =
      frontEnd->getOpticalFlowQueue();
#else
  // no direct connection: communiate through ROS2 messages,
  // with some overhead involved
  basalt_ros2::VIOBackEnd::OpticalFlowResultQueue **q = NULL;
#endif
  const auto backEnd = std::make_shared<basalt_ros2::VIOBackEnd>(
      backEndNode, calib, imu_topics, q);

  frontEnd->start();
  backEnd->start();

  // actually run the node
#ifdef TEST_DATA_DROPPING
  rclcpp::spin(frontEndNode);
#else
  exec.add_node(frontEndNode);
  exec.add_node(backEndNode);
  exec.spin();
#endif

  rclcpp::shutdown();
  return 0;
}
