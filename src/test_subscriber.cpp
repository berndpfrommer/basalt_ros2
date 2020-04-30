/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros2/imu_subscriber.h>

#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <memory>
#include <thread>
#include <vector>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("test_subscriber");
  const std::vector<std::string> topics = {"gyro", "accel"};
  auto imuSub = std::make_shared<basalt_ros2::IMUSubscriber>(node, topics);
  RCLCPP_INFO(node->get_logger(), "test subscriber started!");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
