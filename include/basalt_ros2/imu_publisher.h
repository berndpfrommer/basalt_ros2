/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <basalt/imu/imu_types.h>

#include <tbb/concurrent_queue.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <memory>
#include <thread>

namespace basalt_ros2 {
class IMUPublisher {
 public:
  typedef sensor_msgs::msg::Imu ImuMsg;
  typedef std::shared_ptr<const ImuMsg> ImuMsgConstPtr;
  typedef tbb::concurrent_bounded_queue<std::shared_ptr<basalt::ImuData>>
      ImuDataQueue;
  IMUPublisher(const std::shared_ptr<rclcpp::Node>& node,
               const std::string& topic);

  ImuDataQueue* getInputQueue() { return (&inputQueue_); }
  void start();

 private:
  void processingThread();
  // ----- variables --
  std::shared_ptr<rclcpp::Node> node_;
  std::string topic_;
  std::shared_ptr<rclcpp::Publisher<ImuMsg>> pub_;
  ImuDataQueue inputQueue_;
  std::thread thread_;
  uint64_t framesPublished_{0};
  rclcpp::Time lastTime_;
};
}  // namespace basalt_ros2
