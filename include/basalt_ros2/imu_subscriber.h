/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <basalt/imu/imu_types.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tbb/concurrent_queue.h>

#include <memory>
#include <string>
#include <vector>

namespace basalt_ros2 {
class IMUSubscriber {
 public:
  typedef sensor_msgs::msg::Imu ImuMsg;
  typedef std::shared_ptr<const ImuMsg> ImuMsgConstPtr;
  typedef tbb::concurrent_bounded_queue<std::shared_ptr<basalt::ImuData>>
      ImuDataQueue;
  typedef std::shared_ptr<ImuDataQueue> ImuDataQueuePtr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IMUSubscriber(const std::shared_ptr<rclcpp::Node>& node,
                const std::vector<std::string>& topics);

  ~IMUSubscriber(){};

  ImuDataQueue* getQueue() { return queue_; }
  void setQueue(ImuDataQueue* q) { queue_ = q; }
  void printFrameRate();

 private:
  void callback_gyro(const ImuMsgConstPtr msg);
  void callback_accel(const ImuMsgConstPtr msg);
  void callback_combined(const ImuMsgConstPtr msg);
  // ----- variables --
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<ImuMsg>::SharedPtr gyroSub_;
  rclcpp::Subscription<ImuMsg>::SharedPtr accelSub_;
  rclcpp::Subscription<ImuMsg>::SharedPtr combinedSub_;
  std::list<ImuMsgConstPtr> gyroQueue_;
  ImuMsgConstPtr prevAccel_;
  ImuDataQueue* queue_{nullptr};
  long int max_q_{0};
  uint64_t combinedFramesReceived_{0};
  rclcpp::Time lastTime_;
};
}  // namespace basalt_ros2
