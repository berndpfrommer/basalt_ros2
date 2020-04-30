/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <basalt/optical_flow/optical_flow.h>

#include <tbb/concurrent_queue.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>

namespace basalt_ros2 {
class ImageSubscriber {
  typedef sensor_msgs::msg::Image Image;
  typedef std::shared_ptr<const Image> ImageConstPtr;
  typedef tbb::concurrent_bounded_queue<basalt::OpticalFlowInput::Ptr>
      OpticalFlowInputQueue;
  typedef std::shared_ptr<OpticalFlowInputQueue> OpticalFlowInputQueuePtr;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImageSubscriber(const std::shared_ptr<rclcpp::Node>& node,
                  const std::string& topic_left,
                  const std::string& topic_right);

  ~ImageSubscriber(){};

  OpticalFlowInputQueue* getQueue() { return queue_; }
  void setQueue(OpticalFlowInputQueue* q) { queue_ = q; }

 private:
  void callback(const ImageConstPtr& left, const ImageConstPtr& right);
  // ----- variables --
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<message_filters::TimeSynchronizer<Image, Image>> sync_;
  std::vector<std::shared_ptr<message_filters::Subscriber<Image>>> subs_;
  OpticalFlowInputQueue* queue_{nullptr};
  long int max_q_{0};
  uint64_t framesReceived_{0};
  rclcpp::Time lastTime_;
};
}  // namespace basalt_ros2
