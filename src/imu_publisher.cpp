/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros2/imu_publisher.h>
#include <boost/range/irange.hpp>

using boost::irange;

namespace basalt_ros2 {
IMUPublisher::IMUPublisher(const std::shared_ptr<rclcpp::Node> &node,
                           const std::string &topic)
    : node_(node), topic_(topic) {
  pub_ = node_->create_publisher<ImuMsg>(topic, 10);
  lastTime_ = node_->now();
}

void IMUPublisher::processingThread() {
  basalt::ImuData::Ptr data;
  for (;;) {
    inputQueue_.pop(data);
    if (!data) {
      break;  // done!
    }
    if (node_->count_subscribers(topic_) != 0) {
      ImuMsg msg;
      msg.header.frame_id = "frame_id";  // XXX make configurable
      msg.header.stamp.sec = data->t_ns / 1000000000LL;
      msg.header.stamp.nanosec = data->t_ns % 1000000000LL;

      msg.linear_acceleration.x = data->accel(0);
      msg.linear_acceleration.y = data->accel(1);
      msg.linear_acceleration.z = data->accel(2);

      msg.angular_velocity.x = data->gyro(0);
      msg.angular_velocity.y = data->gyro(1);
      msg.angular_velocity.z = data->gyro(2);
      pub_->publish(msg);
    }
    framesPublished_++;
    if (framesPublished_ == 1000) {
      const auto t = node_->now();
      const auto dt = t - lastTime_;
      RCLCPP_INFO_STREAM(node_->get_logger(),
                         "published imu frame rate: "
                             << framesPublished_ * 1e9 / dt.nanoseconds());
      framesPublished_ = 0;
      lastTime_ = t;
    }
  }
}

void IMUPublisher::start() {
  thread_ = std::thread(&IMUPublisher::processingThread, this);
}

}  // namespace basalt_ros2
