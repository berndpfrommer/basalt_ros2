/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros2/imu_subscriber.h>
#include <functional>

using namespace std::placeholders;

namespace basalt_ros2 {
IMUSubscriber::IMUSubscriber(const std::shared_ptr<rclcpp::Node>& node,
                             const std::vector<std::string>& topics)
    : node_(node) {
  if (topics.size() > 1) {
    // gyro and acceleration are on different topics!
    gyroSub_ = node_->create_subscription<ImuMsg>(
        // topics[0], rclcpp::SensorDataQoS(),
        topics[0], rclcpp::QoS(100),
        std::bind(&IMUSubscriber::callback_gyro, this, _1));
    accelSub_ = node_->create_subscription<ImuMsg>(
        // topics[1], rclcpp::SensorDataQoS(),
        topics[1], rclcpp::QoS(100),
        std::bind(&IMUSubscriber::callback_accel, this, _1));
    RCLCPP_INFO_STREAM(
        node_->get_logger(),
        "imu subscribing to topics: " << topics[0] << ", " << topics[1]);
  } else if (topics.size() == 1) {
    combinedSub_ = node_->create_subscription<ImuMsg>(
        // topics[0], rclcpp::SensorDataQoS(),
        topics[0], rclcpp::QoS(100),
        std::bind(&IMUSubscriber::callback_combined, this, _1));
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "imu subscribing to topic: " << topics[0]);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "must specify 1 or 2 imu topics");
    throw std::invalid_argument("must specify 1 or 2 imu topics");
  }
  lastTime_ = node_->now();
}

void IMUSubscriber::callback_gyro(const ImuMsgConstPtr msg) {
  // gyro data is stored until acceleration data comes in
  gyroQueue_.push_back(msg);
  printFrameRate();
}

static rclcpp::Time stamp(const IMUSubscriber::ImuMsgConstPtr& m) {
  return (rclcpp::Time(m->header.stamp.sec, m->header.stamp.nanosec));
}

static double stamp_to_float(const IMUSubscriber::ImuMsgConstPtr& m) {
  return (m->header.stamp.sec * 1e3 + m->header.stamp.nanosec * 1e-6);
}

void IMUSubscriber::callback_accel(const ImuMsgConstPtr accel) {
  if (!prevAccel_) {
    // need previous acceleration for interpolation
    prevAccel_ = accel;
    return;
  }
  const rclcpp::Time prevAccelStamp = stamp(prevAccel_);
  while (!gyroQueue_.empty() && stamp(gyroQueue_.front()) < prevAccelStamp) {
    // drain old stuff
    gyroQueue_.pop_front();
  }
  const Eigen::Vector3d curr_acc(accel->linear_acceleration.x,
                                 accel->linear_acceleration.y,
                                 accel->linear_acceleration.z);
  const Eigen::Vector3d prev_acc(prevAccel_->linear_acceleration.x,
                                 prevAccel_->linear_acceleration.y,
                                 prevAccel_->linear_acceleration.z);
  // time of current accel message
  const double t_accel = stamp_to_float(accel);
  // time of previous accel message;
  const double t_prev_accel = stamp_to_float(prevAccel_);
  // total time passed since last accel message
  const double dt = t_accel - t_prev_accel;

  while (!gyroQueue_.empty()) {
    // pull gyro data out of queue (makes copy of pointer)
    const ImuMsgConstPtr gyroMsg = gyroQueue_.front();
    const double t_gyro = stamp_to_float(gyroMsg);
    if (t_gyro >= t_accel) {
      break;  // done
    }
    gyroQueue_.pop_front();
    // interpolate acceleration data to the
    // same time stamp as the gyro data
    const double w0 = (t_accel - t_gyro) / dt;
    const double w1 = (t_gyro - t_prev_accel) / dt;

    const Eigen::Vector3d accel_interp = w0 * prev_acc + w1 * curr_acc;

    // make data packet in basalt format
    basalt::ImuData::Ptr data(new basalt::ImuData());
    data->t_ns = t_gyro * 1e6;
    data->accel = accel_interp;
    data->gyro << gyroMsg->angular_velocity.x, gyroMsg->angular_velocity.y,
        gyroMsg->angular_velocity.z;
    if (queue_) {
      if (!queue_->try_push(data)) {
        RCLCPP_WARN_STREAM(
            node_->get_logger(),
            "IMU data dropped due to overflow: q_len = " << queue_->size());
      }
      max_q_ = std::max(queue_->size(), max_q_);
    }
  }
  prevAccel_ = accel;
}

void IMUSubscriber::printFrameRate() {
  if (++combinedFramesReceived_ == 1000) {
    const auto t = node_->now();
    const auto dt = t - lastTime_;
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "received combined IMU frame rate: "
                           << combinedFramesReceived_ * 1e9 / dt.nanoseconds());

    lastTime_ = node_->now();
    combinedFramesReceived_ = 0;
  }
}

void IMUSubscriber::callback_combined(const ImuMsgConstPtr msg) {
  const double t = stamp_to_float(msg);
  // make data packet in basalt format
  basalt::ImuData::Ptr data(new basalt::ImuData());
  data->t_ns = t * 1e6;
  data->accel << msg->linear_acceleration.x, msg->linear_acceleration.y,
      msg->linear_acceleration.z;
  data->gyro << msg->angular_velocity.x, msg->angular_velocity.y,
      msg->angular_velocity.z;
  if (queue_) {
    if (!queue_->try_push(data)) {
      RCLCPP_WARN_STREAM(
          node_->get_logger(),
          "IMU data dropped due to overflow: q_len = " << queue_->size());
    }
    max_q_ = std::max(queue_->size(), max_q_);
  }
  printFrameRate();
}
}  // namespace basalt_ros2
