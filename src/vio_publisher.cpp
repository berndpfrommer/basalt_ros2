/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros2/vio_publisher.h>

#include <fstream>
#include <ios>
#include <stdexcept>

static geometry_msgs::msg::Point to_ros_point(const Eigen::Vector3d &v) {
  geometry_msgs::msg::Point vv;
  vv.x = v[0];
  vv.y = v[1];
  vv.z = v[2];
  return (vv);
}

static geometry_msgs::msg::Vector3 to_ros_vec(const Eigen::Vector3d &v) {
  geometry_msgs::msg::Vector3 vv;
  vv.x = v[0];
  vv.y = v[1];
  vv.z = v[2];
  return (vv);
}

static geometry_msgs::msg::Quaternion to_ros_quat(const Eigen::Quaterniond &q) {
  geometry_msgs::msg::Quaternion qq;
  qq.x = q.x();
  qq.y = q.y();
  qq.z = q.z();
  qq.w = q.w();
  return (qq);
}

static geometry_msgs::msg::TransformStamped to_tf_msg(
    const rclcpp::Time &t, const Eigen::Quaterniond &q,
    const Eigen::Vector3d &trans, const std::string &parent,
    const std::string &child) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = t;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;

  tf.transform.translation = to_ros_vec(trans);
  tf.transform.rotation = to_ros_quat(q);
  return (tf);
}

namespace basalt_ros2 {
VIOPublisher::VIOPublisher(const std::shared_ptr<rclcpp::Node> &node)
    : node_(node) {
  RCLCPP_INFO(node_->get_logger(), "starting publisher");

  tfBroadCaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node.get());
  pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  // we don't do covariance quite yet....
  cov_ = {0.01, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.01, 0.00,
          0.00, 0.00, 0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.00,
          0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.00, 0.00, 0.00,
          0.00, 0.01, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.01};
}

void VIOPublisher::publish(const basalt::PoseVelBiasState::Ptr &data) {
  const Eigen::Vector3d T = data->T_w_i.translation();
  const Eigen::Quaterniond q = data->T_w_i.unit_quaternion();
  const Eigen::Vector3d ang_vel = data->vel_w_i;

  // make odometry message
  nav_msgs::msg::Odometry msg;
  msg.header.frame_id = "world";
  msg.child_frame_id = "body";

  msg.header.stamp.sec = data->t_ns / 1000000000LL;
  msg.header.stamp.nanosec = data->t_ns % 1000000000LL;

  msg.pose.pose.position = to_ros_point(T);
  msg.pose.pose.orientation = to_ros_quat(q);

  msg.pose.covariance = cov_;
  msg.twist.twist.linear = to_ros_vec(ang_vel);
  msg.twist.covariance = cov_;  // zero matrix

  pub_->publish(msg);
#if 0
  std::cout << "position: " << msg.pose.pose.position.x << " "
            << msg.pose.pose.position.y << " " << msg.pose.pose.position.z
            << std::endl;
#endif
  // make transform message
  const rclcpp::Time t(msg.header.stamp.sec, msg.header.stamp.nanosec);
  const geometry_msgs::msg::TransformStamped tf =
      to_tf_msg(t, q, T, msg.header.frame_id, msg.child_frame_id);

  tfBroadCaster_->sendTransform(tf);
}

}  // namespace basalt_ros2
