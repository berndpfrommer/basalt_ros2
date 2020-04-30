/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt/optical_flow/optical_flow.h>
#include <basalt_ros2/image_subscriber.h>
#include <boost/range/irange.hpp>
#include <functional>
#include <stdexcept>

using namespace std::placeholders;
using boost::irange;

static const rmw_qos_profile_t profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST, 50,
    // RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE, RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT, RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT, false};

namespace basalt_ros2 {
ImageSubscriber::ImageSubscriber(const std::shared_ptr<rclcpp::Node>& node,
                                 const std::string& topic_left,
                                 const std::string& topic_right)
    : node_(node) {
  subs_.push_back(std::make_shared<message_filters::Subscriber<Image>>(
      node.get(), topic_left));
  //      node.get(), topic_left, profile));
  subs_.push_back(std::make_shared<message_filters::Subscriber<Image>>(
      // node.get(), topic_right, profile));
      node.get(), topic_right));
  sync_ = std::make_shared<message_filters::TimeSynchronizer<Image, Image>>(
      *subs_[0], *subs_[1], 100);
  sync_->registerCallback(std::bind(&ImageSubscriber::callback, this, _1, _2));
  lastTime_ = node_->now();
}

void ImageSubscriber::callback(const ImageConstPtr& left,
                               const ImageConstPtr& right) {
  basalt::OpticalFlowInput::Ptr data(new basalt::OpticalFlowInput);

  const int NUM_CAMS = 2;
  data->img_data.resize(NUM_CAMS);
  data->t_ns =
      left->header.stamp.sec * 1000000000LL + left->header.stamp.nanosec;
  const ImageConstPtr msg[2] = {left, right};
  for (const auto& i : irange(0, NUM_CAMS)) {
    const auto& img = *msg[i];
    data->img_data[i].exposure =
        0;  /// XXX  RS2_FRAME_METADATA_ACTUAL_EXPOSURE * 1e-6

    data->img_data[i].img.reset(
        new basalt::ManagedImage<uint16_t>(img.width, img.height));

    const uint8_t* data_in = (const uint8_t*)&img.data[0];
    uint16_t* data_out = data->img_data[i].img->ptr;
    // TODO: this 8-bit to 16bit conversion can probably be done
    // more efficiently with opencv
    size_t full_size = img.width * img.height;
    for (size_t j = 0; j < full_size; j++) {
      int val = data_in[j];
      val = val << 8;
      data_out[j] = val;
    }
  }

  framesReceived_++;
  if (framesReceived_ == 100) {
    const auto t = node_->now();
    const auto dt = t - lastTime_;
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "received imag frame rate: " << framesReceived_ * 1e9 /
                                                           dt.nanoseconds());
    framesReceived_ = 0;
    lastTime_ = t;
  }

  if (queue_) {
    if (!queue_->try_push(data)) {
      RCLCPP_WARN_STREAM(
          node_->get_logger(),
          "image frame " << data->t_ns << " dropped due to queue overflow");
    }
    max_q_ = std::max(queue_->size(), max_q_);
  }
}
}  // namespace basalt_ros2
