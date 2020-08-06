/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once
#include <basalt/utils/vio_config.h>
#include <basalt/calibration/calibration.hpp>

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace basalt_ros2 {
void load_calib(basalt::Calibration<double> *calib,
                const std::string &calib_path);

void load_calib_and_config(const std::shared_ptr<rclcpp::Node> &node,
                           basalt::Calibration<double> *calib,
                           basalt::VioConfig *config);

}  // namespace basalt_ros2









