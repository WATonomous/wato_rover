// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GPS_SIM_NODE_HPP_
#define GPS_SIM_NODE_HPP_

#include <memory>
#include <random>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class GpsSimNode : public rclcpp::Node
{
public:
  GpsSimNode();

private:
  void timerCallback();
  void processParameters();
  double generateGaussianNoise(double mean, double stddev);

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;

  // Transform Utilities
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Timer to periodically publish GPS data
  rclcpp::TimerBase::SharedPtr timer_;

  // Random number generator for noise
  std::random_device rd_;
  std::mt19937 gen_;
  std::normal_distribution<double> gaussian_noise_;

  // Configuration parameters
  std::string source_frame_;
  std::string target_frame_;
  double gps_noise_std_;  // Standard deviation for GPS position noise (meters)
  double gps_update_rate_;  // GPS update rate (Hz)
  double gps_dropout_probability_;  // Probability of GPS dropout (0.0-1.0)
  bool simulate_dropouts_;  // Whether to simulate GPS dropouts
};

#endif
