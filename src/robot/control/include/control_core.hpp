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

#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class ControlCore
{
public:
  ControlCore(const rclcpp::Logger & logger);

  void initControlCore(double kp, double ki, double kd, double max_steering_angle, double linear_velocity);

  void updatePath(nav_msgs::msg::Path path);

  bool isPathEmpty();

  unsigned int findClosestPoint(double robot_x, double robot_y);

  geometry_msgs::msg::Twist calculateControlCommand(double robot_x, double robot_y, double robot_theta, double dt);

private:
  nav_msgs::msg::Path path_;
  rclcpp::Logger logger_;

  double kp_;
  double ki_;
  double kd_;
  double max_steering_angle_;
  double linear_velocity_;

  double prev_error_;
  double integral_error_;
};

}  // namespace robot

#endif
