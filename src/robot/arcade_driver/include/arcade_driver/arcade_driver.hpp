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

#ifndef ARCADE_DRIVER_NODE_HPP_
#define ARCADE_DRIVER_NODE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "drivetrain_msgs/msg/arcade_speed.hpp"

class ArcadeDriver : public rclcpp::Node
{
public:
  ArcadeDriver();

private:
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr joystick_sub_;
  rclcpp::Publisher<drivetrain_msgs::msg::ArcadeSpeed>::SharedPtr arcade_pub_;

  drivetrain_msgs::msg::ArcadeSpeed joystick_to_speed_mapper(const float joystick_rotate, const float joystick_drive);
  void joystick_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  bool is_negligible_joystick_change(const float new_joystick_rotate, const float new_joystick_drive);
  const float THRESHOLD = 0.05;
};

#endif
