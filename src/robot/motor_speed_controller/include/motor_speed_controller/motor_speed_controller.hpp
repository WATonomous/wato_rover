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

#ifndef MOTOR_SPEED_CONTROLLER_NODE_HPP_
#define MOTOR_SPEED_CONTROLLER_NODE_HPP_

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "drivetrain_msgs/msg/arcade_speed.hpp"
#include "drivetrain_msgs/msg/motor_speeds.hpp"

class MotorSpeedController : public rclcpp::Node
{
public:
  MotorSpeedController();

private:
  // subscriber to arcade speed topic (/arcade_speed)
  rclcpp::Subscription<drivetrain_msgs::msg::ArcadeSpeed>::SharedPtr arcade_sub_;

  // publishers to set motor speeds
  rclcpp::Publisher<drivetrain_msgs::msg::MotorSpeeds>::SharedPtr motor_speeds_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr odrive_pub_;

  // functions to compute motor speeds
  void arcade_callback(const drivetrain_msgs::msg::ArcadeSpeed arcade_msg);
  drivetrain_msgs::msg::MotorSpeeds compute_motor_speeds(const float arcade_l, const float arcade_r);

  void publish_speeds_odrive(drivetrain_msgs::msg::MotorSpeeds speeds);
  const nlohmann::json odrive_speed_req_json = {
    {"Stage", "run"}, {"Type", "request"}, {"Target", "Drivetrain"}, {"Command", "Set_Input_Vel"}};
  const float VEL_SCALER = 40;
};

#endif
