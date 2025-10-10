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

#include "arcade_driver.hpp"

#include <cmath>

using namespace std::placeholders;

ArcadeDriver::ArcadeDriver()
: Node("arcade_driver")
{
  // Create publisher for joystick messages
  joystick_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/cmd_vel_stamped", 10, std::bind(&ArcadeDriver::joystick_callback, this, _1));

  arcade_pub_ = this->create_publisher<drivetrain_msgs::msg::ArcadeSpeed>("/arcade_speed", 10);
}

// joystick input is "negligible" if it is very close to (0, 0), basically
bool ArcadeDriver::is_negligible_joystick_change(const float new_joystick_rotate, const float new_joystick_drive)
{
  (void)new_joystick_rotate;
  (void)new_joystick_drive;
  return false;
}

void ArcadeDriver::joystick_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  // Ignore Twist msg if component is inactive
  // if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
  // 	RCLCPP_WARN(get_logger(), "Received twist message while not active, ignoring...");
  // 	return;
  // }

  RCLCPP_INFO(
    get_logger(),
    "Received Twist message - linear.x: %.2f, angular.z: %.2f",
    msg->twist.linear.x,
    msg->twist.angular.z);

  float joystick_drive = msg->twist.linear.x;
  float joystick_rotate = msg->twist.angular.z;

  if (is_negligible_joystick_change(joystick_rotate, joystick_drive)) {
    RCLCPP_INFO(rclcpp::get_logger("ArcadeDriver"), "Negligibe joystick change");
    return;
  }

  drivetrain_msgs::msg::ArcadeSpeed arcade_msg =
    ArcadeDriver::joystick_to_speed_mapper(joystick_rotate, joystick_drive);
  RCLCPP_INFO(get_logger(), "Publishing ArcadeSpeed - left: %.2f, right: %.2f", arcade_msg.l, arcade_msg.r);
  arcade_pub_->publish(std::move(arcade_msg));
}

drivetrain_msgs::msg::ArcadeSpeed ArcadeDriver::joystick_to_speed_mapper(
  const float joystick_rotate, const float joystick_drive)
{
  const float MAX = fmax(fabs(joystick_drive), fabs(joystick_rotate));
  const float DIFF = joystick_drive - joystick_rotate;
  const float TOTAL = joystick_drive + joystick_rotate;

  /*
	maximum = max(abs(drive), abs(rotate))
	total, difference = drive + rotate, drive - rotate
		# set speed according to the quadrant that the values are in
	if drive >= 0:
		if rotate >= 0:  # I quadrant
			left_motor(maximum)
			right_motor(difference)
		else:            # II quadrant
			left_motor(total)
			right_motor(maximum)
	else:
		if rotate >= 0:  # IV quadrant
			left_motor(total)
			right_motor(-maximum)
		else:            # III quadrant
			left_motor(-maximum)
			right_motor(difference)
	*/

  float left_motor;
  float right_motor;

  if (joystick_drive >= 0) {
    if (joystick_rotate >= 0) {
      left_motor = MAX;
      right_motor = DIFF;
    } else {
      left_motor = TOTAL;
      right_motor = MAX;
    }
  } else {
    if (joystick_rotate >= 0) {
      left_motor = TOTAL;
      right_motor = -1 * MAX;
    } else {
      left_motor = -1 * MAX;
      right_motor = DIFF;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ArcadeDriver"),
    "joystick: x=%.2f, y=%.2f, speed: l=%.2f, r=%.2f",
    joystick_rotate,
    joystick_drive,
    left_motor,
    right_motor);

  auto arcade_msg = drivetrain_msgs::msg::ArcadeSpeed();
  arcade_msg.l = left_motor;
  arcade_msg.r = right_motor;
  return arcade_msg;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArcadeDriver>());
  rclcpp::shutdown();
  return 0;
}
