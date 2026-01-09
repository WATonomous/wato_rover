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

#ifndef ODOMETRY_SPOOF_NODE_HPP_
#define ODOMETRY_SPOOF_NODE_HPP_

#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class OdometrySpoofNode : public rclcpp::Node
{
public:
  OdometrySpoofNode();

private:
  void timerCallback();

  // Odom Publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // Transform Utilities
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Timer to periodically lookup transforms
  rclcpp::TimerBase::SharedPtr timer_;

  // Check if last transform was found
  bool has_last_transform_;

  // Vars to store previous transform
  rclcpp::Time last_time_;
  tf2::Vector3 last_position_;
  tf2::Quaternion last_orientation_;
};

#endif
