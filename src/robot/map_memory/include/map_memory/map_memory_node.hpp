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

#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "map_memory/map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class MapMemoryNode : public rclcpp::Node
{
public:
  MapMemoryNode();

  void localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  // Internal utility to convert quaternion to yaw
  double quaternionToYaw(double x, double y, double z, double w);

  void processParameters();

private:
  // core logic for processing the global costmap
  robot::MapMemoryCore map_memory_;

  // ROS2 Constructs
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ROS2 Parameters
  std::string local_costmap_topic_;
  std::string odom_topic_;
  std::string map_topic_;

  int map_pub_rate_;

  double resolution_;
  int width_;
  int height_;
  geometry_msgs::msg::Pose origin_;

  double update_distance_;

  // To keep track of robot pose in sim frame
  double robot_x_;  // [m]
  double robot_y_;  // [m]
  double robot_theta_;  // [rad]

  // Last position used to check if robot moved > distance_
  double last_robot_x_;
  double last_robot_y_;
};

#endif
