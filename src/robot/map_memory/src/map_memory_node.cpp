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

#include "map_memory/map_memory_node.hpp"

#include <string>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

MapMemoryNode::MapMemoryNode()
: Node("map_memory")
, map_memory_(robot::MapMemoryCore(this->get_logger()))
, robot_x_(0.0)
, robot_y_(0.0)
, robot_theta_(0.0)
, has_odom_(false)
, last_robot_x_(std::numeric_limits<double>::quiet_NaN())
, last_robot_y_(std::numeric_limits<double>::quiet_NaN())
, last_update_time_(this->now())
{
  // load ROS2 yaml parameters
  processParameters();

  // Subscribe to local costmap
  local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    local_costmap_topic_, 10, std::bind(&MapMemoryNode::localCostmapCallback, this, std::placeholders::_1));

  // Subscribe to odometry
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Publish a global costmap for downstream path planning
  global_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, 10);

  timer_ =
    this->create_wall_timer(std::chrono::milliseconds(map_pub_rate_), std::bind(&MapMemoryNode::timerCallback, this));

  map_memory_.setFusionParameters(blend_alpha_);
  map_memory_.initMapMemory(resolution_, width_, height_, origin_);

  RCLCPP_INFO(this->get_logger(), "Initialized Map Memory Core");
}

void MapMemoryNode::processParameters()
{
  // Declare all ROS2 Parameters
  this->declare_parameter<std::string>("local_costmap_topic", "/costmap");
  this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  this->declare_parameter<std::string>("map_topic", "/map");
  this->declare_parameter<std::string>("map_fusion.expected_local_costmap_frame", "");
  this->declare_parameter<std::string>("map_fusion.fusion_frame", "sim_world");
  this->declare_parameter<double>("map_fusion.blend_alpha", 0.35);
  this->declare_parameter<int>("map_pub_rate", 500);
  this->declare_parameter<double>("update_distance", 1.0);
  this->declare_parameter<int>("min_update_period_ms", 0);
  this->declare_parameter<double>("global_map.resolution", 0.4);
  this->declare_parameter<int>("global_map.width", 120);
  this->declare_parameter<int>("global_map.height", 120);
  this->declare_parameter<double>("global_map.origin.position.x", -24.0);
  this->declare_parameter<double>("global_map.origin.position.y", -24.0);
  this->declare_parameter<double>("global_map.origin.position.z", 0.1);
  this->declare_parameter<double>("global_map.origin.orientation.w", 1.0);

  // Retrieve parameters and store them in member variables
  local_costmap_topic_ = this->get_parameter("local_costmap_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  map_topic_ = this->get_parameter("map_topic").as_string();
  expected_local_costmap_frame_ = this->get_parameter("map_fusion.expected_local_costmap_frame").as_string();
  fusion_frame_ = this->get_parameter("map_fusion.fusion_frame").as_string();
  blend_alpha_ = this->get_parameter("map_fusion.blend_alpha").as_double();
  map_pub_rate_ = this->get_parameter("map_pub_rate").as_int();
  update_distance_ = this->get_parameter("update_distance").as_double();
  min_update_period_ms_ = this->get_parameter("min_update_period_ms").as_int();
  resolution_ = this->get_parameter("global_map.resolution").as_double();
  width_ = this->get_parameter("global_map.width").as_int();
  height_ = this->get_parameter("global_map.height").as_int();
  origin_.position.x = this->get_parameter("global_map.origin.position.x").as_double();
  origin_.position.y = this->get_parameter("global_map.origin.position.y").as_double();
  origin_.position.z = this->get_parameter("global_map.origin.position.z").as_double();
  origin_.orientation.w = this->get_parameter("global_map.origin.orientation.w").as_double();
}

void MapMemoryNode::localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!has_odom_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Skipping map fusion until odometry has been received.");
    return;
  }

  if (!expected_local_costmap_frame_.empty() && msg->header.frame_id != expected_local_costmap_frame_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Skipping local costmap fusion due to frame mismatch. Expected '%s', got '%s'.",
      expected_local_costmap_frame_.c_str(), msg->header.frame_id.c_str());
    return;
  }

  // Update map if either robot moved enough OR timeout elapsed.
  bool moved_enough = true;
  if (!std::isnan(last_robot_x_)) {
    double dist = std::hypot(robot_x_ - last_robot_x_, robot_y_ - last_robot_y_);
    moved_enough = dist >= update_distance_;
  }

  bool period_elapsed = true;
  if (min_update_period_ms_ > 0) {
    const auto now = this->now();
    period_elapsed = (now - last_update_time_) >= rclcpp::Duration::from_nanoseconds(
      static_cast<int64_t>(min_update_period_ms_) * 1000000LL);
  }

  if (!moved_enough && !period_elapsed) {
    return;
  }

  // Update last position
  last_robot_x_ = robot_x_;
  last_robot_y_ = robot_y_;
  last_update_time_ = this->now();

  // Update the global map
  map_memory_.updateMap(msg, robot_x_, robot_y_, robot_theta_);
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!fusion_frame_.empty() && msg->header.frame_id != fusion_frame_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Odometry frame '%s' does not match configured fusion frame '%s'.",
      msg->header.frame_id.c_str(), fusion_frame_.c_str());
  }

  // Extract the robot’s position and orientation from the Odometry message.
  // Assume this odometry is in the "sim_world" frame or a frame equivalent to it.
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  // Convert quaternion to yaw
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  robot_theta_ = quaternionToYaw(qx, qy, qz, qw);
  has_odom_ = true;
}

void MapMemoryNode::timerCallback()
{
  // Publish the map every map_pub_rate [ms]
  nav_msgs::msg::OccupancyGrid map_msg = *map_memory_.getMapData();
  map_msg.header.stamp = this->now();
  map_msg.header.frame_id = fusion_frame_;
  // Set z-offset to render map slightly above ground plane (on top of URDF)
  map_msg.info.origin.position.z = origin_.position.z;
  global_costmap_pub_->publish(map_msg);
}

// Utility: Convert quaternion to yaw
double MapMemoryNode::quaternionToYaw(double x, double y, double z, double w)
{
  // Using tf2 to convert to RPY
  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
