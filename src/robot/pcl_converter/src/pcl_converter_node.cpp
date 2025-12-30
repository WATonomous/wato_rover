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

#include "pcl_converter/pcl_converter_node.hpp"

PCLConverterNode::PCLConverterNode()
: Node("pcl_converter")
, pcl_converter_(robot::PCLConverterCore(this->get_logger()))
{
  // subscribers for each camera
  cam1_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/sim/realsense1/depth/points", 10, std::bind(
      &PCLConverterNode::cam1_callback, this, std::placeholders::_1));
  cam2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/sim/realsense2/depth/points", 10, std::bind(
      &PCLConverterNode::cam2_callback, this, std::placeholders::_1));
}

void PCLConverterNode::cam1_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud)
{
  RCLCPP_INFO(this->get_logger(), "Cam 1 Recieved");
}

void PCLConverterNode::cam2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud)
{
  RCLCPP_INFO(this->get_logger(), "Cam 2 Recieved");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCLConverterNode>());
  rclcpp::shutdown();
  return 0;
}
