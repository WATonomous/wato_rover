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

#ifndef PCL_CONVERTER_NODE_HPP_
#define PCL_CONVERTER_NODE_HPP_

#include "pcl_converter/pcl_converter_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PCLConverterNode : public rclcpp::Node
{
public:
  PCLConverterNode();

private:
  robot::PCLConverterCore pcl_converter_;

  // subscribers to depth cameras
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cam1_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cam2_sub_;

  // camera recieved flags
  bool has_cam1;
  bool has_cam2;


  // camera callback functions
  void cam1Callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud);
  void cam2Callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud);

  // publish function
  void publishPointCloud();

  // publisher for merged point cloud
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  
  // the actual message to publish
  sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg;
};

#endif
