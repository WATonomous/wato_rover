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
      &PCLConverterNode::cam1Callback, this, std::placeholders::_1));
  cam2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/sim/realsense2/depth/points", 10, std::bind(
      &PCLConverterNode::cam2Callback, this, std::placeholders::_1));

  // publisher for merged pointcloud
  point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
  "pcl_converter/mergedpointcloud", 10);

  // set flags to false
  has_cam1 = false;
  has_cam2 = false;

  // initialize the msg
  point_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

}

// callback for camera 1 and processes what is recieved 
// and store in pcl_converter_core for merge
void PCLConverterNode::cam1Callback(const 
  sensor_msgs::msg::PointCloud2::SharedPtr point_cloud)
{
  pcl_converter_.processPointCloudMsg(point_cloud, 1);
  has_cam1 = true;

  RCLCPP_INFO(this->get_logger(), "Frame_ID of cam1: %s", point_cloud->header.frame_id.c_str());

  // only publishes if both recieved
  publishPointCloud();
}

// callback for camera 2 and processes what is recieved
// and store in pcl_converter_core for merge
void PCLConverterNode::cam2Callback(const 
  sensor_msgs::msg::PointCloud2::SharedPtr point_cloud)
{
  pcl_converter_.processPointCloudMsg(point_cloud, 2);
  has_cam2 = true;

  RCLCPP_INFO(this->get_logger(), "Frame_ID of cam2: %s", point_cloud->header.frame_id.c_str());

  // only publishes if both recieved
  publishPointCloud();
}

// publishes the pointcloud
void PCLConverterNode::publishPointCloud()
{
  // dont publish if no point cloud
  if(!has_cam1 || !has_cam2)
  {
    return;
  }
  
  // merge the point clouds
  point_cloud_msg = pcl_converter_.mergePointCloudMsgs();

  // set flags back to false
  has_cam1 = false;
  has_cam2 = false;

  // set the header
  point_cloud_msg->header.frame_id = "robot/chassis";
  point_cloud_msg->header.stamp = this->now();

  // publish and log
  point_cloud_pub_->publish(*point_cloud_msg);
  // RCLCPP_INFO(this->get_logger(), "Point Cloud published");
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCLConverterNode>());
  rclcpp::shutdown();
  return 0;
}
