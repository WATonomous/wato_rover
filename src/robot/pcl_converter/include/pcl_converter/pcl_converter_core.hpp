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

#ifndef PCL_CONVERTER_CORE_HPP_
#define PCL_CONVERTER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/impl/point_types.hpp"

namespace robot
{

class PCLConverterCore
{
public:
  explicit PCLConverterCore(const rclcpp::Logger & logger);

  // processes the point cloud
  void processPointCloudMsg(
    const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msg_, int cam_num);

  // function that merges the point clouds
  sensor_msgs::msg::PointCloud2::SharedPtr mergePointCloudMsgs();

private:
  rclcpp::Logger logger_;
  
  // pcl pointcloud pointers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_point_cloud_1_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_point_cloud_2_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_point_cloud_merged_;
  sensor_msgs::msg::PointCloud2::SharedPtr merged_msg;
};

}  // namespace robot

#endif