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

#include "pcl_converter/pcl_converter_core.hpp"

namespace robot
{

PCLConverterCore::PCLConverterCore(const rclcpp::Logger & logger)
: logger_(logger)
{
    // make sure they are not null ptr
    pcl_point_cloud_1_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl_point_cloud_2_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl_point_cloud_merged_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    merged_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
}


// processes pointclouds and stores them to be used for the merge
void PCLConverterCore::processPointCloudMsg(
    const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msg_, int cam_num) 
{
    // convert based on the camera it comes from
    if(cam_num == 1)
    {
        pcl::fromROSMsg(*point_cloud2_msg_, *pcl_point_cloud_1_);
        
    } else if(cam_num == 2)
    {
        pcl::fromROSMsg(*point_cloud2_msg_, *pcl_point_cloud_2_);          
    }
}

// merges point cloud msgs and converts them to ros msgs
sensor_msgs::msg::PointCloud2::SharedPtr PCLConverterCore::mergePointCloudMsgs()
{
    // ** NEEDS WORK **
    *pcl_point_cloud_merged_ = *pcl_point_cloud_1_ + *pcl_point_cloud_2_;

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*pcl_point_cloud_merged_, output_msg);


    return std::make_shared<sensor_msgs::msg::PointCloud2>(output_msg);}

}  // namespace robot
