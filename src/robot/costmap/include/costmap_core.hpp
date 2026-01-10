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

#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace robot
{

class CostmapCore
{
public:
  // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
  explicit CostmapCore(const rclcpp::Logger & logger);

  // Initializes the Costmap with the parameters that we get from the params.yaml
  void initCostmap(double resolution, int width, int height, geometry_msgs::msg::Pose origin, double inflation_radius);

  // Update the costmap based on the current laserscan reading
  void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan) const;

  // Update the costmap based on point cloud data from RGBD camera
  void updateCostmapFromPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) const;

  // Inflate the obstacle in the laserscan on the costmap because we want of range of values
  // where we can and cannot go
  void inflateObstacle(int origin_x, int origin_y) const;

  // Retrieves costmap data
  nav_msgs::msg::OccupancyGrid::SharedPtr getCostmapData() const;

private:
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;
  rclcpp::Logger logger_;

  double inflation_radius_;
  int inflation_cells_;
};

}  // namespace robot

#endif
