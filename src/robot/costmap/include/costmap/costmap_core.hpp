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

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace robot
{

class CostmapCore
{
public:
  explicit CostmapCore(const rclcpp::Logger & logger);

  void initCostmap(
    double resolution,
    int width,
    int height,
    geometry_msgs::msg::Pose origin,
    double inflation_radius);

  void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan) const;
  void updateCostmapFromPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) const;

  nav_msgs::msg::OccupancyGrid::SharedPtr getCostmapData() const;

private:
  void inflateObstacle(int origin_x, int origin_y) const;

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;
  rclcpp::Logger logger_;
  double inflation_radius_;
  int inflation_cells_;
};

}  // namespace robot

#endif
