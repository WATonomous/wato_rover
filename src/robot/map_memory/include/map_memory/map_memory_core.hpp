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

#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class MapMemoryCore
{
public:
  explicit MapMemoryCore(const rclcpp::Logger & logger);

  void initMapMemory(double resolution, int width, int height, geometry_msgs::msg::Pose origin);

  void updateMap(
    nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap, double robot_x, double robot_y, double robot_theta);

  bool robotToMap(double rx, double ry, int & mx, int & my);

  // Load elevation cost grid from a preprocessed CSV file
  void loadElevationGrid(const std::string & csv_path);

  // Apply elevation costs as a base layer on the global map (call once after init)
  void applyElevationLayer();

  // Retrieves map data
  nav_msgs::msg::OccupancyGrid::SharedPtr getMapData() const;

private:
  nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
  rclcpp::Logger logger_;

  std::vector<int8_t> elevation_grid_;
  bool elevation_loaded_ = false;
};

}  // namespace robot

#endif
