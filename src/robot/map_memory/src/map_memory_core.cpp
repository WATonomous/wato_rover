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

#include "map_memory/map_memory_core.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger & logger)
: global_map_(std::make_shared<nav_msgs::msg::OccupancyGrid>())
, logger_(logger)
, blend_alpha_(0.35)
{}

void MapMemoryCore::setFusionParameters(double blend_alpha)
{
  blend_alpha_ = std::clamp(blend_alpha, 0.0, 1.0);
}

void MapMemoryCore::initMapMemory(double resolution, int width, int height, geometry_msgs::msg::Pose origin)
{
  global_map_->info.resolution = resolution;
  global_map_->info.width = width;
  global_map_->info.height = height;
  global_map_->info.origin = origin;
  global_map_->data.assign(width * height, 0);

  RCLCPP_INFO(
    logger_, "Global Map initialized with resolution: %.2f, width: %d, height: %d", resolution, width, height);
}

void MapMemoryCore::updateMap(
  nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap, double robot_x, double robot_y, double robot_theta)
{
  // Get local costmap specs
  double local_res = local_costmap->info.resolution;
  double local_origin_x = local_costmap->info.origin.position.x;
  double local_origin_y = local_costmap->info.origin.position.y;
  unsigned int local_w = local_costmap->info.width;
  unsigned int local_h = local_costmap->info.height;
  const auto & local_data = local_costmap->data;

  const size_t global_cells = static_cast<size_t>(global_map_->info.width) * global_map_->info.height;
  std::vector<int> observed_sum(global_cells, 0);
  std::vector<int> observed_count(global_cells, 0);

  // For each cell in local costmap, transform to sim_world
  for (unsigned int j = 0; j < local_h; ++j) {
    for (unsigned int i = 0; i < local_w; ++i) {
      int8_t occ_val = local_data[j * local_w + i];
      if (occ_val < 0) {
        // Unknown => skip or handle differently
        continue;
      }
      // Convert (i,j) to local metric coords relative to "robot" frame
      double lx = local_origin_x + (i + 0.5) * local_res;  // center of cell
      double ly = local_origin_y + (j + 0.5) * local_res;

      // Now transform (lx, ly) from "robot" frame to "sim_world" frame
      // using the robot pose (robot_x_, robot_y_, robot_theta_).
      // Basic 2D transform:
      //   wx = robot_x + cos(theta)*lx - sin(theta)*ly
      //   wy = robot_y + sin(theta)*lx + cos(theta)*ly
      double cos_t = std::cos(robot_theta);
      double sin_t = std::sin(robot_theta);
      double wx = robot_x + (lx * cos_t - ly * sin_t);
      double wy = robot_y + (lx * sin_t + ly * cos_t);

      // Convert (wx, wy) to indices in the global map
      int gx, gy;
      if (!robotToMap(wx, wy, gx, gy)) {
        // Out of global map bounds
        continue;
      }

      const size_t global_index = static_cast<size_t>(gy) * global_map_->info.width + gx;
      observed_sum[global_index] += static_cast<int>(occ_val);
      observed_count[global_index] += 1;
    }
  }

  // Blend each observed cell toward the local costmap value, preserving inflation gradients.
  for (size_t idx = 0; idx < global_cells; ++idx) {
    if (observed_count[idx] == 0) {
      continue;
    }

    const double observed_cost = static_cast<double>(observed_sum[idx]) / observed_count[idx];
    const double current_cost = std::max(0, static_cast<int>(global_map_->data[idx]));
    const double fused_cost = (1.0 - blend_alpha_) * current_cost + blend_alpha_ * observed_cost;
    global_map_->data[idx] = static_cast<int8_t>(std::round(std::clamp(fused_cost, 0.0, 100.0)));
  }
}

bool MapMemoryCore::robotToMap(double rx, double ry, int & mx, int & my)
{
  double origin_x = global_map_->info.origin.position.x;
  double origin_y = global_map_->info.origin.position.y;
  double resolution = global_map_->info.resolution;

  // offset from origin
  if (rx < origin_x || ry < origin_y) {
    return false;
  }

  my = static_cast<int>((ry - origin_y) / resolution);
  mx = static_cast<int>((rx - origin_x) / resolution);

  if (
    mx < 0 || mx >= static_cast<int>(global_map_->info.width) || my < 0 ||
    my >= static_cast<int>(global_map_->info.height))
  {
    return false;
  }
  return true;
}

// Retrieves map data
nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryCore::getMapData() const
{
  return global_map_;
}

}  // namespace robot
