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

#include "costmap_core.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger & logger)
: costmap_data_(std::make_shared<nav_msgs::msg::OccupancyGrid>())
, logger_(logger)
{}

void CostmapCore::initCostmap(
  double resolution, int width, int height, geometry_msgs::msg::Pose origin, double inflation_radius)
{
  costmap_data_->info.resolution = resolution;
  costmap_data_->info.width = width;
  costmap_data_->info.height = height;
  costmap_data_->info.origin = origin;
  costmap_data_->data.assign(width * height, -1);

  inflation_radius_ = inflation_radius;
  inflation_cells_ = static_cast<int>(inflation_radius / resolution);

  RCLCPP_INFO(logger_, "Costmap initialized with resolution: %.2f, width: %d, height: %d", resolution, width, height);
}

void CostmapCore::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan) const
{
  // Reset the costmap to free space
  std::fill(costmap_data_->data.begin(), costmap_data_->data.end(), 0);

  double angle = laserscan->angle_min;
  for (size_t i = 0; i < laserscan->ranges.size(); ++i, angle += laserscan->angle_increment) {
    double range = laserscan->ranges[i];

    // Check if the range is within the valid range
    if (range >= laserscan->range_min && range <= laserscan->range_max) {
      // Calculate obstacle position in the map frame
      double x = range * std::cos(angle);
      double y = range * std::sin(angle);

      // Convert to grid coordinates
      int grid_x = static_cast<int>((x - costmap_data_->info.origin.position.x) / costmap_data_->info.resolution);
      int grid_y = static_cast<int>((y - costmap_data_->info.origin.position.y) / costmap_data_->info.resolution);

      if (
        grid_x >= 0 && grid_x < static_cast<int>(costmap_data_->info.width) && grid_y >= 0 &&
        grid_y < static_cast<int>(costmap_data_->info.height))
      {
        // Mark the cell as occupied
        int index = grid_y * costmap_data_->info.width + grid_x;
        costmap_data_->data[index] = 100;  // 100 indicates an occupied cell

        // Inflate around the obstacle
        inflateObstacle(grid_x, grid_y);
      }
    }
  }
}

void CostmapCore::updateCostmapFromPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) const
{
  // Reset the costmap to free space
  std::fill(costmap_data_->data.begin(), costmap_data_->data.end(), 0);

  // Create iterators for x, y, z fields in the point cloud
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");

  // Take a thin horizontal slice at a specific height (like a 2D laser scan)
  // Filter by z-height to get points at obstacle level (ignore ground)
  const double min_height = 0.05;  // Minimum height - ignore ground points below this
  const double max_height = 0.15;  // Maximum height - only 10cm band

  // Downsample: only process every Nth point to reduce computation
  // A 640x480 cloud has 307k points, we only need ~256-500 for a laser-scan-like result
  const int point_skip = 20;  // Process every 20th point (307k/20 ≈ 15k points checked)

  int point_count = 0;

  // Iterate through points in the cloud with downsampling
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    // Downsample: skip most points
    if (++point_count % point_skip != 0) {
      continue;
    }

    float x = *iter_x;
    float y = *iter_y;
    float z = *iter_z;

    // Skip invalid points (NaN or Inf)
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    // Only consider points in a thin horizontal slice at a specific height
    // This creates a 2D laser-scan-like view, ignoring ground points
    if (z < min_height || z > max_height) {
      continue;
    }

    // Use x and y directly from the point cloud (in the sensor's frame)
    // This is exactly like the laser scan approach
    double obstacle_x = x;
    double obstacle_y = y;

    // Convert to grid coordinates (same logic as laser scan)
    int grid_x =
      static_cast<int>((obstacle_x - costmap_data_->info.origin.position.x) / costmap_data_->info.resolution);
    int grid_y =
      static_cast<int>((obstacle_y - costmap_data_->info.origin.position.y) / costmap_data_->info.resolution);

    // Check if within costmap bounds
    if (
      grid_x >= 0 && grid_x < static_cast<int>(costmap_data_->info.width) && grid_y >= 0 &&
      grid_y < static_cast<int>(costmap_data_->info.height))
    {
      // Mark the cell as occupied
      int index = grid_y * costmap_data_->info.width + grid_x;
      costmap_data_->data[index] = 100;  // 100 indicates an occupied cell

      // Inflate around the obstacle
      inflateObstacle(grid_x, grid_y);
    }
  }
}

void CostmapCore::inflateObstacle(int origin_x, int origin_y) const
{
  // Use a simple breadth-first search (BFS) to mark cells within the inflation radius
  std::queue<std::pair<int, int>> queue;
  queue.emplace(origin_x, origin_y);

  std::vector<std::vector<bool>> visited(
    costmap_data_->info.width, std::vector<bool>(costmap_data_->info.height, false));
  visited[origin_x][origin_y] = true;

  while (!queue.empty()) {
    auto [x, y] = queue.front();
    queue.pop();

    // Iterate over neighboring cells
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;  // Skip the center cell

        int nx = x + dx;
        int ny = y + dy;

        // Ensure the neighbor cell is within bounds
        if (
          nx >= 0 && nx < static_cast<int>(costmap_data_->info.width) && ny >= 0 &&
          ny < static_cast<int>(costmap_data_->info.height) && !visited[nx][ny])
        {
          // Calculate the distance to the original obstacle cell
          double distance = std::hypot(nx - origin_x, ny - origin_y) * costmap_data_->info.resolution;

          // If within inflation radius, mark as inflated and add to queue
          if (distance <= inflation_radius_) {
            int index = ny * costmap_data_->info.width + nx;
            if (costmap_data_->data[index] < (1 - (distance / inflation_radius_)) * 100) {
              costmap_data_->data[index] = (1 - (distance / inflation_radius_)) * 100;
            }
            queue.emplace(nx, ny);
          }

          visited[nx][ny] = true;
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::getCostmapData() const
{
  return costmap_data_;
}

}  // namespace robot
