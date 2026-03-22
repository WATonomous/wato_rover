
#include "costmap/costmap_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger & logger)
: costmap_data_(std::make_shared<nav_msgs::msg::OccupancyGrid>())
, logger_(logger)
{}

void CostmapCore::initCostmap(
  double resolution, int width, int height,
  double inflation_radius, double step_threshold, double max_range)
{
  costmap_data_->info.resolution = resolution;
  costmap_data_->info.width = width;
  costmap_data_->info.height = height;

  costmap_data_->info.origin.position.x = -(width * resolution) / 2.0;
  costmap_data_->info.origin.position.y = -(height * resolution) / 2.0;
  costmap_data_->info.origin.orientation.w = 1.0;

  costmap_data_->data.assign(width * height, 0);

  inflation_radius_ = inflation_radius;
  inflation_cells_ = static_cast<int>(std::ceil(inflation_radius / resolution));
  step_threshold_ = step_threshold;
  max_range_ = max_range;

  RCLCPP_INFO(
    logger_, "Costmap initialized: %dx%d @ %.2fm/cell, inflation=%.2fm, step=%.2fm, range=%.1fm",
    width, height, resolution, inflation_radius, step_threshold, max_range);
}

void CostmapCore::updateCostmapFromPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud, double pitch) const
{
  const int w = costmap_data_->info.width;
  const int h = costmap_data_->info.height;
  const int n = w * h;
  const double res = costmap_data_->info.resolution;
  const double ox = costmap_data_->info.origin.position.x;
  const double oy = costmap_data_->info.origin.position.y;

  // Per-cell height tracking
  const float NEG_INF = -std::numeric_limits<float>::infinity();
  const float POS_INF = std::numeric_limits<float>::infinity();
  std::vector<float> min_z(n, POS_INF);
  std::vector<float> max_z(n, NEG_INF);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");

  // Precompute pitch correction (rotate points around Y axis by -pitch to level the frame)
  const float cos_p = std::cos(-pitch);
  const float sin_p = std::sin(-pitch);

  // Pass 1: accumulate min/max z per grid cell
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    float px = *iter_x;
    float py = *iter_y;
    float pz = *iter_z;

    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) continue;

    // Compensate for robot pitch: rotate (px, pz) to level frame
    float corrected_x = px * cos_p + pz * sin_p;
    float corrected_z = -px * sin_p + pz * cos_p;
    px = corrected_x;
    pz = corrected_z;

    if (std::hypot(px, py) > max_range_) continue;

    int gx = static_cast<int>((px - ox) / res);
    int gy = static_cast<int>((py - oy) / res);
    if (gx < 0 || gx >= w || gy < 0 || gy >= h) continue;

    int idx = gy * w + gx;
    min_z[idx] = std::min(min_z[idx], pz);
    max_z[idx] = std::max(max_z[idx], pz);
  }

  // Pass 2: mark obstacle cells where height gradient exceeds threshold
  // A cell is an obstacle if:
  //   (a) it has a large internal height span (e.g. boulder face), OR
  //   (b) its max_z is much higher than a neighbor's max_z (step/cliff edge)
  std::fill(costmap_data_->data.begin(), costmap_data_->data.end(), 0);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      int idx = y * w + x;
      if (max_z[idx] == NEG_INF) continue;  // no points in this cell

      // Check internal height span
      float span = max_z[idx] - min_z[idx];
      if (span > step_threshold_) {
        costmap_data_->data[idx] = 100;
        continue;
      }

      // Check height difference to neighbors
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;
          int nx = x + dx, ny = y + dy;
          if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
          int ni = ny * w + nx;
          if (max_z[ni] == NEG_INF) continue;

          float diff = std::abs(max_z[idx] - max_z[ni]);
          if (diff > step_threshold_) {
            costmap_data_->data[idx] = 100;
            goto next_cell;
          }
        }
      }
      next_cell:;
    }
  }

  inflate(costmap_data_->data, w, h);
}

void CostmapCore::inflate(std::vector<int8_t> & grid, int width, int height) const
{
  if (inflation_cells_ <= 0) return;

  std::queue<std::pair<int, int>> queue;
  std::vector<int> dist(width * height, -1);

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (grid[y * width + x] == 100) {
        queue.emplace(x, y);
        dist[y * width + x] = 0;
      }
    }
  }

  const double res = costmap_data_->info.resolution;
  const int dirs[][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

  while (!queue.empty()) {
    auto [x, y] = queue.front();
    queue.pop();

    for (auto & d : dirs) {
      int nx = x + d[0];
      int ny = y + d[1];
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

      int ni = ny * width + nx;
      if (dist[ni] >= 0) continue;

      int new_dist = dist[y * width + x] + 1;
      double real_dist = new_dist * res;
      if (real_dist > inflation_radius_) continue;

      dist[ni] = new_dist;

      int8_t cost = static_cast<int8_t>((1.0 - real_dist / inflation_radius_) * 99);
      if (cost > grid[ni]) {
        grid[ni] = cost;
      }

      queue.emplace(nx, ny);
    }
  }
}

nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::getCostmapData() const
{
  return costmap_data_;
}

}  // namespace robot
