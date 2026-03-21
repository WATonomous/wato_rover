
#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace robot
{

class CostmapCore
{
public:
  explicit CostmapCore(const rclcpp::Logger & logger);

  void initCostmap(
    double resolution, int width, int height,
    double inflation_radius, double step_threshold, double max_range);

  void updateCostmapFromPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) const;

  nav_msgs::msg::OccupancyGrid::SharedPtr getCostmapData() const;

private:
  void inflate(std::vector<int8_t> & grid, int width, int height) const;

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;
  rclcpp::Logger logger_;

  double inflation_radius_;
  int inflation_cells_;
  double step_threshold_;
  double max_range_;
};

}  // namespace robot

#endif
