
#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <string>

#include "costmap/costmap_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class CostmapNode : public rclcpp::Node
{
public:
  CostmapNode();

private:
  void processParameters();
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  robot::CostmapCore costmap_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

  std::string pointcloud_topic_;
  std::string costmap_topic_;
  std::string imu_topic_;

  double resolution_;
  int width_;
  int height_;
  double inflation_radius_;
  double step_threshold_;
  double max_range_;
  double pitch_threshold_;

  double current_pitch_ = 0.0;
};

#endif
