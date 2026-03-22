
#include "costmap/costmap_node.hpp"

#include <cmath>

CostmapNode::CostmapNode()
: Node("costmap")
, costmap_(robot::CostmapCore(this->get_logger()))
{
  processParameters();

  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, 10, std::bind(&CostmapNode::pointCloudCallback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, 10, std::bind(&CostmapNode::imuCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic_, 10);

  costmap_.initCostmap(resolution_, width_, height_, inflation_radius_, step_threshold_, max_range_);

  RCLCPP_INFO(this->get_logger(), "Costmap node ready, subscribed to: %s", pointcloud_topic_.c_str());
}

void CostmapNode::processParameters()
{
  this->declare_parameter<std::string>("pointcloud_topic", "/camera/points");
  this->declare_parameter<std::string>("costmap_topic", "/costmap");
  this->declare_parameter<double>("costmap.resolution", 0.2);
  this->declare_parameter<int>("costmap.width", 100);
  this->declare_parameter<int>("costmap.height", 100);
  this->declare_parameter<double>("costmap.inflation_radius", 0.5);
  this->declare_parameter<double>("costmap.step_threshold", 0.3);
  this->declare_parameter<double>("costmap.max_range", 10.0);
  this->declare_parameter<std::string>("imu_topic", "/imu/simulated");
  this->declare_parameter<double>("costmap.pitch_threshold", 0.1);

  pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
  costmap_topic_ = this->get_parameter("costmap_topic").as_string();
  imu_topic_ = this->get_parameter("imu_topic").as_string();
  resolution_ = this->get_parameter("costmap.resolution").as_double();
  width_ = this->get_parameter("costmap.width").as_int();
  height_ = this->get_parameter("costmap.height").as_int();
  inflation_radius_ = this->get_parameter("costmap.inflation_radius").as_double();
  step_threshold_ = this->get_parameter("costmap.step_threshold").as_double();
  max_range_ = this->get_parameter("costmap.max_range").as_double();
  pitch_threshold_ = this->get_parameter("costmap.pitch_threshold").as_double();
}

void CostmapNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Extract pitch from quaternion using atan2(2(qw*qy - qz*qx), 1 - 2(qx^2 + qy^2))
  double qw = msg->orientation.w;
  double qx = msg->orientation.x;
  double qy = msg->orientation.y;
  double qz = msg->orientation.z;
  double sinp = 2.0 * (qw * qy - qz * qx);
  current_pitch_ = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
}

void CostmapNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
{
  // Only apply pitch compensation when pitch exceeds threshold
  double pitch = (std::abs(current_pitch_) > pitch_threshold_) ? current_pitch_ : 0.0;
  costmap_.updateCostmapFromPointCloud(msg, pitch);

  nav_msgs::msg::OccupancyGrid costmap_msg = *costmap_.getCostmapData();
  costmap_msg.header = msg->header;
  costmap_pub_->publish(costmap_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
