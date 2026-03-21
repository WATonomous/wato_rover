
#include "costmap/costmap_node.hpp"

CostmapNode::CostmapNode()
: Node("costmap")
, costmap_(robot::CostmapCore(this->get_logger()))
{
  processParameters();

  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, 10, std::bind(&CostmapNode::pointCloudCallback, this, std::placeholders::_1));

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

  pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
  costmap_topic_ = this->get_parameter("costmap_topic").as_string();
  resolution_ = this->get_parameter("costmap.resolution").as_double();
  width_ = this->get_parameter("costmap.width").as_int();
  height_ = this->get_parameter("costmap.height").as_int();
  inflation_radius_ = this->get_parameter("costmap.inflation_radius").as_double();
  step_threshold_ = this->get_parameter("costmap.step_threshold").as_double();
  max_range_ = this->get_parameter("costmap.max_range").as_double();
}

void CostmapNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
{
  costmap_.updateCostmapFromPointCloud(msg);

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
