#include "camera_processing_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>

CameraProcessingNode::CameraProcessingNode()
    : Node("camera_processing"),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
      camera_processing_(robot::CameraProcessingCore(this->get_logger()))
{
  sub_cam1_.subscribe(this, "/sim/realsense1/depth/points");
  sub_cam2_.subscribe(this, "/sim/realsense2/depth/points");

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10),
                                                                      sub_cam1_,
                                                                      sub_cam2_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));
  sync_->registerCallback(std::bind(&CameraProcessingNode::cloudCallback,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2));
}

void CameraProcessingNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &c1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &c2)
{
  RCLCPP_INFO(this->get_logger(),
              "Received synced clouds @ %.3f and %.3f",
              rclcpp::Time(c1->header.stamp).seconds(),
              rclcpp::Time(c2->header.stamp).seconds());

  // TODO: transform, costmap update, inflation, publish OccupancyGrid
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraProcessingNode>());
  rclcpp::shutdown();
  return 0;
}
