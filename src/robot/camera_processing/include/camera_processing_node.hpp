#ifndef CAMERA_PROCESSING_NODE_HPP_
#define CAMERA_PROCESSING_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "camera_processing_core.hpp"

class CameraProcessingNode : public rclcpp::Node
{
public:
  CameraProcessingNode();

private:
  // message-filter sync for the two cameras' point clouds
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::PointCloud2,
          sensor_msgs::msg::PointCloud2>;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_cam1_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_cam2_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  robot::CameraProcessingCore camera_processing_;

  // Callback fired by synchronizer
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &c1,
                     const sensor_msgs::msg::PointCloud2::ConstSharedPtr &c2);
};

#endif // CAMERA_PROCESSING_NODE_HPP_
