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
  // 1. Print raw cloud metadata
  RCLCPP_INFO(this->get_logger(),
              "Received clouds: cam1=%u points, cam2=%u points",
              c1->width * c1->height,
              c2->width * c2->height);

  // 2. Transform both clouds to "robot/chassis"
  sensor_msgs::msg::PointCloud2 c1_transformed;
  sensor_msgs::msg::PointCloud2 c2_transformed;

  try
  {
    c1_transformed = tf_buffer_->transform(*c1, "robot/chassis", tf2::durationFromSec(0.1));
    c2_transformed = tf_buffer_->transform(*c2, "robot/chassis", tf2::durationFromSec(0.1));
    RCLCPP_INFO(this->get_logger(), "c1_transformed: frame_id='%s', stamp=%.3f, width=%u, height=%u, point_step=%u, row_step=%u",
                c1_transformed.header.frame_id.c_str(),
                rclcpp::Time(c1_transformed.header.stamp).seconds(),
                c1_transformed.width,
                c1_transformed.height,
                c1_transformed.point_step,
                c1_transformed.row_step);

    RCLCPP_INFO(this->get_logger(), "c2_transformed: frame_id='%s', stamp=%.3f, width=%u, height=%u, point_step=%u, row_step=%u",
                c2_transformed.header.frame_id.c_str(),
                rclcpp::Time(c2_transformed.header.stamp).seconds(),
                c2_transformed.width,
                c2_transformed.height,
                c2_transformed.point_step,
                c2_transformed.row_step);
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
    return;
  }

  float x = 0, y = 0, z = 0;
  memcpy(&x, &c1_transformed.data[0], sizeof(float));
  memcpy(&y, &c1_transformed.data[4], sizeof(float));
  memcpy(&z, &c1_transformed.data[8], sizeof(float));
  RCLCPP_INFO(this->get_logger(), "First point (raw): x=%.2f y=%.2f z=%.2f", x, y, z);
  for (const auto &field : c1_transformed.fields)
  {
    RCLCPP_INFO(this->get_logger(), "Field name='%s', offset=%u, datatype=%u, count=%u",
                field.name.c_str(), field.offset, field.datatype, field.count);
  }
  // 3. Convert to PCL
  pcl::PointCloud<pcl::PointXYZRGB> cloud1, cloud2;
  pcl::fromROSMsg(c1_transformed, cloud1);
  pcl::fromROSMsg(c2_transformed, cloud2);

  RCLCPP_INFO(this->get_logger(),
              "Transformed clouds: cam1=%zu points, cam2=%zu points",
              cloud1.size(), cloud2.size());

  // 4. Check for empty or corrupted clouds
  // Print first valid point (cloud 1)
  bool found_valid_1 = false;
  for (const auto &pt : cloud1)
  {
    if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z))
    {
      RCLCPP_INFO(this->get_logger(), "Cloud 1 first valid point: x=%.2f y=%.2f z=%.2f", pt.x, pt.y, pt.z);
      found_valid_1 = true;
      break;
    }
  }
  if (!found_valid_1)
    RCLCPP_WARN(this->get_logger(), "Cloud 1 has no valid (non-NaN) points");

  // Cloud 2
  bool found_valid_2 = false;
  for (const auto &pt : cloud2)
  {
    if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z))
    {
      RCLCPP_INFO(this->get_logger(), "Cloud 2 first valid point: x=%.2f y=%.2f z=%.2f", pt.x, pt.y, pt.z);
      found_valid_2 = true;
      break;
    }
  }
  if (!found_valid_2)
    RCLCPP_WARN(this->get_logger(), "Cloud 2 has no valid (non-NaN) points");

  // TODO: transform, costmap update, inflation, publish OccupancyGrid
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraProcessingNode>());
  rclcpp::shutdown();
  return 0;
}
