#include "camera_processing_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
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

  pub_cloud1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cam1_transformed", 10);
  pub_cloud2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cam2_transformed", 10);
}

void CameraProcessingNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &c1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &c2)
{
  RCLCPP_INFO(this->get_logger(),
              "Received clouds: cam1=%u points, cam2=%u points",
              c1->width * c1->height,
              c2->width * c2->height);

  pcl::PointCloud<pcl::PointXYZRGB> cloud1, cloud2;
  pcl::fromROSMsg(*c1, cloud1);
  pcl::fromROSMsg(*c2, cloud2);

  RCLCPP_INFO(this->get_logger(),
              "Transformed clouds: cam1=%zu points, cam2=%zu points",
              cloud1.size(), cloud2.size());
  
  float angle_rot1 = M_PI / 6; // 30 degrees, placeholder  
  Eigen::Affine3f cam1_to_chassis = Eigen::Affine3f::Identity(); 
  cam1_to_chassis.translation() << 0.0f, 0.0f, 0.0f; // placeholder
  cam1_to_chassis.rotate(Eigen::AngleAxisf(angle_rot1, Eigen::Vector3f::UnitY())); 

  float angle_rot2 = -M_PI / 6; // -30 degrees, placeholde
  Eigen::Affine3f cam2_to_chassis = Eigen::Affine3f::Identity();
  cam2_to_chassis.translation() << 0.0f, 0.0f, 0.0f; // placeholde
  cam2_to_chassis.rotate(Eigen::AngleAxisf(angle_rot2, Eigen::Vector3f::UnitY()));

  for (auto &pt : cloud1.points)
  {
    Eigen::Vector3f point(pt.x, pt.y, pt.z); 
    Eigen::Vector3f p = cam1_to_chassis * point;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
  }

  for (auto &pt : cloud2.points)
  {
    Eigen::Vector3f point(pt.x, pt.y, pt.z); 
    Eigen::Vector3f p = cam2_to_chassis * point; 
    p = cam2_to_chassis * point; 
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
  }

  RCLCPP_INFO(this->get_logger(),
              "Transformed clouds: cam1=%zu points, cam2=%zu points",
              cloud1.size(), cloud2.size());

  /*
  // transform both clouds to "robot/chassis" (from robot/chassis/sim_realsense_d435_1_color and robot/chassis/sim_realsense_d435_2_color frames)
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
    RCLCPP_WARN(this->get_logger(), "tf2 transform failed: %s", ex.what());
    return;
  }



  // This code will show you what fields are in a PointCloud2 message.
  // See https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointField.html for more info :)
  for (const auto &field : c1_transformed.fields)
  {
    RCLCPP_INFO(this->get_logger(), "Field name='%s', offset=%u, datatype=%u, count=%u",
                field.name.c_str(), field.offset, field.datatype, field.count);
  }

  // Convert to PointCloud2 messages to PCL (in particular pcl::PointXYZRGB)
  pcl::PointCloud<pcl::PointXYZRGB> cloud1, cloud2;
  pcl::fromROSMsg(c1_transformed, cloud1);
  pcl::fromROSMsg(c2_transformed, cloud2);

  RCLCPP_INFO(this->get_logger(),
              "Transformed clouds: cam1=%zu points, cam2=%zu points",
              cloud1.size(), cloud2.size());

  */ 
  // Check for empty or corrupted clouds, print first valid point
  // Cloud 1
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
  {
    RCLCPP_WARN(this->get_logger(), "Cloud 2 has no valid (non-NaN) points");
  }

  sensor_msgs::msg::PointCloud2 cloud1_msg;
  pcl::toROSMsg(cloud1, cloud1_msg);
  cloud1_msg.header.frame_id = "robot/chassis";
  cloud1_msg.header.stamp = c1->header.stamp;
  pub_cloud1_->publish(cloud1_msg);

  sensor_msgs::msg::PointCloud2 cloud2_msg;
  pcl::toROSMsg(cloud2, cloud2_msg);
  cloud2_msg.header.frame_id = "robot/chassis";
  cloud2_msg.header.stamp = c2->header.stamp;
  pub_cloud2_->publish(cloud2_msg);

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraProcessingNode>());
  rclcpp::shutdown();
  return 0;
}
