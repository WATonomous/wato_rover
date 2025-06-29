#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>

class CameraFallbackNode : public rclcpp::Node
{
public:
  CameraFallbackNode()
  : Node("camera_fallback_node"), use_real_camera_(false), last_real_msg_time_(0.0), timeout_(30.0)
  {
    // Publishers
    unified_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/depth/points", 10);

    // Subscribers
    real_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/camera/depth/color/points", 10,
      std::bind(&CameraFallbackNode::real_callback, this, std::placeholders::_1));
    sim_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/sim/realsense1/depth/points", 10,
      std::bind(&CameraFallbackNode::sim_callback, this, std::placeholders::_1));

    // Timer to check real camera activity
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(120000),
      std::bind(&CameraFallbackNode::check_camera_activity, this));

    // Static transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publish_static_transform();
  }

private:
  void real_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    last_real_msg_time_ = this->get_clock()->now().seconds();
    use_real_camera_ = true;
    unified_pub_->publish(*msg);
  }

  void sim_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!use_real_camera_) {
      unified_pub_->publish(*msg);
    }
  }

  void check_camera_activity()
  {
    double current_time = this->get_clock()->now().seconds();
    if (current_time - last_real_msg_time_ > timeout_) {
      use_real_camera_ = false;
      RCLCPP_INFO(this->get_logger(), "Real camera inactive, falling back to simulated camera");
    } else {
      RCLCPP_INFO(this->get_logger(), "Using real camera");
    }
  }

  void publish_static_transform()
  {
    // Publish static transform from chassis to camera_link

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "robot/chassis";
    transform.child_frame_id = "camera_link";
    transform.transform.translation.x = 0.8;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.5;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(transform);
    RCLCPP_INFO(this->get_logger(), "Published static transform from chassis to camera_link");
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unified_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr real_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sim_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  bool use_real_camera_;
  double last_real_msg_time_;
  double timeout_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraFallbackNode>());
  rclcpp::shutdown();
  return 0;
}