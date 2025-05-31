#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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
      "/sim/realsense/depth/points", 10,
      std::bind(&CameraFallbackNode::sim_callback, this, std::placeholders::_1));

    // Timer to check real camera activity
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(120000),
      std::bind(&CameraFallbackNode::check_camera_activity, this));
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

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unified_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr real_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sim_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
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