#include <chrono>
#include <memory>
#include <cmath>

#include "gps_sim_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

GpsSimNode::GpsSimNode() 
  : Node("gps_sim_node"),
    gen_(rd_()),
    gaussian_noise_(0.0, 1.0)
{
    processParameters();

    // Create publisher
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);

    // Create TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create timer to publish GPS at specified rate
    int period_ms = static_cast<int>(1000.0 / gps_update_rate_);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&GpsSimNode::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "GPS Simulation Node initialized");
    RCLCPP_INFO(this->get_logger(), "  Update rate: %.1f Hz", gps_update_rate_);
    RCLCPP_INFO(this->get_logger(), "  Noise std dev: %.2f m", gps_noise_std_);
    RCLCPP_INFO(this->get_logger(), "  Dropout probability: %.2f", gps_dropout_probability_);
}

void GpsSimNode::processParameters() {
    // Declare parameters
    this->declare_parameter<std::string>("source_frame", "sim_world");
    this->declare_parameter<std::string>("target_frame", "robot/chassis/camera");
    this->declare_parameter<double>("gps_noise_std", 2.0);
    this->declare_parameter<double>("gps_update_rate", 1.0);
    this->declare_parameter<double>("gps_dropout_probability", 0.0);
    this->declare_parameter<bool>("simulate_dropouts", false);

    // Get parameters
    source_frame_ = this->get_parameter("source_frame").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    gps_noise_std_ = this->get_parameter("gps_noise_std").as_double();
    gps_update_rate_ = this->get_parameter("gps_update_rate").as_double();
    gps_dropout_probability_ = this->get_parameter("gps_dropout_probability").as_double();
    simulate_dropouts_ = this->get_parameter("simulate_dropouts").as_bool();
}

void GpsSimNode::timerCallback() {
    // Look up transform from source_frame to target_frame
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_->lookupTransform(
            source_frame_,
            target_frame_,
            tf2::TimePointZero  // latest available transform
        );
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Could not transform %s to %s: %s",
                            source_frame_.c_str(), target_frame_.c_str(), ex.what());
        return;
    }

    // Check for GPS dropout
    if (simulate_dropouts_) {
        double random_value = static_cast<double>(gen_()) / static_cast<double>(gen_.max());
        if (random_value < gps_dropout_probability_) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "GPS dropout simulated - not publishing fix");
            return;
        }
    }

    // Create GPS message
    sensor_msgs::msg::NavSatFix gps_msg;

    // Fill header
    gps_msg.header.stamp = transform_stamped.header.stamp;
    gps_msg.header.frame_id = source_frame_;

    // Get ground truth position
    double gt_x = transform_stamped.transform.translation.x;
    double gt_y = transform_stamped.transform.translation.y;
    double gt_z = transform_stamped.transform.translation.z;

    // Add noise to simulate GPS measurement
    // Note: In a real system, you would convert world coordinates to lat/lon
    // For simulation, we use x,y directly as lat/lon (scaled appropriately)
    double noisy_x = gt_x + generateGaussianNoise(0.0, gps_noise_std_);
    double noisy_y = gt_y + generateGaussianNoise(0.0, gps_noise_std_);

    // Set position (using x,y as lat/lon for simulation purposes)
    // In real GPS: these would be actual latitude and longitude
    gps_msg.latitude = noisy_x;
    gps_msg.longitude = noisy_y;
    gps_msg.altitude = gt_z;

    // Set status (assuming good fix when not dropped out)
    gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    // Set position covariance (3x3 matrix stored as 9-element array)
    // Format: [xx, xy, xz, yx, yy, yz, zx, zy, zz]
    gps_msg.position_covariance[0] = gps_noise_std_ * gps_noise_std_;  // xx
    gps_msg.position_covariance[1] = 0.0;                              // xy
    gps_msg.position_covariance[2] = 0.0;                              // xz
    gps_msg.position_covariance[3] = 0.0;                              // yx
    gps_msg.position_covariance[4] = gps_noise_std_ * gps_noise_std_;  // yy
    gps_msg.position_covariance[5] = 0.0;                              // yz
    gps_msg.position_covariance[6] = 0.0;                              // zx
    gps_msg.position_covariance[7] = 0.0;                              // zy
    gps_msg.position_covariance[8] = 1.0;                              // zz (altitude uncertainty)
    gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    // Publish GPS message
    gps_pub_->publish(gps_msg);
}

double GpsSimNode::generateGaussianNoise(double mean, double stddev) {
    return mean + stddev * gaussian_noise_(gen_);
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsSimNode>());
    rclcpp::shutdown();
    return 0;
}

