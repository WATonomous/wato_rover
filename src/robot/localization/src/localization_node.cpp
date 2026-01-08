#include <chrono>
#include <memory>
#include <cmath>

#include "localization_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

LocalizationNode::LocalizationNode()
  : Node("localization_node"),
    ekf_(this->get_logger()),
    has_cmd_vel_(false),
    has_imu_(false)
{
    processParameters();
    
    // Create publisher
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    
    // Create subscribers
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic_, 10,
        std::bind(&LocalizationNode::gpsCallback, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10,
        std::bind(&LocalizationNode::imuCallback, this, std::placeholders::_1));
    
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 10,
        std::bind(&LocalizationNode::cmdVelCallback, this, std::placeholders::_1));
    
    // Create TF buffer and listener (for initialization)
    if (use_tf_for_init_) {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
    
    // Create timer for EKF updates
    int period_ms = static_cast<int>(1000.0 / ekf_update_rate_);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&LocalizationNode::timerCallback, this)
    );
    
    last_update_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Localization Node initialized");
    RCLCPP_INFO(this->get_logger(), "  GPS topic: %s", gps_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  IMU topic: %s", imu_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Cmd vel topic: %s", cmd_vel_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  EKF update rate: %.1f Hz", ekf_update_rate_);
}

void LocalizationNode::processParameters() {
    // Declare parameters
    this->declare_parameter<std::string>("gps_topic", "/gps/fix");
    this->declare_parameter<std::string>("imu_topic", "/imu/simulated");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>("odom_topic", "/odom/ekf_filtered");
    this->declare_parameter<std::string>("source_frame", "sim_world");
    this->declare_parameter<std::string>("target_frame", "robot/chassis/camera");
    this->declare_parameter<double>("process_noise_position", 0.1);
    this->declare_parameter<double>("process_noise_orientation", 0.05);
    this->declare_parameter<double>("process_noise_velocity", 0.1);
    this->declare_parameter<double>("gps_measurement_noise", 0.5);
    this->declare_parameter<double>("imu_measurement_noise_orientation", 0.001);
    this->declare_parameter<double>("imu_measurement_noise_angular_vel", 0.001);
    this->declare_parameter<double>("ekf_update_rate", 10.0);
    this->declare_parameter<bool>("use_tf_for_init", true);
    
    // Get parameters
    gps_topic_ = this->get_parameter("gps_topic").as_string();
    imu_topic_ = this->get_parameter("imu_topic").as_string();
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    source_frame_ = this->get_parameter("source_frame").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    process_noise_position_ = this->get_parameter("process_noise_position").as_double();
    process_noise_orientation_ = this->get_parameter("process_noise_orientation").as_double();
    process_noise_velocity_ = this->get_parameter("process_noise_velocity").as_double();
    gps_measurement_noise_ = this->get_parameter("gps_measurement_noise").as_double();
    imu_measurement_noise_orientation_ = this->get_parameter("imu_measurement_noise_orientation").as_double();
    imu_measurement_noise_angular_vel_ = this->get_parameter("imu_measurement_noise_angular_vel").as_double();
    ekf_update_rate_ = this->get_parameter("ekf_update_rate").as_double();
    use_tf_for_init_ = this->get_parameter("use_tf_for_init").as_bool();
}

void LocalizationNode::timerCallback() {
    // Initialize EKF from TF if not initialized and TF is enabled
    if (!ekf_.isInitialized() && use_tf_for_init_ && tf_buffer_) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_->lookupTransform(
                source_frame_, target_frame_, tf2::TimePointZero);
            
            double x = transform_stamped.transform.translation.x;
            double y = transform_stamped.transform.translation.y;
            double theta = quaternionToYaw(
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z,
                transform_stamped.transform.rotation.w);
            
            ekf_.initialize(x, y, theta);
            last_update_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "EKF initialized from TF: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);
            return;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Waiting for TF transform for initialization: %s", ex.what());
            return;
        }
    }
    
    if (!ekf_.isInitialized()) {
        return;
    }
    
    // Compute time delta
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt <= 0.0) {
        return;
    }
    
    // Predict using wheel odometry (cmd_vel)
    if (has_cmd_vel_) {
        double vx = latest_cmd_vel_.linear.x;
        double vy = latest_cmd_vel_.linear.y;
        double omega = latest_cmd_vel_.angular.z;
        
        ekf_.predict(vx, vy, omega, dt,
                     process_noise_position_,
                     process_noise_orientation_,
                     process_noise_velocity_);
    }
    
    // Publish filtered odometry
    nav_msgs::msg::Odometry filtered_odom;
    ekf_.getOdometry(filtered_odom, source_frame_, target_frame_, current_time);
    odom_pub_->publish(filtered_odom);
    
    last_update_time_ = current_time;
}

void LocalizationNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (!ekf_.isInitialized()) {
        return;
    }
    
    // Check GPS status
    if (msg->status.status != sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
        return;
    }
    
    // Extract measurement noise from covariance if available
    double measurement_noise = gps_measurement_noise_;
    if (msg->position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN ||
        msg->position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN) {
        double variance = msg->position_covariance[0];
        if (variance > 0.0) {
            measurement_noise = std::sqrt(variance);
        }
    }
    
    // Update EKF with GPS
    ekf_.updateGPS(msg->latitude, msg->longitude, measurement_noise);
}

void LocalizationNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    latest_imu_ = *msg;
    has_imu_ = true;
    
    if (!ekf_.isInitialized()) {
        return;
    }
    
    // Extract orientation
    double theta = quaternionToYaw(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    
    // Extract measurement noise from covariance
    double orientation_noise = imu_measurement_noise_orientation_;
    if (msg->orientation_covariance[8] > 0.0) {
        orientation_noise = std::sqrt(msg->orientation_covariance[8]);
    }
    
    // Update EKF with IMU orientation
    ekf_.updateIMUOrientation(theta, orientation_noise);
    
    // Update EKF with IMU angular velocity
    double angular_vel_noise = imu_measurement_noise_angular_vel_;
    if (msg->angular_velocity_covariance[8] > 0.0) {
        angular_vel_noise = std::sqrt(msg->angular_velocity_covariance[8]);
    }
    
    ekf_.updateIMUAngularVel(msg->angular_velocity.z, angular_vel_noise);
}

void LocalizationNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    latest_cmd_vel_ = *msg;
    has_cmd_vel_ = true;
    last_cmd_vel_time_ = this->now();
}

double LocalizationNode::quaternionToYaw(double x, double y, double z, double w) {
    tf2::Quaternion q(x, y, z, w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}

