#ifndef LOCALIZATION_NODE_HPP_
#define LOCALIZATION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "localization_core.hpp"

class LocalizationNode : public rclcpp::Node {
  public:
    LocalizationNode();

  private:
    void processParameters();
    void timerCallback();
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    double quaternionToYaw(double x, double y, double z, double w);
    
    // EKF instance
    robot::ExtendedKalmanFilter ekf_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    
    // Transform utilities (for initialization)
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Latest sensor data
    geometry_msgs::msg::Twist latest_cmd_vel_;
    sensor_msgs::msg::Imu latest_imu_;
    bool has_cmd_vel_;
    bool has_imu_;
    rclcpp::Time last_cmd_vel_time_;
    rclcpp::Time last_update_time_;
    
    // Configuration parameters
    std::string gps_topic_;
    std::string imu_topic_;
    std::string cmd_vel_topic_;
    std::string odom_topic_;
    std::string source_frame_;
    std::string target_frame_;
    
    double process_noise_position_;
    double process_noise_orientation_;
    double process_noise_velocity_;
    double gps_measurement_noise_;
    double imu_measurement_noise_orientation_;
    double imu_measurement_noise_angular_vel_;
    
    double ekf_update_rate_;
    bool use_tf_for_init_;
};

#endif

