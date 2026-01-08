#ifndef IMU_SIM_NODE_HPP_
#define IMU_SIM_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <random>

class ImuSimNode : public rclcpp::Node {
  public:
    ImuSimNode();

  private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void processParameters();
    double generateGaussianNoise(double mean, double stddev);

    // Subscriber and Publisher
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_sim_pub_;

    // Random number generators for noise
    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<double> gaussian_noise_;

    // IMU bias (simulates sensor drift)
    double orientation_bias_x_;
    double orientation_bias_y_;
    double orientation_bias_z_;
    double orientation_bias_w_;
    double angular_vel_bias_x_;
    double angular_vel_bias_y_;
    double angular_vel_bias_z_;
    double linear_accel_bias_x_;
    double linear_accel_bias_y_;
    double linear_accel_bias_z_;

    // Configuration parameters
    std::string imu_topic_;
    std::string imu_sim_topic_;
    double orientation_noise_std_;        // Standard deviation for orientation noise
    double angular_vel_noise_std_;         // Standard deviation for angular velocity noise
    double linear_accel_noise_std_;       // Standard deviation for linear acceleration noise
    double orientation_bias_std_;        // Standard deviation for orientation bias
    double angular_vel_bias_std_;         // Standard deviation for angular velocity bias
    double linear_accel_bias_std_;       // Standard deviation for linear acceleration bias
    bool add_bias_;                       // Whether to add bias to measurements
    bool initialized_;                   // Whether biases have been initialized
};

#endif

