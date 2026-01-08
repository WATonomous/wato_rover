// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "imu_sim_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>

ImuSimNode::ImuSimNode()
: Node("imu_sim_node")
, gen_(rd_())
, gaussian_noise_(0.0, 1.0)
, initialized_(false)
{
  processParameters();

  // Initialize biases to zero (will be set on first message if add_bias_ is true)
  orientation_bias_x_ = 0.0;
  orientation_bias_y_ = 0.0;
  orientation_bias_z_ = 0.0;
  orientation_bias_w_ = 0.0;
  angular_vel_bias_x_ = 0.0;
  angular_vel_bias_y_ = 0.0;
  angular_vel_bias_z_ = 0.0;
  linear_accel_bias_x_ = 0.0;
  linear_accel_bias_y_ = 0.0;
  linear_accel_bias_z_ = 0.0;

  // Create subscriber
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, 10, std::bind(&ImuSimNode::imuCallback, this, std::placeholders::_1));

  // Create publisher
  imu_sim_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_sim_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "IMU Simulation Node initialized");
  RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s", imu_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Publishing to: %s", imu_sim_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Orientation noise std: %.4f", orientation_noise_std_);
  RCLCPP_INFO(this->get_logger(), "  Angular velocity noise std: %.4f", angular_vel_noise_std_);
  RCLCPP_INFO(this->get_logger(), "  Linear acceleration noise std: %.4f", linear_accel_noise_std_);
  RCLCPP_INFO(this->get_logger(), "  Add bias: %s", add_bias_ ? "true" : "false");
}

void ImuSimNode::processParameters()
{
  // Declare parameters
  this->declare_parameter<std::string>("imu_topic", "/imu");
  this->declare_parameter<std::string>("imu_sim_topic", "/imu/simulated");
  this->declare_parameter<double>("orientation_noise_std", 0.01);
  this->declare_parameter<double>("angular_vel_noise_std", 0.01);
  this->declare_parameter<double>("linear_accel_noise_std", 0.1);
  this->declare_parameter<double>("orientation_bias_std", 0.001);
  this->declare_parameter<double>("angular_vel_bias_std", 0.001);
  this->declare_parameter<double>("linear_accel_bias_std", 0.01);
  this->declare_parameter<bool>("add_bias", true);

  // Get parameters
  imu_topic_ = this->get_parameter("imu_topic").as_string();
  imu_sim_topic_ = this->get_parameter("imu_sim_topic").as_string();
  orientation_noise_std_ = this->get_parameter("orientation_noise_std").as_double();
  angular_vel_noise_std_ = this->get_parameter("angular_vel_noise_std").as_double();
  linear_accel_noise_std_ = this->get_parameter("linear_accel_noise_std").as_double();
  orientation_bias_std_ = this->get_parameter("orientation_bias_std").as_double();
  angular_vel_bias_std_ = this->get_parameter("angular_vel_bias_std").as_double();
  linear_accel_bias_std_ = this->get_parameter("linear_accel_bias_std").as_double();
  add_bias_ = this->get_parameter("add_bias").as_bool();
}

void ImuSimNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Initialize biases on first message if enabled
  if (add_bias_ && !initialized_) {
    orientation_bias_x_ = generateGaussianNoise(0.0, orientation_bias_std_);
    orientation_bias_y_ = generateGaussianNoise(0.0, orientation_bias_std_);
    orientation_bias_z_ = generateGaussianNoise(0.0, orientation_bias_std_);
    orientation_bias_w_ = generateGaussianNoise(0.0, orientation_bias_std_);
    angular_vel_bias_x_ = generateGaussianNoise(0.0, angular_vel_bias_std_);
    angular_vel_bias_y_ = generateGaussianNoise(0.0, angular_vel_bias_std_);
    angular_vel_bias_z_ = generateGaussianNoise(0.0, angular_vel_bias_std_);
    linear_accel_bias_x_ = generateGaussianNoise(0.0, linear_accel_bias_std_);
    linear_accel_bias_y_ = generateGaussianNoise(0.0, linear_accel_bias_std_);
    linear_accel_bias_z_ = generateGaussianNoise(0.0, linear_accel_bias_std_);
    initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "IMU biases initialized");
  }

  // Create simulated IMU message
  sensor_msgs::msg::Imu imu_sim_msg = *msg;

  // Add noise and bias to orientation
  imu_sim_msg.orientation.x =
    msg->orientation.x + generateGaussianNoise(0.0, orientation_noise_std_) + (add_bias_ ? orientation_bias_x_ : 0.0);
  imu_sim_msg.orientation.y =
    msg->orientation.y + generateGaussianNoise(0.0, orientation_noise_std_) + (add_bias_ ? orientation_bias_y_ : 0.0);
  imu_sim_msg.orientation.z =
    msg->orientation.z + generateGaussianNoise(0.0, orientation_noise_std_) + (add_bias_ ? orientation_bias_z_ : 0.0);
  imu_sim_msg.orientation.w =
    msg->orientation.w + generateGaussianNoise(0.0, orientation_noise_std_) + (add_bias_ ? orientation_bias_w_ : 0.0);

  // Normalize quaternion (important for valid orientation)
  double norm = std::sqrt(
    imu_sim_msg.orientation.x * imu_sim_msg.orientation.x + imu_sim_msg.orientation.y * imu_sim_msg.orientation.y +
    imu_sim_msg.orientation.z * imu_sim_msg.orientation.z + imu_sim_msg.orientation.w * imu_sim_msg.orientation.w);
  if (norm > 0.0) {
    imu_sim_msg.orientation.x /= norm;
    imu_sim_msg.orientation.y /= norm;
    imu_sim_msg.orientation.z /= norm;
    imu_sim_msg.orientation.w /= norm;
  }

  // Add noise and bias to angular velocity
  imu_sim_msg.angular_velocity.x = msg->angular_velocity.x + generateGaussianNoise(0.0, angular_vel_noise_std_) +
                                   (add_bias_ ? angular_vel_bias_x_ : 0.0);
  imu_sim_msg.angular_velocity.y = msg->angular_velocity.y + generateGaussianNoise(0.0, angular_vel_noise_std_) +
                                   (add_bias_ ? angular_vel_bias_y_ : 0.0);
  imu_sim_msg.angular_velocity.z = msg->angular_velocity.z + generateGaussianNoise(0.0, angular_vel_noise_std_) +
                                   (add_bias_ ? angular_vel_bias_z_ : 0.0);

  // Add noise and bias to linear acceleration
  imu_sim_msg.linear_acceleration.x = msg->linear_acceleration.x + generateGaussianNoise(0.0, linear_accel_noise_std_) +
                                      (add_bias_ ? linear_accel_bias_x_ : 0.0);
  imu_sim_msg.linear_acceleration.y = msg->linear_acceleration.y + generateGaussianNoise(0.0, linear_accel_noise_std_) +
                                      (add_bias_ ? linear_accel_bias_y_ : 0.0);
  imu_sim_msg.linear_acceleration.z = msg->linear_acceleration.z + generateGaussianNoise(0.0, linear_accel_noise_std_) +
                                      (add_bias_ ? linear_accel_bias_z_ : 0.0);

  // Update covariance matrices to reflect added noise
  // Orientation covariance (3x3 matrix for roll, pitch, yaw)
  double orientation_cov = orientation_noise_std_ * orientation_noise_std_;
  imu_sim_msg.orientation_covariance[0] = orientation_cov;  // roll-roll
  imu_sim_msg.orientation_covariance[4] = orientation_cov;  // pitch-pitch
  imu_sim_msg.orientation_covariance[8] = orientation_cov;  // yaw-yaw

  // Angular velocity covariance
  double angular_vel_cov = angular_vel_noise_std_ * angular_vel_noise_std_;
  imu_sim_msg.angular_velocity_covariance[0] = angular_vel_cov;  // x-x
  imu_sim_msg.angular_velocity_covariance[4] = angular_vel_cov;  // y-y
  imu_sim_msg.angular_velocity_covariance[8] = angular_vel_cov;  // z-z

  // Linear acceleration covariance
  double linear_accel_cov = linear_accel_noise_std_ * linear_accel_noise_std_;
  imu_sim_msg.linear_acceleration_covariance[0] = linear_accel_cov;  // x-x
  imu_sim_msg.linear_acceleration_covariance[4] = linear_accel_cov;  // y-y
  imu_sim_msg.linear_acceleration_covariance[8] = linear_accel_cov;  // z-z

  // Publish simulated IMU message
  imu_sim_pub_->publish(imu_sim_msg);
}

double ImuSimNode::generateGaussianNoise(double mean, double stddev)
{
  return mean + stddev * gaussian_noise_(gen_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuSimNode>());
  rclcpp::shutdown();
  return 0;
}
