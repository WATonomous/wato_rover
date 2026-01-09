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

#ifndef LOCALIZATION_CORE_HPP_
#define LOCALIZATION_CORE_HPP_

#include <array>
#include <cmath>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace robot
{

// Simple 6x6 matrix class for EKF operations
class Matrix6x6
{
public:
  Matrix6x6();
  explicit Matrix6x6(double value);
  Matrix6x6(const Matrix6x6 & other);

  double & operator()(int i, int j)
  {
    return data_[i * 6 + j];
  }

  const double & operator()(int i, int j) const
  {
    return data_[i * 6 + j];
  }

  Matrix6x6 operator+(const Matrix6x6 & other) const;
  Matrix6x6 operator-(const Matrix6x6 & other) const;
  Matrix6x6 operator*(const Matrix6x6 & other) const;
  Matrix6x6 transpose() const;
  Matrix6x6 inverse() const;

  static Matrix6x6 identity();

private:
  std::array<double, 36> data_;
};

// Simple 6x1 vector class
class Vector6
{
public:
  Vector6();
  Vector6(double x, double y, double theta, double vx, double vy, double omega);

  double & operator()(int i)
  {
    return data_[i];
  }

  const double & operator()(int i) const
  {
    return data_[i];
  }

  Vector6 operator+(const Vector6 & other) const;
  Vector6 operator-(const Vector6 & other) const;
  Vector6 operator*(double scalar) const;

  double norm() const;

  double x() const
  {
    return data_[0];
  }

  double y() const
  {
    return data_[1];
  }

  double theta() const
  {
    return data_[2];
  }

  double vx() const
  {
    return data_[3];
  }

  double vy() const
  {
    return data_[4];
  }

  double omega() const
  {
    return data_[5];
  }

private:
  std::array<double, 6> data_;
};

class ExtendedKalmanFilter
{
public:
  explicit ExtendedKalmanFilter(const rclcpp::Logger & logger);

  // Initialize EKF with initial pose
  void initialize(double x, double y, double theta);

  // Prediction step (process model) - using wheel odometry
  void predict(
    double vx,
    double vy,
    double omega,
    double dt,
    double process_noise_pos,
    double process_noise_theta,
    double process_noise_vel);

  // Update steps (measurement models)
  void updateGPS(double gps_x, double gps_y, double measurement_noise);
  void updateIMUOrientation(double theta, double measurement_noise);
  void updateIMUAngularVel(double omega, double measurement_noise);

  // Get current state estimate
  void getState(double & x, double & y, double & theta, double & vx, double & vy, double & omega);

  // Get state with covariance for odometry message
  void getOdometry(
    nav_msgs::msg::Odometry & odom_msg,
    const std::string & frame_id,
    const std::string & child_frame_id,
    const rclcpp::Time & stamp);

  // Check if EKF is initialized
  bool isInitialized() const
  {
    return initialized_;
  }

private:
  // State vector: [x, y, theta, vx, vy, omega]
  Vector6 state_;
  Matrix6x6 P_;  // 6x6 covariance matrix

  bool initialized_;
  rclcpp::Logger logger_;

  // Helper functions
  Matrix6x6 computeProcessJacobian(double vx, double vy, double omega, double dt);
  void normalizeTheta();
};

}  // namespace robot

#endif
