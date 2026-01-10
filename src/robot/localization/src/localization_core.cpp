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

#include "localization/localization_core.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace robot
{

// ============================================================================
// Matrix6x6 Implementation
// ============================================================================

Matrix6x6::Matrix6x6()
{
  data_.fill(0.0);
}

Matrix6x6::Matrix6x6(double value)
{
  data_.fill(0.0);
  for (int i = 0; i < 6; i++) {
    data_[i * 6 + i] = value;
  }
}

Matrix6x6::Matrix6x6(const Matrix6x6 & other)
: data_(other.data_)
{}

Matrix6x6 Matrix6x6::operator+(const Matrix6x6 & other) const
{
  Matrix6x6 result;
  for (size_t i = 0; i < 36; i++) {
    result.data_[i] = data_[i] + other.data_[i];
  }
  return result;
}

Matrix6x6 Matrix6x6::operator-(const Matrix6x6 & other) const
{
  Matrix6x6 result;
  for (size_t i = 0; i < 36; i++) {
    result.data_[i] = data_[i] - other.data_[i];
  }
  return result;
}

Matrix6x6 Matrix6x6::operator*(const Matrix6x6 & other) const
{
  Matrix6x6 result;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      double sum = 0.0;
      for (int k = 0; k < 6; k++) {
        sum += (*this)(i, k) * other(k, j);
      }
      result(i, j) = sum;
    }
  }
  return result;
}

Matrix6x6 Matrix6x6::transpose() const
{
  Matrix6x6 result;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      result(j, i) = (*this)(i, j);
    }
  }
  return result;
}

Matrix6x6 Matrix6x6::inverse() const
{
  // Simple Gaussian elimination for 6x6 matrix
  Matrix6x6 augmented = *this;
  Matrix6x6 identity = Matrix6x6::identity();

  // Create augmented matrix [A|I]
  std::array<double, 72> aug_data;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      aug_data[i * 12 + j] = (*this)(i, j);
      aug_data[i * 12 + j + 6] = (i == j) ? 1.0 : 0.0;
    }
  }

  // Forward elimination
  for (int i = 0; i < 6; i++) {
    // Find pivot
    int max_row = i;
    double max_val = std::abs(aug_data[i * 12 + i]);
    for (int k = i + 1; k < 6; k++) {
      if (std::abs(aug_data[k * 12 + i]) > max_val) {
        max_val = std::abs(aug_data[k * 12 + i]);
        max_row = k;
      }
    }

    // Swap rows
    if (max_row != i) {
      for (int j = 0; j < 12; j++) {
        std::swap(aug_data[i * 12 + j], aug_data[max_row * 12 + j]);
      }
    }

    // Make diagonal element 1
    double pivot = aug_data[i * 12 + i];
    if (std::abs(pivot) < 1e-10) {
      // Singular matrix, return identity (fallback)
      return Matrix6x6::identity();
    }

    for (int j = 0; j < 12; j++) {
      aug_data[i * 12 + j] /= pivot;
    }

    // Eliminate column
    for (int k = 0; k < 6; k++) {
      if (k != i) {
        double factor = aug_data[k * 12 + i];
        for (int j = 0; j < 12; j++) {
          aug_data[k * 12 + j] -= factor * aug_data[i * 12 + j];
        }
      }
    }
  }

  // Extract inverse
  Matrix6x6 result;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      result(i, j) = aug_data[i * 12 + j + 6];
    }
  }

  return result;
}

Matrix6x6 Matrix6x6::identity()
{
  Matrix6x6 result;
  for (int i = 0; i < 6; i++) {
    result(i, i) = 1.0;
  }
  return result;
}

// ============================================================================
// Vector6 Implementation
// ============================================================================

Vector6::Vector6()
{
  data_.fill(0.0);
}

Vector6::Vector6(double x, double y, double theta, double vx, double vy, double omega)
{
  data_[0] = x;
  data_[1] = y;
  data_[2] = theta;
  data_[3] = vx;
  data_[4] = vy;
  data_[5] = omega;
}

Vector6 Vector6::operator+(const Vector6 & other) const
{
  Vector6 result;
  for (size_t i = 0; i < 6; i++) {
    result.data_[i] = data_[i] + other.data_[i];
  }
  return result;
}

Vector6 Vector6::operator-(const Vector6 & other) const
{
  Vector6 result;
  for (size_t i = 0; i < 6; i++) {
    result.data_[i] = data_[i] - other.data_[i];
  }
  return result;
}

Vector6 Vector6::operator*(double scalar) const
{
  Vector6 result;
  for (size_t i = 0; i < 6; i++) {
    result.data_[i] = data_[i] * scalar;
  }
  return result;
}

double Vector6::norm() const
{
  double sum = 0.0;
  for (size_t i = 0; i < 6; i++) {
    sum += data_[i] * data_[i];
  }
  return std::sqrt(sum);
}

// ============================================================================
// ExtendedKalmanFilter Implementation
// ============================================================================

ExtendedKalmanFilter::ExtendedKalmanFilter(const rclcpp::Logger & logger)
: logger_(logger)
, initialized_(false)
{
  state_ = Vector6();
  P_ = Matrix6x6::identity() * 0.1;  // Initial uncertainty
}

void ExtendedKalmanFilter::initialize(double x, double y, double theta)
{
  state_ = Vector6(x, y, theta, 0.0, 0.0, 0.0);

  // Initialize covariance with high uncertainty
  P_ = Matrix6x6::identity();
  P_(0, 0) = 1.0;  // x uncertainty
  P_(1, 1) = 1.0;  // y uncertainty
  P_(2, 2) = 0.1;  // theta uncertainty
  P_(3, 3) = 0.1;  // vx uncertainty
  P_(4, 4) = 0.1;  // vy uncertainty
  P_(5, 5) = 0.1;  // omega uncertainty

  initialized_ = true;
}

void ExtendedKalmanFilter::predict(
  double vx,
  double vy,
  double omega,
  double dt,
  double process_noise_pos,
  double process_noise_theta,
  double process_noise_vel)
{
  if (!initialized_) {
    return;
  }

  // Process model: constant velocity model
  double theta = state_.theta();
  double cos_theta = std::cos(theta);
  double sin_theta = std::sin(theta);

  // Predict state
  double x_new = state_.x() + (vx * cos_theta - vy * sin_theta) * dt;
  double y_new = state_.y() + (vx * sin_theta + vy * cos_theta) * dt;
  double theta_new = state_.theta() + omega * dt;

  state_ = Vector6(x_new, y_new, theta_new, vx, vy, omega);
  normalizeTheta();

  // Compute process Jacobian
  Matrix6x6 F = computeProcessJacobian(vx, vy, omega, dt);

  // Process noise covariance
  Matrix6x6 Q;
  Q(0, 0) = process_noise_pos * process_noise_pos;
  Q(1, 1) = process_noise_pos * process_noise_pos;
  Q(2, 2) = process_noise_theta * process_noise_theta;
  Q(3, 3) = process_noise_vel * process_noise_vel;
  Q(4, 4) = process_noise_vel * process_noise_vel;
  Q(5, 5) = process_noise_vel * process_noise_vel;

  // Update covariance: P = F * P * F^T + Q
  Matrix6x6 F_transpose = F.transpose();
  P_ = F * P_ * F_transpose + Q;
}

void ExtendedKalmanFilter::updateGPS(double gps_x, double gps_y, double measurement_noise)
{
  if (!initialized_) {
    return;
  }

  // Measurement model: h(x) = [x, y]
  Vector6 h;
  h(0) = state_.x();
  h(1) = state_.y();

  // Measurement
  Vector6 z;
  z(0) = gps_x;
  z(1) = gps_y;

  // Innovation
  Vector6 y = z - h;

  // Measurement Jacobian (2x6)
  // H = [1 0 0 0 0 0]
  //     [0 1 0 0 0 0]

  // Measurement noise covariance (2x2)
  // R = [σ² 0 ]
  //     [0  σ²]
  double R_val = measurement_noise * measurement_noise;

  // Innovation covariance: S = H * P * H^T + R
  // For 2x6 H and 6x6 P, we need to compute H*P*H^T
  // H*P is 2x6, then (H*P)*H^T is 2x2
  double S_00 = P_(0, 0) + R_val;
  double S_01 = P_(0, 1);
  double S_10 = P_(1, 0);
  double S_11 = P_(1, 1) + R_val;

  // S inverse (2x2)
  double det_S = S_00 * S_11 - S_01 * S_10;
  if (std::abs(det_S) < 1e-10) {
    return;  // Singular, skip update
  }

  double S_inv_00 = S_11 / det_S;
  double S_inv_01 = -S_01 / det_S;
  double S_inv_10 = -S_10 / det_S;
  double S_inv_11 = S_00 / det_S;

  // Kalman gain: K = P * H^T * S^-1
  // P*H^T is 6x2, then multiply by S^-1 (2x2) gives 6x2
  Vector6 K_x, K_y;
  for (int i = 0; i < 6; i++) {
    K_x(i) = P_(i, 0) * S_inv_00 + P_(i, 1) * S_inv_10;
    K_y(i) = P_(i, 0) * S_inv_01 + P_(i, 1) * S_inv_11;
  }

  // Update state: x = x + K * y
  state_ = state_ + K_x * y(0) + K_y * y(1);
  normalizeTheta();

  // Update covariance: P = (I - K*H) * P
  // K*H is 6x6, where (K*H)(i,j) = K_x(i)*H(0,j) + K_y(i)*H(1,j)
  Matrix6x6 KH;
  for (int i = 0; i < 6; i++) {
    KH(i, 0) = K_x(i);
    KH(i, 1) = K_y(i);
  }

  Matrix6x6 I = Matrix6x6::identity();
  Matrix6x6 I_minus_KH = I - KH;
  P_ = I_minus_KH * P_;
}

void ExtendedKalmanFilter::updateIMUOrientation(double theta, double measurement_noise)
{
  if (!initialized_) {
    return;
  }

  // Measurement model: h(x) = theta
  double h = state_.theta();
  double z = theta;

  // Innovation (normalize angle difference)
  double y = z - h;
  while (y > M_PI) y -= 2.0 * M_PI;
  while (y < -M_PI) y += 2.0 * M_PI;

  // Measurement Jacobian: H = [0 0 1 0 0 0]
  // Innovation covariance: S = H * P * H^T + R
  double S = P_(2, 2) + measurement_noise * measurement_noise;

  if (std::abs(S) < 1e-10) {
    return;
  }

  // Kalman gain: K = P * H^T / S
  Vector6 K;
  for (int i = 0; i < 6; i++) {
    K(i) = P_(i, 2) / S;
  }

  // Update state
  state_ = state_ + K * y;
  normalizeTheta();

  // Update covariance: P = (I - K*H) * P
  Matrix6x6 KH;
  for (int i = 0; i < 6; i++) {
    KH(i, 2) = K(i);
  }

  Matrix6x6 I = Matrix6x6::identity();
  Matrix6x6 I_minus_KH = I - KH;
  P_ = I_minus_KH * P_;
}

void ExtendedKalmanFilter::updateIMUAngularVel(double omega, double measurement_noise)
{
  if (!initialized_) {
    return;
  }

  // Measurement model: h(x) = omega
  double h = state_.omega();
  double z = omega;
  double y = z - h;

  // Measurement Jacobian: H = [0 0 0 0 0 1]
  // Innovation covariance: S = H * P * H^T + R
  double S = P_(5, 5) + measurement_noise * measurement_noise;

  if (std::abs(S) < 1e-10) {
    return;
  }

  // Kalman gain: K = P * H^T / S
  Vector6 K;
  for (int i = 0; i < 6; i++) {
    K(i) = P_(i, 5) / S;
  }

  // Update state
  state_ = state_ + K * y;

  // Update covariance: P = (I - K*H) * P
  Matrix6x6 KH;
  for (int i = 0; i < 6; i++) {
    KH(i, 5) = K(i);
  }

  Matrix6x6 I = Matrix6x6::identity();
  Matrix6x6 I_minus_KH = I - KH;
  P_ = I_minus_KH * P_;
}

void ExtendedKalmanFilter::getState(double & x, double & y, double & theta, double & vx, double & vy, double & omega)
{
  x = state_.x();
  y = state_.y();
  theta = state_.theta();
  vx = state_.vx();
  vy = state_.vy();
  omega = state_.omega();
}

void ExtendedKalmanFilter::getOdometry(
  nav_msgs::msg::Odometry & odom_msg,
  const std::string & frame_id,
  const std::string & child_frame_id,
  const rclcpp::Time & stamp)
{
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = frame_id;
  odom_msg.child_frame_id = child_frame_id;

  // Set pose
  odom_msg.pose.pose.position.x = state_.x();
  odom_msg.pose.pose.position.y = state_.y();
  odom_msg.pose.pose.position.z = 0.0;

  // Convert theta to quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, state_.theta());
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();

  // Set twist
  odom_msg.twist.twist.linear.x = state_.vx();
  odom_msg.twist.twist.linear.y = state_.vy();
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = state_.omega();

  // Set covariance (6x6 matrix flattened to 36 elements)
  // Pose covariance (6x6: x, y, z, roll, pitch, yaw)
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      if (i < 3 && j < 3) {
        // Position covariance
        odom_msg.pose.covariance[i * 6 + j] = P_(i, j);
      } else if (i == 5 && j == 5) {
        // Yaw covariance (theta is index 2 in state)
        odom_msg.pose.covariance[35] = P_(2, 2);
      }
    }
  }

  // Twist covariance
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      if (i < 3 && j < 3) {
        // Linear velocity covariance
        odom_msg.twist.covariance[i * 6 + j] = P_(i + 3, j + 3);
      } else if (i == 5 && j == 5) {
        // Angular z velocity covariance
        odom_msg.twist.covariance[35] = P_(5, 5);
      }
    }
  }
}

Matrix6x6 ExtendedKalmanFilter::computeProcessJacobian(double vx, double vy, double omega, double dt)
{
  Matrix6x6 F = Matrix6x6::identity();
  double theta = state_.theta();
  double cos_theta = std::cos(theta);
  double sin_theta = std::sin(theta);

  // Partial derivatives of process model
  F(0, 2) = (-vx * sin_theta - vy * cos_theta) * dt;  // dx/dtheta
  F(0, 3) = cos_theta * dt;  // dx/dvx
  F(0, 4) = -sin_theta * dt;  // dx/dvy

  F(1, 2) = (vx * cos_theta - vy * sin_theta) * dt;  // dy/dtheta
  F(1, 3) = sin_theta * dt;  // dy/dvx
  F(1, 4) = cos_theta * dt;  // dy/dvy

  F(2, 5) = dt;  // dtheta/domega

  return F;
}

void ExtendedKalmanFilter::normalizeTheta()
{
  double theta = state_.theta();
  while (theta > M_PI) theta -= 2.0 * M_PI;
  while (theta < -M_PI) theta += 2.0 * M_PI;
  state_(2) = theta;
}

}  // namespace robot
