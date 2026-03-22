#include "control_core.hpp"
#include <cmath>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : path_(nav_msgs::msg::Path()), logger_(logger) {}

void ControlCore::initControlCore(
  double lookahead_distance,
  double max_steering_angle,
  double steering_gain,
  double linear_velocity
) {
  lookahead_distance_ = lookahead_distance;
  max_steering_angle_ = max_steering_angle;
  steering_gain_ = steering_gain;
  linear_velocity_ = linear_velocity;
}

void ControlCore::updatePath(nav_msgs::msg::Path path) {
  RCLCPP_INFO(logger_, "Path Updated");
  path_ = path;
}

bool ControlCore::isPathEmpty() {
  return path_.poses.empty();
}

geometry_msgs::msg::Twist ControlCore::calculateControlCommand(double robot_x, double robot_y, double robot_theta) {
  geometry_msgs::msg::Twist twist;

  unsigned int lookahead_index = findLookaheadPoint(robot_x, robot_y, robot_theta);

  if (lookahead_index >= path_.poses.size())
  {
    return twist;
  }

  // Get the lookahead point coordinates
  double lookahead_x = path_.poses[lookahead_index].pose.position.x;
  double lookahead_y = path_.poses[lookahead_index].pose.position.y;

  // Calculate the distance to the lookahead point
  double dx = lookahead_x - robot_x;
  double dy = lookahead_y - robot_y;

  // Calculate the angle to the lookahead point
  double angle_to_lookahead = std::atan2(dy, dx);

  // Calculate the steering angle (difference between robot's heading and lookahead direction)
  double steering_angle = angle_to_lookahead - robot_theta;

  // Normalize the steering angle to the range [-pi, pi] for proper handling of sharp turns
  if (steering_angle > M_PI) 
  {
    steering_angle -= 2 * M_PI;
  }
  else if (steering_angle < -M_PI) 
  {
    steering_angle += 2 * M_PI;
  }

  // Scale speed based on steering angle — always move forward, slow down in turns
  double abs_steer = std::abs(steering_angle);
  double scale = std::max(0.2, 1.0 - (abs_steer / M_PI) * 0.8);
  twist.linear.x = linear_velocity_ * scale;

  // Clamp steering angle for angular velocity calculation
  steering_angle = std::max(-max_steering_angle_, std::min(steering_angle, max_steering_angle_));

  // Set an angular velocity according to gain
  double angular_velocity = steering_angle * steering_gain_; // Gain for steering

  twist.angular.z = angular_velocity;

  return twist;
}


unsigned int ControlCore::findLookaheadPoint(double robot_x, double robot_y, double robot_theta) {
  double min_distance = std::numeric_limits<double>::max();
  int lookahead_index = path_.poses.size();  // default to past-end so caller stops
  bool found_forward = false;

  // Loop through all path points to find the closest lookahead point
  for (size_t i = 0; i < path_.poses.size(); ++i)
  {
    double dx = path_.poses[i].pose.position.x - robot_x;
    double dy = path_.poses[i].pose.position.y - robot_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Skip points that are too close (within lookahead distance)
    if (distance < lookahead_distance_)
      continue;

    // Calculate the angle to the lookahead point
    double angle_to_point = std::atan2(dy, dx);

    // Calculate the difference between the robot's heading and the angle to the lookahead point
    double angle_diff = angle_to_point - robot_theta;

    // Normalize the angle difference to be within [-pi, pi]
    if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    if (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    // Check if the angle to the point is within the forward-facing direction range (-π/2 to π/2)
    if (std::abs(angle_diff) < M_PI / 2) {
      // We found a valid forward point, update the closest point
      if (distance < min_distance)
      {
        min_distance = distance;
        lookahead_index = i;
        found_forward = true;
      }
    }
  }

  // If no forward point was found, allow the robot to reverse
  if (!found_forward) {
    // Find the closest point regardless of direction
    for (size_t i = 0; i < path_.poses.size(); ++i)
    {
      double dx = path_.poses[i].pose.position.x - robot_x;
      double dy = path_.poses[i].pose.position.y - robot_y;
      double distance = std::sqrt(dx * dx + dy * dy);

      // Skip points that are too close
      if (distance < lookahead_distance_)
        continue;

      // Always select the closest point if no valid forward point was found
      if (distance < min_distance)
      {
        min_distance = distance;
        lookahead_index = i;
      }
    }
  }

  return lookahead_index;
}

}  
