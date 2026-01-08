#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore {
  public:
    ControlCore(const rclcpp::Logger& logger);

    void initControlCore(
      double kp,
      double ki,
      double kd,
      double max_steering_angle, 
      double linear_velocity
    );

    void updatePath(nav_msgs::msg::Path path);

    bool isPathEmpty();

    unsigned int findClosestPoint(double robot_x, double robot_y);

    geometry_msgs::msg::Twist calculateControlCommand(double robot_x, double robot_y, double robot_theta, double dt);
  
  private:
    nav_msgs::msg::Path path_;
    rclcpp::Logger logger_;

    double kp_;
    double ki_;
    double kd_;
    double max_steering_angle_;
    double linear_velocity_;

    double prev_error_;
    double integral_error_;
};

} 

#endif 
