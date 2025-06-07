#ifndef ARCADE_DRIVER_NODE_HPP_
#define ARCADE_DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "drivetrain_msgs/msg/arcade_speed.hpp"

class ArcadeDriver : public rclcpp::Node {
  public:
    ArcadeDriver();

  private:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr joystick_sub_;
    rclcpp::Publisher<drivetrain_msgs::msg::ArcadeSpeed>::SharedPtr arcade_pub_;

    drivetrain_msgs::msg::ArcadeSpeed joystick_to_speed_mapper(const float joystick_rotate, const float joystick_drive);
    void joystick_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    bool is_negligible_joystick_change(const float new_joystick_rotate, const float new_joystick_drive);
    const float THRESHOLD = 0.05;
};

#endif 
