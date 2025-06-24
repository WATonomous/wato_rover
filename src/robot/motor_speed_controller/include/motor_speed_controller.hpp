#ifndef MOTOR_SPEED_CONTROLLER_NODE_HPP_
#define MOTOR_SPEED_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "drivetrain_msgs/msg/arcade_speed.hpp"
#include "drivetrain_msgs/msg/motor_speeds.hpp"
#include <nlohmann/json.hpp>
#include <std_msgs/msg/string.hpp>

class MotorSpeedController : public rclcpp::Node {
  public:
    MotorSpeedController();

  private:
    // subscriber to arcade speed topic (/arcade_speed)
    rclcpp::Subscription<drivetrain_msgs::msg::ArcadeSpeed>::SharedPtr arcade_sub_;
    
    // publishers to set motor speeds
    rclcpp::Publisher<drivetrain_msgs::msg::MotorSpeeds>::SharedPtr motor_speeds_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr odrive_pub_;

    // functions to compute motor speeds
    void arcade_callback(const drivetrain_msgs::msg::ArcadeSpeed arcade_msg);
    drivetrain_msgs::msg::MotorSpeeds compute_motor_speeds(const float arcade_l, const float arcade_r);
    
    void publish_speeds_odrive(drivetrain_msgs::msg::MotorSpeeds speeds);
    const nlohmann::json odrive_speed_req_json = {
        {"Stage", "run"},
        {"Type", "request"},
        {"Target", "Drivetrain"},
        {"Command", "Set_Input_Vel"}
    };
    const float VEL_SCALER = 40;
};

#endif 
