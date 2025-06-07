#include "motor_speed_controller.hpp"

using namespace std::placeholders;

MotorSpeedController::MotorSpeedController() : Node("motor_speed_controller") {
    // Create publisher for joystick messages
    arcade_sub_ = this->create_subscription<drivetrain_msgs::msg::ArcadeSpeed>(
        "/arcade_speed", 10,
        std::bind(&MotorSpeedController::arcade_callback, this, _1));

    motor_speeds_pub_ = this->create_publisher<drivetrain_msgs::msg::MotorSpeeds>(
        "/cmd_vel_out", 10);

    odrive_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/OdriveJsonSub", 10);
}

void MotorSpeedController::arcade_callback(const drivetrain_msgs::msg::ArcadeSpeed arcade_msg) {
    // Ignore arcade msg if component is inactive. This component shouldn't be inactive
    //  when ArcadeDriver is publishing ArcadeSpeed messages though.
	if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
		RCLCPP_WARN(get_logger(), "Received arcade message while not active, ignoring...");
		return;
	}
    float arcade_left = arcade_msg.l;
    float arcade_right = arcade_msg.r;

    drivetrain_msgs::msg::MotorSpeeds motor_speeds_msg = MotorSpeedController::compute_motor_speeds(arcade_left, arcade_right);
    motor_speeds_pub->publish(motor_speeds_msg);
    publish_speeds_odrive(motor_speeds_msg);
}

void MotorSpeedController::publish_speeds_odrive(drivetrain_msgs::msg::MotorSpeeds speeds) {
    std_msgs::msg::String req_msg;
    nlohmann::json req_json = odrive_speed_req_json;

    req_json["Payload"] = {
        {"1", speeds.m1},
        {"2", speeds.m2},
        {"3", speeds.m3},
        {"4", speeds.m4},
        {"5", speeds.m5},
        {"6", speeds.m6}
    };
    req_msg.data = req_json.dump();

    odrive_pub->publish(req_msg);
}

drivetrain_msgs::msg::MotorSpeeds MotorSpeedController::compute_motor_speeds(const float arcade_l, const float arcade_r) {
    auto motor_speeds_msg = drivetrain_msgs::msg::MotorSpeeds();
    motor_speeds_msg.m1 = arcade_l * 0.8 * VEL_SCALER;
    motor_speeds_msg.m2 = arcade_l * VEL_SCALER;
    motor_speeds_msg.m3 = arcade_l * -1 * VEL_SCALER;
    motor_speeds_msg.m4 = arcade_r * 0.8 * VEL_SCALER * -1;
    motor_speeds_msg.m5 = arcade_r * -1 * VEL_SCALER;
    motor_speeds_msg.m6 = arcade_r * VEL_SCALER;
    return motor_speeds_msg;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArcadeDriver>());
    rclcpp::shutdown();
    return 0;
}
