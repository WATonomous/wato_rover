#include <chrono>
#include <memory>

#include "camera_processor_node.hpp"

CameraProcessorNode::CameraProcessorNode() : Node("camera_processor_node"), camera_processor_(this->get_logger()) {}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraProcessorNode>());
    rclcpp::shutdown();
    return 0;
}