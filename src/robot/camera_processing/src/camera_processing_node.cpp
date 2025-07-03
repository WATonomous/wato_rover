#include "camera_processing_node.hpp"

CameraProcessingNode::CameraProcessingNode(): Node("camera_processing"), camera_processing_(robot::CameraProcessingCore(this->get_logger())) {}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraProcessingNode>());
  rclcpp::shutdown();
  return 0;
}
