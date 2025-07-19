#include "object_detection_node.hpp"

Object_detectionNode::Object_detectionNode()
  : Node("object_detection"),
    object_detection_(robot::Object_detectionCore(this->get_logger())) {}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Object_detectionNode>());
  rclcpp::shutdown();
  return 0;
}
