#ifndef PACKAGE_NAME_NODE_HPP_
#define PACKAGE_NAME_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "object_detection_core.hpp"

class Object_detectionNode : public rclcpp::Node
{
public:
    Object_detectionNode();

private:
    robot::Object_detectionCore object_detection_;
};

#endif
