#ifndef CAMERA_PROCESSOR_NODE_HPP_
#define CAMERA_PROCESSOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "camera_processor_core.hpp"

class CameraProcessorNode : public rclcpp::Node {
  public:
    CameraProcessorNode();

  private:
    robot::CameraProcessorCore camera_processor_;
};

#endif