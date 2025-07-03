#ifndef CAMERA_PROCESSING_NODE_HPP_
#define CAMERA_PROCESSING_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "camera_processing_core.hpp"

class CameraProcessingNode : public rclcpp::Node {
  public:
    CameraProcessingNode();

  private:
    robot::CameraProcessingCore camera_processing_;
};

#endif
