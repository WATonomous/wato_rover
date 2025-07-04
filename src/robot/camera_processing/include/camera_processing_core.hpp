#ifndef CAMERA_PROCESSING_CORE_HPP_
#define CAMERA_PROCESSING_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{

  class CameraProcessingCore
  {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    CameraProcessingCore(const rclcpp::Logger &logger);

  private:
    rclcpp::Logger logger_;
  };

}

#endif
