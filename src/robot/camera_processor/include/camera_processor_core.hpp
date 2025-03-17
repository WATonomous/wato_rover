#ifndef CAMERA_PROCESSOR_CORE_HPP_
#define CAMERA_PROCESSOR_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{
  
class CameraProcessorCore {
  public:
    explicit CameraProcessorCore(const rclcpp::Logger& logger);

  private:
    rclcpp::Logger logger_;
};

}

#endif