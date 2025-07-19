#ifndef PACKAGE_NAME_CORE_HPP_
#define PACKAGE_NAME_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{

    class Object_detectionCore
    {
    public:
        explicit Object_detectionCore(const rclcpp::Logger &logger);

    private:
        rclcpp::Logger logger_;
    };

} // namespace robot

#endif
