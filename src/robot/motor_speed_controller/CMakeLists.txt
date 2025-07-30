cmake_minimum_required(VERSION 3.10)
project(motor_speed_controller)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(drivetrain_msgs)

# Create ROS2 node executable from source files
add_executable(motor_speed_controller src/motor_speed_controller.cpp)
target_include_directories(motor_speed_controller
  PUBLIC include)
# Add ROS2 dependencies required by package
ament_target_dependencies(motor_speed_controller
  "rclcpp"
  # "rclcpp_components"
  # "rclcpp_lifecycle"
  # "lifecycle_msgs"
  "geometry_msgs"
  "std_msgs"
  "drivetrain_msgs"
)

# Copy executable to installation location
install(TARGETS
  motor_speed_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
