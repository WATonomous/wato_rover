cmake_minimum_required(VERSION 3.8)
project(camera_fallback)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executable
add_executable(camera_fallback_node src/camera_fallback_node.cpp)

# Link dependencies
ament_target_dependencies(camera_fallback_node
  rclcpp
  sensor_msgs
  tf2_ros
  geometry_msgs
)

# Install the executable
install(TARGETS
  camera_fallback_node
  DESTINATION lib/${PROJECT_NAME}
)

# Optional: Add test support if needed
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export package dependencies
ament_package()