cmake_minimum_required(VERSION 3.10)
project(gps_sim)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED) # ROS2 build tool
find_package(rclcpp REQUIRED)      # ROS2 C++ package
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create ROS2 node executable from source files
add_executable(gps_sim_node src/gps_sim_node.cpp)
target_include_directories(gps_sim_node
  PUBLIC include)
# Add ROS2 dependencies required by package
ament_target_dependencies(gps_sim_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
)

# Copy executable to installation location
install(TARGETS
  gps_sim_node
  DESTINATION lib/${PROJECT_NAME})

# Copy config files to installation location
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()

