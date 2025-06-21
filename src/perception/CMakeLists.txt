cmake_minimum_required(VERSION 3.10)
project(pointcloud_converter)

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
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(perception_node src/pointcloud_converter.cpp)

# Compiles source files into a library
# A library is not executed, instead other executables can link
# against it to access defined methods and classes.
# We build a library so that the methods defined can be used by
# both the unit test and ROS2 node executables.
add_library(perception_lib
  src/pointcloud_converter.cpp)
# Indicate to compiler where to search for header files
target_include_directories(perception_lib
  PUBLIC
  include)

# Add ROS2 dependencies required by package
ament_target_dependencies(perception_lib
  rclcpp
  sensor_msgs 
  pcl_conversions
  PCL
)

# Create ROS2 node executable from source files
add_executable(perception_node src/pointcloud_converter.cpp)
# Link to the previously built library to access Transformer classes and methods
target_link_libraries(perception_node perception_lib)

# Copy executable to installation location
install(TARGETS
  perception_node
  DESTINATION lib/${PROJECT_NAME})

# Copy launch and config files to installation location
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()