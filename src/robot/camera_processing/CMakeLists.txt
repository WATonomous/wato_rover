cmake_minimum_required(VERSION 3.10)
project(camera_processing)

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

# Compiles source files into a library
# A library is not executed, instead other executables can link
# against it to access defined methods and classes.
# We build a library so that the methods defined can be used by
# both the unit test and ROS2 node executables.
add_library(camera_processing_lib
  src/camera_processing_core.cpp)
# Indicate to compiler where to search for header files
target_include_directories(camera_processing_lib
  PUBLIC
  include)

# Add ROS2 dependencies required by package
ament_target_dependencies(camera_processing_lib
  rclcpp
)

# Create ROS2 node executable from source files
add_executable(camera_processing_node src/camera_processing_node.cpp)
# Link to the previously built library to access Transformer classes and methods
target_link_libraries(camera_processing_node camera_processing_lib)

# Copy executable to installation location
install(TARGETS
camera_processing_node
  DESTINATION lib/${PROJECT_NAME})

# Copy launch and config files to installation location
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()