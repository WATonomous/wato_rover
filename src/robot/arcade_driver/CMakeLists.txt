cmake_minimum_required(VERSION 3.10)
project(arcade_driver)

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
find_package(rclcpp_components REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
# find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)
# find_package(lifecycle_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(nlohmann_json REQUIRED)

# Create ROS2 node executable from source files
add_executable(arcade_driver src/arcade_driver.cpp)
target_include_directories(arcade_driver
  PUBLIC include)
# Add ROS2 dependencies required by package
ament_target_dependencies(arcade_driver
  "rclcpp"
  # "rclcpp_components"
  # "rclcpp_lifecycle"
  # "lifecycle_msgs"
  "geometry_msgs"
  "std_msgs"
)

# Copy executable to installation location
install(TARGETS
  arcade_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
