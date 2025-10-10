# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
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
find_package(drivetrain_msgs)
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
  "drivetrain_msgs"
)

# Copy executable to installation location
install(TARGETS
  arcade_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
