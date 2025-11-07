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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    ld = LaunchDescription()  # Begin building a launch description

    yolo_pkg_share = get_package_share_directory("yolo_inference")
    default_model_path = os.path.join(yolo_pkg_share, "models", "yolov8.onnx")

    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value=default_model_path,
        description="Absolute path to the YOLO ONNX/PT weights file",
    )
    ld.add_action(model_path_arg)

    #################### Odometry Spoof Node #####################
    odometry_spoof_node = Node(
        package="odometry_spoof",
        name="odometry_spoof",
        executable="odometry_spoof",
    )
    ld.add_action(odometry_spoof_node)

    #################### RealSense Node #####################
    realsense_launch_file = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
    )
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={"pointcloud.enable": "true", "filters": "spatial"}.items(),
    )
    ld.add_action(realsense_launch)

    #################### Camera Fallback Node #####################
    camera_fallback_node = Node(
        package="camera_fallback",
        name="camera_fallback_node",
        executable="camera_fallback_node",
    )
    ld.add_action(camera_fallback_node)

    #################### ArcadeDriver Node #####################
    arcade_driver_node = Node(
        package="arcade_driver",
        name="arcade_driver",
        executable="arcade_driver",
    )
    ld.add_action(arcade_driver_node)

    #################### MotorSpeedController Node #####################
    motor_speed_controller_node = Node(
        package="motor_speed_controller",
        name="motor_speed_controller",
        executable="motor_speed_controller",
    )
    ld.add_action(motor_speed_controller_node)

    #################### Object_detection Node #####################
    object_detection_node = Node(
        package="object_detection",
        name="object_detection_node",
        executable="object_detection_node",
    )
    ld.add_action(object_detection_node)

    ld.add_action(
        Node(
            package="yolo_inference",
            executable="yolo_inference_node",
            name="yolo_inference_node",
            output="screen",
            parameters=[
                {
                    "model_path": LaunchConfiguration("model_path"),
                    "input_size": 640,  # TODO: adjust DEPENDING on model needs 416, 512, etc
                }
            ],
            remappings=[("/image", "/sim/realsense1/depth/image")],
        )
    )
    return ld
