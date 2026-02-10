# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Launch only the camera-from-URL and YOLO object detection nodes for testing
# with your Mac's built-in camera. Run scripts/mac_camera_stream.py on the host first.
# Use with Foxglove (vis_tools) to view /image and /detections_image.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    image_url_arg = DeclareLaunchArgument(
        "image_url",
        default_value="http://host.docker.internal:9999/frame",
        description="URL for the Mac camera stream (from scripts/mac_camera_stream.py)",
    )
    ld.add_action(image_url_arg)

    camera_node = Node(
        package="camera_from_url",
        executable="camera_from_url_node",
        name="camera_from_url_node",
        parameters=[{"image_url": LaunchConfiguration("image_url")}],
    )
    ld.add_action(camera_node)

    yolo_node = Node(
        package="yolo_inference",
        executable="yolo_inference_node",
        name="yolo_inference_node",
    )
    ld.add_action(yolo_node)

    return ld
