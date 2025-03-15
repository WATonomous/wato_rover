from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

def generate_launch_description():
    ld = LaunchDescription()  # Begin building a launch description

    #################### Odometry Spoof Node #####################
    odometry_spoof_node = Node(
        package='odometry_spoof',
        name='odometry_spoof',
        executable='odometry_spoof',
    )
    ld.add_action(odometry_spoof_node)

    #################### RealSense D435 Node #####################
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_d435',
        output='screen',
        parameters=[
            {'serial_no': ''},  # Leave blank to use the first connected camera, or specify the serial number
            {'usb_port_id': ''},  # Leave blank unless targeting a specific USB port
            {'enable_pointcloud': True},  # Enable point cloud output
            {'pointcloud_ordered': True},  # Ordered point cloud (optional, for better visualization)
            {'depth_module.profile': '1280x720x30'},  # Depth stream: 1280x720 at 30 FPS
            {'rgb_camera.profile': '640x480x30'},  # RGB stream: 640x480 at 30 FPS (optional)
            {'enable_color': True},  # Enable RGB stream (optional)
            {'enable_depth': True},  # Enable depth stream
            {'align_depth.enable': True},  # Align depth to RGB (optional, affects point cloud)
            {'base_frame_id': 'camera_link'},  # TF frame for the camera base
            {'depth_frame_id': 'camera_depth_frame'},  # TF frame for depth
            {'pointcloud_frame_id': 'camera_depth_frame'},  # TF frame for point cloud
        ],
        remappings=[
            ('/pointcloud', '/realsense/depth/points'),  # Remap to match your simulation naming
            ('/depth/image_rect_raw', '/realsense/depth/image'),  # Remap depth image
            ('/color/image_raw', '/realsense/color/image'),  # Optional RGB image
            ('/depth/camera_info', '/realsense/depth/camera_info'),  # Depth camera info
            ('/color/camera_info', '/realsense/color/camera_info'),  # RGB camera info
        ]
    )
    ld.add_action(realsense_node)

    return ld