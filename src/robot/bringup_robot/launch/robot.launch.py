from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    #################### RealSense Node #####################
    realsense_launch_file = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'pointcloud.enable': 'true',
            'filters': 'spatial'
        }.items()
    )
    ld.add_action(realsense_launch)

    #################### Camera Fallback Node #####################
    camera_fallback_node = Node(
        package='camera_fallback',
        name='camera_fallback_node',
        executable='camera_fallback_node',
    )
    ld.add_action(camera_fallback_node)

    #################### ArcadeDriver Node #####################
    arcade_driver_node = Node(
        package='arcade_driver',
        name='arcade_driver',
        executable='arcade_driver',
    )
    ld.add_action(arcade_driver_node)

    #################### MotorSpeedController Node #####################
    motor_speed_controller_node = Node(
        package='motor_speed_controller',
        name='motor_speed_controller',
        executable='motor_speed_controller',
    )
    ld.add_action(motor_speed_controller_node)

    #################### CameraProcessing Node #####################
    camera_processing_node = Node(
        package='camera_processing',
        name='camera_processing_node',
        executable='camera_processing_node',
    )
    ld.add_action(camera_processing_node)
    
    return ld