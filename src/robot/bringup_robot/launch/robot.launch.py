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

    return ld