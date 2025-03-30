from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Get the package share directory
    robot_navigation_pkg = get_package_share_directory('robot_navigation')
    bringup_robot_pkg = get_package_share_directory('bringup_robot')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = os.path.join(robot_navigation_pkg, 'config', 'nav2_params.yaml')
    ekf_params_file = os.path.join(robot_navigation_pkg, 'config', 'ekf.yaml')

    # Include the bringup_robot launch file to start the robot and sensors
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_robot_pkg, 'launch', 'sim.launch.py')
        )
    )

    # Launch robot_localization for EKF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}]
    )

    # Launch Nav2
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        bringup_launch,
        ekf_node,
        nav2_bringup_launch,
    ])