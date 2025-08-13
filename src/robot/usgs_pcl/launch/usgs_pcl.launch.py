from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    usgs_pcl_node = Node(
            package='usgs_pcl',
            executable='usgs_pcl',
            name='usgs_pcl_node',
    )

    ld.add_action(usgs_pcl_node)

    return ld

