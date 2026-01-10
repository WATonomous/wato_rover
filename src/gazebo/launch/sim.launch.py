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
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess

from launch_ros.actions import Node


def generate_launch_description():
    gazebo_pkg_prefix = get_package_share_directory("gazebo")
    gazebo_sim_ign = os.path.join(gazebo_pkg_prefix, "launch", "sim.ign")
    sdf_file_path = os.path.join(gazebo_pkg_prefix, "launch", "robot_env.sdf")

    gz_sim = ExecuteProcess(cmd=["ign", "launch", "-v 4", f"{gazebo_sim_ign}"])
    gz_sim_server = ExecuteProcess(
        cmd=["ign", "gazebo", "-s", "-v 4", "-r", f"{sdf_file_path}"]
    )

    # Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/robot/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
            "/model/robot/pose_static@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
            "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU",
            #    '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            "/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            "/model/robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
        ],
        parameters=[
            {"qos_overrides./model/vehicle_blue.subscriber.reliability": "reliable"}
        ],
        output="screen",
        remappings=[
            ("/model/robot/pose", "/tf"),
            ("/model/robot/pose_static", "/tf"),
            ("/camera/camera_info", "/camera_info"),
        ],
    )

    return LaunchDescription([gz_sim, gz_sim_server, bridge])
