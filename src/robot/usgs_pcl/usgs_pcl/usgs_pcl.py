#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
import laspy
import numpy as np

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2


class USGSPointCloudPublisher(Node):
    def __init__(self):
        super().__init__("usgs_pcl_publisher")

        # Parameter for the LAS path (default points to the file you copied)
        self.declare_parameter("las_path", "/root/terrain.las")
        las_path = self.get_parameter("las_path").get_parameter_value().string_value

        if not os.path.exists(las_path):
            raise FileNotFoundError(f"LAS file not found: {las_path}")

        self.get_logger().info(f"Loading LAS: {las_path}")
        las = laspy.read(las_path)

        # Build Nx3 float32 array
        self.points = np.vstack((las.x, las.y, las.z)).astype(np.float32).T

        # Publisher and timer (1 Hz so Foxglove won’t miss it)
        self.pub = self.create_publisher(PointCloud2, "/terrain_cloud", 10)
        self.timer = self.create_timer(1.0, self._tick)

    def _tick(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"  # Set Foxglove 3D "Fixed frame" to "map"
        msg = pc2.create_cloud_xyz32(header, self.points)
        self.pub.publish(msg)
        # Optional: uncomment for logs
        # self.get_logger().info(f"Published {self.points.shape[0]} points")


def main(args=None):
    rclpy.init(args=args)
    node = USGSPointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
