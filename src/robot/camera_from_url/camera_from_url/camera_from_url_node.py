#!/usr/bin/env python3
# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Fetches JPEG images from an HTTP URL (e.g. Mac camera stream from
# scripts/mac_camera_stream.py) and publishes sensor_msgs/Image to /image
# for YOLO or other nodes.

import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraFromUrlNode(Node):
    def __init__(self):
        super().__init__("camera_from_url_node")
        self.declare_parameter("image_url", "http://host.docker.internal:9999/frame")
        self.declare_parameter("output_topic", "/image")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("timeout_sec", 2.0)

        self.image_url = self.get_parameter("image_url").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        self.timeout_sec = self.get_parameter("timeout_sec").get_parameter_value().double_value

        self.pub = self.create_publisher(Image, self.output_topic, 10)
        self.bridge = CvBridge()
        period_ms = int(1000.0 / rate_hz)
        self.timer = self.create_timer(period_ms / 1000.0, self.timer_callback)

        self.get_logger().info(
            "Publishing %s from %s at %.1f Hz" % (self.output_topic, self.image_url, rate_hz)
        )

    def timer_callback(self):
        try:
            req = urllib.request.Request(self.image_url)
            with urllib.request.urlopen(req, timeout=self.timeout_sec) as resp:
                data = resp.read()
        except Exception as e:
            self.get_logger().warn_throttle(5.0, "Failed to fetch image: %s" % e)
            return
        arr = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn_throttle(5.0, "Failed to decode JPEG")
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraFromUrlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
