#!/usr/bin/env python3
"""
Load a YOLO model (ONNX by default, optional .pt via torch)
subscribe to /image topic and log detection results
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import onnxruntime as ort # rosdep: python3-onnxruntime
from ament_index_python.packages import get_package_share_directory

try:
    import torch # rosdep: python3-torch (optional)
    TORCH_OK = True
except ImportError:
    TORCH_OK = False


class YoloInference(Node):
    def __init__(self):
        super().__init__("yolo_inference_node")

        pkg_share = get_package_share_directory("yolo_inference")
        default_model = os.path.join(pkg_share, "models", "yolov8.onnx")

        self.declare_parameter("model_path", default_model)
        self.declare_parameter("input_size", 640)

        self.model_path: str = self.get_parameter(
            "model_path").get_parameter_value().string_value
        self.input_size: int = self.get_parameter(
            "input_size").get_parameter_value().integer_value

        if not self.model_path:
            self.get_logger().fatal("Parameter 'model_path' is required.")
            raise RuntimeError("model_path parameter unset")

        # Load model
        ext = os.path.splitext(self.model_path)[1]
        if ext == ".onnx":
            self.session = ort.InferenceSession(
                self.model_path,
                providers=["CPUExecutionProvider"],
            )
            self.input_name = self.session.get_inputs()[0].name
            self.get_logger().info(f"Loaded ONNX model: {self.model_path}")
            self.run = self.run_onnx
        elif ext in (".pt", ".pth"):
            if not TORCH_OK:
                raise RuntimeError(
                    ".pt model requested but 'torch' not available")
            self.model = torch.jit.load(self.model_path).eval()
            self.get_logger().info(f"Loaded TorchScript model: {self.model_path}")
            self.run = self.run_torch
        else:
            raise RuntimeError(f"Unsupported model extension: {ext}")
        
        self.annotated_pub = self.create_publisher(Image, "/detections_image", 10)

        # Bridge & subscription
        self.bridge = CvBridge()
        self.create_subscription(Image, "/image", self.cb_image, 10)

    ## inference 
    def run_onnx(self, img: np.ndarray):
        inp = self.preprocess(img)
        preds = self.session.run(None, {self.input_name: inp})[0]
        return self.postprocess(preds)

    def run_torch(self, img: np.ndarray):
        inp = self.preprocess(img, torch_tensor=True)
        with torch.no_grad():
            preds = self.model(inp)[0].cpu().numpy()
        return self.postprocess(preds)

    ## ros callback
    def cb_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        detections = self.run(frame)

        for x1, y1, x2, y2, conf, cls in detections:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{cls}:{conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Publish the annotated image
        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.annotated_pub.publish(annotated_msg)

        self.get_logger().info(f"Detected {len(detections)} objects")


    ## helpers
    def preprocess(self, img: np.ndarray, torch_tensor=False):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (self.input_size, self.input_size))
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))   # HWC to CHW
        img = np.expand_dims(img, 0) # NCHW
        if torch_tensor:
            import torch
            return torch.from_numpy(img)
        return img

    def postprocess(self, raw: np.ndarray):
        """
        Convert YOLOv8 raw output of shape [1, N, 85] to usable boxes.
        Format: [x_center, y_center, width, height, conf, cls_probs...]
        """
        conf_threshold = 0.5
        results = []

        predictions = raw[0]  # shape: [N, 85]
        for det in predictions:
            conf = det[4]
            if conf < conf_threshold:
                continue
            class_id = np.argmax(det[5:])
            class_conf = det[5 + class_id]
            if class_conf < conf_threshold:
                continue

            xc, yc, w, h = det[0:4]
            x1 = int((xc - w / 2) * self.input_size)
            y1 = int((yc - h / 2) * self.input_size)
            x2 = int((xc + w / 2) * self.input_size)
            y2 = int((yc + h / 2) * self.input_size)

            results.append((x1, y1, x2, y2, float(class_conf), int(class_id)))

        return results




def main(args=None):
    rclpy.init(args=args)
    try:
        node = YoloInference()
        rclpy.spin(node)
    except RuntimeError as e:
        rclpy.get_logger("yolo_inference_node").error(str(e))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
