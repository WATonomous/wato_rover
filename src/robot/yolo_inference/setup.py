from setuptools import setup

package_name = "yolo_inference"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/models", ["models/yolov8.onnx"]),
    ],
    install_requires=[
        "onnxruntime", 
        "numpy",
        "opencv-python",
        "torch ; extra == 'torch'",   # optional - if we want .pt support
    ],
    entry_points={
        "console_scripts": [
            "yolo_inference_node = yolo_inference.yolo_inference_node:main",
        ],
    },
)
