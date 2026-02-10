# Copyright (c) 2025-present WATonomous. All rights reserved.

from setuptools import setup

package_name = "camera_from_url"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["opencv-python-headless", "numpy"],
    entry_points={
        "console_scripts": [
            "camera_from_url_node = camera_from_url.camera_from_url_node:main",
        ],
    },
)
