from setuptools import setup
import os
from glob import glob

package_name = "smc_demo"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Sliding Mode Control visualization demo for ROS 2 + RViz",
    license="MIT",
    entry_points={
        "console_scripts": [
            "smc_controller = smc_demo.smc_controller_node:main",
        ],
    },
)
