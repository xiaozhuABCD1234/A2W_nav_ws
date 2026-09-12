import os
from glob import glob

from setuptools import setup

package_name = "a2w_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.json") + glob("config/*.xml") + glob("config/*.rviz") + glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.sh")),
        (os.path.join("share", package_name), ["README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xiaozhu",
    maintainer_email="xiaozhuABCD1234@163.com",
    description="A2W 机器人数据 → ROS2 桥接（点云/IMU/里程计/栅格，JSON 配置）",
    license="MIT",
    entry_points={
        "console_scripts": [
            "a2w_bridge_node = a2w_bridge.node:main",
            "joint_relay = a2w_bridge.joint_relay:main",
            "a2w_base_height = a2w_bridge.base_height:main",
            "a2w_base_footprint = a2w_bridge.base_footprint:main",
        ],
    },
)