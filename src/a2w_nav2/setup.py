import os
from glob import glob

from setuptools import setup

package_name = "a2w_nav2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml") + glob("config/*.rviz")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "behavior_trees"), glob("behavior_trees/*.xml")),
        (os.path.join("share", package_name), ["README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xiaozhu",
    maintainer_email="xiaozhuABCD1234@163.com",
    description="A2W 2D 导航：前雷达→扫描/过滤云、PCD→占据栅格、Nav2 组合式 bringup",
    license="MIT",
    entry_points={
        "console_scripts": [
            "a2w_scan = a2w_nav2.scan:main",
            "a2w_pcd_to_map = a2w_nav2.pcd_to_map:main",
        ],
    },
)
