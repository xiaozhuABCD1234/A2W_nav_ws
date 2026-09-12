import os
from glob import glob

from setuptools import setup

package_name = "a2w_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xiaozhu",
    maintainer_email="xiaozhu@todo.todo",
    description="Xbox 手柄遥操作：joy + teleop_twist_joy 发布 /cmd_vel",
    license="Apache-2.0",
    python_requires=">=3.8",
)