"""a2w_description 的 URDF 显示启动器（滑条/假值，纯离线看模型）。

    ros2 launch a2w_description display.launch.py            # joint_state_publisher_gui 拖滑条

⚠️ 本启动器**不接实机**：关节角来自 joint_state_publisher_gui 的滑条。
   想看**实机关节角**（只读、不控制），用桥接包里的显示启动器：

       source src/a2w_bridge/scripts/a2w_env.sh
       ros2 launch a2w_bridge a2w_joint_display.launch.py

   它会把 /a2w/joint_states（SDK 名）转成 URDF 名喂给 robot_state_publisher。
   两边都要 source a2w_env.sh，因为桥默认把 ROS2 隔离在独立域（防触发机器狗软急停）。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('a2w_description')
    default_urdf = os.path.join(pkg_share, 'urdf', 'a2w_description.urdf')

    gui = LaunchConfiguration('gui')
    model = LaunchConfiguration('model')

    # 直接读取 URDF 文件内容作为字符串参数,避免 launch_ros 对路径的歧义处理
    try:
        with open(default_urdf, 'r', encoding='utf-8') as f:
            robot_description = f.read()
    except OSError as e:
        raise RuntimeError('Failed to read URDF file: {}'.format(default_urdf)) from e

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=default_urdf,
            description='Path to the URDF file'),
        DeclareLaunchArgument('gui', default_value='true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui),
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'urdf.rviz')],
            output='screen',
        ),
        LogInfo(msg='URDF model: ' + default_urdf),
    ])