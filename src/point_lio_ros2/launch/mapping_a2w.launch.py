"""A2W 前雷达(JT128) + Point-LIO 建图/里程计。

    ros2 launch point_lio mapping_a2w.launch.py                  # 带 RViz
    ros2 launch point_lio mapping_a2w.launch.py rviz:=false       # 只看话题/存 PCD
    ros2 launch point_lio mapping_a2w.launch.py pcd_save:=true     # 退出时把点云落盘

⚠️ 点云/IMU 由 **a2w_bridge** 提供，本 launch 不起桥。完整链路：

    # 终端 1：桥（必须用 LIO 那份配置：它打开了 ring/timestamp 字段、IMU 钉在前雷达）
    source src/a2w_bridge/scripts/a2w_env.sh
    ros2 launch a2w_bridge a2w_bridge.launch.py config:=$(ros2 pkg prefix --share a2w_bridge)/config/a2w_bridge_lio.json

    # 终端 2：本 launch（同样先 source a2w_env.sh，否则不在同一 ROS 域）
    ros2 launch point_lio mapping_a2w.launch.py

懒人版（一条命令起桥 + LIO + RViz）：``ros2 launch a2w_bridge a2w_lio.launch.py``。

为什么这里不像 mapping_mid360.launch.py 那样再传一个参数字典（LOCAL PATCH P5 的教训）：
节点 parameters 列表里**后面的项会覆盖前面的**，把参数写进 launch 就等于让 yaml 静默失效。
本 launch 只传 config/a2w.yaml 一个来源，改参数只改 yaml。
"""

from typing import Any

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _laser_mapping_node(context):
    """组装 laserMapping 节点：唯一参数来源 = config/a2w.yaml（+ 可选的 pcd_save 开关）。

    LOCAL PATCH P9(见 docs/PATCHES.md)：A2W 用的这份 launch。
    ``pcd_save`` 留空 = 用 yaml 的 ``pcd_save.pcd_save_en``；显式 true/false 才追加覆盖项
    （写法与 mapping_mid360.launch.py 的 P7 一致：节点 parameters 列表**后者覆盖前者**，
    所以不能无条件把这项塞进字典，否则 yaml 又被顶掉）。
    """
    params: list[Any] = [
        PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'config', 'a2w.yaml'
        ]).perform(context),
    ]

    raw = context.perform_substitution(LaunchConfiguration('pcd_save')).strip().lower()
    if raw:
        if raw not in ('true', 'false', '1', '0', 'yes', 'no', 'on', 'off'):
            # 启动前报错，而不是把字符串塞进 bool 参数触发
            # InvalidParameterTypeException（那种报错看不出是哪来的）
            raise RuntimeError(f"pcd_save 只接受 true/false，收到: {raw!r}")
        params.append({'pcd_save': {'pcd_save_en': raw in ('true', '1', 'yes', 'on')}})

    return [Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=params,
    )]


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='是否同时起 RViz（配置：point_lio 的 rviz_cfg/loam_livox.rviz，'
                    'Fixed Frame = camera_init）')

    pcd_save_arg = DeclareLaunchArgument(
        'pcd_save', default_value='',
        description='点云落盘开关。留空 = 用 config/a2w.yaml 的 pcd_save.pcd_save_en；'
                    '显式传 true/false 才覆盖。注意：只在节点【正常退出】(Ctrl+C) 时写一次，'
                    '写到 ./PCD/scans.pcd（相对运行本 launch 的目录）')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg', 'loam_livox.rviz'
        ])],
        prefix='nice',
    )

    return LaunchDescription([
        rviz_arg,
        pcd_save_arg,
        OpaqueFunction(function=_laser_mapping_node),
        GroupAction(
            actions=[rviz_node],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
