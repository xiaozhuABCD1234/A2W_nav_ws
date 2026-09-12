"""A2W 一键：桥（LIO 配置）+ Point-LIO（+ 可选 RViz）。

    ros2 launch a2w_bridge a2w_lio.launch.py
    ros2 launch a2w_bridge a2w_lio.launch.py show_rviz:=false      # 只看话题/存 PCD
    ros2 launch a2w_bridge a2w_lio.launch.py pcd_save:=true        # 退出时落盘 ./PCD/scans.pcd
    ros2 launch a2w_bridge a2w_lio.launch.py lio:=false            # 只起桥（自己再起 LIO）

数据流（全程只读，不下发任何控制指令）：

    rt/unitree/slam_lidar/points1 ─采集器(只 subscribe)─▶ /a2w/points ─┐
    rt/unitree/slam_lidar/imu1    ─采集器(只 subscribe)─▶ /a2w/imu   ─┴─▶ Point-LIO
                                                                        └─▶ /cloud_registered(_body)
                                                                            /path, TF: camera_init→body

为什么默认用 ``config/a2w_bridge_lio.json``（extends 主配置）而不是主配置：
Point-LIO 的 HESAI 分支要 ``ring(u2) + timestamp(f8)`` 两个字段（逐点时间去畸变），
主配置只发 x/y/z/intensity；这份继承配置把 fields 打开、并把 IMU 钉在**前雷达那一路**
（``imu.source: lidar_front``）—— 因为 ``auto`` 会降级到后雷达/本体 IMU，坐标系数就变了，
LIO 的外参（单位阵）随即失效。

⚠️ 同 ``a2w_points_rviz.launch.py``：本 launch 会给**自己起的子进程**设好 ROS 域隔离
（JSON 的 ros.domain_id + FastDDS 只走回环），所以它在哪个终端跑都行；
但你自己另开终端看话题/起 RViz 前仍要先 ``source src/a2w_bridge/scripts/a2w_env.sh``。
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 静态检查看不到这个导入（launch/ 目录不在包搜索路径），运行时由 ROS 环境提供：
# 编译后 install 会把 a2w_bridge 包加到 PYTHONPATH，launch 进程可以直接 import。
from a2w_bridge.launch_common import (  # pyright: ignore[reportMissingImports]
    config_defaults,
    isolation_actions,
    package_share,
)

################### user configure parameters for ros2 start ###################
lio_config_name = "a2w_bridge_lio.json"   # 桥的 LIO 配置（share/a2w_bridge/config/ 下）
start_bridge = True                       # 由本 launch 起桥；已在别处跑桥就设 false
start_lio = True                          # 是否起 Point-LIO
start_rviz = True                         # 是否起 RViz2
################### user configure parameters for ros2 end #####################

cur_path = os.path.split(os.path.realpath(__file__))[0] + "/"
cur_config_path = os.path.join(cur_path, "..", "config")


def _default_bridge_config() -> str:
    """LIO 用的桥配置路径：优先包 share（安装后），退回源码树 config/。"""
    share = package_share("a2w_bridge")
    installed = share / "config" / lio_config_name if share is not None else None
    if installed is not None and installed.is_file():
        return str(installed)
    return os.path.normpath(os.path.join(cur_config_path, lio_config_name))


def _default_rviz_config() -> str:
    """建图 RViz 配置：用 point_lio 自带的（Fixed Frame = camera_init，显示 LIO 点云/path）。"""
    share = package_share("point_lio")
    if share is not None:
        for candidate in (share / "rviz_cfg" / "loam_livox.rviz",):
            if candidate.is_file():
                return str(candidate)
    return ""


def _lio_launch() -> IncludeLaunchDescription:
    """point_lio 的 mapping_a2w.launch.py；RViz 由本 launch 起，所以给它 rviz:=false。"""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("point_lio"),
            "launch", "mapping_a2w.launch.py",
        ])),
        launch_arguments={
            # 用 rviz:=false 而不是让 LIO 自己起 RViz：本 launch 要控制窗口生命周期
            # （关 RViz = 全部退出）。注意 include 的 launch 参数会写进**共享**的
            # launch_configurations，所以本文件自己的开关叫 show_rviz 不叫 rviz
            # —— 同 mid360_bringup/mid360.launch.py 踩过的坑。
            "rviz": "false",
            "pcd_save": LaunchConfiguration("pcd_save"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("lio")),
    )


def generate_launch_description() -> LaunchDescription:
    cfg = config_defaults(_default_bridge_config())
    isolate = LaunchConfiguration("isolate")
    rviz_config = LaunchConfiguration("rviz_config")
    default_rviz_cfg = _default_rviz_config()

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("show_rviz")),
        # 找不到 point_lio 的 rviz 配置（包没 build）就不传参数，让 rviz2 用默认窗口，
        # 而不是把空串当配置路径传进去（rviz2 会直接报错退出）。
        arguments=["--display-config", rviz_config] if default_rviz_cfg else [],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=cfg["config_path"],
                description="桥的 JSON 配置路径（默认 = LIO 那份继承配置）",
            ),
            DeclareLaunchArgument(
                "iface",
                default_value="",
                description="覆盖 JSON 里的网卡名（如 enx00e04c2c4260）",
            ),
            DeclareLaunchArgument(
                "bridge",
                default_value=str(start_bridge).lower(),
                description="是否启动桥接节点（点云/IMU 来源）；已在别处跑桥就设 false",
            ),
            DeclareLaunchArgument(
                "lio",
                default_value=str(start_lio).lower(),
                description="是否启动 Point-LIO（mapping_a2w.launch.py）",
            ),
            DeclareLaunchArgument(
                "pcd_save",
                default_value="",
                description="Point-LIO 点云落盘（留空=用 a2w.yaml；true/false 才覆盖）。"
                            "只在 Ctrl+C 正常退出时写 ./PCD/scans.pcd",
            ),
            DeclareLaunchArgument(
                "show_rviz",
                default_value=str(start_rviz).lower(),
                description="是否启动 RViz2（关掉窗口 = 整个 launch 退出）",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_cfg,
                description="RViz 配置（默认 point_lio 的 rviz_cfg/loam_livox.rviz）",
            ),
            DeclareLaunchArgument(
                "isolate",
                default_value=str(cfg["isolate"]).lower(),
                description="ROS2 与机器人控制网隔离（true/false，默认取 JSON 的 ros.isolate）",
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value=str(cfg["domain_id"]),
                description="ROS 域（默认取 JSON 的 ros.domain_id；隔离时不能为 0）",
            ),
            # 注意：必须在 DeclareLaunchArgument 之后，IfCondition 里的 'isolate' 才存在
            *isolation_actions(cfg, isolate, LaunchConfiguration("ros_domain_id")),
            LogInfo(
                msg=[
                    "[a2w] LIO 链路: 桥(",
                    LaunchConfiguration("config"),
                    ") → /a2w/points + /a2w/imu → point_lio（",
                    "config/a2w.yaml）",
                ]
            ),
            Node(
                package="a2w_bridge",
                executable="a2w_bridge_node",
                name="a2w_bridge",
                output="screen",
                condition=IfCondition(LaunchConfiguration("bridge")),
                parameters=[
                    {
                        "config": LaunchConfiguration("config"),
                        "iface": LaunchConfiguration("iface"),
                    }
                ],
            ),
            _lio_launch(),
            rviz_node,
            # 关掉 RViz 窗口 = 整个 launch 退出
            RegisterEventHandler(
                OnProcessExit(
                    target_action=rviz_node,
                    on_exit=[EmitEvent(event=Shutdown())],
                )
            ),
        ]
    )
