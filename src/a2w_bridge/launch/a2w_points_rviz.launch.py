"""A2W 点云 RViz 显示启动器（模仿 livox_ros_driver2/launch_ROS2/rviz_MID360_launch.py）。

    ros2 launch a2w_bridge a2w_points_rviz.launch.py                    # 桥 + RViz
    ros2 launch a2w_bridge a2w_points_rviz.launch.py bridge:=false      # 桥已在别的终端跑
    ros2 launch a2w_bridge a2w_points_rviz.launch.py rviz:=false        # 只起桥，不看图

数据流（全程只读）：

    机器人雷达 rt/unitree/slam_lidar/points(1|2) ─采集器(只 subscribe)─▶ /a2w/points ─▶ RViz

⚠️ 必须先 ``source src/a2w_bridge/scripts/a2w_env.sh``（或在同一终端里 launch）：
   桥默认把 ROS2 隔离在独立域 + FastDDS 只走回环，不然本终端的 RViz 看不见话题，
   而且没隔离的 DDS 发现包打进机器人控制网会让机器狗进软急停（阻尼）。
   本 launch 只会给**自己起的子进程**设隔离环境，管不到你当前这个终端。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 静态检查看不到这个导入（launch/ 目录不在包搜索路径），运行时由 ROS 环境提供：
# 编译后 install 会把 a2w_bridge 包加到 PYTHONPATH，launch 进程可以直接 import。
from a2w_bridge.launch_common import (  # pyright: ignore[reportMissingImports]
    config_defaults,
    isolation_actions,
    package_share,
)

################### user configure parameters for ros2 start ###################
cloud_topic = "a2w/points"      # 显示的话题（single 模式输出；multi 模式见 a2w_points.rviz 里的三路）
frame_id = "a2w/lidar"          # 必须与 JSON 的 pointcloud.frame_id 一致（= RViz 的 Fixed Frame）
start_bridge = True             # 由本 launch 起桥（提供点云）；已在别处跑桥就设 false
start_rviz = True               # 是否起 RViz2
rviz_name = "a2w_points.rviz"   # share/a2w_bridge/config/ 下的 RViz 配置
################### user configure parameters for ros2 end #####################

cur_path = os.path.split(os.path.realpath(__file__))[0] + "/"
cur_config_path = os.path.join(cur_path, "..", "config")


def _default_rviz_config() -> str:
    """RViz 配置路径：优先用包 share（安装后），退回源码树 config/。"""
    share = package_share("a2w_bridge")
    installed = share / "config" / rviz_name if share is not None else None
    if installed is not None and installed.is_file():
        return str(installed)
    return os.path.normpath(os.path.join(cur_config_path, rviz_name))


def generate_launch_description() -> LaunchDescription:
    cfg = config_defaults()
    isolate = LaunchConfiguration("isolate")

    rviz_config = LaunchConfiguration("rviz_config")
    common_params = [
        {
            "config": LaunchConfiguration("config"),
            "iface": LaunchConfiguration("iface"),
        }
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["--display-config", rviz_config],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=cfg["config_path"],
                description="a2w_bridge JSON 配置路径（点云源/frame_id/限幅都在这里）",
            ),
            DeclareLaunchArgument(
                "iface",
                default_value="",
                description="覆盖 JSON 里的网卡名（如 enx00e04c2c4260）",
            ),
            DeclareLaunchArgument(
                "bridge",
                default_value=str(start_bridge).lower(),
                description="是否启动桥接节点（点云来源）；已在别处跑桥就设 false",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value=str(start_rviz).lower(),
                description="是否启动 RViz2",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=_default_rviz_config(),
                description="RViz 配置（默认包内 config/a2w_points.rviz）",
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
                    "[a2w] 点云显示: ",
                    rviz_config,
                    " ← 订阅 /",
                    cloud_topic,
                    "，Fixed Frame=",
                    frame_id,
                    "（RViz 里换话题/坐标系改这份配置）",
                ]
            ),
            # 桥（= 采集器 + ROS2 发布；见 a2w_bridge.launch.py 的说明）
            Node(
                package="a2w_bridge",
                executable="a2w_bridge_node",
                name="a2w_bridge",
                output="screen",
                condition=IfCondition(LaunchConfiguration("bridge")),
                parameters=common_params,
            ),
            rviz_node,
            # 关掉 RViz 窗口 = 整个 launch 退出（参照文件里那段被注释掉的写法）
            RegisterEventHandler(
                OnProcessExit(
                    target_action=rviz_node,
                    on_exit=[EmitEvent(event=Shutdown())],
                )
            ),
        ]
    )
