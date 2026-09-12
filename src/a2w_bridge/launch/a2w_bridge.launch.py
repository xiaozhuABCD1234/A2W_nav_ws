"""A2W 桥接启动器：``ros2 launch a2w_bridge a2w_bridge.launch.py``。

参数：
    config        JSON 配置文件路径（默认安装目录里的 a2w_bridge.json）
    iface         网卡名覆盖（默认取 JSON 里的 iface）
    isolate       ROS2(FastDDS) 与机器人控制网隔离（默认取 JSON 的 ros.isolate）
    ros_domain_id ROS 域（默认取 JSON 的 ros.domain_id，缺省 42；隔离时不能是 0）

为什么默认隔离（实测 + 官方文档）：
    A2W 的 192.168.123.0/24（交换机1）是官方写明的“DDS控制信号”局域网。主机 ROS2 默认
    FastDDS 域 0，会把发现组播 239.255.0.1（含类型对象）打到那张网卡；机器人侧是
    CycloneDDS 0.10.2，跨实现类型对象会让它出问题，运控随即落到“阻尼＝软急停”
    （官方 error_code 1001）。所以这里给节点进程设 ROS_DOMAIN_ID + FastDDS 只走回环。

    隔离实现与其他启动器共用 ``a2w_bridge.launch_common``。

    ⚠️ 自己开终端看 /a2w/* 话题（`ros2 topic echo`、rviz2、point_lio…）也要先
       ``source src/a2w_bridge/scripts/a2w_env.sh``，否则不在同一 ROS 域、互相看不见。

想在 RViz 里看真实关节角，另开一个终端：
    ``ros2 launch a2w_bridge a2w_joint_display.launch.py``
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 静态检查看不到这个导入（launch/ 目录不在包搜索路径），运行时由 ROS 环境提供：
# 编译后 install 会把 a2w_bridge 包加到 PYTHONPATH，launch 进程可以直接 import。
from a2w_bridge.launch_common import (  # pyright: ignore[reportMissingImports]
    config_defaults,
    isolation_actions,
)


def generate_launch_description() -> LaunchDescription:
    cfg = config_defaults()
    isolate = LaunchConfiguration("isolate")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=cfg["config_path"],
                description="a2w_bridge JSON 配置文件路径",
            ),
            DeclareLaunchArgument(
                "iface",
                default_value="",
                description="覆盖 JSON 里的网卡名（如 enx00e04c2c4260）",
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
            Node(
                package="a2w_bridge",
                executable="a2w_bridge_node",
                name="a2w_bridge",
                output="screen",
                parameters=[
                    {
                        "config": LaunchConfiguration("config"),
                        "iface": LaunchConfiguration("iface"),
                    }
                ],
            ),
        ]
    )
