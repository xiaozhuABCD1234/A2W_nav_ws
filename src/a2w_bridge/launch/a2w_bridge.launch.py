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

    ⚠️ 自己开终端看 /a2w/* 话题（`ros2 topic echo`、rviz2、point_lio…）也要先
       ``source src/a2w_bridge/scripts/a2w_env.sh``，否则不在同一 ROS 域、互相看不见。
"""

import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _ros_defaults(cfg_path: str) -> tuple[str, str]:
    """从 JSON 读 ros.isolate / ros.domain_id（读不到回安全默认）。"""
    isolate, domain = True, 42
    try:
        ros = (json.loads(Path(cfg_path).read_text(encoding="utf-8")) or {}).get("ros", {}) or {}
        isolate = bool(ros.get("isolate", True))
        domain = int(ros.get("domain_id", 42))
    except (OSError, ValueError, TypeError):
        pass  # 配置读不到/格式不对 → 用上面的安全默认值，节点启动时还会再校验一次
    if isolate and domain == 0:
        domain = 42
    return str(isolate).lower(), str(domain)


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("a2w_bridge")
    cfg_default = PathJoinSubstitution([pkg_share, "config", "a2w_bridge.json"])
    profile = PathJoinSubstitution([pkg_share, "config", "fastdds_iso.xml"])

    try:  # 环境已 source 时读包内 JSON 作为默认值
        from ament_index_python.packages import get_package_share_directory

        cfg_path = str(Path(get_package_share_directory("a2w_bridge")) / "config" / "a2w_bridge.json")
        iso_default, domain_default = _ros_defaults(cfg_path)
    except Exception:  # noqa: BLE001 —— 未 source ROS 环境时退回默认
        iso_default, domain_default = "true", "42"

    isolate = LaunchConfiguration("isolate")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=cfg_default,
                description="a2w_bridge JSON 配置文件路径",
            ),
            DeclareLaunchArgument(
                "iface",
                default_value="",
                description="覆盖 JSON 里的网卡名（如 enx00e04c2c4260）",
            ),
            DeclareLaunchArgument(
                "isolate",
                default_value=iso_default,
                description="ROS2 与机器人控制网隔离（true/false，默认取 JSON 的 ros.isolate）",
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value=domain_default,
                description="ROS 域（默认取 JSON 的 ros.domain_id；隔离时不能为 0）",
            ),
            SetEnvironmentVariable(
                "ROS_DOMAIN_ID",
                LaunchConfiguration("ros_domain_id"),
                condition=IfCondition(isolate),
            ),
            SetEnvironmentVariable(
                "FASTDDS_DEFAULT_PROFILES_FILE",
                profile,
                condition=IfCondition(isolate),
            ),
            LogInfo(
                msg=[
                    "[a2w_bridge] ROS 已隔离: domain=",
                    LaunchConfiguration("ros_domain_id"),
                    "，FastDDS 只走回环。看 /a2w/* 话题前先 source "
                    "src/a2w_bridge/scripts/a2w_env.sh",
                ],
                condition=IfCondition(isolate),
            ),
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
