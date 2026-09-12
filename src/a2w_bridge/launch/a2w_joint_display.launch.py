"""在 RViz 里用 a2w_description 的 URDF 显示 A2W 的**真实关节角**（只读，不控制）。

用法::

    # 终端 A：先起桥（提供 /a2w/joint_states）
    ros2 launch a2w_bridge a2w_bridge.launch.py
    # 终端 B：再起显示
    source src/a2w_bridge/scripts/a2w_env.sh
    ros2 launch a2w_bridge a2w_joint_display.launch.py
    ros2 launch a2w_bridge a2w_joint_display.launch.py rviz:=false   # 只要 /tf，不开 RViz

数据流（全程只读，没有任何控制指令）::

    rt/lowstate ─采集器(只 subscribe)─▶ /a2w/joint_states ─joint_relay─▶ /joint_states
                                                                        └─▶ robot_state_publisher ─▶ /tf ─▶ RViz

关节名映射表：``dds_topics.A2W_URDF_JOINT_NAMES``
（FR_hip → right_front_joint1，FL_wheel → left_front_joint4，…）

参数：
    urdf          URDF 路径（默认 a2w_description/urdf/a2w_description.urdf）
    rviz          是否启动 RViz2（true/false，默认 true）
    rviz_config   RViz 配置（默认 a2w_description/urdf.rviz）
    relay         是否启动关节转发节点（默认取 JSON 的 display.enabled）
    input_topic   桥上 SDK 命名的关节话题（默认取 JSON 的 display.input_topic）
    output_topic  robot_state_publisher 订阅的话题（默认 joint_states）
    rate_hz       /joint_states 频率（默认取 JSON 的 display.rate_hz = 50）
    stale_sec     多久没新数据就停止发布（默认取 JSON 的 display.stale_sec）
    flip          需要反号的 SDK 关节名，逗号分隔（默认取 JSON 的 display.flip）
    frame_prefix  TF 前缀（默认空）
    isolate       ROS2 与机器人控制网隔离（默认取 JSON 的 ros.isolate）
    ros_domain_id ROS 域（默认取 JSON 的 ros.domain_id = 42）
    config        a2w_bridge JSON 配置路径

⚠️ 本终端自己的 ROS 工具（rviz2/ros2 CLI）也得在同一 ROS 域：先
   ``source src/a2w_bridge/scripts/a2w_env.sh``（launch 只给子进程设隔离环境）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 静态检查看不到这个导入（launch/ 目录不在包搜索路径），运行时由 ROS 环境提供：
# 编译后 install 会把 a2w_bridge 包加到 PYTHONPATH，launch 进程可以直接 import。
from a2w_bridge.launch_common import (  # pyright: ignore[reportMissingImports]
    config_defaults,
    default_rviz_config,
    default_urdf,
    isolation_actions,
)


def generate_launch_description() -> LaunchDescription:
    # 安装目录里的 JSON 作为默认值（桥启动器同样这么做）
    cfg = config_defaults()
    disp = cfg["display"]

    urdf_default = disp["urdf"] or str(default_urdf() or "")
    rviz_default = disp["rviz_config"] or str(default_rviz_config() or "")
    if not urdf_default:
        raise RuntimeError(
            "找不到 URDF：确认已编译 a2w_description，或用 urdf:=<绝对路径> 指定"
        )

    isolate = LaunchConfiguration("isolate")
    rviz_on = LaunchConfiguration("rviz")

    # URDF 文本（顺手修正历史 mesh 包名）交给 robot_state_publisher。
    # 必须包 ParameterValue(value_type=str)，否则 launch_ros 会把整段 XML 当 YAML 解析。
    robot_description = ParameterValue(
        Command(["python3 -m a2w_bridge.launch_common ", LaunchConfiguration("urdf")]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf", default_value=urdf_default, description="URDF 路径"
            ),
            DeclareLaunchArgument(
                "rviz", default_value="true", description="是否启动 RViz2"
            ),
            DeclareLaunchArgument(
                "rviz_config", default_value=rviz_default, description="RViz 配置文件"
            ),
            DeclareLaunchArgument(
                "relay",
                default_value=str(disp["enabled"]).lower(),
                description="是否启动关节转发节点（SDK 命名 → URDF 命名）",
            ),
            DeclareLaunchArgument(
                "input_topic",
                default_value=disp["input_topic"],
                description="桥发布的 SDK 命名关节话题",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value=disp["output_topic"],
                description="robot_state_publisher 订阅的关节话题",
            ),
            DeclareLaunchArgument(
                "rate_hz",
                default_value=f"{disp['rate_hz']:g}",
                description="/joint_states 发布频率（Hz）",
            ),
            DeclareLaunchArgument(
                "stale_sec",
                default_value=f"{disp['stale_sec']:g}",
                description="多久没新数据就停止发布（秒）",
            ),
            DeclareLaunchArgument(
                "flip",
                default_value=",".join(disp["flip"]),
                description='需要反号的 SDK 关节名，逗号分隔（例如 "FR_thigh,FL_thigh"）',
            ),
            DeclareLaunchArgument("frame_prefix", default_value="", description="TF 前缀"),
            DeclareLaunchArgument(
                "isolate",
                default_value=str(cfg["isolate"]).lower(),
                description="ROS2 与机器人控制网隔离（true/false，默认取 JSON）",
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value=str(cfg["domain_id"]),
                description="ROS 域（默认取 JSON；隔离时不能为 0）",
            ),
            DeclareLaunchArgument(
                "config",
                default_value=cfg["config_path"],
                description="a2w_bridge JSON 配置路径",
            ),
            # 注意：必须在 DeclareLaunchArgument 之后，IfCondition 里的 'isolate' 才存在
            *isolation_actions(cfg, isolate, LaunchConfiguration("ros_domain_id")),
            LogInfo(
                msg=[
                    "[a2w] 显示真实关节: ",
                    LaunchConfiguration("input_topic"),
                    " → ",
                    LaunchConfiguration("output_topic"),
                    " @ ",
                    LaunchConfiguration("rate_hz"),
                    " Hz（只读）。若一直没数据，先起桥: "
                    "ros2 launch a2w_bridge a2w_bridge.launch.py",
                ]
            ),
            # 只做运动学：把 /joint_states 变成 /tf（不读写任何机器人接口）
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "frame_prefix": ParameterValue(
                            LaunchConfiguration("frame_prefix"), value_type=str
                        ),
                    }
                ],
            ),
            # SDK 命名 → URDF 命名（只转发，不下发控制）
            Node(
                package="a2w_bridge",
                executable="joint_relay",
                name="a2w_joint_relay",
                output="screen",
                condition=IfCondition(LaunchConfiguration("relay")),
                parameters=[
                    {
                        "config": LaunchConfiguration("config"),
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "rate_hz": ParameterValue(
                            LaunchConfiguration("rate_hz"), value_type=float
                        ),
                        "stale_sec": ParameterValue(
                            LaunchConfiguration("stale_sec"), value_type=float
                        ),
                        "flip": ParameterValue(
                            LaunchConfiguration("flip"), value_type=str
                        ),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(rviz_on),
                arguments=["-d", LaunchConfiguration("rviz_config")],
            ),
        ]
    )
