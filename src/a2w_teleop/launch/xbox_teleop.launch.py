"""Xbox 手柄 → /cmd_vel 遥操作，全部用 ROS 2 自带包（joy + teleop_twist_joy）。

用法：
    ① 确认手柄被识别（一般插上就有 /dev/input/js0，也可 Bluetooth 连接）：
        ls /dev/input/js*

    ② A2W 工程里要和 bridge 互见，先隔离 ROS 环境（ROS_DOMAIN_ID=42，只走回环）：
        source src/a2w_bridge/scripts/a2w_env.sh

    ③ 启动本包：
        ros2 launch a2w_teleop xbox_teleop.launch.py

    ④ 另开终端（同样先 source a2w_env.sh）验证：
        ros2 topic echo /cmd_vel

Xbox 键位（实机测出：LB=buttons[6]，左摇杆 axes 0/1，右摇杆 axes 2/3）：
    左摇杆 上/下   = 前进/后退        （默认 0.7 m/s，axis 1）
    左摇杆 左/右   = 横移            （默认 0.7 m/s，axis 0）
    右摇杆 左/右   = 原地转向          （默认 0.4 rad/s，axis 2）
    按住 LB(左肩键) = 使能，不按不输出   （默认 enable_button=6，以实机 echo /joy 为准）
    按住 RB(右肩键) = 涡轮 ×1.5        （默认 enable_turbo_button=5）

所有数值都能用 launch 参数覆盖，例如：
    ros2 launch a2w_teleop xbox_teleop.launch.py \
        cmd_vel:=/nav/cmd_vel scale_linear:=0.3 scale_angular:=0.2 device:=/dev/input/js1

⚠️ 安全：先在开阔场地、小 scale 下试车；松开 LB 立即停（teleop 内部自带松手超时）。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # 预检：A2W 工程里 bridge 隔离在 ROS_DOMAIN_ID=42（只走回环）。
    # 未 source a2w_env.sh 时手柄节点和 bridge 互相看不见，这里只提醒、不拦。
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    domain_warning = (
        [
            LogInfo(
                msg="[a2w_teleop] 当前 ROS_DOMAIN_ID 未设置/为 0：如需与 a2w_bridge"
                " 互见，请先 `source src/a2w_bridge/scripts/a2w_env.sh` 再启动。"
            )
        ]
        if domain in ("", "0")
        else []
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cmd_vel",
                default_value="/cmd_vel",
                description="速度指令话题（teleop 输出 remap 到这里）",
            ),
            DeclareLaunchArgument(
                "device",
                default_value="/dev/input/js0",
                description="手柄设备节点",
            ),
            DeclareLaunchArgument(
                "deadzone",
                default_value="0.05",
                description="摇杆死区（joy_node 参数）",
            ),
            DeclareLaunchArgument(
                "scale_linear",
                default_value="0.7",
                description="最大线速度 m/s（左摇杆前后，LB 按住生效）",
            ),
            DeclareLaunchArgument(
                "scale_angular",
                default_value="0.4",
                description="最大角速度 rad/s（左摇杆左右，LB 按住生效）",
            ),
            DeclareLaunchArgument(
                "scale_turbo",
                default_value="1.5",
                description="涡轮倍率（RB 按住时线速度 ×此值）",
            ),
            DeclareLaunchArgument(
                "enable_button",
                default_value="6",
                description="使能键：按住才输出（实机测出 LB=buttons[6]）",
            ),
            DeclareLaunchArgument(
                "enable_turbo_button",
                default_value="5",
                description="涡轮键：Xbox 为 RB（右肩键）",
            ),
            *domain_warning,
            # 手柄驱动：/dev/input/jsX → /joy（sensor_msgs/Joy）
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[
                    {
                        "device": LaunchConfiguration("device"),
                        "deadzone": LaunchConfiguration("deadzone"),
                    }
                ],
            ),
            # 映射：/joy → geometry_msgs/Twist（默认 /cmd_vel）
            # 参数结构与官方 xbox.config.yaml 一致（axis_linear.x=1 等）
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",  # lyrical 里的可执行名（旧发行版叫 teleop_twist_joy_node）
                name="teleop_node",
                output="screen",
                parameters=[
                    {
                        "axis_linear": {"x": 1, "y": 0},  # 左摇杆：垂直=前后(x)，水平=横移(y)
                        "scale_linear": {"x": LaunchConfiguration("scale_linear"), "y": LaunchConfiguration("scale_linear")},
                        "scale_linear_turbo": {"x": LaunchConfiguration("scale_turbo"), "y": LaunchConfiguration("scale_turbo")},
                        "axis_angular": {"yaw": 2},  # 右摇杆水平 = 转向(yaw)
                        "scale_angular": {"yaw": LaunchConfiguration("scale_angular")},
                        "enable_button": LaunchConfiguration("enable_button"),
                        "enable_turbo_button": LaunchConfiguration("enable_turbo_button"),
                    }
                ],
                remappings=[("cmd_vel", LaunchConfiguration("cmd_vel"))],
            ),
        ]
    )