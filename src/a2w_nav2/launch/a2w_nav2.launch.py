"""A2W 2D 导航一键 bringup（map_server + AMCL + 规划/控制/恢复/BT + 速度平滑）。

为什么自己组 launch 而不是用 nav2_bringup
========================================

本机是 **ROS 2 Lyrical**：apt 里 nav2 组件齐（1.5.1），但**没有 nav2_bringup 元包**
（`ros-lyrical-nav2-bringup` 不存在），所以这里按上游 `navigation_launch.py` 的写法
把各节点自己拼起来 —— 反正 A2W 也要定制：坐标系（camera_init 当 odom、base_footprint
当底盘）、足迹（0.68×0.76 m 矩形）、前雷达只有 180°（禁倒车/旋转限速）。

话题链路（速度侧，注意平滑器在最末端）::

    controller_server ──cmd_vel_nav──▶ velocity_smoother ──cmd_vel──▶ a2w_bridge 运动通道
                                                                      └─ sport Move(vx,vy,vyaw)

    ⚠️ 手柄（a2w_teleop）和导航**不能同时开**：两边都发 /cmd_vel。
       想让机器人绝不动（只验证定位/代价地图）：把桥的运动通道关掉
       （config/a2w_bridge.json 的 motion.enabled=false）——那才是真正的"干跑"开关。

数据侧::

    /a2w/points ─▶ a2w_scan ─▶ /scan ─▶ AMCL + 局部/全局代价地图
                          └─▶ /a2w/points_nav ─▶ 局部 voxel 层（低于扫描带的障碍）

一次起全链（桥 + LIO + odom_tf + URDF/足迹 + 扫描 + Nav2 + RViz）::

    ros2 launch a2w_nav2 a2w_nav2.launch.py map:=$PWD/maps/a2w_map.yaml

桥/LIO/显示已经在别的终端跑着::

    ros2 launch a2w_nav2 a2w_nav2.launch.py map:=... lio:=false display:=false scan:=false

不起 RViz（无窗口机器 / ssh）::

    ros2 launch a2w_nav2 a2w_nav2.launch.py map:=... rviz:=false
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 生命周期节点：由一个 lifecycle_manager 统一 configure/activate
# （map_server/amcl 不设开关 —— 本 launch 的用途就是"地图 + AMCL"，少一个这套就废了）
LIFECYCLE_NODES = [
    "map_server",
    "amcl",
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "velocity_smoother",
]


def _check_map(context, *args, **kwargs):
    """map 必须给：不给的话 map_server 会起来但加载失败，lifecycle 卡住，报错很难看懂。"""
    raw = context.perform_substitution(LaunchConfiguration("map")).strip()
    if not raw:
        raise RuntimeError(
            "没有给地图：请用 map:=<你的 .yaml> 指定 2D 地图。\n"
            "  还没建图的话先跑：\n"
            "    ① 驱动狗走一圈（手柄 a2w_teleop 或直接发 /cmd_vel），存 Point-LIO 的 PCD：\n"
            "       ros2 launch a2w_bridge a2w_lio.launch.py show_rviz:=false pcd_save:=true\n"
            "       （Ctrl+C 退出时写 ./PCD/scans.pcd）\n"
            "    ② 离线投影成 2D 栅格图：\n"
            "       ros2 run a2w_nav2 a2w_pcd_to_map PCD/scans.pcd -o maps/a2w_map\n"
            "  详见 src/a2w_nav2/README.md"
        )
    return []


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("a2w_nav2")
    bridge_pkg = FindPackageShare("a2w_bridge")

    params_file = LaunchConfiguration("params_file")
    scan_params = LaunchConfiguration("scan_params")
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    default_params = PathJoinSubstitution([pkg, "config", "a2w_nav2_params.yaml"])
    default_scan_params = PathJoinSubstitution([pkg, "config", "a2w_scan.yaml"])
    # A2W 专用行为树：把恢复动作里的盲退收窄（见文件头注释）
    bt_xml = PathJoinSubstitution([pkg, "behavior_trees", "a2w_navigate_to_pose.xml"])
    bt_xml_through = PathJoinSubstitution([
        FindPackageShare("nav2_bt_navigator"), "behavior_trees",
        "navigate_through_poses_w_replanning_and_recovery.xml",
    ])

    args = [
        DeclareLaunchArgument(
            "map", default_value="",
            description="2D 地图 yaml（由 a2w_pcd_to_map 从 Point-LIO 的 PCD 生成），必填"),
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="Nav2 参数文件（默认 = 本包 A2W 定制那份）"),
        DeclareLaunchArgument("scan_params", default_value=default_scan_params,
                              description="a2w_scan（前雷达→扫描）参数文件"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="仿真时间（实机 false）"),
        DeclareLaunchArgument("autostart", default_value="true",
                              description="lifecycle_manager 是否自动 configure/activate"),
        DeclareLaunchArgument("rviz", default_value="true", description="是否起 RViz（本包的导航配置）"),
        DeclareLaunchArgument("scan", default_value="true",
                              description="是否起 a2w_scan（桥的点云 → 扫描/过滤云）"),
        DeclareLaunchArgument("lio", default_value="true",
                              description="是否顺带起 a2w_lio（桥 + Point-LIO + odom_tf）"),
        DeclareLaunchArgument("display", default_value="true",
                              description="是否顺带起 joint display（URDF/关节 + base_footprint）"),
    ]

    # ── 顺带起上游链路（一条命令跑通全链）──────────────────────────────
    # joint display 必须 bridge:=false —— 桥的采集器独占 TCP 42610，只能有一个
    #
    # ⚠️ 每个 include 都套 GroupAction(scoped=True)：include 的 launch_arguments 是
    #    **平铺的 SetLaunchConfiguration，不隔离作用域**（launch/actions/
    #    include_launch_description.py 的 execute()），所以子 launch 拿到的
    #    rviz:=false 会直接写回本文件的 `rviz`。而 rviz_node 的 IfCondition 在
    #    这些 include 之后才求值 —— 结果就是：全链都起来了、RViz 一个进程都不起，
    #    而且日志里连报错都没有（现象极难猜）。scoped 之后同名开关只在本组内生效。
    lio_launch = GroupAction(
        [IncludeLaunchDescription(
            PythonLaunchDescriptionSource([bridge_pkg, "/launch/a2w_lio.launch.py"]),
            launch_arguments={
                "bridge": "true",
                "lio": "true",
                "odom_tf": "true",
                "show_rviz": "false",
            }.items(),
        )],
        scoped=True,
        condition=IfCondition(LaunchConfiguration("lio")),
    )
    display_launch = GroupAction(
        [IncludeLaunchDescription(
            PythonLaunchDescriptionSource([bridge_pkg, "/launch/a2w_joint_display.launch.py"]),
            launch_arguments={
                "bridge": "false",     # 桥已由上面那条起过（scoped 也保住了用户传的 bridge:=）
                "rviz": "false",       # RViz 由本 launch 用导航配置起
                "footprint": "true",   # 动态 base_footprint（贴地）—— 代价地图的底盘帧
            }.items(),
        )],
        scoped=True,
        condition=IfCondition(LaunchConfiguration("display")),
    )

    # ── 传感器：点云 → 扫描（干净带）+ 过滤云（宽带 − 自车体）──────────────
    scan_node = Node(
        package="a2w_nav2", executable="a2w_scan", name="a2w_scan", output="screen",
        parameters=[scan_params, {"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("scan")),
    )

    # ── 地图与定位 ──────────────────────────────────────────────────────
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server", output="screen",
        parameters=[params_file, {"yaml_filename": map_yaml, "use_sim_time": use_sim_time}],
    )
    amcl = Node(
        package="nav2_amcl", executable="amcl", name="amcl", output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # ── 控制 / 规划 / 恢复 / 行为树 ─────────────────────────────────────
    controller_server = Node(
        package="nav2_controller", executable="controller_server", name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        # 控制器出 cmd_vel_nav，交给速度平滑器再出 cmd_vel（见模块 docstring 的链路）
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )
    planner_server = Node(
        package="nav2_planner", executable="planner_server", name="planner_server", output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )
    behavior_server = Node(
        package="nav2_behaviors", executable="behavior_server", name="behavior_server", output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )
    bt_navigator = Node(
        package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="screen",
        parameters=[params_file, {
            "use_sim_time": use_sim_time,
            "default_nav_to_pose_bt_xml": bt_xml,
            "default_nav_through_poses_bt_xml": bt_xml_through,
        }],
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager",
        output="screen",
        parameters=[{"autostart": autostart, "use_sim_time": use_sim_time,
                     "node_names": LIFECYCLE_NODES}],
    )

    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", PathJoinSubstitution([pkg, "config", "a2w_nav2.rviz"])],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        *args,
        OpaqueFunction(function=_check_map),   # map 必填（不给就带着用法报错退出）
        lio_launch,
        display_launch,
        scan_node,
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager,
        rviz_node,
    ])
