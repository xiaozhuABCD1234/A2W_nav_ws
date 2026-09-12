"""Point-LIO 点云 -> octomap 三维占据栅格。

前置(另开终端,先跑起来):
    ros2 launch livox_ros_driver2 msg_MID360_launch.py
    ros2 launch point_lio mapping_mid360.launch.py rviz:=false   # 关掉 point_lio 自己的 rviz

本 launch 只负责 octomap_server + rviz 这一层,方便单独重启/调参。

    ros2 launch mid360_bringup octomap_mid360.launch.py
    ros2 launch mid360_bringup octomap_mid360.launch.py rviz:=true   # 单窗口:octomap + LIO 点云 + path
    ros2 launch mid360_bringup octomap_mid360.launch.py resolution:=0.2 max_range:=15.0  # 临时覆盖

参数来源(唯一真相源):
    config/octomap_mid360.yaml 是 resolution / max_range 的唯一真相源。
    只有命令行**显式**传参才覆盖 yaml;不传就用 yaml 里的值。
    (旧写法在 DeclareLaunchArgument 上给了 default_value='0.1'/'20.0',
     等于无条件把 yaml 顶掉 —— 改 yaml 不生效。详见 docs/PATCHES.md 的 A 项。)

日常一键启动(驱动 + point_lio + octomap + rviz):
    ros2 launch mid360_bringup mid360.launch.py

输出:
    /octomap_full, /octomap_binary   octomap_msgs/Octomap(3D 栅格)
    /octomap_point_cloud_centers     占据体素中心点云
    /occupied_cells_vis_array        MarkerArray(粒度随 octree 深度)
    /projected_map                   nav_msgs/OccupancyGrid(2D 投影,给 nav2/costmap)
服务:
    /octomap_server/octomap_full, /octomap_binary  (用 octomap_saver 存盘)
    /octomap_server/clear_bbox
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# 命令行可临时覆盖的两个参数:arg 名 -> (参数名, 类型)
OVERRIDABLE = {
    'resolution': ('resolution', float),
    'max_range': ('sensor_model.max_range', float),
}


def _octomap_node(context):
    """按「yaml 优先,显式传参才覆盖」组装 octomap_server。

    为什么不直接在 DeclareLaunchArgument 上给 default_value:
    节点 parameters 列表里排在后面的项覆盖前面的,而 LaunchConfiguration
    总是会被求值 —— default_value='0.1' 就等于**无条件**追加一个覆盖项,
    把 yaml 里的值顶掉。于是「改了 config/*.yaml 却不生效」,而且
    src/ 与 install/ 两份副本容易对不上,排查起来像玄学。
    现在 default_value=''(= 没传,LaunchConfiguration 求值为空串),
    只有显式传参才追加覆盖项,配置源头收敛到一处。
    """
    params = [PathJoinSubstitution([
        FindPackageShare('mid360_bringup'), 'config', 'octomap_mid360.yaml'
    ]).perform(context)]

    override = {}
    for arg_name, (param_name, cast) in OVERRIDABLE.items():
        raw = context.perform_substitution(LaunchConfiguration(arg_name)).strip()
        if not raw:
            continue
        value = cast(raw)          # 显式传参时自己转换类型,避免
        top, _, leaf = param_name.partition('.')   # declare_parameter 抛
        if leaf:                    # InvalidParameterTypeException
            override.setdefault(top, {})[leaf] = value
        else:
            override[top] = value
    if override:
        params.append(override)

    return [Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=params,
        # 关键:必须喂机体系点云,见 config/octomap_mid360.yaml 顶部的说明。
        remappings=[('cloud_in', '/cloud_registered_body')],
    )]


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='启动 rviz2(octomap_rviz_plugins 的 OccupancyGrid 显示, '
                    'subscribes /octomap_binary,按占据概率着色)')

    resolution_arg = DeclareLaunchArgument(
        'resolution', default_value='',
        description='体素边长 [m]。留空 = 用 config/octomap_mid360.yaml 里的值;'
                    '显式传参才覆盖。运行时改不了,须重启节点')

    max_range_arg = DeclareLaunchArgument(
        'max_range', default_value='',
        description='射线截断距离 [m]。留空 = 用 config/octomap_mid360.yaml 里的值;'
                    '显式传参才覆盖。越大 CPU/内存越高,也越容易在远处出重影')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('mid360_bringup'), 'rviz', 'octomap_mid360.rviz'
        ])],
        # 关键:octomap_rviz_plugins 2.1.0 的 .so 编译时没把 liboctomap 写进
        # DT_NEEDED,而 rviz2 自己又不加载 octomap → dlopen 插件时符号解析失败
        # (undefined symbol: _ZTIN7octomap13OcTreeStampedE),OccupancyGrid 显示
        # 加载报错(以前只能退化成 FlatColor 橙色点云就是这个原因)。
        # 实测:预加载 liboctomap.so.1.9.7 后插件全部符号可解析、dlopen 成功。
        # 只对 rviz2 注入,不影响同 launch 里的其他节点。
        # 注意必须用 additional_env 而不是 env:Jazzy 的 launch 里 env= 是
        # **替换整个环境**(launch/descriptions/executable.py:env 不为 None 时
        # 就不继承 context.environment),会把 LD_LIBRARY_PATH 冲掉 ——
        # rviz2 随即找不到 rviz_ogre_vendor 提供的 libOgreMain.so.1.12.10,
        # 报 "error while loading shared libraries" 并以 127 退出(2026-09-11 实测)。
        additional_env={'LD_PRELOAD': '/usr/lib/x86_64-linux-gnu/liboctomap.so.1.9.7'},
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        resolution_arg,
        max_range_arg,
        OpaqueFunction(function=_octomap_node),
        rviz_node,
    ])
