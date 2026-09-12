from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _laser_mapping_node(context):
    """组装 laserMapping 节点(参数约定 A:yaml 优先,只有显式传参才覆盖)。

    LOCAL PATCH P7(见 docs/PATCHES.md):新增 launch 参数 `pcd_save`,
    用来在**不改 yaml、不重新 colcon build** 的前提下开关 PCD 落盘。

    为什么非得用 OpaqueFunction:节点 parameters 列表里排在后面的项覆盖
    前面的,所以「无条件把 pcd_save.pcd_save_en 塞进字典」= 把 yaml 顶掉。
    default_value='' 表示「没传」,求值为空串 → 不追加覆盖项。
    """
    laser_mapping_params = [
        PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'config', 'mid360.yaml'
        ]).perform(context),
        # LOCAL PATCH P5(见 docs/PATCHES.md):
        # 下面这个字典排在 params_file 之后,会**覆盖** config/mid360.yaml 的同名参数。
        # 原先这里还硬编码着 'filter_size_surf': 0.5 与 'filter_size_map': 0.5,
        # 结果在 yaml 里改这两项**静默失效**(实测踩过)。现已删掉,让 yaml 成为
        # 唯一真相源(约定 A)。
        # ⚠️ 以下各项仍然盖住 yaml,要改它们必须改这里:
        #    use_imu_as_input / prop_at_freq_of_imu / check_satu / init_map_size /
        #    point_filter_num / space_down_sample / cube_side_length /
        #    runtime_pos_log_enable
        {
            'use_imu_as_input': False,  # Change to True to use IMU as input of Point-LIO
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 3,  # Options: 1, 3
            'space_down_sample': True,
            # filter_size_surf / filter_size_map:见 config/mid360.yaml(唯一真相源)
            'cube_side_length': 1000.0,  # Option: 1000
            'runtime_pos_log_enable': False,  # Option: True
        }
    ]

    # LOCAL PATCH P7:只有显式传了 pcd_save 才覆盖 yaml。
    raw = context.perform_substitution(LaunchConfiguration('pcd_save')).strip().lower()
    if raw:
        if raw not in ('true', 'false', '1', '0', 'yes', 'no', 'on', 'off'):
            # 启动前就报错,而不是把字符串塞进 bool 参数触发
            # InvalidParameterTypeException(那种报错看不出是哪来的)
            raise RuntimeError(f"pcd_save 只接受 true/false,收到: {raw!r}")
        laser_mapping_params.append(
            {'pcd_save': {'pcd_save_en': raw in ('true', '1', 'yes', 'on')}})

    return [Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
        # prefix='gdb -ex run --args'
    )]


def generate_launch_description():
    # Declare the RViz argument
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Flag to launch RViz.')

    # LOCAL PATCH P7:建图点云落盘开关(先验地图用,见 docs/LOCALIZATION.md)
    pcd_save_arg = DeclareLaunchArgument(
        'pcd_save', default_value='',
        description='是否把建图点云落盘。留空 = 用 config/mid360.yaml 的 pcd_save.pcd_save_en;'
                    '显式传 true/false 才覆盖。注意:只在节点【正常退出】(Ctrl+C)时写一次,'
                    '写到 ./PCD/scans.pcd(相对运行本 launch 的目录,P8)')

    # Conditional RViz node launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg', 'loam_livox.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )

    # Assemble the launch description
    ld = LaunchDescription([
        rviz_arg,
        pcd_save_arg,
        OpaqueFunction(function=_laser_mapping_node),
        GroupAction(
            actions=[rviz_node],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])

    return ld
