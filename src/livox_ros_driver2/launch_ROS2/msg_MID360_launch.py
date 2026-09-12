import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import launch

################### user configure parameters for ros2 start ###################
# 以下变量为命令行参数的默认值，可通过命令行覆盖，例如：
#   ros2 launch livox_ros_driver2 msg_MID360_launch.py \
#       publish_freq:=20.0 frame_id:=livox_2 \
#       user_config_path:=/path/to/your_config.json
# LOCAL PATCH P3(见 docs/PATCHES.md):上游默认 1(CustomMsg)。
# P1 补的是 PointCloud2 通路,默认必须为 0,否则 point_lio 静默收不到点云。
# xfer_format 默认 0 —— point_lio 订阅的是 sensor_msgs/PointCloud2,
# 只有 xfer_format=0 才发这个类型；要发 CustomMsg 请显式传 xfer_format:=1。
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### user configure parameters for ros2 end #####################


def generate_launch_description():
    # 声明命令行参数（默认值取上面的用户配置）
    declared_args = [
        DeclareLaunchArgument(
            'xfer_format', default_value=str(xfer_format),
            description='0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format'),
        DeclareLaunchArgument(
            'multi_topic', default_value=str(multi_topic),
            description='0-All LiDARs share the same topic, 1-One LiDAR one topic'),
        DeclareLaunchArgument(
            'data_src', default_value=str(data_src),
            description='0-lidar, 1-hub, 2-lvx file'),
        DeclareLaunchArgument(
            'publish_freq', default_value=str(publish_freq),
            description='frequency of publish, 5.0, 10.0, 20.0, 50.0, etc.'),
        DeclareLaunchArgument(
            'output_type', default_value=str(output_type),
            description='0-Output to ROS topic, 1-Output to rosbag file'),
        DeclareLaunchArgument(
            'frame_id', default_value=frame_id,
            description='frame_id of the point cloud message'),
        DeclareLaunchArgument(
            'lvx_file_path', default_value=lvx_file_path,
            description='lvx file path (used when data_src:=2)'),
        DeclareLaunchArgument(
            'user_config_path', default_value=user_config_path,
            description='path of the lidar config json (e.g. MID360_config.json)'),
        DeclareLaunchArgument(
            'cmdline_bd_code', default_value=cmdline_bd_code,
            description='lidar broadcast code'),
    ]

    # 命令行参数 -> 节点参数（数字类参数做类型转换）
    livox_ros2_params = [
        {'xfer_format': PythonExpression(["int('", LaunchConfiguration('xfer_format'), "')"])},
        {'multi_topic': PythonExpression(["int('", LaunchConfiguration('multi_topic'), "')"])},
        {'data_src': PythonExpression(["int('", LaunchConfiguration('data_src'), "')"])},
        {'publish_freq': PythonExpression(["float('", LaunchConfiguration('publish_freq'), "')"])},
        {'output_data_type': PythonExpression(["int('", LaunchConfiguration('output_type'), "')"])},
        {'frame_id': LaunchConfiguration('frame_id')},
        {'lvx_file_path': LaunchConfiguration('lvx_file_path')},
        {'user_config_path': LaunchConfiguration('user_config_path')},
        {'cmdline_input_bd_code': LaunchConfiguration('cmdline_bd_code')}
    ]

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    return LaunchDescription([
        *declared_args,
        livox_driver,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])