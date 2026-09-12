"""``sensor_msgs/msg/Imu`` 的 DDS IDL 绑定（Cyclone DDS Python 后端风格）。

**为什么手写**：A2W 的 `rt/unitree/slam_lidar/imu{1,2}` 话题上是标准
`sensor_msgs::msg::dds_::Imu_` 类型，但官方 `unitree_sdk2py` 的
`idl/sensor_msgs/msg/dds_/` 里只有 `PointCloud2_` / `PointField_`，没有 `Imu_`
（`unitree/idl/ros2/Imu_.hpp` 只存在于 C++ SDK）。

Cyclone DDS 的类型匹配基于 IDL 结构本身（成员名/类型/顺序 + `@final` +
`@autoid(sequential)`），因此按 ROS2 `sensor_msgs/msg/Imu.idl` 的等价写法手工声明
即可与机器人发布端匹配——已在本机 A2W 上实测收到 ~200 Hz 数据。

⚠️ 本模块**禁用** ``from __future__ import annotations``：PEP 563 会把字符串字面量注解
再包一层引号（变成 ``"'unitree_sdk2py...Header_'"``），cyclonedds 类型解析器把
“带引号的字符串”当模块名 import 会直接失败（`Type ... cannot be resolved`）。

本模块**只在采集器进程（Python 3.10 + cyclonedds 0.10.2）里导入**。
"""

# 其余 isort:skip / pyright 内联抑制说明：
# 本文件等价于 idlc 生成的 DDS 绑定代码，cyclonedds 的装饰器/类型 API 是运行时动态的
# （typenname 关键字、types.array[float64, 9] 下标、字符串前向引用），静态检查无法建模，
# 属预期，逐行加 pyright ignore 而不是整文件禁用，避免盖掉真实的拼写错误。

from dataclasses import dataclass

import cyclonedds.idl as idl  # pyright: ignore[reportMissingImports]
import cyclonedds.idl.annotations as annotate  # pyright: ignore[reportMissingImports]
import cyclonedds.idl.types as types  # pyright: ignore[reportMissingImports]


@dataclass
@annotate.final  # pyright: ignore[reportCallIssue, reportGeneralTypeIssues, reportUndefinedVariable] -- idlc 动态 API
@annotate.autoid("sequential")  # pyright: ignore[reportCallIssue, reportUndefinedVariable] -- idlc 动态 API
class Imu_(  # pyright: ignore[reportGeneralTypeIssues] -- __init_subclass__ 报错来自 idl.IdlStruct 动态基类
    idl.IdlStruct, typename="sensor_msgs.msg.dds_.Imu_"  # pyright: ignore[reportCallIssue, reportGeneralTypeIssues] -- typename 是 cyclonedds 约定
):
    """等价于 ``sensor_msgs/msg/Imu``（IDL 文件 Imu_.idl）。"""

    header: "unitree_sdk2py.idl.std_msgs.msg.dds_.Header_"  # pyright: ignore[reportUndefinedVariable] -- 运行时注入 SDK 路径
    orientation: "unitree_sdk2py.idl.geometry_msgs.msg.dds_.Quaternion_"  # pyright: ignore[reportUndefinedVariable] -- 同上
    orientation_covariance: types.array[types.float64, 9]  # pyright: ignore[reportGeneralTypeIssues] -- idlc 下标语法
    angular_velocity: "unitree_sdk2py.idl.geometry_msgs.msg.dds_.Vector3_"  # pyright: ignore[reportUndefinedVariable] -- 同上
    angular_velocity_covariance: types.array[types.float64, 9]  # pyright: ignore[reportGeneralTypeIssues] -- idlc 下标语法
    linear_acceleration: "unitree_sdk2py.idl.geometry_msgs.msg.dds_.Vector3_"  # pyright: ignore[reportUndefinedVariable] -- 同上
    linear_acceleration_covariance: types.array[types.float64, 9]  # pyright: ignore[reportGeneralTypeIssues] -- idlc 下标语法