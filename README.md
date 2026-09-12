# A2W_nav_ws —— ROS2 工作空间

ROS2 (Lyrical) 工作空间,当前包含:

| 包 | 内容 | 状态 |
| --- | --- | --- |
| `a2w_bridge` | **A2W 机器人 → ROS2 桥接**:点云 / IMU / 关节状态(16 关节含轮足) / 电池 / SLAM 广播 / 栅格,全部标准 ROS2 消息,行为由 JSON 配置(网卡、点云源 fused/front/rear、IMU 源等);**含一条命令把点云喂给 Point-LIO 的 `a2w_lio.launch.py`;**动态 `base_footprint` TF 与 2D 足迹(`a2w_base_footprint`,Nav2 定位/代价地图用) | ✅ 本机实测可用 |
| `a2w_description` | A2W URDF/网格与显示 launch(轮式 X2-0807);要看**实机关节角**,用 `a2w_bridge` 的 `a2w_joint_display.launch.py` | 新增 |
| `point_lio_ros2` | 上游 Point-LIO(新增 `config/a2w.yaml` + `launch/mapping_a2w.launch.py` 适配 A2W 前雷达;为在本机 ROS Lyrical 能编译,CMakeLists 加了 `LOCAL PATCH P10`) | 原有 + 适配 |

## A2W 桥接快速上手

```bash
# 0) 连接机器人网卡(192.168.123.0/24),确认 iface 名(默认 enx00e04c2c4260,在 JSON 里改)
# 1) 采集器 Python 3.10 环境(首次)
bash src/a2w_bridge/scripts/setup_collector_venv.sh

# 2) 编译
colcon build --packages-select a2w_bridge
source install/setup.bash

# 2.5) 隔离 ROS2 与机器人控制网(重要: 不隔离时 ros2 CLI 的 DDS 发现包会打到
#     机器人 192.168.123.0/24 控制网,可能触发机器狗软急停/阻尼)
source src/a2w_bridge/scripts/a2w_env.sh

# 3) 启动(隔离已由 launch 默认设好)
ros2 launch a2w_bridge a2w_bridge.launch.py

# 4) 验证
ros2 topic hz /a2w/points /a2w/imu
ros2 topic hz /a2w/joint_states      # 16 关节(需机器人底层服务在跑)
ros2 topic echo /a2w/sport_state --once  # 运控状态机(error_code 1001=阻尼/软急停)
ros2 topic echo /a2w/status --once   # 机器人状态: 运控/模式/关节新鲜度
ip maddr show lo | grep 239.255.0.1 && echo 'ROS2 只在回环组播（机器人网卡上那条属采集器，正常）'

# 5) (可选)RViz 里按实机关节角看 URDF(只读,另开一个终端;同样先 source a2w_env.sh)；
#    默认同时启动 base_footprint 节点：动态离地高 TF（base_footprint → base_link）
#    + Nav2 足迹话题 a2w/footprint（footprint:=false 可关）
ros2 launch a2w_bridge a2w_joint_display.launch.py
```

## 点云 + Point-LIO(一条命令)

A2W 前雷达(JT128)的点云字段布局与 Hesai ROS 驱动一致(`x,y,z,intensity`+`ring(u2)`+`timestamp(f8)`),
桥按原生类型透传后 Point-LIO 走 `lidar_type: 4` 分支即可拿到**逐点时间**做运动补偿:

```bash
# 桥(用 LIO 那份继承配置: 打开 ring/timestamp、IMU 钉在前雷达) + Point-LIO + RViz
ros2 launch a2w_bridge a2w_lio.launch.py

# 无窗口(建图存 PCD: Ctrl+C 退出时写 ./PCD/scans.pcd)
ros2 launch a2w_bridge a2w_lio.launch.py show_rviz:=false pcd_save:=true
```

- Point-LIO 侧新增:`point_lio_ros2/config/a2w.yaml`、`point_lio_ros2/launch/mapping_a2w.launch.py`
- 桥侧新增:`a2w_bridge/config/a2w_bridge_lio.json`(用 `extends` 继承主配置)、`a2w_bridge/launch/a2w_lio.launch.py`
- 实测(静止 30 s):`/cloud_registered` 10 Hz、位置漂移 2.3 mm、CPU 23%;时序补偿用桥状态行里的
  `点云−IMU滞后差` 直接拄(见 [`src/a2w_bridge/README.md`](src/a2w_bridge/README.md) 的「喂给 Point-LIO」一节)

详见 [`src/a2w_bridge/README.md`](src/a2w_bridge/README.md) 与 [`src/point_lio_ros2/config/a2w.yaml`](src/point_lio_ros2/config/a2w.yaml)。
