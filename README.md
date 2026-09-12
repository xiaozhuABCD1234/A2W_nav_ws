# A2W_nav_ws —— ROS2 工作空间

ROS2 (Lyrical) 工作空间,当前包含:

| 包 | 内容 | 状态 |
| --- | --- | --- |
| `a2w_bridge` | **A2W 机器人 → ROS2 桥接**:点云 / IMU / 关节状态(16 关节含轮足) / 电池 / SLAM 广播 / 栅格,全部标准 ROS2 消息,行为由 JSON 配置(网卡、点云源 fused/front/rear、IMU 源等) | ✅ 本机实测可用 |
| `a2w_description` | A2W URDF/网格与显示 launch(轮式 X2-0807) | 新增 |
| `mid360_bringup` | MID360 点云 + Point-LIO 周边集成(launch/config) | 原有 |
| `point_lio_ros2` / `livox_ros_driver2` | 上游包 | 原有 |

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
ip maddr show $IF | grep 239.255.0.1 # 期望无输出 = 隔离生效
```

详见 [`src/a2w_bridge/README.md`](src/a2w_bridge/README.md)。
