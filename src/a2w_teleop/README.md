# a2w_teleop —— Xbox 手柄遥操作（ROS 2 自带包）

用 ROS 2 自带的 `joy` + `teleop_twist_joy` 把 Xbox 手柄映射成 `/cmd_vel`，
不需要写任何 C++/Python 节点。

## 快速开始

```bash
# 1. 确认手柄被系统识别
ls /dev/input/js*

# 2. 加载 A2W 隔离环境（与 a2w_bridge 同一 ROS 域；不在这个工程外可跳过）
source src/a2w_bridge/scripts/a2w_env.sh

# 3. 启动（joy_node + teleop_twist_joy_node）
ros2 launch a2w_teleop xbox_teleop.launch.py

# 4. 验证（另开终端，先 source a2w_env.sh）
ros2 topic echo /cmd_vel
```

## Xbox 键位

| 操作 | 效果 | 映射 |
| --- | --- | --- |
| 左摇杆 上/下 | 前进/后退（linear.x） | `axis_linear.x=1` |
| 左摇杆 左/右 | 横移（linear.y） | `axis_linear.y=0` |
| 右摇杆 左/右 | 原地转向（angular.z） | `axis_angular.yaw=2` |
| **按住 LB**（左肩键） | 使能，不按不输出 | `enable_button=6` |
| **按住 RB**（右肩键） | 涡轮 ×1.5 | `enable_turbo_button=5` |

> 安全：必须按住 LB 才有速度输出；松手后 teleop_twist_joy 自带超时（默认 0.5 s）归零。

## 谁消费 /cmd_vel

默认情况下 `/cmd_vel` **没有消费者**——手柄只能发、机器人不动。要让它真的驱动 A2W，
打开 `a2w_bridge` 的运动通道（`config/a2w_bridge.json` 里 `motion.enabled=true`），
由它把 Twist 转成 `sport_client.Move(vx, vy, vyaw)`；建议先用 `motion.dry_run=true`
验证链路（机器人不会动）。限幅/死区/看门狗/阻尼状态门都在桥侧，详见
[`a2w_bridge/README.md`](../a2w_bridge/README.md) 的「运动通道」一节。

## 常用参数

```bash
ros2 launch a2w_teleop xbox_teleop.launch.py \
    cmd_vel:=/nav/cmd_vel \      # 输出话题（默认 /cmd_vel）
    scale_linear:=0.3 \          # 最大线速度 m/s
    scale_angular:=0.2 \         # 最大角速度 rad/s
    scale_turbo:=1.0 \           # 关掉涡轮倍率
    device:=/dev/input/js1       # 多手柄时指定设备
```

## 常见问题

- **`permission denied: /dev/input/js0`**：用户不在 `input` 组。
  `sudo usermod -aG input $USER` 后重新登录。
- **手柄没反应，`/dev/input/js*` 不存在**：有线先换 USB 口；蓝牙手柄先配对
  （`bluetoothctl` / 系统设置里连上后一般自动出现 js0）。
- **按键和摇杆编号对不上**：跑 `ros2 run joy joy_node` + `ros2 topic echo /joy`，
  对照 axes[]/buttons[] 索引改 `enable_button` / `axis_linear` 等参数。
- **`/cmd_vel` 没数据**：确认按住 LB；确认 `source a2w_env.sh` 后与订阅方在同一域。

## 话题 / 节点

- `/joy`（sensor_msgs/Joy）—— `joy_node` 发布
- `/cmd_vel`（geometry_msgs/Twist）—— `teleop_twist_joy_node` 发布
