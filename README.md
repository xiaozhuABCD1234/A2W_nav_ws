# 自研机器狗 ROS 2 导航工作空间

## 一、运行步骤

```bash
# 0) 连接机器人网卡（192.168.123.0/24），确认 iface 名（默认 enx00e04c2c4260，在 JSON 中修改）
# 1) 创建采集器 Python 3.10 环境（首次）
bash src/a2w_bridge/scripts/setup_collector_venv.sh

# 2) 编译（需包含 point_lio；PCD 输出位于源码树，见 .gitignore）
colcon build --symlink-install
source install/setup.bash

# 2.5) 隔离 ROS 2 与机器人控制网。未隔离时 ros2 CLI 的 DDS 发现报文会发往
#      192.168.123.0/24，可能触发机器人进入阻尼/软急停状态
source src/a2w_bridge/scripts/a2w_env.sh

# 3) 启动桥（隔离已由 launch 默认设置）
ros2 launch a2w_bridge a2w_bridge.launch.py

# 4) 验证
ros2 topic hz /a2w/points /a2w/imu
ros2 topic hz /a2w/joint_states           # 16 关节（需机器人底层服务运行中）
ros2 topic echo /a2w/sport_state --once   # 运控状态机（error_code 1001 = 阻尼/软急停）
ros2 topic echo /a2w/status --once        # 运控状态、模式、关节新鲜度
ip maddr show lo | grep 239.255.0.1       # ROS 2 仅在回环组播（机器人网卡上的条目属采集器，正常）
```

### 1.1 建图（Nav2 地图栈的前置）

```bash
# 手动驱动：另开终端运行 ros2 launch a2w_teleop xbox_teleop.launch.py
ros2 launch a2w_bridge a2w_lio.launch.py show_rviz:=false pcd_save:=true
#    Ctrl+C 正常退出时写入 ./PCD/scans.pcd（非正常退出不写入）

ros2 run a2w_nav2 a2w_pcd_to_map PCD/scans.pcd -o maps/a2w_map --dry-run   # 先检查高度带
ros2 run a2w_nav2 a2w_pcd_to_map PCD/scans.pcd -o maps/a2w_map
```

### 1.2 导航

```bash
# Nav2 地图栈：RViz 中设初始位姿，再下发目标
ros2 launch a2w_nav2 a2w_nav2.launch.py map:=$PWD/maps/a2w_map.yaml

# 无图栈：Leg-KILO + SCAN-Planner（不需要地图）
ros2 launch a2w_nav2 a2w_scan_nav.launch.py
#    ★ 等 LIO 稳定输出位姿后再起规划器：规划器把第一帧机体位姿当作目标点高度基准

# 全局重定位：BBS 全局搜索 + NDT_OMP 打分（耗时数秒至十余秒，手动触发）
ros2 launch a2w_nav2 a2w_relocalize.launch.py map:=$PWD/maps/a2w_map.yaml
ros2 service call /a2w_relocalize/run std_srvs/srv/Trigger {}

# 仿真：Gazebo Harmonic（无需机器人）
ros2 launch a2w_gz_sim sim.launch.py gui:=false
```

> ⚠️ 上述链路与手柄 `a2w_teleop` **不可同时运行**：都会发布 `/cmd_vel`。
> 若需在不驱动机器人的前提下验证定位与代价地图，应将桥配置中的 `motion.enabled` 设为 `false`；
> 首次启用运动通道时先设 `dry_run: true` 验证链路。

---

## 二、架构与数据流

### 2.1 桥接：进程划分

```text
机器人(192.168.123.0/24, DDS 域 0)   点云 2.4 MB/帧 · IMU 196 Hz · 关节 16 路
   │
┌──▼──────────────────────────────────────────────────┐
│ 采集器 collector.py    Python 3.10 venv             │
│   cyclonedds 0.10.2 + 机器人 SDK 的 Python 绑定      │
│   点云订阅 BEST_EFFORT + KEEP_LAST(1)                │
│   运动指令的唯一出口                                 │
└──┬──────────────────────────────────────────────────┘
   │  本机 TCP 帧（127.0.0.1:42610；cmd 帧经同一连接反向传输）
┌──▼──────────────────────────────────────────────────┐
│ a2w_bridge_node.py     rclpy（系统 Python 3.14）     │
│   采集器异常退出后自动重启（退避 1/2/5/10 s）         │
│   发布标准消息与静态 TF；运动通道限幅/看门狗/状态门   │
└─────────────────────────────────────────────────────┘
```

进程拆分的直接原因是 Python SDK 依赖的 `cyclonedds==0.10.2`（PyPI 仅提供 cp37~cp310 wheel）
与本机 `rclpy` 运行所在的 Python 3.14 无法共存于同一解释器；该结构同时使“机器人网卡上的 DDS”
与“ROS 2 域内的 DDS”在物理上分离。

**发布的话题**：数据原样透传，统一使用采集端墙钟时间戳（机器人自带时间戳仅记录日志）。

| 数据 | ROS 2 话题 | 类型 |
| --- | --- | --- |
| 点云（融合/前/后，可配置） | `a2w/points` | `sensor_msgs/PointCloud2` |
| 雷达 IMU / 本体 IMU（可配置，支持自动降级） | `a2w/imu` | `sensor_msgs/Imu` |
| 关节状态（16 关节，含轮足） | `a2w/joint_states` | `sensor_msgs/JointState` |
| 运控状态机（只读） | `a2w/sport_state` | `std_msgs/String`(JSON) |
| 电池（默认关闭） | `a2w/battery` | `sensor_msgs/BatteryState` |
| SLAM 广播 | `a2w/slam_info`、`a2w/slam_key_info` | `std_msgs/String`(JSON) |
| 全局占据栅格（默认关闭） | `a2w/map/grid` | `nav_msgs/OccupancyGrid` |
| 桥运行状态汇总 | `a2w/status` | `std_msgs/String`(JSON) |
| 机体中心里程计（由 TF 转换） | `a2w/body_odom` | `nav_msgs/Odometry` |
| 结构外参 | `/tf_static` | `base_link → a2w/lidar`、`a2w/lidar_rear`、`a2w/imu` |

桥接中有两处必须按实测覆盖机器人侧字段，不能直接透传：

- 机器人将 5 个话题的 `frame_id` 统一写为 `hesai_lidar`（该字符串由机器人侧固定给出），但后雷达 IMU 实际位于**后雷达坐标系**
  （两雷达绕 y 轴互转 180°）。直接透传会使下游的加速度与角速度方向解算错误，故按数据源覆盖
  `frame_id`；
- 点云为 **128×900 固定网格**，无回波位置填 `(0,0,0)`，且 `is_dense` 字段标注为 `true`，
  与实测稀疏性不符（115200 点中 68% 为零点）。桥侧默认过滤，输出约 36,500 有效点/帧。

### 2.2 两种「里程计」语义（规划器必须区分）

LIO 输出的是**雷达系**位姿，而规划/控制需要**机体中心**位姿。雷达安装在 `base_link` 前方
**0.338 m**（见静态外参），若混用同一条边，整条规划轨迹会整体前偏 0.34 m，表现为贴障碍行驶或在
门前顶住。

```text
camera_init ──▶ body            /Odomtry（legkilo）或 /aft_mapped_to_init（point_lio）
                                body ≡ a2w/lidar = 雷达系 → 用作 sensor_pose（射线清除原点）
camera_init ──▶ base_footprint  a2w/body_odom（a2w_body_odom 由 TF 转出）
                                机体中心（贴地）→ 用作 body_pose（足迹位置、轨迹起点）
```

`a2w_body_odom` 不重新计算几何：从 TF 取 `camera_init → base_footprint` 后转成 Odometry，
`twist` 由相邻两帧有限差分得到（规划器把 `odom.twist` 当作轨迹初始速度，不能恒为 0）；
若查到的是静态 TF（`stamp = 0`）则改用当前时钟。

### 2.3 TF 树

Point-LIO（或 Leg-KILO）单独运行时产生两棵互不相连的 TF 树（LIO 的 `camera_init → body`
与机器人的 `base_link → 传感器`），Nav2/AMCL 无法在其上工作。本仓库补充以下变换：

```text
map ──(AMCL)──▶ camera_init ──(point_lio / legkilo)──▶ body
                    │
                    └──(a2w_odom_tf)──▶ base_footprint ──(a2w_base_footprint)──▶ base_link ──▶ a2w/lidar…
```

| 变换 | 发布者 | 频率 |
| --- | --- | --- |
| `map → camera_init` | `nav2_amcl`（`amcl:=false` 时由静态 TF 发布者发布单位阵） | 定位更新时 |
| `camera_init → body` | `point_lio` 或 `legkilo` | ≈10 Hz |
| `camera_init → base_footprint` | `a2w_odom_tf` | 同上游位姿频率 |
| `base_footprint → base_link` | `a2w_base_footprint` | 10 Hz（z = 实测离地高） |
| `base_link → a2w/*` | 桥（标定 JSON） | 静态 |

约定 **`camera_init` 即本链路的 odom 坐标系**（Nav2 的 `odom_frame_id` 直接使用该帧），
不应再发布名为 `odom` 的帧；底盘坐标系使用 `base_footprint`（贴地，z 随姿态变化）。
外参变更只需修改 JSON 中的 `tf.transforms`。两条 LIO（Point-LIO 与 Leg-KILO）**不能同时运行**：
二者发布同名话题与同名 TF 边。

### 2.4 全链路

```text
/a2w/points ─┬─▶ point_lio ──▶ /cloud_registered + TF(camera_init→body)
/a2w/imu    ─┤   legkilo  ──▶ /cloud_registered + /Odomtry + TF(camera_init→body)
             │        │
             │        └─▶ a2w_odom_tf ─▶ TF(camera_init→base_footprint)
             │                              └─▶ a2w_body_odom ─▶ /a2w/body_odom
             ▼
  Nav2 地图栈：a2w_scan ─▶ /scan ─▶ AMCL + 全局/局部代价地图 ─▶ Nav2(DWB) ─┐
             └─▶ /a2w/points_nav ─▶ 局部 voxel 层                        │
  无图栈：/cloud_registered + /Odomtry + /a2w/body_odom ─▶ SCAN-Planner ──┼─▶ /cmd_vel ─▶ 桥运动通道 ─▶ sport Move
  全局重定位：/a2w/points_nav ─▶ BBS + NDT_OMP ─▶ a2w_relocalize ─▶ /initialpose ─▶ AMCL（修正定位）
```

---

## 三、数据与开发工具目录

| 目录 / 文件 | 内容 |
| --- | --- |
| `PCD/` | 建图原始点云（Point-LIO 输出，约 167 MB） |
| `bags/` | 实机 rosbag2 录制（mcap，约 3.8 GB，两组） |
| `maps/` | `a2w_map`（实机，由 PCD 投影，约 7.5 MB）、`a2w_sim_map`（仿真会话生成） |
| `datasets/` | 预留（当前为空） |
| `rules/tree-sitter-queries/` | pi-lens 自定义静态检查规则（tree-sitter query） |
| `.pi-lens.json`、`pyrightconfig.json` | 静态检查与类型检查配置 |
| `build/`、`install/`、`log/` | colcon 构建产物（已由 `.gitignore` 忽略） |

---

## 四、验证状态与后续工作

**已验证**：

- 实机：桥接全部话题、DDS 域隔离、运动通道五项安全线、TF 树补全、关节显示、
  `a2w_scan` 输出 `/scan`、`a2w_pcd_to_map` 投影与地平面自动识别、Nav2 全套装配（1.5.1）、
  一键 bringup；
- 开发机：SCAN-Planner 6 个包干净编译并完成"点云/里程计 → 规划 → 控制器 → `/cmd_vel`"全链
  （FSM 循环重规划 7 次，输出 `vx = 0.5 m/s` = 配置上限）；Leg-KILO 编译通过并跑通端到端数据链路
  （合成数据 116 帧无失败，≈10 Hz）；`a2w_body_odom` 位姿与偏航正确；
- 仿真：Gazebo Harmonic 下全链建图与自主导航。

**未验证**（以下各项不应被视为可用）：

1. **实机端到端导航未验收**：`/scan` 与 PCD 地图形状的一致性、AMCL 收敛性、定位与最终误差均需
   完整走行一次后确认；
2. **行走状态的 LIO 精度未验证**：漂移数据为**静止**实测；行走与转向过程中的点云拖影、
   `time_lag_imu_to_lidar`（当前值 −0.042）需要重新标定；两条 LIO 的轨迹质量需要用同一段
   实机数据对比后再决定取舍；
3. **SCAN-Planner 的实机表现未验证**：真实点云下的避障与轨迹质量、双圆柱足迹包络
   （半径 0.40 / 偏移 0.10 为按实测足迹推导的保守值）是否过保守或过松、闭环控制器跟踪参数、
   速度上限（现 0.5 m/s）均需上机确认；
4. **Leg-KILO 的腿足运动学未适配**：上游模型为 12 关节四足 + 足端接触力，本机为 16 关节轮足且
   桥仅提供关节力矩（无 `footForce`），因此默认 `only_imu_use: true`（纯 IMU + LiDAR 紧耦合）。
   即 Leg-KILO 相对 Point-LIO 的差异目前主要来自体素面元地图，而非腿足约束；
5. **全局重定位未验收**：BBS 搜索参数、打分门控与 `/initialpose` 的实际收敛效果未在实机确认；
6. **`/odom` 话题仍未发布**：Nav2 侧 DWB 的 `odom_topic` 与速度平滑器的闭环反馈指向它。
   补上之前，DWB 依赖 TF 估算速度（速度反馈偏乐观），速度平滑器保持 `OPEN_LOOP`；
   `/a2w/body_odom` 是**机体中心**里程计，可作为该话题的实现基础；
7. **低矮障碍**（低于扫描带 0.30 m）仅进入局部 voxel 层，而 voxel 层覆盖范围限于机器人所在区域，
   全局路径上仍存在不可见区域；
8. **后向盲区防护**：建议接入 `nav2_collision_monitor`（已安装，未启动），或将桥的软急停门
   （`error_code 1001`）联动至导航暂停；目前仅具备桥侧防护。

各包的算法推导、逐参数依据与排错手册见包内 README：
[`a2w_bridge`](src/a2w_bridge/README.md) · [`a2w_nav2`](src/a2w_nav2/README.md) ·
[`a2w_teleop`](src/a2w_teleop/README.md)（`a2w_gz_sim`、`legkilo`、`scan_planner` 暂无包内 README）。
