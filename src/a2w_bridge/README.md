# a2w_bridge —— A2W 机器人数据 → ROS2 桥接

把 Unitree A2W 机器人（PC2 上的 `slam_operate` 服务）发布的 DDS 数据转成**标准 ROS2 消息**：

| 数据 | DDS 话题（机器人侧） | ROS2 话题（默认） | 消息类型 |
| --- | --- | --- | --- |
| 点云（融合/前/后） | `rt/unitree/slam_lidar/points{,1,2}` | `a2w/points` | `sensor_msgs/PointCloud2` |
| 雷达 IMU | `rt/unitree/slam_lidar/imu{1,2}` | `a2w/imu` | `sensor_msgs/Imu` |
| 本体 IMU（低电平） | `rt/lowstate` 内 `imu_state` | `a2w/imu`（降级源） | `sensor_msgs/Imu` |
| **关节状态（16 关节）** | `rt/lowstate` 内 `motor_state` | **`a2w/joint_states`** | `sensor_msgs/JointState` |
| **运控状态（只读）** | **`rt/sportmodestate`** | **`a2w/sport_state`** | `std_msgs/String`（JSON，限频10Hz） |
| 电池（默认关） | `rt/bms_state` | `a2w/battery` | `sensor_msgs/BatteryState` |
| SLAM 广播 | `rt/slam_info` / `rt/slam_key_info` | `a2w/slam_info` / `a2w/slam_key_info` | `std_msgs/String`（JSON 透传） |
| 全局占据栅格 | `rt/unitree/slam_relocation/global_map` | `a2w/map/grid`（默认关） | `nav_msgs/OccupancyGrid` |
| 机器人状态 | —（桥把 lowstate/bms 汇总） | `a2w/status` | `std_msgs/String`（JSON，默认 5 s） |
| 静态 TF | — | `/tf_static` | 结构外参：base_link → 前雷达/后雷达/IMU（见下节） |

所有话题时间戳用**采集端墙钟**统一打点；机器人自带时间戳只进日志不做时钟源。

## 结构外参与坐标系（TF）

标定给出的结构外参（JT128 雷达头/尾各一，2026-09），桥按此发布静态 TF：

```text
base_link ─T=[0.33767, 0, 0.08134] R=[[0,0,1],[1,0,0],[0,1,0]] (rpy=[90°,0,90°])─▶ a2w/lidar       前雷达（= hesai_lidar）
a2w/lidar ─T=[0, 0.00599, -0.61764] R=diag(-1,1,-1) (rpy=[180°,0,180°])──────────▶ a2w/lidar_rear  后雷达
base_link ─（单位阵）─▶ a2w/imu        机身上的低电平 IMU
base_link ─（单位阵）─▶ a2w/base       旧名别名（向后兼容）
```

两雷达相距 0.61764 m，后雷达在 `base_link` 后方 0.280 m（= 0.33767 − 0.61764）。
`rpy` 是 ZYX 顺序、单位弧度；**重新标定只改 JSON 的 `tf.transforms`，不用改代码**。

**各话题的坐标系归属**（机器人侧定义 + 桥发布的 frame_id）：

| 机器人话题 | 数据原点坐标系（机器人侧定义） | 桥发布的 `frame_id` |
| --- | --- | --- |
| `…slam_lidar/points`（融合） | **前雷达** | `a2w/lidar` |
| `…slam_lidar/points1`（前雷达） | **前雷达** | `a2w/lidar` |
| `…slam_lidar/points2`（后雷达） | **前雷达**（机器人已变换过去） | `a2w/lidar` |
| `…slam_lidar/imu1`（前雷达 IMU） | **前雷达**（JT128 内部 IMU 与雷达同姿） | `a2w/lidar` |
| `…slam_lidar/imu2`（后雷达 IMU） | **后雷达** | `a2w/lidar_rear` ← 见下面的坑 |
| `rt/lowstate` 里的 `imu_state`（机身 IMU） | 机身 | `a2w/imu` |

⚠️ **坑：机器人把上述５个话题的 `frame_id` 全写成 `hesai_lidar`（本机实测）**，但 `imu2` 实际是
后雷达坐标系（两雷达绕 y 互转 180°）。照搬机器人的 frame_id 会让后 IMU 的朝向差 180°,
下游（Point-LIO / robot_localization）会把加速度/角速度解错。所以桥**按源覆盖** frame_id，不照搬。

按源覆盖的位置（都在 JSON，改标定不改代码）：

- `imu.frames`：`lidar_front → a2w/lidar`、`lidar_rear → a2w/lidar_rear`、`lowstate → a2w/imu`
  （`imu.frame_id` 是没列在 `frames` 里的源的兜底值）
- `pointcloud.frame_id`：`a2w/lidar`（三路点云都是前雷达坐标系，共用一个）

改完配置**重启节点**生效；若 `install/...share/a2w_bridge/config/*.json` 是**普通文件**
（之前用过不带 `--symlink-install` 的 `colcon build`），必须先
`colcon build --packages-select a2w_bridge` 才会刷新（是软链时则即时生效）。

## 量机器人高度（按 TF）

```bash
ros2 run a2w_bridge a2w_base_height                      # base_link 离地高度（每秒一行）
ros2 run a2w_bridge a2w_base_height --frame a2w/lidar     # 雷达离地高度（任意坐标系都行）
ros2 run a2w_bridge a2w_base_height --once                # 只打一行就退出（脚本取值用）
ros2 run a2w_bridge a2w_base_height --ground-frame a2w/ground
      # 另发动态 TF base_link → a2w/ground；RViz 把 Fixed Frame 改成 a2w/ground，
      # 机器人就“站”在网格地面上（可直观检查高度对不对）
```

原理：URDF 四个轮子 mesh 的最低点 = 轮心 − 半径（`meshes/*_Link4.STL` 量得半径
**0.09486 m**），四轮最低点在 `base_link` 系里的 z 就是地面（取最低的轮 → 平地/单轮悬空都能用），
反号就是 `base_link` 离地高度；量别的坐标系再做一次 TF 换算。

- 需要 TF 在位：`ros2 launch a2w_bridge a2w_joint_display.launch.py`（URDF + 关节）+ 桥（提供关节角、外参）
- 高度**随姿势变**：比如 `运控=待机` 腿收着停放时 `base_link` 只有 ~0.10 m，站起来会变高
- 输出里四轮接地点差 > 20 mm 会提醒“不在平地或某轮没落地”
- 实测（2026-09，待机停放姿势）：`base_link` 0.104 m、`a2w/lidar` 0.185 m；
  与点云地面法（云里最低点约在雷达下方 0.15 m）相差 ~3 cm —— 差在
  URDF 是**刚性轮**、实际橡胶胎受压会扁 ~2–3 cm

## ⚠️ ROS 域隔离（必读：不隔离会触发机器狗软急停）

A2W 的 `192.168.123.0/24`（交换机1）是官方文档写明的**“DDS控制信号”局域网**。本机 ROS2
默认用 FastDDS 域 0，会把发现组播 `239.255.0.1`（含类型对象）发到那张网卡；机器人侧是
CycloneDDS 0.10.2，跨实现类型对象会让它出问题，运控随即落到**阻尼 ＝ 软急停**
（官方 `error_code=1001`；`Damp()` 备注“该模式具有最高的优先级，用于突发情况下的急停”）。
实测：不隔离时跑一条 `ros2 topic echo`，机器人网卡上就会加入 `239.255.0.1`。

**本包默认已隔离**，两处生效：

```bash
# ① launch 自动给节点进程设好（取 JSON 的 ros.isolate / ros.domain_id）
ros2 launch a2w_bridge a2w_bridge.launch.py

# ② 你自己的终端要先 source 一次，否则不在同一 ROS 域，看不见 /a2w/* 话题
source src/a2w_bridge/scripts/a2w_env.sh      # ROS_DOMAIN_ID=42 + FastDDS 只走回环
ros2 topic echo /a2w/joint_states --once
```

验证隔离是否生效（实测方法见下）：

```bash
# ① ROS2/FastDDS 侧只应在**回环**加入该组播（users 数 ≈ 你的 ROS2 进程数）
ip maddr show lo | grep 239.255.0.1

# ② 机器人网卡上**会**有 239.255.0.1 —— 那是采集器的 CycloneDDS（机器人自己的 DDS 域 0），
#    它就是靠这个收机器人数据的，属于设计内。想确认那几条确实只是采集器：
#    （杀掉采集器后 0.5 s 内看，随后监督器会把它重启）
pkill -9 -f 'collector.py --config'; sleep 0.5
ip maddr show enx00e04c2c4260 | grep 239.255.0.1 || echo '已消失 = 确实只是采集器，ROS2 没漏'
```

⚠️ 别把②当成“隔离失效”：判据是**ROS2 侧有没有把发现包发到那张网卡**。本机实测：
五个 ROS2 进程（桥/LIO/RViz 生态）只在 `lo` 上加入该组播；机器人网卡上那两条来自采集器，
杀掉采集器就立刻消失。

> **为什么可以彻底隔离**：机器人数据不走 ROS2 DDS——采集器(CycloneDDS，只读订阅) → TCP →
> ROS2 节点，ROS2 侧纯属本机消费，根本不需要碰机器人网络。
> 也正因如此，`ROS_LOCALHOST_ONLY=1` 在本机 FastDDS 3.6 **实测不生效**（仍会绑机器人网卡），
> 要用本包 `config/fastdds_iso.xml` 这份 profile 才行。

## 看实时点云（RViz：`a2w_points_rviz.launch.py`）

模仿 `livox_ros_driver2/launch_ROS2/rviz_MID360_launch.py` 的写法：一条命令 = 桥 + RViz。

```bash
source src/a2w_bridge/scripts/a2w_env.sh     # 必须：本终端的 RViz 也要在同一 ROS 域
ros2 launch a2w_bridge a2w_points_rviz.launch.py

# 桥已经在别的终端跑 → 不要重复起桥
ros2 launch a2w_bridge a2w_points_rviz.launch.py bridge:=false
# 只起桥不看图 / 换 RViz 配置
ros2 launch a2w_bridge a2w_points_rviz.launch.py rviz:=false
ros2 launch a2w_bridge a2w_points_rviz.launch.py rviz_config:=/path/my.rviz
```

- RViz 配置：`config/a2w_points.rviz` —— `Fixed Frame = base_link`（= TF 的根，也是 URDF 的根，
  所以点云能和机器人模型叠着看；改成 `a2w/lidar` 就是雷达视角）；**默认只显示一项**：
  话题 `/a2w/points`（single 模式下这一话题里就是 JSON 里 `pointcloud.source` 指定的那一路，
  当前配置是 `front` 前雷达；也可设 `fused` 融合、`auto` 断流轮换）；
  `multi` 模式的三路（`points_fused|front|rear`）在配置里**注释保留**，需要时打开注释
  或 RViz 里 Add 三个 PointCloud2
- 改话题名/坐标系/点大小：直接编辑这份 rviz 配置（RViz 里改完也可另存）
- 关掉 RViz 窗口 = 整个 launch 退出（参照文件里那段被注释掉的 `OnProcessExit` 写法）
- ⚠️ 实测提醒：单帧 115200 点里 **68%** 是雷达无回波的 `(0,0,0)`（机器人把 128×900 网格整帧发过来），
  不丢就会在传感器中心形成一个很密的亮点团。本包默认 `pointcloud.filter_zero: true` 已经把它们丢掉了；
  还嫌近处脏就加 `pointcloud.min_range`（去自车体/轮子回波），见「常见问题」第一条

## 喂给 Point-LIO（`a2w_lio.launch.py` + `config/a2w_bridge_lio.json`）

一条命令 = 桥（LIO 配置）+ Point-LIO（+ 可选 RViz）：

```bash
ros2 launch a2w_bridge a2w_lio.launch.py                  # 桥 + LIO + RViz
ros2 launch a2w_bridge a2w_lio.launch.py show_rviz:=false  # 无窗口（看日志 / 存 PCD 时用）
ros2 launch a2w_bridge a2w_lio.launch.py lio:=false        # 只起桥（LIO 自己另开终端起）
ros2 launch a2w_bridge a2w_lio.launch.py pcd_save:=true    # Ctrl+C 退出时把建图点云写 ./PCD/scans.pcd
```

数据流（全程只读，不下发任何控制指令）：

```text
rt/unitree/slam_lidar/points1 ─采集器─▶ /a2w/points ─┐
rt/unitree/slam_lidar/imu1    ─采集器─▶ /a2w/imu    ─┴─▶ point_lio（config/a2w.yaml）
                                                          └─▶ /cloud_registered(_body)、/path、TF camera_init→body
```

不想一键，也可以两边分开起（两个终端都要先 `source src/a2w_bridge/scripts/a2w_env.sh`）：

```bash
# 终端 1：桥（关键是带上 LIO 那份配置）
ros2 launch a2w_bridge a2w_bridge.launch.py config:=$(ros2 pkg prefix --share a2w_bridge)/config/a2w_bridge_lio.json
# 终端 2：Point-LIO
ros2 launch point_lio mapping_a2w.launch.py
```

### 为什么单开一份配置：`config/a2w_bridge_lio.json`

它用本包新支持的 `extends` 继承主配置（只写差异项，**标定/网卡/话题仍然只有一处真相源**），
一共只改了三件事：

| 键 | 值 | 为什么 |
| --- | --- | --- |
| `pointcloud.fields` | `["x","y","z","intensity","ring","timestamp"]` | Point-LIO 的 HESAI 分支要 `ring(u2) + timestamp(f8)` 才能拿逐点时间做运动补偿 |
| `imu.source` | `lidar_front` | LIO 的外参是**单位阵**（点云与 IMU 同在前雷达坐标系）；`auto` 会降级到后雷达/本体 IMU，坐标系数说变就变，外参随即失效 |

（第三件事不是配置项：默认的 `filter_zero: true` 对 LIO 同样关键 —— 零点会被当成"传感器原点处的障碍物"。）

### 桥为 LIO 做的三件事

1. **字段按原生类型透传**（`fields` 里写 `ring`/`timestamp` 时生效）：
   机器人原始点云就是 Hesai 驱动那套布局 —— `x,y,z,intensity`(f4) + `ring`(**u2**) + `timestamp`(**f8**，
   绝对 Unix 秒)，`point_step=26`。桥以前把一切都当 f4 发：f8 压成 f4 会掉到 0.1 s 级精度，
   u2 的 ring 直接变成垃圾浮点。现在采集器按源类型打包、节点按类型发 `PointField`，
   与 `point_lio_ros2/src/preprocess.h` 的 `hesai_ros::Point` **逐字节一致**。
2. **点云时间戳打"帧首"**：下游把 `header.stamp` 当帧首用（Point-LIO: `lidar_end_time = 帧首 + 帧内跨度`），
   若打"到达时刻"，LIO 的内建时钟就比物理时间晚整整一帧、姿态戳会跑到"未来"。
   桥用点云自带的逐点绝对时间反推：`帧首 = 到达时刻 − (末点时刻 − 机器人 header.stamp)`；
   没有 `timestamp` 字段时保持旧行为（到达时刻）。
3. **不再白丢帧**：解码线程原先是 30 ms 轮询唤醒，实测 DDS 进 9.4 Hz、只有 8.2 Hz 发得出去（~13% 帧白丢）。
   改成事件唤醒后，日志里 `云输出Hz[front]` 与 `cloud:front` 基本相等（都是 9.7~10.2 Hz）。

### 时序诊断：`time_lag_imu_to_lidar` 不用猜

周期状态行（每 5 s）里会直接量并给值：

```text
状态: ... | 点云源=front IMU源=lidar_front 点云−IMU滞后差=+42ms（LIO: time_lag=-0.042） 云输出Hz[front=9.8] | {'cloud:front': 9.9, ...}
```

- 原理：分别统计 `点云戳 − 机器人帧首戳` 与 `IMU 主机戳 − 机器人 IMU 戳`，两者都含同一个
  机器人时钟偏差（实测与主机**差 263 s**），相减就抵消了，剩下的正是**点云比 IMU 多出来的那部分链路延迟**。
- 用法：把括号里的值抄进 `point_lio_ros2/config/a2w.yaml` 的 `common.time_lag_imu_to_lidar`
  （当前 **-0.042**，实测抖动 32~49 ms）。换网卡/换交换机/机器人侧改发布频率后重抄一次即可。
- 它只影响**动态**精度（走起来才看得出），静止时与它无关。

### 本机实测（2026-09-13，A2W 实机，机器人静止）

| 项 | 实测 |
| --- | --- |
| 机器人原始点云 | 115200 点/帧（128 线 × 900），10 Hz，`point_step=26`；**68.3% 是 `(0,0,0)`** |
| 桥输出 `/a2w/points` | 36534 点/帧，`point_step=26`，`ring` u2 0~127，`timestamp` f8 帧内跨度 99.8 ms |
| 桥输出 `/a2w/imu` | 前雷达 IMU ~200 Hz，加速度模长 9.85（**m/s²**，重力在 +Y） |
| Point-LIO 输出 | `/cloud_registered` 10.1 Hz、`/path` 10.5 Hz、`/aft_mapped_to_init` 10.3 Hz |
| 精度 | 静止 30 s 位置漂移 **2.3 mm**（逐帧位移中位 4 mm），姿态戳落后墙钟 63 ms（= 传输+发布延迟，物理正确） |
| 资源 | `pointlio_mapping` CPU **23%**、RSS 186 MB |

> 还**没有**在机器人行走/转向时实测过（那需要有人遥控它），走起来才是真正的验收；
> 如果发现转向时点云有拖影，优先看 `point_lio_ros2/config/a2w.yaml` 里的 `time_lag_imu_to_lidar`
> 与 `preprocess.blind`（后者负责丢掉腿/轮子回波）。

### 在本机编译 point_lio 的坑（LOCAL PATCH P10）

本工作区此前**从未成功编译过 C++ 包**，因为本机 ROS（Lyrical，`ament_cmake 2.8.8`）已经
**删除 `ament_target_dependencies` 宏**，而 CMake 4 也删了 `FindPythonLibs`（CMP0148）——
上游 `point_lio` 两处都还在用。已在 `point_lio_ros2/CMakeLists.txt` 就地补上（语义不变，见文件里的
`LOCAL PATCH P10` 注释），`colcon build --symlink-install --packages-select point_lio` 即可通过。

## 关节（`a2w/joint_states`）

A2W 是**轮足机器狗**：4 条腿 × (髋/大腿/小腿) + 4 个轮足电机 = **16 个自由度**。
官方《A2 SDK 开发指南》的命名是 `Leg0=FR / 1=FL / 2=RR / 3=RL`，`Joint0=Hip / 1=Thigh / 2=Calf / 3=Wheel`。

⚠️ **但 `rt/lowstate`（`unitree_hg/LowState`，35 槽）里的实际排列和那张命名表不是一回事**
（本机在 A2W 上实测出来的）：

```text
idx 0..11 = 12 个腿关节，步长 3：0,1,2=FR 髋/大腿/小腿  3,4,5=FL  6,7,8=RR  9,10,11=RL
idx 12..15 = 4 个轮足（GO2W/B2W 也是“腿关节在前、轮在后”，而不是每腿紧跟一个轮）
```

实测依据（静止、电机通电）：① 12 个腿关节的 q 全部落在官方限位内且左右髋镜像对称
（`FR_hip=-0.5725 / FL_hip=+0.5778`，`RR_hip=-0.5784 / RL_hip=+0.5954`）；② 若按“每腿 4 个”
读，FL/RR 组的 q 会超出髋/大腿/小腿限位 → 排除；③ `idx12~15` 的 τ 只有 ±0.018 N·m
（腿部 ±0.4）、温度更低 → 轮足。

- `position` ← `q`（弧度），`velocity` ← `dq`（rad/s），`effort` ← `tau_est`（N·m）
- 随 lowstate 节奏发布（本机实测约 1 kHz），**与 IMU 选源解耦**：即使 IMU 降级到
  lidar_front，关节照常出口
- 默认出口 16 个关节；`joints.names` / `joints.indexes` 可改（例如只出 12 个腿关节）。
  轮足内部顺序（12~15 各自对应哪条腿）官方文档没写，默认按腿序 FR/FL/RR/RL；
  单独转一个轮、看哪个索引的 q 在变即可确认，然后改 `joints.indexes`，不用改代码
- 想喂给 `a2w_description` 的 URDF（`left_front_joint1..4` 那套名字）：本包已内置映射
  （`dds_topics.A2W_URDF_JOINT_NAMES`），**符号不需要换算** —— URDF/CAD 的限位与官方 SDK
  完全一致（小腿 `−2.77~−0.54 rad` = −158.7°~−30.9°、大腿 `−2.34~3.15 rad` = −134°~180°），
  实机站立/卧倒的小腿角都是负值、正好落在 URDF 的负区间内；2026-09 已在 RViz 里对实机
  逐条腿核对通过。要用就在 RViz 里看：见下一节。

## 在 RViz 里看真实关节（只读：`a2w_joint_display.launch.py`）

把 `a2w_description` 的 URDF 按**实机关节角**动起来（纯只读，不发任何控制指令）：

```bash
# 终端 A：桥（提供 a2w/joint_states）
source src/a2w_bridge/scripts/a2w_env.sh
ros2 launch a2w_bridge a2w_bridge.launch.py

# 终端 B：显示（RViz）
source src/a2w_bridge/scripts/a2w_env.sh
ros2 launch a2w_bridge a2w_joint_display.launch.py
```

数据流（全部只读）：

```text
rt/lowstate ─采集器(只 subscribe)─▶ /a2w/joint_states ─joint_relay─▶ /joint_states
                                                                    └─▶ robot_state_publisher ─▶ /tf ─▶ RViz
```

`joint_relay`（`a2w_bridge/joint_relay.py`）只做三件事：

1. **改名**：SDK 名 → URDF 名（`dds_topics.A2W_URDF_JOINT_NAMES`）；
2. **限频**：按 `display.rate_hz`（默认 50 Hz）发 `/joint_states`，桥上原始是 ~1.1 kHz；
3. **断流保护**：超过 `display.stale_sec` 没有新数据就停发并打 WARN，
   免得 RViz 里定格一个假姿态。

映射表（已在实机上逐条腿核对）：

| SDK 关节名 | URDF 关节名 | 对应腿 |
| --- | --- | --- |
| `FR_hip/thigh/calf/wheel` | `right_front_joint1/2/3/4` | 右前 |
| `FL_hip/thigh/calf/wheel` | `left_front_joint1/2/3/4` | 左前 |
| `RR_hip/thigh/calf/wheel` | `right_hind_joint1/2/3/4` | 右后 |
| `RL_hip/thigh/calf/wheel` | `left_hind_joint1/2/3/4` | 左后 |

（URDF 前腿在 +x、左腿在 +y，与 ROS REP-103 / 宇树约定一致。）

- **万一某条腿转向反了**（RViz 里膝盖朝反方向弯腰）：把该 SDK 关节名加进配置的
  `display.flip`，或临时用 launch 参数试：
  `ros2 launch a2w_bridge a2w_joint_display.launch.py flip:="FR_thigh,FL_thigh"`
  —— 不用改 URDF、不用改代码。
- 其他 launch 参数：`rviz:=false`（只发 `/tf`，不开 RViz）、`rate_hz:=100`、
  `urdf:=<别的 URDF>`、`input_topic:=` / `output_topic:=` / `frame_prefix:=`。
- 一致性核对方法（当时用它验证的）：RViz 的 TF 与**独立 FK 计算**逐位一致 ——
  站立足实测 `base_link → left_front_Link4` = `[0.251, 0.154, -0.404]`
  （用 URDF 链手算 0.2508/0.1536/−0.4045）。
- 只想要数据不要 RViz：`ros2 run a2w_bridge joint_relay`（可带 `-p rate_hz:=30.0`）。

## 机器人状态（`a2w/status`）

**不再是桥自身健康报告**；内容是机器人本体数据（std_msgs/String，JSON，默认 5 s）：

```json
{
  "ts": 1789220268.06,        // 采集端墙钟
  "mode_machine": 2,          // 低电平控制模式（原始值透传）
  "mode_pr": 0,               // 模式优先级
  "tick": 287393,             // lowstate 帧号
  "joints": { "count": 16, "fresh_sec": 0.03 },   // 距上一帧 lowstate（-1=从未收到）
  "sport": { "error_code": 1001, "name": "阻尼(软急停)", "fresh_sec": 0.01 },
  "battery": null                                   // 默认关，开的话是 {voltage, current, soc, soh}
}
```

其中 `sport.error_code` 是官方《运控服务接口 V2.0》里那套运动状态机：
`0` 待机、`100` 灵动、**`1001` 阻尼（软急停）**、`1002` 站立锁定、`1013` 平衡站立、
`1015` 常规行走…（完整表见 `a2w_bridge/dds_topics.py:SPORT_STATE_NAMES`）。
状态**跳变**时采集器会额外发一条日志（1001 为 WARN），不用盯话题也能从 ROS 日志看到软急停发生。

桥自身健康（当前点云源/IMU 源、各话题实测频率、队列深度）不再上话题，
只打印在 ROS 日志里（采集器 stderr → 节点日志）。

## 架构

```
┌─────────────────────────────── 机器(192.168.123.0/24) ───────────────────────────────┐
│ PC2(192.168.123.162) unitree_slam.service：点云/IMU/里程计/栅格（标准 rosidl DDS 类型）│
└──────────────┬────────────────────────────────────────────────────────────────────────┘
               │ DDS 域 0（千兆网卡，点云 ~2.4 MB/帧）
┌──────────────▼────────────────────────────────────────────────────┐
│ 采集器 collector.py（Python 3.10 + cyclonedds 0.10.2 + unitree_sdk2py）│
│   · 网卡/来源/传感器开关全部来自 JSON 配置                          │
│   · 点云 BEST_EFFORT + KEEP_LAST(1)，同一时刻只订阅一个点云话题      │
└──────────────┬────────────────────────────────────────────────────┘
               │ 本机 TCP 帧（127.0.0.1:42610，JSON 头 + 二进制点云）
┌──────────────▼────────────────────────────────────────────────────┐
│ a2w_bridge_node（rclpy，ROS2 的 Python —— 本机为 3.14）              │
│   · 采集器崩溃自动重启（退避 1/2/5/10 s）                           │
│   · 发布上表全部标准消息 + 静态 TF                                  │
└───────────────────────────────────────────────────────────────────┘
```

**为什么要两个进程**：`unitree_sdk2py` 依赖 `cyclonedds==0.10.2`（PyPI 只有 cp37~cp310 wheel），
而本机 ROS2 Lyrical 的 `rclpy` 跑在 Python 3.14——两个依赖集无法共存于一个解释器。
采集器进程专门用 3.10 venv（见下），ROS2 进程保持用系统 ROS2 Python。

## 快速开始

```bash
# 0) 前置：连接机器人的网卡（192.168.123.0/24），例如 enx00e04c2c4260
# 1) 一键创建采集器 3.10 venv（uv 自动下载 Python 3.10）
bash src/a2w_bridge/scripts/setup_collector_venv.sh

# 2) 编译安装
cd /home/xiaozhu/Projects/A2W_nav_ws
colcon build --packages-select a2w_bridge
source install/setup.bash

# 2.5) 隔离 ROS2 与机器人控制网（否则 ros2 CLI 可能触发机器狗软急停）
source src/a2w_bridge/scripts/a2w_env.sh

# 3) 启动（网卡/点云源等全部由 JSON 配置决定；隔离已由 launch 自动设好）
ros2 launch a2w_bridge a2w_bridge.launch.py
# 或指定自己的配置：
ros2 launch a2w_bridge a2w_bridge.launch.py config:=/path/to/my.json

# 4) 验证
ros2 topic hz a2w/points a2w/imu
ros2 topic echo /a2w/joint_states --once    # 16 关节（需机器人底层服务在跑）
ros2 topic echo /a2w/status --once         # 运控状态机/关节新鲜度
ip maddr show lo | grep 239.255.0.1         # ROS2 只在回环组播（机器人网卡上那条属采集器，见「ROS 域隔离」）
```

`unitree_sdk2py` 源码路径：`collector.sdk_path`（默认优先 `~/Downloads/unitree_sdk2_python`
官方 SDK，回落 A2W-nav 的 vendor 拷贝）。官方 Python SDK 的 `sensor_msgs` 里没有
`Imu_` 生成类（只有 C++ SDK 有 `unitree/idl/ros2/Imu_.hpp`），本包在
`a2w_bridge/imu_idl.py` 里按 IDL 等价声明了 `Imu_`（`@final + @autoid(sequential)`，
与本机实测匹配，~200 Hz 收数正常）。

## 配置（config/a2w_bridge.json）

所有行为由这份 JSON 决定，改完**重启节点**生效：

| 键 | 可选值 / 默认 | 说明 |
| --- | --- | --- |
| `iface` | 如 `enx00e04c2c4260` | 连接机器人 192.168.123.0/24 的网卡（`ip -br addr` 查看），必填 |
| `log_level` | `debug/info/warn/error` | 采集器日志级别 |
| `collector.python` | 路径数组 | 采集器解释器候选，取第一个存在的 |
| `collector.sdk_path` | 路径数组 | unitree_sdk2py 源码目录候选 |
| `collector.port` | `42610` | 本机 TCP 端口（冲突时改） |
| `pointcloud.enabled` | `true/false` | 是否收点云 |
| `pointcloud.mode` | `single`（默认）/ `multi` | `single`=同一时刻只订阅一个点云话题；`multi`=同时收 fused/front/rear 发到 `multi_topics` |
| `pointcloud.source` | **`front`**（当前配置）/ `fused` / `rear` / `mapping` / `relocation` / `auto` | `single` 模式的点云源。`front` = 只订阅前雷达（本配置现状）；`fused` = 前后雷达融合点云、不轮换；`auto` = 按 `failover_order` 轮换（当前源长时间无数据就换下一个） |
| `pointcloud.failover_order` | `["fused","front","rear"]` | `auto` 时的轮换顺序（无数据超过 `stale_sec` 换下一个） |
| `pointcloud.stale_sec` | `6.0` | 多少秒收不到数据判定“断了” |
| `pointcloud.topic` / `frame_id` | `a2w/points` / `a2w/lidar` | `single` 模式输出话题与坐标系 |
| `pointcloud.fields` | `["x","y","z","intensity"]` | PointCloud2 输出字段（可加 `ring`/`timestamp`）。**字段按源原生类型发**：`ring`→u2、`timestamp`→f8（绝对 Unix 秒）、坐标/强度→f4；要喂 Point-LIO 就看上一节 |
| `pointcloud.max_points` / `max_range` / `voxel` | `0`=关闭 | 每帧随机抽稀 / 距离裁剪（米）/ 体素下采样（米） |
| `pointcloud.filter_zero` | `true`（默认开） | 丢掉机器人填的 `(0,0,0)` 无效点——见「常见问题」第一条（实测占单帧 **68%**） |
| `pointcloud.min_range` | `0.0`=关闭 | 丢掉比该距离更近的点（米）：去自车体/轮子回波时用 |
| `imu.enabled` | `true` | 是否发布 IMU |
| `imu.source` | `auto`（默认，front→rear→lowstate 自动降级）/ `all`（三路各发各的话题）/ `lidar_front` / `lidar_rear` / `lowstate` | IMU 来源 |
| `imu.topic` / `frame_id` | `a2w/imu` / `a2w/imu` | `auto` 与单源模式的输出话题 |
| `imu.topics` | `{lidar_front, lidar_rear, lowstate}` | `all` 模式各自的输出话题 |
| `joints.enabled` / `topic` / `frame_id` | `true` / `a2w/joint_states` / `a2w/base` | 关节状态开关/话题/坐标系 |
| `joints.names` / `indexes` | 16 个（官方腿序 FR/FL/RR/RL × 髋/大腿/小腿/轮） | 关节名与 motor_state 槽位（实机对不上就改这里） |
| `battery.enabled` / `topic` / `dds_topic` | `false`（默认关！） / `a2w/battery` / `rt/bms_state` | 电池（mV/mA 自动换成 V/A）。**默认关**：本机实测订阅 `rt/bms_state` 会让 CycloneDDS 0.10.2 约 25s 后段错误（固件侧 XTypes 类型不兼容），代码路径完整保留，待 SDK/固件问题解决后再开 |
| `ros.isolate` / `domain_id` / `fastdds_profile` | `true` / `42` / `fastdds_iso.xml` | **ROS2 与机器人控制网的隔离**（见上一节）。`domain_id` 不能是 0，否则配置直接报错 |
| `sport_state.enabled` / `topic` / `dds_topic` / `rate_hz` | `true` / `a2w/sport_state` / `rt/sportmodestate` / `10.0` | 只读运控状态机（1001=阻尼/软急停）；机器人侧 ~300 Hz，输出限频 |
| `display.enabled` / `input_topic` / `output_topic` | `true` / `a2w/joint_states` / `joint_states` | RViz 显示用的关节转发（SDK 名 → URDF 名），见「在 RViz 里看真实关节」 |
| `display.rate_hz` / `stale_sec` | `50.0` / `2.0` | `/joint_states` 限频与断流保护（秒） |
| `display.flip` | `[]` | 需要反号的 SDK 关节名列表（RViz 里腿方向反了才加） |
| `display.frame_id` / `urdf` / `rviz_config` | 留空 | frame_id 留空=沿用桥上；urdf/rviz_config 留空=用 `a2w_description` 包里的 |
| `sensors.*.enabled` | 各传感器开关 | slam_info / slam_key_info / global_map |
| `sensors.*.topic` | | 各传感器输出话题 |
| `sensors.*.frame_id` | 留空=跟随机器人 | 非空则覆盖机器人消息里的 frame_id |
| `tf.transforms` | base→lidar、base→imu（单位阵） | 静态 TF 列表，按实机安装尺寸改 `xyz`/`rpy`（弧度） |
| `publish_status_period_sec` | `5.0` | 状态帧周期 |
| `extends` | 如 `"a2w_bridge.json"` | **配置继承**（相对本文件目录）：先读被继承的那份，再把本文件的键合上去（数组整个替掉）。用来只写差异项：`config/a2w_bridge_lio.json` 就只改了 `fields` 与 `imu.source`，标定/网卡仍只有一处真相源 |

### ⚠️ 链路是最重要的约束（改动配置前必读）

点云每帧 ~2.4 MB（23 万点），被切成上千个 IP 分片；机器人**对每个订阅者复制一份**。
实测订阅 4 个点云话题会把千兆口打满（`Ip ReasmFails` 飙升 → 一帧都收不到）：
小报文（sport/slam_info）正常、点云全丢的典型症状。

因此：

1. **`single` 模式一次只订阅一个点云话题**（默认；源固定为 `pointcloud.source`，当前配置是
   `front` 前雷达，不轮换）；想要熔断降级就把 `pointcloud.source` 改成 `auto` —— 它在当前源
   `stale_sec` 无数据时自动轮换（实测融合源时有时无，`front` 通常最稳）；
2. 点云 reader 固定 `BEST_EFFORT + KEEP_LAST(1)`：不触发重传、不积压历史帧；
3. `multi` 模式会同时拉 fused/front/rear 三路（~240 Mbps 起），只在交换机/网卡
   带宽确认有余量时再开。

排查：`cat /sys/class/net/$IF/speed`；连续两次
`cat /sys/class/net/$IF/statistics/rx_bytes` 看流量；
`awk '/^Ip:/{print "InDelivers="$10, "ReasmFails="$17}' /proc/net/snmp` 看分片丢失。

## 本机实测（2026-09，A2W-minipc / Lyrical）

| 话题 | 频率 | 备注 |
| --- | --- | --- |
| `rt/unitree/slam_lidar/imu{1,2}` | ~200 Hz | 标准 `sensor_msgs/Imu`，含协方差 |
| `rt/lowstate` | ~1 kHz（实测 1055 Hz） | 内含 imu_state + motor_state[35]（A2W 用 0~15） |
| `rt/sportmodestate` | ~300 Hz（实测 300.4 Hz） | `error_code` = 运动状态机（1001=阻尼/软急停） |
| `rt/slam_info` | ~5.5 Hz | JSON 广播 |
| `rt/unitree/slam_lidar/points`（fused） | 间歇 | 时有时无，靠 auto 轮换兜底 |
| `rt/unitree/slam_lidar/points1`（front） | ~2.4 Hz | 当前最稳 |
| `rt/unitree/slam_lidar/points2`（rear） | 间歇 | 同上 |

> ⚠️ `rt/lowstate` 只有机器人**低电平/底层服务在跑**时才发布（开机、运控服务运行时都在发）；
> 正式关机/服务未起时会静默——此时 `a2w/status` 的 `joints.fresh_sec` 会一直增长，属正常。
> 若 `ros2 topic hz /a2w/joint_states` 是 0 Hz，先看 `ros2 topic echo /a2w/status --once`
> 的 `joints.fresh_sec` 是 -1（没收到 lowstate）还是有值（桥的问题）。
>
> 📐 关节槽位实测对照：静止时 `motor_state[0..11].q` 依次为
> `-0.573, 1.060, -2.757 | 0.578, 1.062, -2.758 | -0.578, 1.065, -2.764 | 0.595, 1.059, -2.768`
> —— 每三个一组正好是 髋(≈±0.58)/大腿(1.06)/小腿(−2.76，官方限位 −158°~−30°)，且左右髋镜像；
> `idx12..15` 的 τ 只有 ±0.018 N·m → 4 个轮足。即 **0~11 腿关节（步长 3）+ 12~15 轮**。

## 与官方 unitree_ros2 的关系

官方 `unitree_ros2` 仓库走的是另一条路线：把整台机器的 RMW 换成 `rmw_cyclonedds_cpp`
并用 `CYCLONEDDS_URI` 绑定网卡，让 ROS2 节点**直连**机器人 DDS。该路线官方只支持
foxy/humble，与本机 Lyrical（默认 FastDDS）不兼容，且 A2W 的
`rt/lowstate` 还需官方 `unitree_hg` 自定义消息包。
本包选用 Python 采集器（复用 A2W-nav 已验证的 `unitree_sdk2py` + CycloneDDS 0.10.2 栈），
对本机 ROS2 零侵入：其它节点（point_lio、rviz2 等）的 RMW/网卡配置完全不受影响。

## 常见问题

- **RViz 里点云中心有个密集亮点团 / 下游把原点当障碍物** → 雷达把**没回波的格子填 `(0,0,0)`**：
  机器人整帧发的是 **128×900 固定网格**（115200 点），实测（前雷达 `points1`）其中 **78750 个是零点 = 68%**，
  且 `is_dense=true`（在骗人）。本包默认 `pointcloud.filter_zero: true` 把它们丢掉
  （启动日志会打一行 `点云 front: 原始 115200 → 输出 3xxxx 点（丢掉 …%）`）。
  还嫌近处脏就再加 `pointcloud.min_range`（如 `0.3`）去自车体/轮子回波；
  想看原始帧就设 `filter_zero: false`。
- **`a2w_points_rviz.launch.py` 里 RViz 空的 / 没有点** → ① 确认起 launch 的终端已
  `source src/a2w_bridge/scripts/a2w_env.sh`；② 确认 `Fixed Frame`（默认 `a2w/lidar`）
  与 JSON 的 `pointcloud.frame_id` 一致；③ 桥已在别处跑时加 `bridge:=false`。
- **RViz 里关节不动 / 一直定格同一个姿态** → ① 先确认桥在跑、且**本终端也在隔离域**
  （`source src/a2w_bridge/scripts/a2w_env.sh`）；② `ros2 topic hz /joint_states` 应为 50 Hz；
  ③ 没数据时 `joint_relay` 会打 WARN 并在超过 `display.stale_sec` 后**停发**（设计如此，
  免得显示假姿态）。
- **RViz 里某条腿转向和实机相反** → 在 `display.flip` 里加那条腿的 SDK 关节名
  （如 `["FR_calf"]`），或临时 `ros2 launch a2w_bridge a2w_joint_display.launch.py flip:="FR_calf"`。
- **RViz 里只有网格没有狗** → `robot_state_publisher` 没起来（看显示启动器日志），或
  `a2w_description` 没编译（`colcon build --packages-select a2w_description`）；
  网格路径用的是 `package://a2w_description/meshes/...`，RViz 靠 ament index 解析。
- **一执行 `ros2 topic echo …` 机器狗就进软急停（阻尼）** → 是 **ROS2(FastDDS) 与机器人控制网的
  干扰**，不是你在做底层控制：`ros2 topic echo` 会让主机 DDS 域 0 的发现组播（239.255.0.1，
  含类型对象）打到 `192.168.123.0/24` —— 官方写明的“DDS控制信号”网；机器人侧 CycloneDDS 0.10.2
  遇到跨实现类型对象会出问题，运控随即落到阻尼（官方 `error_code` **1001** = 软急停）。
  **修复：`source src/a2w_bridge/scripts/a2w_env.sh`**（launch 默认已对节点做同样隔离），
  再用 `ip maddr show lo | grep 239.255.0.1` 确认 ROS2 只在回环组播（机器人网卡上那条属采集器，
  见「ROS 域隔离」）。
  注意 `ROS_LOCALHOST_ONLY=1` 在本机 FastDDS 3.6 实测**无效**，要用包里的 `config/fastdds_iso.xml`。
  监视是否再次发生：`ros2 topic echo /a2w/sport_state --once`（`error_code=1001` 就是软急停）。
- **点云一个字节都收不到、但 IMU 正常** → 多半是链路被打满或网卡选错。
  先 `ip -br addr` 确认 `iface`；再把 `pointcloud.mode` 保持 `single`、`source` 设 `auto`。
- **`没有 cyclonedds` / Python 版本错误** → 采集器一定要用 3.10 venv：
  `bash scripts/setup_collector_venv.sh`，并确认 `collector.python` 指向它。
- **IMU 先有数据后消失** → `auto` 模式会自动降级；想看三路原始值用 `"source": "all"`。
- **`/a2w/joint_states` 没数据 / `a2w/status` 里 `joints.fresh_sec` 是 -1** → 机器人的低电平
  服务没在发 `rt/lowstate`（待机、关机、或运控服务没启动），不是桥的问题。
- **关节名/位置对不上实机**（比如轮子槽位不对）→ 改 `joints.indexes`（0~34）与
  `joints.names`，无需改代码；对照官方《A2 SDK 开发指南》about_a2w 的腿/关节序号。
- **`a2w/battery` 没数据 / 打开会崩** → 默认就是关的。本机实测订阅 `rt/bms_state`
  （unitree_hg/BmsState_）会让 CycloneDDS 0.10.2 约 25s 后 SIGSEGV
  （`ddsi_xt_type_init_impl: invalid type object`，固件侧类型不兼容，已用对照实验确认：
  关掉该订阅 90s 稳定，打开必崩）。等新版 cyclonedds 或确认固件类型后再开
  （`battery.enabled: true`），代码路径已就绪。
- **想要前后雷达同时进 ROS2** → `"mode": "multi"`（注意带宽约束，见上）。
- **采集器偶发段错误重启（`采集器退出 (code=-11)`，日志里先出现
  `dq.builtin: ... ddsi_xt_type_init_impl with invalid type object`）** → 属既存问题，
  与关节/状态功能无关：单独跑采集器 60s+ 稳定；ROS2 节点和采集器同时跑时，
  机器人侧周期性重新宣告的某个 XTypes 类型对象会让 CycloneDDS 0.10.2 崩（
  推测为 rclpy FastDDS 与机器人 CycloneDDS 同在 domain 0/同网卡的发现干扰）。
  节点已带自动重启（1/2/5/10s 退避），丢几秒数据就自行恢复；
  彻底解决方向：升级 cyclonedds、给 ROS2 侧错开 domain（`ROS_DOMAIN_ID`），
  或改成官方 C++ SDK 路线。
