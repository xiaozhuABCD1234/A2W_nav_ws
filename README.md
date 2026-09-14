# A2W_nav_ws —— A2W（宇树轮足机器人）ROS 2 导航工作空间

目标平台为 **ROS 2 Lyrical Luth**（当前最新 LTS，2026-05 发布，支持期至 2031-05）。
本仓库在该版本上为 A2W 建立完整链路：厂商私有 DDS 数据接入 → 标准 ROS 2 消息 →
LiDAR-Inertial 里程计（Point-LIO / Leg-KILO）→ 导航（Nav2 地图栈或 SCAN-Planner 无图栈），
并可在 Gazebo Harmonic 中复用同一条下游链路做验证。

仓库内容分为三个方向：

1. **版本适配**——最新 LTS 下，机器人厂商与上游算法包尚未完成适配：官方 `unitree_ros2`
   仅支持 foxy/humble；上游 Point-LIO 依赖 `ament_cmake` 中已被移除的宏；apt 源中缺少
   `nav2_bringup` 元包与 `octomap_server`；SCAN-Planner、Leg-KILO 等第三方包需要逐项移植。
   见第一节。
2. **硬件适配**——A2W 的雷达视场与机身几何使通用传感器转换节点与 Nav2 默认参数不适用，
   见第五节。
3. **导航链路实现与验证**——四条可切换链路（三条实机、一条仿真），见第三节。

---

## 一、最新 LTS 上的版本不兼容项与处理

### 1.1 工具链与 apt 包

| # | 不兼容点 | 现象 | 处理 | 位置 |
| --- | --- | --- | --- | --- |
| 1 | `ament_cmake` 2.8.8 已移除 `ament_target_dependencies` 宏 | 上游 Point-LIO 编译失败 | 在 CMakeLists 中就地补齐该宏（`LOCAL PATCH P10`），不改变下游依赖声明与语义 | `point_lio_ros2/CMakeLists.txt`（同一份 shim 复用于 legkilo、scan_planner、lidar_localization_ros2） |
| 2 | CMake 4 已移除 `FindPythonLibs`（CMP0148） | 同上 | 同上（与第 1 项在同一处补丁） | 同上 |
| 3 | apt 源中无 `nav2_bringup` 元包（`ros-lyrical-nav2-bringup` 不存在） | `ros2 launch nav2_bringup …` 不可用；Nav2 组件（1.5.1）虽已安装，但缺少顶层装配 | 参照上游 `navigation_launch.py` 组装 bringup：map_server、AMCL、planner、controller、behavior、bt_navigator、velocity_smoother，统一由 lifecycle_manager 管理 | `a2w_nav2/launch/a2w_nav2.launch.py` |
| 4 | apt 源中无 `octomap_server` | 点云 PCD 无法转换为 `map_server` 可用的占据栅格 | 实现按高度带投影的栅格化工具（本链路只需 2D 栅格，不需要 3D 占据树） | `a2w_nav2/a2w_nav2/pcd_to_map.py` |
| 5 | 官方 `unitree_ros2` 仅支持 foxy/humble，且要求将整机 RMW 切换为 `rmw_cyclonedds_cpp` 并绑定网卡，使节点直连机器人 DDS | 与 Lyrical 默认的 FastDDS 冲突；且 A2W 的 `rt/lowstate` 需要官方自定义消息包 | 不采用官方 ROS 2 方案，改为 Python 采集器（官方 `unitree_sdk2py` + cyclonedds）转换数据；对 ROS 2 侧零侵入，其余节点（point_lio、rviz2 等）的 RMW 与网卡配置不受影响 | `a2w_bridge/a2w_bridge/collector.py`、`node.py` |
| 6 | `unitree_sdk2py` 依赖 `cyclonedds==0.10.2`（PyPI 仅提供 cp37~cp310 wheel），而本机 `rclpy` 运行于 Python 3.14 | 两组依赖无法共存于同一解释器 | 拆分为两个进程：3.10 采集器与 3.14 ROS 节点，二者通过本机 TCP 帧协议通信（JSON 头 + 二进制点云，控制指令经同一连接反向传输） | `a2w_bridge/a2w_bridge/protocol.py` |
| 7 | `ROS_LOCALHOST_ONLY=1` 在本机 FastDDS 3.6 实测无效 | ROS 2 的 DDS 发现报文经机器人控制网卡（`192.168.123.0/24`）发送，触发机器人进入阻尼/软急停状态（运控 `error_code 1001`） | 以 FastDDS XML profile 显式限定网卡与组播地址；launch 自动为节点设置，交互终端需手动 source `a2w_env.sh` | `a2w_bridge/config/fastdds_iso.xml`、`scripts/a2w_env.sh` |
| 8 | 上游 Point-LIO 未提供 A2W 前雷达（JT128）配置 | 缺少对应的 `lidar_type` 与时序参数，运动补偿无法获得逐点时间 | 新增配置文件走 Hesai 分支（字段布局逐字节一致，见第五节末），以获得逐点绝对时间用于运动补偿 | `point_lio_ros2/config/a2w.yaml` |

### 1.2 第三方上游包在 Lyrical 上的移植

| 上游 | 上游基线 | 本仓库所做修改 |
| --- | --- | --- |
| **SCAN-Planner**（社区 ROS 2 移植分支，6 个 ament 包） | Humble / Ubuntu 22.04 | ① `ament_target_dependencies` shim（与 1.1 第 1 项同一份）；② `cv_bridge`、`message_filters/*`、`tf2/LinearMath/Quaternion` 的 `.h` → `.hpp` 改名（用 `__has_include` 同时兼容两版）；③ `message_filters::Subscriber::subscribe(node, topic, rmw_qos_profile_t)` 重载已删除 → 改传 `rclcpp::SensorDataQoS()`；④ VTK 9.5 的 `VTK::jsoncpp` 引用了发行版不导出的 `JsonCpp::JsonCpp` 目标，需在 `find_package(PCL/VTK)` 之前补齐；⑤ 10 个头文件的 include guard `_FOO_H` → `FOO_H`；⑥ 拼写 `falg_use_jerk` → `flag_use_jerk` |
| **Leg-KILO 2.0** | ROS 1（`roscpp`） | ① 接口层重写为 ROS 2：单主节点发布器 + 3 个 spin 子订阅节点 + 5000 Hz 主循环；② `sensor_msgs::ImuPtr` → 本地 `ImuMeas`（去 ROS 依赖）；③ `unitree_legged_msgs::HighState` → `sensor_msgs/JointState`；④ `glog_utils`（gflags + boost）→ `std::filesystem`；⑤ 删除体素图/面元 RViz 可视化；⑥ CMake 由 catkin 改 ament。算法核心（`core/`、`preprocess/`）未改 |
| **lidar_localization_ros2**、**ndt_omp_ros2** | Jazzy / Humble | ① 同一份 ament shim；② 上游 `global_localization_node` 按设计只提供 `~/query` 服务与候选话题，不发行驶位姿 → 由 `a2w_nav2/a2w_nav2/relocalize.py` 桥接为 `/initialpose`（见 3.3） |

### 1.3 移植中发现的两处物理语义错误（Leg-KILO）

这两处不是 API 变化，而是上游实现依赖的假设在 A2W 上不成立，必须修改：

**(a) 时间戳不能相加。** A2W 桥的约定是：点云 `header.stamp` = 帧首主机墙钟；逐点 `timestamp` 列 =
机器人时钟绝对秒（与主机相差约 263 s）；IMU `header.stamp` = 主机墙钟。上游 Hesai 分支将二者
相加作为同一时基，在 A2W 上必然错乱。移植改为只取**帧内相对差**叠加到 `header.stamp`。

同处还有一个 `rclcpp::Time` 的隐式转换陷阱：`rclcpp::Time` 没有 `(double)` 构造函数，写
`rclcpp::Time(1789305121.789)` 时 double 会被隐式转为 `int64_t` 并按**纳秒**解释，时间戳变成
1.789 s。后果是整条 TF/里程计链的时间戳停留在 1.78 s：`a2w_odom_tf` 的停更判定暂停发布 TF，
`/a2w/body_odom` 无 TF 可取，RViz 无机器人模型，规划器取不到位姿。代码内统一经 `secToRosTime()`
构造（出口共 4 处）。

**(b) 世界系必须对齐重力。** 上游将初始姿态设为单位阵（世界系 = 初始化时刻的 IMU 系），
而 A2W 前雷达 IMU 的轴系相对机身转过（实测静止 |a| = 9.85，重力在 +Y）。直接使用会使世界系 z 轴
落在水平方向，表现为 `base_footprint z = -0.44 m`、静止时俯仰角在 -25°~-58° 间跳变、位置随时间
发散。移植在初始化中用静止加速度计对齐（`rot_` 与 `grav_` 两行必须同时修改，仅改前者会使滤波器
不自洽，位置在数十秒内发散）。对齐后静止读数与安装几何一致（`camera_init → base_footprint` 的
y = +0.337 m 对应雷达前装偏移、z = -0.177 m 对应离地高与雷达高之和、yaw = -90° 对应安装方位差）。

---

## 二、包组成

| 包 | 来源 | 内容 | 状态 |
| --- | --- | --- | --- |
| `a2w_bridge` | 本仓库新增 | 厂商 DDS → 标准 ROS 2 消息：点云、IMU、16 关节、运控状态、电池、SLAM 广播、占据栅格，行为全部由 JSON 配置；静态外参 TF 与动态 `base_footprint`；LIO 位姿接入机器人 TF 树（`a2w_odom_tf`）；机体中心里程计 `a2w_body_odom`；唯一下行通道 `/cmd_vel` → `sport_client.Move`（默认关闭） | 实机实测可用 |
| `a2w_nav2` | 本仓库新增 | `a2w_scan`（点云 → LaserScan 与过滤云）、`a2w_pcd_to_map`（PCD → 2D 占据栅格）、`a2w_relocalize`（全局重定位候选 → `/initialpose`）、Nav2 组合式 bringup、SCAN-Planner 一键链路、A2W 定制参数与行为树 | 节点与装配已实测；实机端到端导航未验收 |
| `a2w_gz_sim` | 本仓库新增 | Gazebo Harmonic 仿真：由实机 URDF 生成模型、`sim_bridge` 把仿真数据整形成 `a2w_bridge` 接口（同名话题/帧/时间戳基准），下游 `point_lio`、`a2w_scan`、`a2w_nav2` 原样复用 | 仿真全链已跑通 |
| `a2w_teleop` | 本仓库新增（仅 launch） | 调用 ROS 2 官方 `joy` 与 `teleop_twist_joy` 发布 `/cmd_vel`，无自写节点；仅用于导航未就绪时的手动调试 | 可用 |
| `a2w_description` | 随机器人提供 | A2W（X2-0807 轮式）URDF 与网格；本仓库仅新增按实机关节角只读驱动 RViz 的显示 launch | 可用 |
| `point_lio_ros2` | 上游 + 本仓库适配 | 上游 Point-LIO 本体；新增 `config/a2w.yaml`、`launch/mapping_a2w.launch.py` 与编译补丁 P10 | 实机跑通 |
| `legkilo` | 上游 ROS 1 代码 + 本仓库 ROS 2 移植 | Leg-KILO 2.0（腿足运动学-惯性-激光紧耦合 ESKF，体素八叉树 + 面元地图）。话题、帧名与 odom 系（`camera_init`）与 Point-LIO 完全一致，可直接互换 | 编译与数据链路已验；实机轨迹质量未验 |
| `scan_planner` | 上游社区移植分支 + 本仓库适配 | SCAN-Planner：机体系滑动栅格地图 + 双圆柱足迹 + projected A* + B 样条优化的局部规划器，输出 `/cmd_vel`。含 6 个 ament 包与 A2W 配置/launch | 编译与规划链路已验；实机避障未验 |
| `lidar_localization_ros2` | 上游 | 地图级 3D 全局重定位（BBS 2D 分支限界 + NDT_OMP 打分），服务触发式 | 随重定位链接入 |
| `ndt_omp_ros2` | 上游 | OpenMP 加速的 NDT / GICP 配准库 | 依赖 |

---

## 三、四条导航链路

运行时 `/cmd_vel` **同一时刻只能有一个发布者**——下列链路与手柄 `a2w_teleop` 均会写入该话题。

| # | 链路 | 一键命令 | 组成 | 是否需要地图 |
| --- | --- | --- | --- | --- |
| A | Nav2 地图栈 | `ros2 launch a2w_nav2 a2w_nav2.launch.py map:=$PWD/maps/a2w_map.yaml` | 桥 + LIO + AMCL + 代价地图 + DWB + BT | 需要（先建图） |
| B | Leg-KILO + SCAN-Planner | `ros2 launch a2w_nav2 a2w_scan_nav.launch.py` | 桥 + LIO + `a2w_body_odom` + SCAN-Planner + 闭环控制器 | 不需要 |
| C | 3D 全局重定位 | `ros2 launch a2w_nav2 a2w_relocalize.launch.py map:=…` | BBS 全局搜索 + NDT_OMP 打分 → `/initialpose` | 需要 |
| D | Gazebo 仿真 | `ros2 launch a2w_gz_sim sim_nav.launch.py` | 仿真传感器 → `sim_bridge` → 与 A 相同的下游 | 需要（`maps/a2w_sim_map.yaml`） |

### 3.1 链路 A/B 的常用参数

```bash
# A：不跑 AMCL，改由静态 TF 发 map→camera_init（位姿 100% 等于 LIO）
ros2 launch a2w_nav2 a2w_nav2.launch.py map:=... amcl:=false
# A：顺带启动重定位链（默认 false，见 3.2）
ros2 launch a2w_nav2 a2w_nav2.launch.py map:=... relocalize:=true

# B：只规划不发 /cmd_vel（干跑调参）
ros2 launch a2w_nav2 a2w_scan_nav.launch.py controller:=false
ros2 launch a2w_nav2 a2w_scan_nav.launch.py lio_flavor:=point_lio   # 换回 Point-LIO
ros2 launch a2w_nav2 a2w_scan_nav.launch.py navi_mode:=3            # 路线引导（/initial_path）
```

链路 B 的 `navi_mode`：`1` = RViz 目标点、`2` = 预设航点、`3` = 参考路径（对应论文主场景）。
若需要"Nav2 负责全局规划、SCAN-Planner 负责局部"，将 Nav2 全局路径发到 `/initial_path`、
设 `navi_mode:=3`，同时停掉 Nav2 的 `controller_server`。

### 3.2 链路 A 的定位策略：默认「零自动纠正」

`a2w_nav2_params.yaml` 将 AMCL 的 `update_min_d` / `update_min_a` 设为 `1e9`，即
`shouldUpdateFilter()` 闸门不再打开，导航过程中不做激光似然匹配（`save_pose_rate: 0.0`
同时关闭定时存盘）。重定位链（`relocalize`）默认也是 `false`。因此
**位姿 = `initial_pose` ⊕ LIO 里程计**，LIO 漂多少导航就错多少，且不会被纠正回来。

改为依赖激光定位：将 `update_min_d` / `update_min_a` 改回 `0.15`（`amcl:=true` 时无需其他改动）；
需要主动重定位时再 `relocalize:=true` 并调用服务。两种模式的开机前提相同：机器人位于**建图起点**
且朝向一致（Point-LIO 每次开机重建 `camera_init`）。

---

## 四、架构与数据流

### 4.1 桥接：进程划分

```text
机器人(192.168.123.0/24, DDS 域 0)   点云 2.4 MB/帧 · IMU 196 Hz · 关节 16 路
   │
┌──▼──────────────────────────────────────────────────┐
│ 采集器 collector.py    Python 3.10 venv             │
│   cyclonedds 0.10.2 + unitree_sdk2py（官方 SDK）     │
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

进程拆分的直接原因是第一节 1.1 第 6 项；该结构同时使"机器人网卡上的 DDS"与"ROS 2 域内的 DDS"
在物理上分离。

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

- 机器人将 5 个话题的 `frame_id` 统一写为 `hesai_lidar`，但后雷达 IMU 实际位于**后雷达坐标系**
  （两雷达绕 y 轴互转 180°）。直接透传会使下游的加速度与角速度方向解算错误，故按数据源覆盖
  `frame_id`；
- 点云为 **128×900 固定网格**，无回波位置填 `(0,0,0)`，且 `is_dense` 字段标注为 `true`，
  与实测稀疏性不符（115200 点中 68% 为零点）。桥侧默认过滤，输出约 36,500 有效点/帧。

### 4.2 两种「里程计」语义（规划器必须区分）

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

### 4.3 TF 树

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

### 4.4 全链路

```text
/a2w/points ─┬─▶ point_lio ──▶ /cloud_registered + TF(camera_init→body)
/a2w/imu    ─┤   legkilo  ──▶ /cloud_registered + /Odomtry + TF(camera_init→body)
             │        │
             │        └─▶ a2w_odom_tf ─▶ TF(camera_init→base_footprint)
             │                              └─▶ a2w_body_odom ─▶ /a2w/body_odom
             ▼
  链路 A：a2w_scan ─▶ /scan ─▶ AMCL + 全局/局部代价地图 ─▶ Nav2(DWB) ─┐
             └─▶ /a2w/points_nav ─▶ 局部 voxel 层                      │
  链路 B：/cloud_registered + /Odomtry + /a2w/body_odom ─▶ SCAN-Planner ─┼─▶ /cmd_vel ─▶ 桥运动通道 ─▶ sport Move
  链路 C：/a2w/points_nav ─▶ BBS + NDT_OMP ─▶ a2w_relocalize ─▶ /initialpose ─▶ AMCL（修正定位）
```

---

## 五、硬件约束与实测数据

### 5.1 决定导航参数的三项硬约束

A2W 实机实测（2026-09）得到的三项约束直接决定参数取值：

| 约束 | 实测数据 | 处理 |
| --- | --- | --- |
| 前雷达仅有前向 180° 视场 | 方位覆盖 270°→0°→90°；后方 135°~225° 无回波（受机身遮挡） | `/scan` 视场为 180°；DWB 设 `min_vel_x=0` 禁止倒车；旋转角速度限制为 0.8 rad/s；恢复行为中的盲退由上游默认 0.30 m/0.15 m/s 收窄至 0.10 m/0.05 m/s（注意：Nav2 默认 BT 的 backup 恢复在此约束下必然失败） |
| 雷达扫掠平面位于机身高度区间内 | 机身占地面以上 0.014~0.224 m，雷达离地 0.185 m；r<0.5 m 的自车体回波占 19.8%；实测 z≥离地 0.30 m 后自车体回波为零 | 扫描带不得切入机身高度区间；通用 `pointcloud_to_laserscan` 的高度基准为固定值，无法满足该要求 |
| 低于扫描带的障碍在 2D 扫描中不可见 | 扫描带下沿为离地 0.30 m | 增加第二路输出 `/a2w/points_nav`（宽带减去自车体盒）供局部 voxel 层与全局重定位使用；自车体盒按实测足迹 0.68×0.76 m 设定 |

### 5.2 扫描带必须与建图带对齐（`a2w_scan` 的 `band_reference`）

`a2w_pcd_to_map` 的 `--z-min/--z-max` 切的是**离地绝对高度**，因此 `a2w_scan` 默认使用
`band_reference: ground`，取绝对带 `[0.50, 1.00]`，与建图所用高度带一致。若两者不在同一高度切片上，
AMCL 的似然场评分与 BBS 全局搜索都会失去意义（用现场切片去匹配地图中另一个高度的切片）。

旧的 `band_reference: body` 行为（带 = `[h+0.21, h+1.00]`，h 为实时机身离地高）会使切片随站立/趴伏
上下移动：实测站立时 h≈0.49，得到 `[0.70, 1.49]`，比建图带高 0.2~0.5 m。仅在确认绝对带会引入
自车体回波时才切回该模式；其余近场残留（含站立时进入带内的前雷达保护罩，罩顶 = h+0.40）由
`self_mask_*` 一组盒掩膜处理。

### 5.3 其余实测数据

| 实测项 | 结果 | 影响 |
| --- | --- | --- |
| 点云频率 / IMU 频率 | 7.83 Hz / 196 Hz | 控制 10 Hz、代价地图 5 Hz 已足够，无需提高 |
| 点云与 IMU 滞后差 | 51~55 ms（由桥状态行实时输出） | `time_lag_imu_to_lidar` 可直接据此配置（现值 −0.042），无需试凑 |
| 静止漂移（118.8 s，Point-LIO） | 水平末值 30.7 mm、最大值 41.1 mm，不呈单向累积；yaw 极差 0.32° | 静止精度足以支撑 AMCL；注：早期记录的"静止 30 s 漂移 2.3 mm"为单次快照，不具代表性，已弃用 |
| 运动通道 | 限幅、死区、看门狗、状态门、退出即停均已实机验证 | — |

**`a2w_pcd_to_map` 的栅格判据**（不使用 octomap）：

| 栅格状态 | 判据 |
| --- | --- |
| 占据 | 高度带内落点数 ≥ `--min-hits`（默认 2） |
| 空闲 | 有任意高度的落点，但带内落点为 0（该格可见地面） |
| 未知 | 整格无落点 |

地平面自动识别（取 z 直方图最下方主峰，可用 `--floor-z` 覆盖）。支持 Point-LIO 实际输出的
binary PCD；建议先执行 `--dry-run` 检查统计量与 ASCII 预览后再落盘。

### 5.4 Point-LIO 侧的时序适配

A2W 前雷达（JT128）的点云字段布局与 **Hesai ROS 驱动逐字节一致**
（`x,y,z,intensity` + `ring(u2)` + `timestamp(f8)`，point_step 26，逐点绝对 Unix 时间），
因此 `config/a2w.yaml` 使 Point-LIO 走 `lidar_type: 4`（hesai 分支），即可获得逐点时间用于
运动补偿，无需修改上游算法代码。另注：机器人 IMU 的加速度单位为 m/s²
（实测静止时 |a| = 9.85，重力方向为 +Y），`acc_norm` 须配置为 9.81。

### 5.5 仿真验证结果（Gazebo Harmonic，下游零改动）

站立 → 直行（0.2 m/s）→ Point-LIO（静止漂移 <7 mm）→ slam_toolbox 建图（12×8 房间全覆盖）
→ AMCL 定位（误差 ≤0.15 m）→ Nav2 自主导航到达目标（误差 0.17 m，容差内）→ 柱间绕行。
相对实机参数，仿真的 AMCL 运动模型噪声收紧（`alpha1-5` 0.2 → 0.05，因 LIO 里程计漂移 <1%）、
激光模型锐化（`z_hit/z_rand` 0.9/0.1、`sigma_hit` 0.15）、转速上限降至 0.6 rad/s
（快转时 180° 视场下扫描匹配退化）。其中与里程计质量相关的两项建议原样迁移到实机；
与接触特性相关的容差/转速需在实机小步验证。已知问题：长途绕障时 AMCL 瞬态误差会在代价地图
留下幻影障碍（完成率非 100%）、backup 恢复在禁倒车约束下必然失败。

---

## 六、运行步骤

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

### 6.1 建图（链路 A 的前置）

```bash
# 手动驱动：另开终端运行 ros2 launch a2w_teleop xbox_teleop.launch.py
ros2 launch a2w_bridge a2w_lio.launch.py show_rviz:=false pcd_save:=true
#    Ctrl+C 正常退出时写入 ./PCD/scans.pcd（非正常退出不写入）

ros2 run a2w_nav2 a2w_pcd_to_map PCD/scans.pcd -o maps/a2w_map --dry-run   # 先检查高度带
ros2 run a2w_nav2 a2w_pcd_to_map PCD/scans.pcd -o maps/a2w_map
```

### 6.2 导航

```bash
# 链路 A：Nav2 地图栈（RViz 中设初始位姿，再下发目标）
ros2 launch a2w_nav2 a2w_nav2.launch.py map:=$PWD/maps/a2w_map.yaml

# 链路 B：Leg-KILO + SCAN-Planner（不需要地图）
ros2 launch a2w_nav2 a2w_scan_nav.launch.py
#    ★ 等 LIO 稳定输出位姿后再起规划器：规划器把第一帧机体位姿当作目标点高度基准

# 链路 C：3D 全局重定位（搜索耗时数秒至十余秒，手动触发）
ros2 launch a2w_nav2 a2w_relocalize.launch.py map:=$PWD/maps/a2w_map.yaml
ros2 service call /a2w_relocalize/run std_srvs/srv/Trigger {}

# 链路 D：仿真（无需机器人）
ros2 launch a2w_gz_sim sim.launch.py gui:=false
```

> ⚠️ 链路 A、B 与手柄 `a2w_teleop` **不可同时运行**：三者均发布 `/cmd_vel`。
> 若需在不驱动机器人的前提下验证定位与代价地图，应将桥配置中的 `motion.enabled` 设为 `false`；
> 首次启用运动通道时先设 `dry_run: true` 验证链路。

---

## 七、数据与开发工具目录

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

## 八、验证状态与后续工作

**已验证**：

- 实机：桥接全部话题、DDS 域隔离、运动通道五项安全线、TF 树补全、关节显示、
  `a2w_scan` 输出 `/scan`、`a2w_pcd_to_map` 投影与地平面自动识别、Nav2 全套装配（1.5.1）、
  一键 bringup；
- 开发机：SCAN-Planner 6 个包干净编译并完成"点云/里程计 → 规划 → 控制器 → `/cmd_vel`"全链
  （FSM 循环重规划 7 次，输出 `vx = 0.5 m/s` = 配置上限）；Leg-KILO 编译通过并跑通端到端数据链路
  （合成数据 116 帧无失败，≈10 Hz）；`a2w_body_odom` 位姿与偏航正确；
- 仿真：全链建图与自主导航（见 5.5）。

**未验证**（以下各项不应被视为可用）：

1. **实机端到端导航未验收**：`/scan` 与 PCD 地图形状的一致性、AMCL 收敛性、定位与最终误差均需
   完整走行一次后确认；
2. **行走状态的 LIO 精度未验证**：漂移数据为**静止**实测；行走与转向过程中的点云拖影、
   `time_lag_imu_to_lidar`（当前值 −0.042）需要重新标定；两条 LIO 的轨迹质量需要用同一段
   实机数据对比后再决定取舍；
3. **SCAN-Planner 的实机表现未验证**：真实点云下的避障与轨迹质量、双圆柱足迹包络
   （半径 0.40 / 偏移 0.10 为按实测足迹推导的保守值）是否过保守或过松、闭环控制器跟踪参数、
   速度上限（现 0.5 m/s）均需上机确认；
4. **Leg-KILO 的腿足运动学未适配**：上游模型为 12 关节四足 + 足端接触力，A2W 为 16 关节轮足且
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
[`a2w_gz_sim`](src/a2w_gz_sim/README.md) · [`legkilo`](src/legkilo/README.md) ·
[`scan_planner`](src/scan_planner/README.md) · [`a2w_teleop`](src/a2w_teleop/README.md)。

---

## 九、上游组件与许可

| 组件 | 来源 | 用途 | 许可 |
| --- | --- | --- | --- |
| Point-LIO（ROS 2 移植版） | HKU MARS Lab | LiDAR-Inertial 里程计与建图 | BSD |
| Leg-KILO 2.0 | `asphyxiadie/Leg-KILO`（RA-L 2024） | 腿足运动学-惯性-激光紧耦合里程计 | GPL（上游 2.0；包内 `package.xml` 标 3.0-or-later）⚠️ 与其余包的许可不同 |
| SCAN-Planner | 论文 arXiv 2606.19555；社区 ROS 2 移植分支 | 局部规划器（滑动地图 + projected A* + B 样条） | Apache-2.0 |
| `lidar_localization_ros2`、`ndt_omp_ros2` | `rsasaki0109` | 全局重定位与 NDT/GICP 配准 | BSD-2-Clause |
| `unitree_sdk2py` / A2W 运动服务 | 宇树科技 | 机器人 DDS 数据源、`sport_client` 运动接口 | 见官方仓库 |
| Nav2 1.5.1、`joy`、`teleop_twist_joy`、`slam_toolbox`、`ros_gz` | ROS 2 社区 | 规划/控制/恢复/行为树、手柄驱动、建图、仿真桥 | Apache-2.0 |
| A2W URDF/STL | 随机器人提供（SolidWorks URDF Exporter 导出） | 可视化与碰撞几何参考 | — |

本仓库新增的 `a2w_bridge`、`a2w_nav2`、`a2w_gz_sim` 与各上游包的适配层以 **MIT** 许可发布
（`legkilo/` 因上游为 GPL 而沿用其许可）。
