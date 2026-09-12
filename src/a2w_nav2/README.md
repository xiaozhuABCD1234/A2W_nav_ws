# a2w_nav2 —— A2W（轮足狗）2D 导航

把 A2W 接进 **标准 Nav2**：前雷达 → 2D 扫描、Point-LIO 的 PCD → 2D 占据栅格、
Nav2 组合式 bringup 与 A2W 定制参数。

| 东西 | 文件 | 干什么 |
| --- | --- | --- |
| `a2w_scan` | `a2w_nav2/scan.py` | 前雷达点云 → `LaserScan`（干净高度带）+ 过滤云（宽带 − 自车体盒） |
| `a2w_pcd_to_map` | `a2w_nav2/pcd_to_map.py` | Point-LIO 的 PCD → `.pgm` + `.yaml`（给 `map_server`/AMCL） |
| `a2w_nav2.launch.py` | `launch/` | map_server + AMCL + 规划/控制/恢复/BT + 速度平滑（**Lyrical 没有 nav2_bringup**，自己组） |
| `a2w_nav2_params.yaml` | `config/` | AMCL/代价地图/DWB 的参数（禁倒车、旋转限速、足迹、坐标系） |
| `a2w_navigate_to_pose.xml` | `behavior_trees/` | 基于上游默认树，只把恢复动作里的**盲退**收窄到 0.10 m/0.05 m/s |

## 目前状态：能跑到哪一步

* ✅ **已实测**：扫描节点（实机出 `/scan`）、PCD→栅格工具（合成房间 + 自动找地平面）、Nav2 全套装配（1.5.1）
* ⏳ **还没做**：走一圈建图 → 投影 → 实机定位/导航验收（见文末「待办与未验证」）

## 实测依据（2026-09，A2W 实机静止；这些数字决定了参数怎么配）

| 实测项 | 结果 | 影响 |
| --- | --- | --- |
| 前雷达水平视场 | **只有前向 180°**（方位 270°→0°→90°；后方 135~225° **0 点**，被自家机身挡住） | `/scan` 只有 180°；**禁倒车 + 旋转限速**；恢复动作盲退必须收窄 |
| 机身占高 | mesh z∈[−0.080,+0.130] + 实测离地 **0.094 m** → 占地面以上 **0.014~0.224 m** | 雷达（离地 0.185 m）平面就切在机身里 → 必须选带 |
| 自车体回波 | r<0.5 m 占 19.8%，其中 83.3% 落在 z∈[0,0.3]；**z≥0.30 m 起 0 点** | 扫描带下沿取 **h+0.21**（≈离地 0.30 m） |
| 天花板 | 2.2 m，占前向点数 **29.5%** | 扫描带上沿取 **h+1.00**，压住别打到天花板 |
| 点云/里程计频率 | **7.83 Hz**（桥侧日志 `云输出Hz[front]` 7.0~8.4） | 控制 10 Hz、代价地图 5 Hz 足够；别盲目拉高 |
| 点云−IMU 滞后差 | **51~55 ms**（桥状态行实时量；`a2w.yaml` 里现值 −0.042） | 走起来要按日志重抄一次 `time_lag_imu_to_lidar` |
| 静止漂移（118.8 s） | 水平末值 **30.7 mm**、最大 **41.1 mm**；每 10 s 游走 4~19 mm，**不单向累积**；yaw 极差 0.32° | ⚠️ `a2w_bridge/README.md` 里"静止 30 s 漂移 2.3 mm"**与实测不符**（那是单次快照，不是代表性指标）；本包文档一律用上表数字 |

> **不用的东西**：融合点云（`rt/unitree/slam_lidar/points`）和后雷达（`points2`）都不参与导航
> —— 只用前雷达 `points1`（最稳的一路，也避免把千兆链路打满）。代价就是后向盲区，用参数兜。

## 快速上手

```bash
# 0) 编译（新增包）
colcon build --symlink-install --packages-select a2w_nav2 && source install/setup.bash
source src/a2w_bridge/scripts/a2w_env.sh          # ROS 域隔离，看话题前必须 source

# 1) 走一圈建图（手柄驱动：另开终端 ros2 launch a2w_teleop xbox_teleop.launch.py）
ros2 launch a2w_bridge a2w_lio.launch.py show_rviz:=false pcd_save:=true
#    Ctrl+C 正常退出时把点云写到【当前目录】/PCD/scans.pcd（不退出不写）

# 2) 离线投影成 2D 栅格（先 --dry-run 看高度带选得对不对，会打 ASCII 预览）
ros2 run a2w_nav2 a2w_pcd_to_map PCD/scans.pcd -o maps/a2w_map --dry-run
ros2 run a2w_nav2 a2w_pcd_to_map PCD/scans.pcd -o maps/a2w_map

# 3) 一键导航（桥 + LIO + odom_tf + URDF/足迹 + 扫描 + Nav2 + RViz）
ros2 launch a2w_nav2 a2w_nav2.launch.py map:=$PWD/maps/a2w_map.yaml
#    RViz：Fixed Frame 选 map → "2D Pose Estimate" 点一下初始位姿 → 2D Goal Pose 发目标
```

分步起（桥/LIO 已在别的终端跑）:

```bash
ros2 launch a2w_nav2 a2w_nav2.launch.py map:=... lio:=false display:=false scan:=false
ros2 launch a2w_nav2 a2w_nav2.launch.py map:=... rviz:=false     # 无窗口（ssh）
```

## 坐标系与话题（与补 TF 那条边的人约定好，别改）

```text
map ──────────(AMCL)──────────▶ camera_init ──(point_lio)──▶ body
                                    │
                                    └──(a2w_odom_tf)──▶ base_footprint ──▶ base_link ──▶ a2w/lidar
                                                                                        ├─▶ a2w/lidar_rear
                                                                                        └─▶ a2w/imu
```

| 边 | 谁发 | 频率 |
| --- | --- | --- |
| `map → camera_init` | `nav2_amcl` | 定位更新时 |
| `camera_init → body` | `point_lio` | ≈8 Hz |
| `camera_init → base_footprint` | `a2w_odom_tf` | ≈8 Hz |
| `base_footprint → base_link` | `a2w_base_footprint` | 10 Hz（z=实测离地高） |
| `base_link → a2w/*` | 桥（标定 JSON） | 静态 |

* **`camera_init` 就是 odom**（AMCL 的 `odom_frame_id`、局部代价地图的 `global_frame`）
  —— 不要再另发一个叫 `odom` 的帧，AMCL 发 `map→odom` 时父子会打架。
* 底盘帧用 **`base_footprint`**（贴地，z 随姿态变）——代价地图/足迹都在这个系里。

话题链路：

```text
/a2w/points ─▶ a2w_scan ─▶ /scan ─▶ AMCL + 全局/局部代价地图
                       └─▶ /a2w/points_nav ─▶ 局部 voxel 层（低于扫描带的障碍）

controller_server ──cmd_vel_nav──▶ velocity_smoother ──cmd_vel──▶ a2w_bridge 运动通道 ─▶ sport Move
```

⚠️ **手柄（`a2w_teleop`）和导航不能同时开**：两边都发 `/cmd_vel`。
想让机器人绝不动、只看定位与代价地图：把桥的 `motion.enabled` 设成 `false` 再起桥。

## 参数怎么来的（两个文件，改之前先读这段）

### `config/a2w_scan.yaml`（点到线：高度带）

高度全部以**实时实测的机身离地高度 h** 为基准（`h` = TF `base_footprint → base_link` 的 z，
由 `a2w_base_footprint` 量；待机 0.094 m，站起来变大）：

| 参数 | 默认 | 依据 |
| --- | --- | --- |
| `band_low_above_body` | 0.21 | 实测 z≥h+0.21（≈离地 0.30 m）后自车体回波为 0 |
| `band_high_above_body` | 1.00 | 天花板 2.2 m，压住 |
| `cloud_min_height` | 0.10 | 地面在 z≈0，直接切掉；过滤云还能看见"低于扫描带"的障碍 |
| `self_mask_*` | 足迹盒 | 实测足迹 x∈[−0.358,0.417]、y∈±0.341、上表面 h+0.130（只作用于过滤云） |
| `angle_min/max` | ∓90° | 前雷达实际视场就是 180° |

### `config/a2w_nav2_params.yaml`（A2W 的硬约束）

| 参数 | 值 | 为什么 |
| --- | --- | --- |
| `min_vel_x`（DWB） | **0.0** | 后方盲区 → 不许倒车 |
| `min_velocity`（平滑器） | **[0.0, 0.0, −0.8]** | 最后一道闸，负 vx 直接钳 0 |
| `max_vel_theta` / `max_rotational_vel` | 0.8 / 0.8 | 旋转限速（官方走路上限 2.0） |
| `max_vel_x` | 0.6 | 官方低速档 0.8 留余量 |
| `max_vel_y` | 0.0 | 先不横移（想开：设 0.3 + `vy_samples: 5`） |
| `footprint` | 矩形 | 实测待机姿态 [[−0.358,−0.341],[0.417,−0.341],[0.417,0.341],[−0.358,0.341]] |
| `inflation_radius` | 0.45 | 足迹半宽 0.34 + 0.11 |
| `robot_model_type`（AMCL） | Omni | 轮足狗能横移、LIO 里程计是完整 3 自由度 |
| `allow_unknown`（NavFn） | true | PCD 投影图里未知区多，不许穿就基本没路 |
| `track_unknown_space` | true | 未知区如实标未知（别当空闲） |
| BT 的 `BackUp` | 0.10 m / 0.05 m/s | 上游默认 0.30 m/0.15 m 在盲区里太激进 |

## `a2w_pcd_to_map` 的算法（三态，不用 octomap）

Lyrical 的 apt 里**没有 octomap_server**，这里也不需要 3D 占据树，只要"按高度带压平"：

| 栅格状态 | 判据 |
| --- | --- |
| 占据（黑 0） | **带内**落点 ≥ `--min-hits`（默认 2） |
| 空闲（白 254） | 有任意高度的落点、但带内 0 点（说明这格看得见地面/别处） |
| 未知（灰 205） | 整格一个点都没有 |

* 地平面**自动找**（z 直方图最下面的主峰），`--floor-z` 可覆盖；输出的 `origin` 是栅格左下角，
  原点就是**建图起点** → AMCL 的 `initial_pose` 用 (0,0,0) = "把狗放回建图起点"
* 支持 Point-LIO 实际写出的 **binary** PCD（`writeBinary`）；`binary_compressed` 会明确报错
* 先 `--dry-run` 看统计与 ASCII 预览再落盘

## 待办与未验证（别当成已经好了）

1. **实机定位没验过**：`/scan` 与 PCD 地图形状对不对、AMCL 收敛不收敛、误差多大 —— 都要走一遍才知道。
2. **行走中的 LIO 精度没验过**：上面的漂移数字是**静止**测的；行走/转向时点云拖影与
   `time_lag_imu_to_lidar`（现值 −0.042，日志实测 51~55 ms）需要重新标。
3. **`/odom` 话题还不存在**：DWB 的 `odom_topic` 与速度平滑器的闭环反馈都指向 `odom`
   （frame `camera_init`、child `base_footprint`）。补 TF 的人已经说可以顺带发一条
   `nav_msgs/Odometry`；发出来之前：DWB 只能靠 TF 估速度（能用、但反馈偏乐观），
   平滑器保持 `OPEN_LOOP`。
4. **低矮障碍**：低于扫描带（离地 0.30 m）的障碍只靠 `/a2w/points_nav` 进局部 voxel 层，
   而 voxel 层只覆盖机器人在的地方；全局路线上仍可能"看不见"。
5. **AMCL 初值**：`set_initial_pose: true` + (0,0,0) 只在"狗放在建图起点、朝向一致"时成立；
   否则 RViz 里点 2D Pose Estimate。
6. **后向盲区的兜底**：建议后续加 `nav2_collision_monitor`（已装、未起）或把
   `a2w_bridge` 的软急停门（error_code 1001）联动到导航暂停 —— 现在只有桥侧的门。

## 排错

* **`/scan` 没数据** → ① 本终端 source 过 `a2w_env.sh`？② 桥在跑且 `/a2w/points` 有数据？
  ③ TF 是否齐：`ros2 run tf2_ros tf2_echo base_footprint a2w/lidar`（缺就检查
  `a2w_base_footprint` 与桥）；④ 看 `a2w_scan` 的状态行（输入点数 / 带内点数 / 束数）。
* **扫描带点数太少**（状态行里"束数"很低）→ 高度带选错了。用 `--dry-run` 那套工具先看
  点的 z 分布，再调 `band_low_above_body`。
* **机器人不动** → ① 桥的运动通道 `motion.enabled` 开了吗？② 运控状态是 1001 阻尼？
  （`ros2 topic echo /a2w/sport_state --once`）③ `cmd_vel` 有没有速度：
  `ros2 topic echo /cmd_vel`（平滑器输出）与 `/cmd_vel_nav`（控制器输出）分别看一眼，
  中间断在哪一段一目了然。
* **代价地图里机器人周围一圈"假障碍"** → 自车体没滤干净：检查 `self_mask_*` 是否与
  当前足迹一致（`ros2 run a2w_bridge a2w_base_footprint --once --print-nav2`）。
