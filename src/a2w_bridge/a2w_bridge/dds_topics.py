"""A2W DDS 话题名常量（node 与 collector 共用，纯标准库）。

来源：A2W 官方 SDK《SLAM与导航服务接口》与 unitree_slam 参考例程，
与本机 A2W-nav 工程 `src/a2w_nav/slam_api.py` 保持一致。
"""

from typing import Any

# ---------------------------------------------------------------------------
# 点云话题：每帧约 2.4 MB（23 万点 / point_step=26），会被切成上千个 IP 分片。
# 机器人对**每个订阅者**复制一份，多订阅会把千兆链路打满（IP 重组失败 → 一帧都收不到）。
# 因此默认同一时刻只订阅一个点云话题，见 README「链路」一节。
# ---------------------------------------------------------------------------
CLOUD_TOPICS = {
    "fused": "rt/unitree/slam_lidar/points",          # 前后雷达融合点云
    "front": "rt/unitree/slam_lidar/points1",         # 前雷达
    "rear": "rt/unitree/slam_lidar/points2",          # 后雷达
    "mapping": "rt/unitree/slam_mapping/points",      # 建图点云（仅建图时发布）
    "relocation": "rt/unitree/slam_relocation/points",  # 定位点云（仅定位时发布）
}

# 点云自动选源顺序（single + auto 模式的看门狗轮换顺序）
CLOUD_FAILOVER_ORDER = ("fused", "front", "rear")

# ---------------------------------------------------------------------------
# IMU / 传感器话题
# ---------------------------------------------------------------------------
IMU_TOPICS = {
    "lidar_front": "rt/unitree/slam_lidar/imu1",  # sensor_msgs/Imu，~200 Hz
    "lidar_rear": "rt/unitree/slam_lidar/imu2",   # sensor_msgs/Imu，~200 Hz
    "lowstate": "rt/lowstate",                    # unitree_hg/LowState，内含 imu_state
}
# IMU 自动选源顺序（auto 模式）
IMU_FAILOVER_ORDER = ("lidar_front", "lidar_rear", "lowstate")

TOPIC_SLAM_INFO = "rt/slam_info"                # std_msgs/String（JSON），~5.5 Hz
TOPIC_SLAM_KEY_INFO = "rt/slam_key_info"        # std_msgs/String（JSON）
TOPIC_RELOCATION_GLOBAL_MAP = "rt/unitree/slam_relocation/global_map"  # nav_msgs/OccupancyGrid

# ---------------------------------------------------------------------------
# A2W 关节序（与 Go2/B2 等“别的狗”不同，见官方《A2 SDK 开发指南》about_a2w）
#
# 官方命名：Leg0=FR 右前  Leg1=FL 左前  Leg2=RR 右后  Leg3=RL 左后
#           Joint0=Hip  Joint1=Thigh  Joint2=Calf  Joint3=Wheel(轮足，A2W 特有)
#
# ⚠️ 但 LowState.motor_state[] 数组里的实际排列与官方命名表不是一回事（本机实测）：
#
#   idx 0..11 = 12 个腿关节，**步长 3**（腿序 FR/FL/RR/RL × 髋/大腿/小腿）：
#       0,1,2 = FR 髋/大腿/小腿   3,4,5 = FL   6,7,8 = RR   9,10,11 = RL
#   idx 12..15 = 4 个轮足（GO2W/B2W 也是这种“腿关节在前、轮在后”的排法，
#                    不是每腿紧跟一个轮）
#
# 实测依据（A2W 静止、关节通电）：
#   * 12 个腿关节的 q 全部落在官方限位内，且左右髋镜像对称
#     （FR_hip=-0.5725 / FL_hip=+0.5778，RR_hip=-0.5784 / RL_hip=+0.5954）；
#   * 若按“每腿 4 个”读，FL/RR 组的 q 会超出髋/大腿/小腿限位 → 排除；
#   * idx12~15 的 tau 只有 ±0.018 N·m（腿部 ±0.4）、温度更低 → 轮足。
#
# 轮足内部顺序（12~15 对应哪条腿）官方文档未写，默认按腿序 FR/FL/RR/RL；
# 想确认就单独转一个轮，看哪个索引的 q 在变，然后改 JSON 里的 joints.indexes。
# ---------------------------------------------------------------------------
A2W_JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
    "FR_wheel", "FL_wheel", "RR_wheel", "RL_wheel",
]
A2W_JOINT_INDEXES = list(range(16))  # motor_state 槽位 [0..15]（0~11 腿关节，12~15 轮）

# 电池管理（unitree_hg/BmsState_，bmsvoltage 单位 mV / current 单位 mA）
TOPIC_BMS = "rt/bms_state"

# ---------------------------------------------------------------------------
# 运控状态（只读）— rt/sportmodestate（unitree_go/SportModeState_）
#
# 官方《运控服务接口 V2.0》(A2-W 适配版)：error_code 字段就是运动状态机；
# **1001 = 阻尼**，也就是遥控器 L2+B 的"软急停"状态，Damp() 接口备注写的是
# "该模式具有最高的优先级，用于突发情况下的急停"。
# 本包只订阅它做监视，不下发任何控制指令。
# ---------------------------------------------------------------------------
TOPIC_SPORT_STATE = "rt/sportmodestate"

SPORT_STATE_NAMES = {
    0: "待机/未进入运控",        # 本机实测：机器人静止未接管时 error_code=0
    100: "灵动",
    1001: "阻尼(软急停)",
    1002: "站立锁定",
    1004: "蹲下",
    1006: "动作(打招呼/舞蹈/…)",
    1007: "坐下",
    1008: "前跳",
    1009: "扑人",
    1013: "平衡站立",
    1015: "常规行走",
    1016: "常规跑步",
    1017: "常规续航",
    1091: "摆姿势",
    2006: "蹲下",
    2007: "闪避",
    2008: "并腿跑",
    2009: "跳跃跑",
    2010: "经典",
    2011: "倒立",
}
# 软急停/阻尼的状态码（监视告警用）
SPORT_STATE_DAMPING = 1001


def sport_state_name(code: Any) -> str:
    """error_code → 状态名（非法/未知码原样返回，不抛异常）。"""
    try:
        key = int(code)
    except (TypeError, ValueError):
        return f"未知({code!r})"
    return SPORT_STATE_NAMES.get(key, f"未知({key})")
