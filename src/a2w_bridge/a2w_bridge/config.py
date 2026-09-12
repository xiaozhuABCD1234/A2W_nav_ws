"""JSON 配置加载 / 校验 / 默认值填充（纯标准库，node 与 collector 共用）。

配置文件是**唯一**的行为开关：网卡、点云来源、IMU 来源、各话题名、坐标系、
滤波参数都在里面。默认文件：``share/a2w_bridge/config/a2w_bridge.json``。

顶层字段一览::

    iface            : 机器人 DDS 网卡（192.168.123.0/24 那个网口），必填
    log_level        : debug / info / warn / error
    status_topic     : 桥接状态（std_msgs/String, JSON）发布话题
    collector        : 采集器进程（Python 3.10 venv / SDK 路径 / 端口 / 重启退避）
    pointcloud       : 点云（single 单源 + 自动轮换 / multi 多源同时收）
    imu              : IMU（lidar_front / lidar_rear / lowstate / auto / all）
    sensors          : slam_info、slam_key_info、全局占据栅格
    tf               : 静态 TF（base → lidar / imu 等）
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import Any

from .dds_topics import (
    A2W_JOINT_INDEXES,
    A2W_JOINT_NAMES,
    CLOUD_FAILOVER_ORDER,
    CLOUD_TOPICS,
    IMU_FAILOVER_ORDER,
    IMU_TOPICS,
    TOPIC_BMS,
    TOPIC_SPORT_STATE,
)

DEFAULT_PORT = 42610

DEFAULTS: dict[str, Any] = {
    "iface": "",
    "log_level": "info",
    "status_topic": "a2w/status",
    "publish_status_period_sec": 5.0,
    "ros": {
        # ROS2(FastDDS) 与机器人控制网隔离。
        # A2W 的 192.168.123.0/24 是官方文档里写明的“DDS控制信号”网；主机 ROS2 若用域 0
        # 默认配置，会把发现组播(239.255.0.1，含类型对象)打到那张网卡，机器人侧
        # CycloneDDS 0.10.2 碰到跨实现类型对象会出问题，运控落到阻尼（软急停）。
        # 隔离做法：换一个 ROS 域 + FastDDS 只走回环（profile 见 config/fastdds_iso.xml）。
        "isolate": True,
        "domain_id": 42,          # 不能等于 0（0 = 机器人所在域）
        "fastdds_profile": "fastdds_iso.xml",
    },
    "collector": {
        # 按顺序取第一个存在的解释器（unitree_sdk2py 需要 cyclonedds==0.10.2 → Python ≤ 3.10）
        "python": [
            ".venv-collector/bin/python",
            "/home/xiaozhu/Projects/A2W_nav_ws/.venv-collector/bin/python",
            "/home/xiaozhu/Projects/A2W-nav/.venv/bin/python",
            "/usr/bin/python3.10",
        ],
        # unitree_sdk2py 源码目录（PYTHONPATH 注入，无需 pip 安装）
        # 首选官方 SDK（~/Downloads/unitree_sdk2_python），回落本仓库 vendor / A2W-nav vendor
        "sdk_path": [
            "/home/xiaozhu/Downloads/unitree_sdk2_python",
            "/home/xiaozhu/Projects/A2W_nav_ws/vendor/unitree_sdk2_python",
            "/home/xiaozhu/Projects/A2W-nav/vendor/unitree_sdk2_python",
        ],
        "port": DEFAULT_PORT,
        "connect_timeout_sec": 60.0,
        "restart_backoff_sec": [1.0, 2.0, 5.0, 10.0],
    },
    "pointcloud": {
        "enabled": True,
        # single: 同一时刻只订阅一个点云话题（默认，保护千兆链路）
        # multi : 同时订阅 fused/front/rear（会成倍占用链路，需自行确认交换机带宽）
        "mode": "single",
        # single 模式的来源: fused / front / rear / mapping / relocation / auto
        # 随包配置里是 front（只订阅前雷达）；也可设 fused（前后雷达融合点云）、
        # auto = 按 failover_order 轮换（当前话题长时间无数据就换下一个）
        "source": "front",
        "failover_order": list(CLOUD_FAILOVER_ORDER),
        "stale_sec": 6.0,
        "topic": "a2w/points",
        "multi_topics": {
            "fused": "a2w/points_fused",
            "front": "a2w/points_front",
            "rear": "a2w/points_rear",
        },
        "frame_id": "a2w/lidar",
        # 注：机器人发的三路点云（融合/前/后）**都已经是前雷达坐标系**（后雷达点云在机器人侧
        # 已变换到前雷达），所以三路共用这一个 frame；外参见 tf.transforms。
        # 输出字段（标准 PointCloud2 字段名）；可用: x y z intensity ring timestamp
        "fields": ["x", "y", "z", "intensity"],
        # 丢掉机器人填的 (0,0,0) 无效点：机器人在 128×900 固定网格里把“没回波”的格子填 0，
        # 实机实测占 68%（115200 点里 78750 个），且 is_dense=true。开着它能替下游
        # （Point-LIO、costmap、RViz）去掉“传感原点处的假障碍物”，并省 2/3 带宽。
        "filter_zero": True,
        "min_range": 0.0,  # >0 = 丢掉比该距离更近的点（米，0 = 不裁）——用来去自车体回波
        "max_points": 0,   # >0 = 每帧随机抽稀到该点数（0 = 全量）
        "max_range": 0.0,  # >0 = 按到原点距离裁剪（米，0 = 不裁剪）
        "voxel": 0.0,      # >0 = 体素下采样边长（米，0 = 不下采样）
    },
    "imu": {
        "enabled": True,
        # auto: lidar_front → lidar_rear → lowstate 自动降级
        # all : 三者都订阅，分别发布到 topics 下的各自话题
        "source": "auto",
        "topic": "a2w/imu",
        "topics": {
            "lidar_front": "a2w/imu_front",
            "lidar_rear": "a2w/imu_rear",
            "lowstate": "a2w/imu_lowstate",
        },
        "frame_id": "a2w/imu",  # 兜底（frames 里没列的源）
        # 每路源各自的坐标系（结构外参，JT128 雷达内部 IMU 与雷达同姿）：
        #   lidar_front 前雷达 IMU → a2w/lidar
        #   lidar_rear  后雷达 IMU → a2w/lidar_rear
        #     ⚠️ 机器人把 imu2 的 frame_id 也写成 hesai_lidar（本机实测），照搬会让
        #        后 IMU 的朝向差 180°（两雷达绕 y 互转 180°），必须在这里覆盖
        #   lowstate    机身上的低电平 IMU → a2w/imu（与 base_link 同姿）
        "frames": {
            "lidar_front": "a2w/lidar",
            "lidar_rear": "a2w/lidar_rear",
            "lowstate": "a2w/imu",
        },
        "stale_sec": 2.0,
    },
    "joints": {
        "enabled": True,
        "topic": "a2w/joint_states",
        # JointState.header.frame_id（参考系；关节角本身属于机器人本体）
        "frame_id": "a2w/base",
        # 关节名与 motor_state 槽位索引（A2W 官方顺序，见 dds_topics.A2W_JOINT_*）。
        # 实机对不上时（例如轮子其实在 12~15 槽）直接改 indexes/names，不用改代码。
        "names": A2W_JOINT_NAMES,
        "indexes": A2W_JOINT_INDEXES,
    },
    "battery": {
        # ⚠️ 默认关闭：本机实测（2026-09）订阅 rt/bms_state 会让 CycloneDDS 0.10.2
        # 在 ~25s 后 SIGSEGV（ddsi_xt_type_init_impl invalid type object，固件侧类型不兼容）。
        # 代码路径保留完整，换新版 cyclonedds / 确认固件类型后可打开。
        "enabled": False,
        "topic": "a2w/battery",
        "dds_topic": TOPIC_BMS,  # unitree_hg/BmsState_，收不到数据自然静默
    },
    "sport_state": {
        # 只读：订阅 rt/sportmodestate 看运控状态机（error_code）
        # 1001 = 阻尼（遥控器 L2+B 的软急停，官方：Damp()“最高优先级，用于突发情况下的急停”）
        "enabled": True,
        "topic": "a2w/sport_state",
        "dds_topic": TOPIC_SPORT_STATE,
        "rate_hz": 10.0,          # 输出限频（机器人侧 ~300 Hz）
    },
    "display": {
        # 在 RViz 里用 a2w_description 的 URDF 显示**真实关节角**（纯只读，见
        # launch/a2w_joint_display.launch.py 与 a2w_bridge/joint_relay.py）
        "enabled": True,
        "input_topic": "a2w/joint_states",  # 桥发布的 SDK 命名关节角
        "output_topic": "joint_states",     # robot_state_publisher 订阅的 URDF 命名关节角
        "rate_hz": 50.0,                     # 限频（桥上 ~1.1 kHz，RSP 不需要那么快）
        "stale_sec": 2.0,                    # 超过这么久没新数据就暂停发布（免定格假姿态）
        "frame_id": "",                      # 空 = 沿用输入消息的 frame_id
        "flip": [],                          # 需要反号的 SDK 关节名（RViz 里腿方向反了再加）
        "urdf": "",                          # 空 = a2w_description 包里的 a2w_description.urdf
        "rviz_config": "",                   # 空 = a2w_description 包里的 urdf.rviz
    },
    "sensors": {
        "slam_info": {"enabled": True, "topic": "a2w/slam_info"},
        "slam_key_info": {"enabled": True, "topic": "a2w/slam_key_info"},
        "global_map": {
            "enabled": False,
            "topic": "a2w/map/grid",
            "frame_id": "",
        },
    },
    "tf": {
        "enabled": True,
        # 静态 TF。根用 URDF 的 base_link（结构外参就是相对它标定的），单位：米/弧度（ZYX）。
        # 2026-09 标定：
        #   base_link ← 前雷达(JT128，机器人里叫 hesai_lidar)：
        #       T=[0.33767, 0, 0.08134]，R=[[0,0,1],[1,0,0],[0,1,0]] → rpy=[90°,0,90°]
        #       （雷达 x→机身 y、y→z、z→x）
        #   前雷达 ← 后雷达：
        #       T=[0, 0.00599, -0.61764]，R=diag(-1,1,-1) → rpy=[180°,0,180°]（绕 y 转 180°）
        #       ⇒ 后雷达在 base_link 后方 0.280 m（= 0.33767-0.61764），两雷达相距 0.61764 m
        # 换机器人/重新标定只改这几条 xyz/rpy 即可。
        "transforms": [
            {"parent": "base_link", "child": "a2w/lidar", "xyz": [0.33767, 0.0, 0.08134],
             "rpy": [1.5707963, 0.0, 1.5707963]},
            {"parent": "a2w/lidar", "child": "a2w/lidar_rear", "xyz": [0.0, 0.00599, -0.61764],
             "rpy": [3.1415927, 0.0, 3.1415927]},
            {"parent": "base_link", "child": "a2w/imu", "xyz": [0.0, 0.0, 0.0],
             "rpy": [0.0, 0.0, 0.0]},
            # 旧名别名：a2w/base 就是 base_link（保持向后兼容）
            {"parent": "base_link", "child": "a2w/base", "xyz": [0.0, 0.0, 0.0],
             "rpy": [0.0, 0.0, 0.0]},
        ],
    },
}


class ConfigError(Exception):
    """配置非法（文件缺失 / 字段取值非法 / 必填缺失）。"""


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    out = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def _as_list(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [value]
    if isinstance(value, (list, tuple)):
        return [str(v) for v in value]
    return []


def _num(cast: type, key: str, value: Any) -> Any:
    """把配置项转成数值类型；非法时抛带键名的 ``ConfigError``（而不是裸 ValueError）。"""
    try:
        return cast(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"配置 {key} 必须是数值: {value!r}") from exc


def _num_list(key: str, values: Any) -> list[float]:
    try:
        return [float(v) for v in values]
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"配置 {key} 必须是数值数组: {values!r}") from exc


def _read_raw(path: Path, seen: tuple[Path, ...]) -> dict[str, Any]:
    """读一份 JSON 配置，并处理 ``extends``（先取被继承的那份，再把本文件的键合上去）。

    ``extends`` 的值是相对**本文件所在目录**的路径（也可以是绝对路径）。只支持一层层合，
    不支持数组合并（数组整个替掉）—— 见 ``_deep_merge``。

    为什么要有它：A2W 的标定/网卡/话题全在那份大 JSON 里，而“喂 Point-LIO 的那份”只差
    ``fields/min_range/imu.source`` 三个键。靠 ``extends`` 继承，标定改了只改一处，不会
    两份配置各自漂移。
    """
    path = path.expanduser().resolve()
    if path in seen:
        chain = " → ".join(str(p) for p in (*seen, path))
        raise ConfigError(f"配置文件 extends 成环: {chain}")
    try:
        with open(path, encoding="utf-8") as fh:
            raw = json.load(fh)
    except FileNotFoundError as exc:
        raise ConfigError(f"配置文件不存在: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ConfigError(f"配置文件 JSON 语法错误: {path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise ConfigError(f"配置根节点必须是 JSON 对象: {path}")

    parent = raw.pop("extends", None)
    if parent in (None, ""):
        return raw
    base_path = Path(str(parent)).expanduser()
    if not base_path.is_absolute():
        base_path = path.parent / base_path
    return _deep_merge(_read_raw(base_path, (*seen, path)), raw)


def load_config(path: str | os.PathLike[str], *, strict: bool = True) -> dict[str, Any]:
    """读取 JSON 配置（支持 ``extends`` 继承）并与默认值合并；``strict=True`` 时做取值校验。"""
    path = Path(path).expanduser()
    raw = _read_raw(path, ())

    cfg = _deep_merge(DEFAULTS, raw)
    cfg["_config_path"] = str(path)
    cfg["_config_dir"] = str(path.parent)
    _normalize(cfg)
    if strict:
        validate(cfg)
    return cfg


def _normalize(cfg: dict[str, Any]) -> None:
    pc = cfg["pointcloud"]
    pc["fields"] = [str(f) for f in _as_list(pc.get("fields"))] or list(DEFAULTS["pointcloud"]["fields"])
    pc["failover_order"] = _as_list(pc.get("failover_order")) or list(CLOUD_FAILOVER_ORDER)
    pc["mode"] = str(pc.get("mode", "single")).lower()
    pc["source"] = str(pc.get("source", "auto")).lower()
    pc["filter_zero"] = bool(pc.get("filter_zero", True))
    pc["min_range"] = _num(float, "pointcloud.min_range", pc.get("min_range", 0.0) or 0.0)
    pc["max_points"] = _num(int, "pointcloud.max_points", pc.get("max_points", 0) or 0)
    pc["max_range"] = _num(float, "pointcloud.max_range", pc.get("max_range", 0.0) or 0.0)
    pc["voxel"] = _num(float, "pointcloud.voxel", pc.get("voxel", 0.0) or 0.0)
    pc["stale_sec"] = _num(float, "pointcloud.stale_sec", pc.get("stale_sec", 6.0) or 6.0)
    pc.setdefault("multi_topics", {})

    imu = cfg["imu"]
    imu["source"] = str(imu.get("source", "auto")).lower()
    imu["stale_sec"] = _num(float, "imu.stale_sec", imu.get("stale_sec", 2.0) or 2.0)
    imu.setdefault("topics", {})
    _raw_frames = imu.get("frames")
    imu["frames"] = (
        {str(k): str(v) for k, v in _raw_frames.items()} if isinstance(_raw_frames, dict) else {}
    )

    joints = cfg["joints"]
    joints["names"] = _as_list(joints.get("names")) or list(A2W_JOINT_NAMES)
    raw_idx = _as_list(joints.get("indexes"))
    try:
        joints["indexes"] = [int(i) for i in raw_idx] or list(A2W_JOINT_INDEXES)
    except (TypeError, ValueError):
        joints["indexes"] = raw_idx  # 留给 validate() 报 ConfigError
    cfg.setdefault("battery", {})
    cfg["battery"].setdefault("enabled", True)
    cfg["battery"].setdefault("topic", "a2w/battery")
    cfg["battery"].setdefault("dds_topic", TOPIC_BMS)

    ros = cfg["ros"]
    ros["isolate"] = bool(ros.get("isolate", True))
    ros["domain_id"] = _num(int, "ros.domain_id", ros.get("domain_id", 42) or 0)
    ros["fastdds_profile"] = str(ros.get("fastdds_profile", "fastdds_iso.xml") or "")

    sport = cfg["sport_state"]
    sport["enabled"] = bool(sport.get("enabled", True))
    sport["rate_hz"] = _num(float, "sport_state.rate_hz", sport.get("rate_hz", 10.0) or 10.0)

    disp = cfg["display"]
    disp["enabled"] = bool(disp.get("enabled", True))
    disp["input_topic"] = str(disp.get("input_topic", "a2w/joint_states") or "a2w/joint_states")
    disp["output_topic"] = str(disp.get("output_topic", "joint_states") or "joint_states")
    disp["rate_hz"] = _num(float, "display.rate_hz", disp.get("rate_hz", 50.0) or 50.0)
    disp["stale_sec"] = _num(float, "display.stale_sec", disp.get("stale_sec", 2.0) or 2.0)
    disp["frame_id"] = str(disp.get("frame_id", "") or "")
    disp["flip"] = _as_list(disp.get("flip"))
    disp["urdf"] = str(disp.get("urdf", "") or "")
    disp["rviz_config"] = str(disp.get("rviz_config", "") or "")

    cfg["collector"]["port"] = _num(int, "collector.port", cfg["collector"].get("port", DEFAULT_PORT) or DEFAULT_PORT)
    cfg["collector"]["connect_timeout_sec"] = _num(
        float, "collector.connect_timeout_sec", cfg["collector"].get("connect_timeout_sec", 60.0) or 60.0
    )
    backoff = _num_list("collector.restart_backoff_sec", _as_list(cfg["collector"].get("restart_backoff_sec")))
    cfg["collector"]["restart_backoff_sec"] = backoff or [1.0, 2.0, 5.0, 10.0]
    cfg["publish_status_period_sec"] = _num(
        float, "publish_status_period_sec", cfg.get("publish_status_period_sec", 5.0) or 5.0
    )


def validate(cfg: dict[str, Any]) -> None:
    """取值校验；非法时抛 ``ConfigError``（消息里给出合法取值）。"""
    if not str(cfg.get("iface", "")).strip():
        raise ConfigError(
            "配置缺少 iface（机器人 DDS 网卡名，例如 enx00e04c2c4260）；"
            "用 `ip -br addr` 找到 192.168.123.x 那张网卡"
        )
    if str(cfg.get("log_level", "info")).lower() not in ("debug", "info", "warn", "warning", "error"):
        raise ConfigError("log_level 只能是 debug/info/warn/error")

    pc = cfg["pointcloud"]
    if pc["mode"] not in ("single", "multi"):
        raise ConfigError(f"pointcloud.mode 只能是 single/multi，当前 {pc['mode']!r}")
    if pc["min_range"] < 0:
        raise ConfigError(f"pointcloud.min_range 不能为负，当前 {pc['min_range']!r}")
    if pc["mode"] == "single":
        allowed = list(CLOUD_TOPICS) + ["auto"]
        if pc["source"] not in allowed:
            raise ConfigError(f"pointcloud.source 只能是 {'/'.join(allowed)}，当前 {pc['source']!r}")
    bad = [k for k in pc["failover_order"] if k not in CLOUD_TOPICS]
    if bad:
        raise ConfigError(f"pointcloud.failover_order 含未知来源 {bad}，可用: {list(CLOUD_TOPICS)}")
    bad_fields = [f for f in pc["fields"] if f not in ("x", "y", "z", "intensity", "ring", "timestamp")]
    if bad_fields:
        raise ConfigError(f"pointcloud.fields 含未知字段 {bad_fields}")
    for axis in ("x", "y", "z"):
        if axis not in pc["fields"]:
            raise ConfigError(f"pointcloud.fields 必须包含 {axis!r}（ROS2 PointCloud2 需要）")
    if pc["mode"] == "multi":
        missing = [k for k in CLOUD_FAILOVER_ORDER if k not in pc.get("multi_topics", {})]
        if missing:
            raise ConfigError(f"pointcloud.mode=multi 时 multi_topics 必须包含 {missing}")

    imu = cfg["imu"]
    if imu["enabled"] and imu["source"] not in ("auto", "all", *IMU_TOPICS.keys()):
        raise ConfigError(
            f"imu.source 只能是 auto/all/{'/'.join(IMU_TOPICS)}，当前 {imu['source']!r}"
        )
    bad_frames = [k for k in imu.get("frames", {}) if k not in IMU_TOPICS]
    if bad_frames:
        raise ConfigError(
            f"imu.frames 含未知来源 {bad_frames}，可用: {list(IMU_TOPICS)}（值是 frame_id，不能为空）"
        )
    empty_frames = [k for k, v in imu.get("frames", {}).items() if not str(v).strip()]
    if empty_frames:
        raise ConfigError(f"imu.frames 里 {empty_frames} 的 frame_id 为空；要沿用兜底就删掉该键")

    joints = cfg["joints"]
    if joints["enabled"]:
        try:
            indexes = [int(i) for i in joints["indexes"]]
        except (TypeError, ValueError) as exc:
            raise ConfigError(f"joints.indexes 必须是整数数组: {joints['indexes']!r}") from exc
        if not joints["names"]:
            raise ConfigError("joints.names 不能为空")
        if len(joints["names"]) != len(indexes):
            raise ConfigError(
                f"joints.names({len(joints['names'])}) 与 indexes({len(indexes)}) 长度不一致"
            )
        if len(set(indexes)) != len(indexes):
            raise ConfigError(f"joints.indexes 含重复槽位: {indexes}")
        bad = [i for i in indexes if not 0 <= i < 35]
        if bad:
            raise ConfigError(f"joints.indexes 超出 motor_state 槽位范围 0~34: {bad}")
        joints["indexes"] = indexes

    ros = cfg["ros"]
    if ros["isolate"] and ros["domain_id"] == 0:
        raise ConfigError(
            "ros.isolate=true 时 ros.domain_id 不能是 0（0 就是机器人所在的 DDS 域，"
            "同域会把发现组播打进机器人控制网，可能触发机器狗软急停）；改成 42 之类"
        )
    if cfg["sport_state"]["enabled"] and cfg["sport_state"]["rate_hz"] <= 0:
        raise ConfigError("sport_state.rate_hz 必须 > 0")

    disp = cfg["display"]
    if disp["rate_hz"] <= 0:
        raise ConfigError("display.rate_hz 必须 > 0")
    if disp["stale_sec"] <= 0:
        raise ConfigError("display.stale_sec 必须 > 0")
    bad_flip = [n for n in disp["flip"] if n not in A2W_JOINT_NAMES]
    if bad_flip:
        raise ConfigError(
            f"display.flip 含未知关节名 {bad_flip}；可用: {list(A2W_JOINT_NAMES)}"
        )
    if disp["enabled"] and not joints.get("enabled", True):
        raise ConfigError(
            "display.enabled=true 需要 joints.enabled=true（关节角来自 joints 那路 rt/lowstate 订阅）"
        )

    tf_items = cfg.get("tf", {}).get("transforms", [])
    children: list[str] = []
    for item in tf_items:
        if not isinstance(item, dict) or not str(item.get("parent", "")).strip() or not str(
            item.get("child", "")
        ).strip():
            raise ConfigError(f"tf.transforms 每项都需要 parent/child，非法项: {item!r}")
        children.append(str(item["child"]))
    dup = sorted({c for c in children if children.count(c) > 1})
    if dup:
        raise ConfigError(
            f"tf.transforms 里同一个子坐标系出现多次 {dup}；一个坐标系只能有一个父（否则 TF 树冲突）"
        )


# ---------------------------------------------------------------------------
# 路径解析
# ---------------------------------------------------------------------------
def resolve_python(cfg: dict[str, Any], ws_root: Path | None = None) -> str:
    """挑一个可用的采集器解释器：按 collector.python 顺序取第一个存在的文件。"""
    candidates = _as_list(cfg["collector"].get("python"))
    for item in candidates:
        path = Path(item).expanduser()
        if not path.is_absolute() and ws_root is not None:
            path = ws_root / item
        if path.is_file():
            return str(path)
    raise ConfigError(
        "找不到可用的采集器解释器（Python 3.10 + cyclonedds==0.10.2）。\n"
        "  请先运行: src/a2w_bridge/scripts/setup_collector_venv.sh\n"
        f"  候选: {candidates}"
    )


def resolve_sdk_path(cfg: dict[str, Any], ws_root: Path | None = None) -> str:
    """挑一个可用的 unitree_sdk2py 源码目录（含 unitree_sdk2py/ 子目录）。"""
    candidates = _as_list(cfg["collector"].get("sdk_path"))
    if ws_root is not None:
        candidates.append(str(ws_root / "vendor" / "unitree_sdk2_python"))
    for item in candidates:
        path = Path(item).expanduser()
        if (path / "unitree_sdk2py" / "__init__.py").is_file():
            return str(path)
    raise ConfigError(
        "找不到 unitree_sdk2py 源码目录（collector.sdk_path）。\n"
        "  可用: 设置 collector.sdk_path 指向 A2W-nav/vendor/unitree_sdk2_python，"
        "或把 SDK clone 到 <ws>/vendor/unitree_sdk2_python"
    )


def find_ws_root() -> Path | None:
    """从本文件位置推断工作空间根目录（src/a2w_bridge/a2w_bridge/config.py → <ws>）。"""
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "src").is_dir() and (parent / "src" / "a2w_bridge").is_dir():
            return parent
    return None


def default_config_path() -> str:
    """包安装后的默认配置路径（share/a2w_bridge/config/a2w_bridge.json）。"""
    try:  # 仅在 ROS2 环境可用
        from ament_index_python.packages import get_package_share_directory

        return str(Path(get_package_share_directory("a2w_bridge")) / "config" / "a2w_bridge.json")
    except Exception:  # noqa: BLE001 —— 未 source ROS2 时退回源码树
        return str(Path(__file__).resolve().parents[1] / "config" / "a2w_bridge.json")


def describe(cfg: dict[str, Any]) -> str:
    """一行摘要（启动日志用）。"""
    pc, imu = cfg["pointcloud"], cfg["imu"]
    cloud = "关闭" if not pc.get("enabled") else (
        f"multi({','.join(pc.get('multi_topics', {}))})" if pc["mode"] == "multi" else pc["source"]
    )
    if pc.get("enabled"):  # 把实际生效的滤波一并印出来（这些参数直接决定点数/带宽）
        flags = ["丢零点" if pc.get("filter_zero", True) else "含零点"]
        if pc.get("min_range", 0.0) > 0:
            flags.append(f">={pc['min_range']:g}m")
        if pc.get("max_range", 0.0) > 0:
            flags.append(f"<={pc['max_range']:g}m")
        if pc.get("voxel", 0.0) > 0:
            flags.append(f"voxel={pc['voxel']:g}")
        cloud = f"{cloud},{','.join(flags)}"
    joints = cfg["joints"]
    joint_desc = f"关节({len(joints['names'])}" if joints.get("enabled") else "关节(关"
    joint_desc = f"{joint_desc}个)"
    battery = "电池" if cfg["battery"].get("enabled") else "电池(关)"
    disp = cfg.get("display", {})
    display = f"显示={disp.get('rate_hz', 50.0):g}Hz" if disp.get("enabled") else "显示(关)"
    ros = cfg["ros"]
    iso = f"ROS域={ros['domain_id']}(隔离)" if ros.get("isolate") else "ROS域未隔离"
    return (
        f"iface={cfg['iface']} 点云={cloud} IMU={imu['source'] if imu['enabled'] else '关闭'} "
        f"{joint_desc} {battery} {display} {iso} 配置文件={cfg.get('_config_path')}"
    )


if __name__ == "__main__":  # 便于手工校验: python -m a2w_bridge.config <path>
    target = sys.argv[1] if len(sys.argv) > 1 else default_config_path()
    parsed = load_config(target)
    print(describe(parsed))
    print(json.dumps({k: v for k, v in parsed.items() if not k.startswith("_")}, indent=2, ensure_ascii=False))
