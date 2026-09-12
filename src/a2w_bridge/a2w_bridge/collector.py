#!/usr/bin/env python3
"""A2W DDS 采集器——在 **Python 3.10** 环境里跑，把机器人话题转成本机 TCP 帧。

为什么单独一个进程：
  * `unitree_sdk2py` 依赖 `cyclonedds==0.10.2`，只有 cp37~cp310 的 wheel；
  * 本机 ROS2（Lyrical）的 `rclpy` 跑在 **Python 3.14** 上。
两者无法共存于同一解释器，所以采集器（3.10 + Cyclone DDS）与 ROS2 节点（3.14 + rclpy）分成
两个进程，用 127.0.0.1 上的 TCP 帧通信（见 `protocol.py`）。

采集内容由 JSON 配置决定（`config.py`）：网卡、点云来源、IMU 来源、各传感器开关。

链路铁律（与 A2W-nav 一致，改动前请先读 README）：
  1. **同一时刻只订阅一个点云话题**（single 模式）——机器人对每个订阅者复制一份
     ~2.4 MB/帧的点云，多订阅会打满千兆口、IP 分片重组失败导致一帧都收不到；
  2. 点云 reader 用 **BEST_EFFORT + KEEP_LAST(1)**，不让发送端重传、不积压历史帧。
"""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import signal
import socket
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np

# 直接当脚本跑（python collector.py）时补路径，使 a2w_bridge.* 可导入；
# 作为包的一部分（python -m a2w_bridge.collector）时跳过。
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from a2w_bridge import protocol
from a2w_bridge.config import ConfigError, load_config
from a2w_bridge.dds_topics import (
    CLOUD_TOPICS,
    IMU_FAILOVER_ORDER,
    IMU_TOPICS,
    SPORT_STATE_DAMPING,
    TOPIC_RELOCATION_GLOBAL_MAP,
    TOPIC_SLAM_INFO,
    TOPIC_SLAM_KEY_INFO,
    sport_state_name,
)
from a2w_bridge.motion import MotionController

# PointCloud2 PointField.datatype -> (numpy 字符, 字节数)
_FIELD_DTYPE = {
    1: ("i1", 1),
    2: ("u1", 1),
    3: ("i2", 2),
    4: ("u2", 2),
    5: ("i4", 4),
    6: ("u4", 4),
    7: ("f4", 4),
    8: ("f8", 8),
}

# 本桥输出用的字段类型白名单（字段名 -> numpy 字符）。
# 为什么输出要带类型、而且不统一成 f4：Point-LIO 的 HESAI 分支按
# ``double timestamp`` + ``uint16 ring`` 读点（见 point_lio_ros2/src/preprocess.h
# 的 hesai_ros::Point）。把 f8 的**绝对 Unix 秒**压成 f4 会掉到 0.1 s 级精度、
# 把 u2 的 ring 当 f4 会变成 NaN 一样的垃圾值 —— 所以按源字段的原生类型透传。
CLOUD_FIELD_KINDS = {"x", "y", "z", "intensity", "ring", "timestamp"}


def log(text: str) -> None:
    """写 stderr（由 ROS2 节点转发进 ROS 日志）。"""
    print(f"[collector] {text}", file=sys.stderr, flush=True)


# ---------------------------------------------------------------------------
# 点云解析 / 滤波
# ---------------------------------------------------------------------------
def _packed_dtype(names: list[str], dtypes: list[str]) -> np.dtype:
    """按 ``[(名称, '<类型')]`` 造**紧凑无填充**的结构化 dtype（一个点 = itemsize 字节）。"""
    return np.dtype([(n, "<" + d) for n, d in zip(names, dtypes)])


def _xyz(points: np.ndarray) -> np.ndarray:
    """结构化点云 -> ``(N, 3)`` float64 坐标（只给滤波用，不动原数组）。"""
    return np.stack((points["x"], points["y"], points["z"]), axis=1).astype(np.float64)


def cloud_to_array(
    msg: Any, fields: list[str], *, warn: Any = None
) -> tuple[np.ndarray, list[str], list[str]]:
    """``sensor_msgs/PointCloud2`` -> ``(结构化数组, 字段名, 字段类型)``。

    按 ``fields`` 顺序取列，**保持源字段的原生数值类型**（只把字节序统一成小端）：
    ``x/y/z/intensity`` 一般是 f4、``ring`` 是 u2、``timestamp`` 是 f8（绝对 Unix 秒）。
    下游 ``node.py`` 会按这里的类型发布 PointField，Point-LIO 的 HESAI 分支才能正确读。

    ``warn``：可选回调（收到“源里没有这个字段”的提示文本）—— 只在第一次出现时由调用方去重。
    """
    by_name = {f.name: f for f in msg.fields}
    names = [n for n in fields if n in by_name]
    missing = [n for n in fields if n not in by_name]
    if missing and warn is not None:
        warn(f"源点云没有字段 {missing}（现有: {list(by_name)}），本次已跳过")
    for axis in ("x", "y", "z"):
        if axis not in names:
            raise ValueError(f"点云缺少字段 {axis!r}（现有: {list(by_name)}）")

    dtypes: list[str] = []
    for name in names:
        try:
            dtypes.append(_FIELD_DTYPE[int(by_name[name].datatype)][0])
        except (KeyError, TypeError, ValueError):
            raise ValueError(
                f"字段 {name!r} 的 datatype 非法: {by_name[name].datatype!r}"
            ) from None
    layout = _packed_dtype(names, dtypes)

    try:
        point_step = int(msg.point_step)
        width = int(msg.width)
        height = int(msg.height)
        row_step = int(msg.row_step)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"点云字段 point_step/width/height/row_step 非法: {exc}") from None
    total = width * height
    if total == 0 or point_step == 0:
        return np.zeros(0, dtype=layout), names, dtypes

    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if row_step <= width * point_step or height <= 1:
        if buf.size < total * point_step:
            raise ValueError(f"点云数据长度不足: {buf.size} < {total * point_step}")
        arr = buf[: total * point_step].reshape(total, point_step)
    else:  # 行间有填充
        rows = [
            buf[r * row_step : r * row_step + width * point_step].reshape(width, point_step)
            for r in range(height)
        ]
        arr = np.vstack(rows)

    endian = ">" if bool(msg.is_bigendian) else "<"
    out = np.empty(arr.shape[0], dtype=layout)
    for name, dt in zip(names, dtypes):
        field = by_name[name]
        try:
            _, size = _FIELD_DTYPE[int(field.datatype)]
            offset = int(field.offset)
        except (KeyError, TypeError, ValueError):
            raise ValueError(
                f"字段 {name!r} 的 datatype/offset 非法: {field.datatype!r}@{field.offset!r}"
            ) from None
        # astype 顺带把小端/大端与目标类型都归一化（int -> float 也可以）
        col = arr[:, offset : offset + size].copy().view(endian + dt).reshape(-1)
        out[name] = col.astype(out.dtype[name], copy=False)

    if out.size:
        # 只对坐标判有效：其它字段（intensity/ring/timestamp）可能是任意数值
        with np.errstate(invalid="ignore"):
            finite = np.isfinite(out["x"]) & np.isfinite(out["y"]) & np.isfinite(out["z"])
        out = out[finite]
    return out, names, dtypes


def drop_zero_points(points: np.ndarray) -> np.ndarray:
    """丢掉机器人填的 ``(0,0,0)`` 无效点。

    机器人（JT128 固件）把 **128×900 的固定网格**（115200 点）整帧发过来，没有回波的格子填
    ``(0,0,0)``，而且 ``is_dense=true`` —— 实机实测这类点占 **68.4%**。不过滤的话，下游
    （Point-LIO、costmap、RViz）会把它们当成“传感器原点处的障碍物”。只比 xyz 三列，
    ``intensity`` 不参与判定。
    """
    if points.shape[0] == 0:
        return points
    keep = ~(
        (points["x"] == 0.0) & (points["y"] == 0.0) & (points["z"] == 0.0)
    )
    return points[keep]


def crop_range(points: np.ndarray, max_range: float, min_range: float = 0.0) -> np.ndarray:
    """按到原点距离裁剪；两边都是 0 = 不裁。"""
    if points.shape[0] == 0 or (max_range <= 0 and min_range <= 0):
        return points
    r = np.linalg.norm(_xyz(points), axis=1)
    keep = np.ones(points.shape[0], dtype=bool)
    if max_range > 0:
        keep &= r <= max_range
    if min_range > 0:
        keep &= r >= min_range
    return points[keep]


def voxel_downsample(points: np.ndarray, voxel: float) -> np.ndarray:
    """体素下采样（每个体素取一个代表点），纯 numpy：三维体素索引打包成 int64 后 1D unique。"""
    if voxel <= 0 or points.shape[0] == 0:
        return points
    # 注意：保留每个体素里**下标最小**的点并按原下标排序 → 输出仍是时间单调的，
    # 逐点时间戳（timestamp/curvature）不会被降采样打乱顺序。
    keys = np.floor(_xyz(points) / voxel).astype(np.int64)
    keys = np.clip(keys + (1 << 16), 0, (1 << 17) - 1)
    packed = (keys[:, 0] << 34) | (keys[:, 1] << 17) | keys[:, 2]
    _, idx = np.unique(packed, return_index=True)
    return points[np.sort(idx)]


def f32_list(values: Any, default: float = 0.0) -> list[float]:
    """IDL 数值序列 -> 普通 float 列表（cyclonedds 未赋值字段可能为 None）。"""
    out: list[float] = []
    try:
        for v in values:
            try:
                out.append(float(v))
            except (TypeError, ValueError):
                out.append(default)
    except TypeError:
        return [default]
    return out


def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def as_text(data: Any) -> str:
    if isinstance(data, (bytes, bytearray)):
        return data.decode("utf-8", errors="replace")
    return str(data)


def int_list(values: Any) -> list[int]:
    """IDL 整数序列 -> 普通 int 列表（cyclonedds 未赋值字段可能为 None）。"""
    out: list[int] = []
    try:
        for v in values:
            out.append(safe_int(v))
    except TypeError:
        pass
    return out


# ---------------------------------------------------------------------------
# 采集器
# ---------------------------------------------------------------------------
class Collector:
    def __init__(self, cfg: dict[str, Any], port: int | None = None) -> None:
        self.cfg = cfg
        self.port = port if port is not None else cfg["collector"]["port"]  # 已规范化成 int
        self.iface = cfg["iface"]

        pc = cfg["pointcloud"]
        self.cloud_enabled = bool(pc.get("enabled", True))
        self.cloud_mode = pc["mode"]
        self.cloud_source = pc["source"]
        self.cloud_order = list(pc["failover_order"])
        self.cloud_stale = pc["stale_sec"]
        self.cloud_fields = list(pc["fields"])
        self.cloud_max_points = pc["max_points"]
        self.cloud_max_range = pc["max_range"]
        self.cloud_min_range = pc["min_range"]
        self.cloud_filter_zero = bool(pc.get("filter_zero", True))
        self.cloud_voxel = pc["voxel"]

        imu = cfg["imu"]
        self.imu_enabled = bool(imu.get("enabled", True))
        self.imu_source = imu["source"]
        self.imu_stale = imu["stale_sec"]

        joints = cfg["joints"]
        self.joints_enabled = bool(joints.get("enabled", True))
        self.joint_indexes = list(joints.get("indexes", []))  # config.py 已规范化成 int 列表
        self.joint_names = list(joints.get("names", []))

        battery = cfg["battery"]
        self.battery_enabled = bool(battery.get("enabled", True))
        self.battery_dds_topic = str(battery.get("dds_topic", "rt/bms_state"))

        sport = cfg["sport_state"]
        self.sport_enabled = bool(sport.get("enabled", True))
        self.sport_dds_topic = str(sport.get("dds_topic", "rt/sportmodestate"))
        self.sport_rate_hz = sport["rate_hz"]  # config.py 已规范化成 float

        motion = cfg["motion"]
        self.motion_enabled = bool(motion.get("enabled", False))
        # 运动通道（下行）：唯一会向机器人发控制指令的地方，配置默认关。
        # 状态门读的就是 _on_sport 维护的 _last_sport（error_code=1001 阻尼时拒发）。
        self.motion = (
            MotionController(motion, log, state_provider=self._sport_snapshot)
            if self.motion_enabled
            else None
        )

        sensors = cfg["sensors"]
        self.want = {
            "slam_info": bool(sensors["slam_info"].get("enabled", True)),
            "slam_key_info": bool(sensors["slam_key_info"].get("enabled", True)),
            "global_map": bool(sensors["global_map"].get("enabled", False)),
        }

        self._lock = threading.Lock()
        # 统计：key -> [count, last_seen, 到达间隔 EMA]
        self._stats: dict[str, list[float]] = {}
        self._cloud_msg: dict[str, Any] = {}     # key -> 最新原始消息（KEEP_LAST(1)）
        self._cloud_gen: dict[str, int] = {}     # key -> 收到帧序号
        self._cloud_ts: dict[str, float] = {}    # key -> 该帧的**到达时刻**（墙钟，打时间戳用）
        self._cloud_hdr: dict[str, float] = {}   # key -> 该帧机器人 header.stamp（帧首，机器人时钟）
        self._cloud_field_warned: dict[str, bool] = {}  # 字段缺失提示去重（只报一次）
        # 时序诊断（EMA）：把两路数据流的“主机墙钟 − 机器人时钟”差出来，机器人时钟
        # 的绝对偏差（实测 263 s）会在相减时抵消，剩下的就是**点云与 IMU 的相对滞后**
        # —— 它正是 Point-LIO 的 common.time_lag_imu_to_lidar 要补的量。
        self._lag_cloud: float | None = None     # 点云 ts − 机器人帧首戳（含点云侧链路/解码延迟）
        self._lag_imu: float | None = None       # IMU 主机戳 − 机器人 IMU 戳（含 IMU 侧链路延迟）
        # 点云输出计数（解码后真正交给 ROS 节点的帧数）：与上面的 DDS 到达率对比，
        # 差值就是桥内丢帧（同一时刻只留最新一帧是设计行为，但丢太多就要看下面这两处）。
        self._cloud_out: dict[str, int] = {}     # key -> 累计输出帧数
        self._cloud_out_prev: dict[str, int] = {}  # 上一报周期末值（算差值用）
        self._active_cloud = self.cloud_source if self.cloud_source != "auto" else self.cloud_order[0]
        self._active_imu = None

        self._small_q: deque = deque(maxlen=2000)   # IMU/slam/status/log
        self._cloud_q: deque = deque(maxlen=8)      # 点云（大帧，只保留最近几帧）
        self._q_lock = threading.Lock()
        self._q_event = threading.Event()
        # 解码线程的唤醒信号：点云到达就置位（原来是 30 ms 轮询，实测会白白丢掉
        # 约 13% 的帧 —— DDS 9.4 Hz 进来、只有 8.2 Hz 发得出去，见 README「点云丢帧」）。
        self._cloud_event = threading.Event()

        self._last_mode: dict[str, Any] | None = None   # rt/lowstate 的 mode/tick 快照
        self._last_bms: dict[str, Any] | None = None    # rt/bms_state 快照
        self._last_sport: dict[str, Any] | None = None  # rt/sportmodestate 快照（只读）
        self._sport_last_emit = 0.0                     # 输出限频用
        self._sport_prev_code: int | None = None        # 状态机跳变检测

        self._subs: list[Any] = []
        self._cloud_channels: dict[str, Any] = {}
        self._stop = threading.Event()
        self._sock: socket.socket | None = None
        self._cmd_thread: threading.Thread | None = None

    # ------------------------------------------------------------- 生命周期
    def run(self) -> int:
        if not os.path.isdir(f"/sys/class/net/{self.iface}"):
            log(f"网卡不存在: {self.iface}（用 `ip -br addr` 找 192.168.123.x 那张网卡）")
            return 2
        try:
            self._subscribe_all()
        except Exception as exc:  # noqa: BLE001 —— 启动失败要带解释退出
            log(f"DDS 订阅初始化失败: {exc}")
            return 2

        threads = [
            threading.Thread(target=self._decode_loop, name="cloud_decode", daemon=True),
            threading.Thread(target=self._source_watchdog, name="source_watchdog", daemon=True),
            threading.Thread(target=self._status_loop, name="status", daemon=True),
        ]
        for t in threads:
            t.start()

        if self.motion is not None:
            try:
                self.motion.start()
            except Exception as exc:  # noqa: BLE001 —— 建 RPC 客户端失败不该拖掉整条只读链路
                log(f"运动通道启动失败（后续不再下发控制指令）: {exc}")
                self.motion = None

        log(
            f"已启动: iface={self.iface} 点云={self._cloud_desc()} IMU={self.imu_source if self.imu_enabled else '关闭'} "
            f"→ tcp://127.0.0.1:{self.port}"
        )
        try:
            self._serve()
        except TimeoutError as exc:
            log(str(exc))
            return 3
        except KeyboardInterrupt:
            pass
        finally:
            self.close()
        return 0

    def close(self) -> None:
        self._stop.set()
        self._q_event.set()
        # 先停运动（StopMove 要走 DDS，不能等 socket 关了又关完 DDS 才做）
        if self.motion is not None:
            with contextlib.suppress(Exception):
                self.motion.close()
            self.motion = None
        if self._sock is not None:
            with contextlib.suppress(Exception):
                self._sock.close()
        for channel in list(self._cloud_channels.values()):
            with contextlib.suppress(Exception):
                channel.CloseReader()
        self._cloud_channels.clear()
        for sub in self._subs:
            with contextlib.suppress(Exception):
                sub.Close()
        self._subs.clear()

    def _cloud_desc(self) -> str:
        if not self.cloud_enabled:
            return "关闭"
        if self.cloud_mode == "multi":
            desc = "multi(" + ",".join(self.cfg["pointcloud"]["multi_topics"]) + ")"
        else:
            desc = self._active_cloud
        flags = ["丢零点" if self.cloud_filter_zero else "含零点"]  # 与 config.describe 对齐
        if self.cloud_min_range > 0:
            flags.append(f">={self.cloud_min_range:g}m")
        if self.cloud_max_range > 0:
            flags.append(f"<={self.cloud_max_range:g}m")
        if self.cloud_voxel > 0:
            flags.append(f"voxel={self.cloud_voxel:g}")
        return f"{desc},{','.join(flags)}"

    # --------------------------------------------------------------- 订阅
    def _subscribe_all(self) -> None:
        # unitree_sdk2py/cyclonedds 只存在于采集器专用 3.10 venv（PYTHONPATH 运行时注入），
        # 静态分析解析不到属预期 —— 见 pyrightconfig.json extraPaths 与 README「为什么两个进程」
        from unitree_sdk2py.core.channel import (  # pyright: ignore[reportMissingImports]
            ChannelFactoryInitialize,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.nav_msgs.msg.dds_ import OccupancyGrid_  # pyright: ignore[reportMissingImports]
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_  # pyright: ignore[reportMissingImports]
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_  # pyright: ignore[reportMissingImports]
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (  # pyright: ignore[reportMissingImports]
            BmsState_,
            LowState_,
        )

        from a2w_bridge.imu_idl import Imu_

        ChannelFactoryInitialize(0, self.iface)

        def add(topic: str, msg_type: Any, handler) -> None:
            sub = ChannelSubscriber(topic, msg_type)
            sub.Init(handler, 10)
            self._subs.append(sub)

        if self.want["slam_info"]:
            add(TOPIC_SLAM_INFO, String_, self._mk_slam("slam_info"))
        if self.want["slam_key_info"]:
            add(TOPIC_SLAM_KEY_INFO, String_, self._mk_slam("slam_key_info"))
        if self.want["global_map"]:
            add(TOPIC_RELOCATION_GLOBAL_MAP, OccupancyGrid_, self._on_grid)

        if self.imu_enabled or self.joints_enabled:
            wanted = list(IMU_FAILOVER_ORDER) if self.imu_source in ("auto", "all") else [self.imu_source]
            if self.joints_enabled and "lowstate" not in wanted:
                wanted.append("lowstate")  # 关节只从 lowstate 出，与 IMU 选源解耦
            if "lidar_front" in wanted:
                add(IMU_TOPICS["lidar_front"], Imu_, self._mk_lidar_imu("lidar_front"))
            if "lidar_rear" in wanted:
                add(IMU_TOPICS["lidar_rear"], Imu_, self._mk_lidar_imu("lidar_rear"))
            if "lowstate" in wanted:
                add(IMU_TOPICS["lowstate"], LowState_, self._on_lowstate)
            if self.imu_enabled:
                self._active_imu = (
                    IMU_FAILOVER_ORDER[0] if self.imu_source in ("auto", "all") else self.imu_source
                )

        if self.battery_enabled:
            # ⚠️ 默认关（battery.enabled=false）：实测订阅 rt/bms_state 会让
            # CycloneDDS 0.10.2 ~25s 后 SIGSEGV（ddsi_xt_type_init_impl invalid type
            # object），固件侧 XTypes 类型不兼容。SDK/固件更新后再开。
            add(self.battery_dds_topic, BmsState_, self._on_bms)

        if self.sport_enabled:
            # 只读：运控状态机（1001=阻尼/软急停），不下发任何指令
            add(self.sport_dds_topic, SportModeState_, self._on_sport)

        if self.cloud_enabled:
            if self.cloud_mode == "multi":
                for key in self.cfg["pointcloud"]["multi_topics"]:
                    self._open_cloud(key)
            else:
                self._open_cloud(self._active_cloud)

    def _open_cloud(self, key: str) -> None:
        """订阅一个点云话题（single 模式先关旧 reader，切换由调用方保证）。"""
        from cyclonedds.qos import Policy, Qos  # pyright: ignore[reportMissingImports]
        from unitree_sdk2py.core.channel import (  # pyright: ignore[reportMissingImports]
            ChannelFactory,
        )
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_  # pyright: ignore[reportMissingImports]

        # BEST_EFFORT + KEEP_LAST(1)：不触发重传、不积压历史帧
        qos = Qos(
            Policy.Reliability.BestEffort,
            Policy.History.KeepLast(1),
            Policy.Durability.Volatile,
        )
        topic = CLOUD_TOPICS[key]
        channel = ChannelFactory().CreateChannel(topic, PointCloud2_)
        channel.SetReader(qos, self._mk_cloud(key), 10)
        with self._lock:
            self._cloud_channels[key] = channel
        log(f"订阅点云 {key} → {topic}")

    def _close_cloud(self, key: str) -> None:
        channel = self._cloud_channels.pop(key, None)
        if channel is not None:
            with contextlib.suppress(Exception):
                channel.CloseReader()

    # --------------------------------------------------------------- 回调
    def _touch(self, key: str) -> float:
        """更新统计（调用方需持锁）；返回当前墙钟时间。"""
        now = time.time()
        entry = self._stats.get(key)
        if entry is None:
            self._stats[key] = [1.0, now, 0.0]
        else:
            entry[0] += 1.0
            if now > entry[1]:
                dt = now - entry[1]
                entry[2] = entry[2] + 0.2 * (dt - entry[2]) if entry[2] > 0 else dt
            entry[1] = now
        return now

    def _mk_slam(self, key: str):
        def handler(msg: Any) -> None:
            with self._lock:
                now = self._touch(key)
            self._enqueue_small({"t": "slam", "k": key, "ts": now, "data": as_text(msg.data)})

        return handler

    def _on_grid(self, msg: Any) -> None:
        with self._lock:
            now = self._touch("global_map")
        try:
            data = np.asarray(list(msg.data), dtype=np.uint8).tobytes()
        except Exception:  # noqa: BLE001
            data = bytes(msg.data)
        info = msg.info
        origin = info.origin
        self._enqueue_small(
            {
                "t": "grid",
                "k": "global_map",
                "ts": now,
                "frame_id": str(getattr(msg.header, "frame_id", "")),
                "resolution": safe_float(info.resolution),
                "width": safe_int(info.width),
                "height": safe_int(info.height),
                "origin_position": [
                    safe_float(origin.position.x),
                    safe_float(origin.position.y),
                    safe_float(origin.position.z),
                ],
                "origin_orientation": [
                    safe_float(origin.orientation.x),
                    safe_float(origin.orientation.y),
                    safe_float(origin.orientation.z),
                    safe_float(origin.orientation.w),
                ],
            },
            payload=data,
        )

    def _mk_lidar_imu(self, key: str):
        def handler(msg: Any) -> None:
            with self._lock:
                now = self._touch(key)
                active = self._active_imu
                # 时序诊断：IMU 侧“主机戳 − 机器人戳”（雷达 IMU 的 header.stamp 是设备时间）
                stamp = safe_float(getattr(msg.header.stamp, "sec", 0)) + \
                    safe_float(getattr(msg.header.stamp, "nanosec", 0)) * 1e-9
                if stamp > 0:
                    lag = now - stamp
                    self._lag_imu = lag if self._lag_imu is None else 0.9 * self._lag_imu + 0.1 * lag
            if self.imu_source not in ("all",) and key != active:
                return
            self._enqueue_small(
                {
                    "t": "imu",
                    "k": key,
                    "ts": now,
                    "frame_id": str(getattr(msg.header, "frame_id", "")),
                    "quat": [
                        safe_float(msg.orientation.x),
                        safe_float(msg.orientation.y),
                        safe_float(msg.orientation.z),
                        safe_float(msg.orientation.w),
                    ],
                    "gyro": [
                        safe_float(msg.angular_velocity.x),
                        safe_float(msg.angular_velocity.y),
                        safe_float(msg.angular_velocity.z),
                    ],
                    "accel": [
                        safe_float(msg.linear_acceleration.x),
                        safe_float(msg.linear_acceleration.y),
                        safe_float(msg.linear_acceleration.z),
                    ],
                    "orientation_covariance": f32_list(msg.orientation_covariance),
                    "angular_velocity_covariance": f32_list(msg.angular_velocity_covariance),
                    "linear_acceleration_covariance": f32_list(msg.linear_acceleration_covariance),
                }
            )

        return handler

    def _on_lowstate(self, msg: Any) -> None:
        with self._lock:
            now = self._touch("lowstate")
            active = self._active_imu
            self._last_mode = {
                "ts": now,
                "mode_machine": safe_int(msg.mode_machine),
                "mode_pr": safe_int(msg.mode_pr),
                "tick": safe_int(msg.tick),
            }

        # --- 关节状态（A2W：16 关节 = 腿序×4，随 lowstate 节奏，与 IMU 选源无关) ---
        if self.joints_enabled:
            q: list[float] = []
            dq: list[float] = []
            tau: list[float] = []
            for i in self.joint_indexes:
                try:
                    m = msg.motor_state[i]
                except (IndexError, TypeError):
                    m = None
                q.append(safe_float(getattr(m, "q", None)))
                dq.append(safe_float(getattr(m, "dq", None)))
                tau.append(safe_float(getattr(m, "tau_est", None)))
            self._enqueue_small(
                {
                    "t": "joints",
                    "k": "lowstate",
                    "ts": now,
                    "tick": safe_int(msg.tick),
                    "q": q,
                    "dq": dq,
                    "tau": tau,
                }
            )

        # --- IMU（受选源 gate：只有 lowstate 是当前源时才发）---
        if not self.imu_enabled:
            return
        if self.imu_source != "all" and "lowstate" != active:
            return
        imu = msg.imu_state
        quat = f32_list(imu.quaternion)  # 顺序 w, x, y, z（unitree 约定）
        if len(quat) != 4:
            quat = [1.0, 0.0, 0.0, 0.0]
        w, x, y, z = quat
        self._enqueue_small(
            {
                "t": "imu",
                "k": "lowstate",
                "ts": now,
                "frame_id": "",          # 由配置的 imu.frame_id 决定
                "quat": [x, y, z, w],    # 统一成 ROS2 的 x, y, z, w
                "gyro": f32_list(imu.gyroscope),
                "accel": f32_list(imu.accelerometer),
                "rpy": f32_list(imu.rpy),
                "temperature": safe_int(imu.temperature),
            }
        )

    def _on_bms(self, msg: Any) -> None:
        """``rt/bms_state``（unitree_hg/BmsState_，电压 mV / 电流 mA）→ 电池帧。"""
        with self._lock:
            now = self._touch("bms")
        vol_mv = f32_list(msg.bmsvoltage)
        cur_ma = safe_float(msg.current)
        soc = safe_int(msg.soc)
        soh = safe_int(msg.soh)
        temps = f32_list(msg.temperature)
        cell = f32_list(msg.cell_vol)
        flags = int_list(msg.bmsstate)
        battery = {
            "ts": now,
            "voltage": round(vol_mv[0] / 1000.0, 3) if vol_mv else 0.0,
            "current": round(cur_ma / 1000.0, 3),
            "soc": soc,
            "soh": soh,
            "temperature": [round(t, 2) for t in temps[:12]],
            "cell_vol": [round(v / 1000.0, 3) for v in cell[:40]],
        }
        with self._lock:
            self._last_bms = dict(battery)
        battery["t"] = "battery"
        battery["k"] = "bms"
        self._enqueue_small(battery)

    def _sport_snapshot(self) -> tuple[int, float] | None:
        """给运动通道状态门用的运控快照：``(error_code, 采样时刻)``；从未收到则 None。"""
        with self._lock:
            ls = self._last_sport
            if not ls:
                return None
        try:
            return int(ls["error_code"]), float(ls["ts"])
        except (KeyError, TypeError, ValueError):  # noqa: BLE001 —— 快照不完整就按无状态处理
            return None

    def _on_sport(self, msg: Any) -> None:
        """``rt/sportmodestate`` → 运控状态帧（**只读**：仅订阅，绝不发控制指令）。

        ``error_code`` 就是运动状态机（官方《运控服务接口 V2.0》）：
        **1001 = 阻尼**（遥控器 L2+B 的软急停，`Damp()` 备注"最高优先级，用于突发情况下的急停"）。
        """
        with self._lock:
            now = self._touch("sport")
            last_emit = self._sport_last_emit
            prev = self._sport_prev_code
        code = safe_int(msg.error_code)
        state = {
            "error_code": code,
            "name": sport_state_name(code),
            "mode": safe_int(msg.mode),
            "gait_type": safe_int(msg.gait_type),
            "progress": round(safe_float(msg.progress), 3),
            "body_height": round(safe_float(msg.body_height), 3),
            "velocity": [round(safe_float(v), 3) for v in f32_list(msg.velocity)[:3]],
            "position": [round(safe_float(v), 3) for v in f32_list(msg.position)[:3]],
            "yaw_speed": round(safe_float(msg.yaw_speed), 3),
        }
        with self._lock:
            self._last_sport = dict(state, ts=now)
            self._sport_prev_code = code
            emit = (now - last_emit) >= (1.0 / self.sport_rate_hz)
            if emit:
                self._sport_last_emit = now

        if code != prev:  # 状态机跳变（含首次）：无缓冲直接进日志
            level = "warn" if code == SPORT_STATE_DAMPING else "info"
            self._enqueue_small(
                {"t": "log", "level": level, "text": f"运控状态: {state['name']}（error_code={code}）"}
            )
        if emit:
            self._enqueue_small({"t": "sport", "k": "sportmodestate", "ts": now, **state})

    def _mk_cloud(self, key: str):
        def handler(msg: Any) -> None:
            with self._lock:
                now = time.time()
                self._cloud_msg[key] = msg             # 只存引用，解码在解码线程
                self._cloud_ts[key] = now              # 到达时刻（墙钟）：解码后可能晚几十 ms
                self._cloud_hdr[key] = safe_float(getattr(msg.header.stamp, "sec", 0)) + \
                    safe_float(getattr(msg.header.stamp, "nanosec", 0)) * 1e-9
                self._cloud_gen[key] = self._cloud_gen.get(key, 0) + 1
                self._touch(f"cloud:{key}")
            self._cloud_event.set()   # 叫醒解码线程（别等轮询）

        return handler

    # --------------------------------------------------------------- 队列
    def _enqueue_small(self, header: dict[str, Any], payload: bytes = b"") -> None:
        with self._q_lock:
            self._small_q.append((header, payload))
        self._q_event.set()

    def _enqueue_cloud(self, header: dict[str, Any], payload: bytes) -> None:
        with self._q_lock:
            self._cloud_q.append((header, payload))
        with self._lock:
            key = str(header.get("k", ""))
            self._cloud_out[key] = self._cloud_out.get(key, 0) + 1
        self._q_event.set()

    # --------------------------------------------------------- 解码 / 看门狗
    def _decode_loop(self) -> None:
        last_gen: dict[str, int] = {}
        while not self._stop.is_set():
            # 事件唤醒（不是轮询）：帧一到就解码，只剩“新帧比解码快”这一种合理丢弃
            self._cloud_event.wait(0.05)
            self._cloud_event.clear()
            with self._lock:
                keys = list(self._cloud_msg)
                jobs = []
                for key in keys:
                    gen = self._cloud_gen.get(key, 0)
                    if gen != last_gen.get(key):
                        last_gen[key] = gen
                        jobs.append((key, gen, self._cloud_msg[key]))
            for key, gen, msg in jobs:
                with self._lock:
                    ts = self._cloud_ts.get(key)
                try:
                    header, payload = self._decode_cloud(key, msg, ts)
                except Exception as exc:  # noqa: BLE001 —— 单帧解码失败不该拖垮采集
                    log(f"点云解码失败({key}): {exc}")
                    continue
                # 解码期间又来了新帧：这张旧帧直接丢弃（KEEP_LAST(1) 语义）
                if self._cloud_gen.get(key, 0) != gen:
                    continue
                self._enqueue_cloud(header, payload)

    def _decode_cloud(self, key: str, msg: Any, ts: float | None = None) -> tuple[dict[str, Any], bytes]:
        t0 = time.time()

        def warn(text: str) -> None:
            if not self._cloud_field_warned.get(text):
                self._cloud_field_warned[text] = True
                log(f"点云 {key}: {text}")

        points, names, dtypes = cloud_to_array(msg, self.cloud_fields, warn=warn)
        try:  # 同 n_pts：只用于日志/统计，转不动就当 0
            n_raw = int(points.shape[0])
        except (TypeError, ValueError):
            n_raw = 0

        # ── 时间戳：把「到达时刻」换成「帧首时刻」─────────────────────────────
        # 为什么：下游（Point-LIO）把 header.stamp 当**帧首**用（lidar_end_time =
        # 帧首 + 帧内跨度），我们若打到达时刻，LIO 的内建时钟就比物理时间晚整整
        # 一帧（还多等一帧的 IMU），姿态戳会跑到“未来”。
        # 点云自带逐点绝对时间（timestamp 列，实测单调、跨度 99.8 ms），所以
        #   帧首 = 到达时刻 − (最后一个有效点时刻 − 机器人 header.stamp)
        # 没有 timestamp 列就退回到达时刻（旧行为）。
        ts_out = ts if ts is not None else time.time()
        hdr_start = self._cloud_hdr.get(key, 0.0)
        frame_span = 0.0
        if "timestamp" in names and points.shape[0] > 1:
            try:  # 时间列异常（NaN/inf）就退回到达时刻，不能因为打戳把整帧丢掉
                tcol = points["timestamp"].astype(np.float64)
                t_last = float(np.max(tcol))
                span = (t_last - hdr_start) if hdr_start > 0 else (t_last - float(np.min(tcol)))
                if np.isfinite(span) and 0.0 < span < 1.0:  # 只信合理量级（帧长 ≤ 1 s）
                    ts_out -= span
                    frame_span = span
            except (TypeError, ValueError):
                frame_span = 0.0

        if self.cloud_filter_zero:
            points = drop_zero_points(points)
        points = crop_range(points, self.cloud_max_range, self.cloud_min_range)
        points = voxel_downsample(points, self.cloud_voxel)
        if self.cloud_max_points > 0 and points.shape[0] > self.cloud_max_points:
            idx = np.random.choice(points.shape[0], size=self.cloud_max_points, replace=False)
            idx.sort()
            points = points[idx]
        # 结构化数组本身就是紧凑小端布局，直接 tobytes = point_step×点数的点表
        payload = np.ascontiguousarray(points).tobytes()
        point_step = points.dtype.itemsize
        try:  # numpy shape 恒为整数；防御性转换（JSON 序列化需要 Python int）
            n_pts = int(points.shape[0])
        except (TypeError, ValueError):
            n_pts = 0
        header = {
            "t": "cloud",
            "k": key,
            # 见上面：有逐点时间就取**帧首**，否则退回到达时刻
            "ts": ts_out,
            "fields": names,
            "field_dtypes": dtypes,   # 与 fields 一一对应的 numpy 类型（node.py 照它发 PointField）
            "point_step": point_step,
            "points": n_pts,
            "points_raw": n_raw,
            "frame_span_ms": round(frame_span * 1000.0, 3),
            "decode_ms": round((time.time() - t0) * 1000.0, 2),
            "stamp_lag_ms": round((time.time() - ts_out) * 1000.0, 2),
            "hdr_stamp": [
                safe_int(getattr(msg.header.stamp, "sec", 0)),
                safe_int(getattr(msg.header.stamp, "nanosec", 0)),
            ],
            "hdr_frame_id": str(getattr(msg.header, "frame_id", "")),
        }
        if hdr_start > 0:  # 时序诊断：点云侧链路/解码滞后（机器人时钟的绝对偏差会与 IMU 相减抵消）
            lag = ts_out - hdr_start
            with self._lock:
                self._lag_cloud = lag if self._lag_cloud is None else 0.9 * self._lag_cloud + 0.1 * lag
        return header, payload

    def _source_watchdog(self) -> None:
        while not self._stop.wait(1.0):
            now = time.time()
            with self._lock:
                stats = {k: v[:] for k, v in self._stats.items()}

            # --- 点云自动轮换（single + auto）
            if self.cloud_enabled and self.cloud_mode == "single" and self.cloud_source == "auto":
                entry = stats.get(f"cloud:{self._active_cloud}")
                age = now - entry[1] if entry else 1e9
                if age > self.cloud_stale:
                    idx = self.cloud_order.index(self._active_cloud)
                    nxt = self.cloud_order[(idx + 1) % len(self.cloud_order)]
                    if self._switch_cloud(nxt):
                        # 给新话题 stale_sec 的观察期，避免连续轮换
                        with self._lock:
                            self._stats[f"cloud:{nxt}"] = [0.0, now, 0.0]

            # --- IMU 自动降级（auto）：优先 lidar_front → lidar_rear → lowstate
            if self.imu_enabled and self.imu_source == "auto":
                for candidate in IMU_FAILOVER_ORDER:
                    entry = stats.get(candidate)
                    if entry and now - entry[1] <= self.imu_stale:
                        with self._lock:
                            changed = self._active_imu != candidate
                            self._active_imu = candidate
                        if changed:
                            log(f"IMU 源切换到 {candidate}（{IMU_TOPICS[candidate]}）")
                        break

    def _switch_cloud(self, key: str) -> bool:
        """先关旧订阅再建新的（同时挂两个话题会成倍放大链路流量）。"""
        if key == self._active_cloud:
            return False
        log(f"点云源 {self._active_cloud} 无数据，切换到 {key}（{CLOUD_TOPICS[key]}）")
        try:
            self._close_cloud(self._active_cloud)
            self._open_cloud(key)
        except Exception as exc:  # noqa: BLE001
            log(f"切换点云话题失败: {exc}")
            return False
        with self._lock:
            self._active_cloud = key
            self._cloud_msg.pop(key, None)
        self._enqueue_small({"t": "log", "level": "warn", "text": f"点云源切换为 {key}"})
        return True

    # ---------------------------------------------------------------- 发送
    def _status_loop(self) -> None:
        """周期发布『机器人状态』帧——不再报桥自身健康。

        帧里只有机器人本体数据：machine 模式 / lowstate tick / 关节新鲜度 / 电池。
        桥自身健康（点云源、IMU 源、各话题实测频率/计数、队列深度）只进 stderr 日志
        （由 ROS2 节点转发进 ROS 日志），不上话题。
        """
        period = self.cfg["publish_status_period_sec"]  # 已在 config.load_config 规范化
        while not self._stop.wait(period):
            now = time.time()
            with self._lock:
                lm = dict(self._last_mode) if self._last_mode else None
                lb = dict(self._last_bms) if self._last_bms else None
                ls = dict(self._last_sport) if self._last_sport else None
                stats = {k: v[:] for k, v in self._stats.items()}

            frame: dict[str, Any] = {"t": "status", "ts": now}
            if lm:
                frame["mode_machine"] = lm["mode_machine"]
                frame["mode_pr"] = lm["mode_pr"]
                frame["tick"] = lm["tick"]
            if self.joints_enabled:
                frame["joints"] = {
                    "count": len(self.joint_indexes),
                    # 距上一帧 rt/lowstate 的秒数（-1 = 从未收到 → 低电平服务可能没开）
                    "fresh_sec": round(max(now - lm["ts"], 0.0), 3) if lm else -1.0,
                }
            else:
                frame["joints"] = None
            if self.sport_enabled and ls:
                frame["sport"] = {
                    "error_code": ls["error_code"],
                    "name": ls["name"],
                    "fresh_sec": round(max(now - ls["ts"], 0.0), 3),
                }
            else:
                frame["sport"] = None
            frame["battery"] = lb
            frame["motion"] = self.motion.status() if self.motion is not None else None

            self._enqueue_small(frame)

            # --- 桥自身健康：只进日志 ------------------------------------------------
            rates = {
                k: round(1.0 / v[2], 2)
                for k, v in stats.items()
                if v[2] > 0 and now - v[1] < period * 2
            }
            jf = frame["joints"]["fresh_sec"] if frame["joints"] else -1.0
            jd = "无数据" if jf < 0 else f"{jf}s"
            bat = f"{lb['voltage']}V/{lb['soc']}%" if lb else "无数据"
            sp = sport_frame["name"] if isinstance((sport_frame := frame.get("sport")), dict) else "无数据"
            with self._lock:
                lc, li = self._lag_cloud, self._lag_imu
            # 时序诊断：点云与 IMU 的**相对**滞后（机器人时钟的绝对偏差相减抵消）。
            # 它就是 Point-LIO 的 common.time_lag_imu_to_lidar 要补的量（负值 = 把 IMU 往后挪）。
            if lc is not None and li is not None:
                timing = f" 点云−IMU滞后差={1000.0 * (lc - li):+.0f}ms（LIO: time_lag={-(lc - li):+.3f}）"
            else:
                timing = ""
            # 点云输出帧率（解码后真发给 ROS 的）vs 上面的 DDS 到达率：差值 = 桥内丢帧
            cloud_out = ""
            if self.cloud_enabled:
                with self._lock:
                    cur = dict(self._cloud_out)
                parts = [
                    f"{k}={round((cnt - self._cloud_out_prev.get(k, 0)) / period, 2)}"
                    for k, cnt in cur.items()
                ]
                self._cloud_out_prev = dict(cur)
                if parts:
                    cloud_out = " 云输出Hz[" + " ".join(parts) + "]"
            mot = frame.get("motion")
            if isinstance(mot, dict):
                if mot.get("blocked"):
                    mline = f"⚠️motion 被拦({mot['blocked']})"
                elif mot.get("moving"):
                    t = mot.get("target") or [0, 0, 0]
                    mline = (
                        f"motion {'试运行' if mot.get('dry_run') else '下发'} "
                        f"vx={t[0]:.2f} vy={t[1]:.2f} vyaw={t[2]:.2f} "
                        f"(帧龄 {mot.get('age_sec')}s, 共 {mot.get('sent')} 次, 失败 {mot.get('errors')})"
                    )
                else:
                    mline = f"motion 待命(共 {mot.get('sent')} 次, 失败 {mot.get('errors')})"
            else:
                mline = "motion 关"
            log(
                f"状态: 运控={sp} mode_machine={frame.get('mode_machine')} 关节={jd} 电池={bat} | "
                f"{mline} | "
                f"点云源={self._active_cloud if self.cloud_enabled else 'off'} "
                f"IMU源={self._active_imu if self.imu_enabled else 'off'}{timing}{cloud_out} | {rates}"
            )

    # ------------------------------------------------------------ 控制指令（下行）
    def _cmd_loop(self) -> None:
        """读 ROS2 节点发来的 ``cmd`` 帧（唯一的下行数据）：move = 速度目标，stop = 立刻刹车。

        这个线程只把目标写给 MotionController（不阻塞、不做 RPC），真正的下发由运动线程
        按 ``motion.rate_hz`` 做——这样 RPC 抖动/超时不会把帧读取堵住（堵住就等于
        看门狗失效，会很危险）。
        """
        assert self._sock is not None
        reader = protocol.FrameReader(self._sock)
        dropped_warn_ts = 0.0
        while not self._stop.is_set():
            try:
                header, _payload = reader.read_frame()
            except (ConnectionError, OSError):
                return  # 连接断了：_serve 会退出，close() 里统一 StopMove
            if str(header.get("t", "")) != "cmd":
                continue
            if self.motion is None:  # 通道启动失败：丢弃但限流告警
                now = time.time()
                if now - dropped_warn_ts >= 5.0:
                    dropped_warn_ts = now
                    log("收到控制指令，但运动通道未启动（启动时失败）→ 已丢弃")
                continue
            kind = str(header.get("k", ""))
            if kind == "move":
                self.motion.submit(
                    header.get("vx", 0.0), header.get("vy", 0.0), header.get("vyaw", 0.0)
                )
            elif kind == "stop":
                self.motion.request_stop(str(header.get("reason", "上游要求停止")))

    def _connect(self) -> None:
        deadline = time.time() + self.cfg["collector"]["connect_timeout_sec"]
        while not self._stop.is_set():
            try:
                sock = socket.create_connection(("127.0.0.1", self.port), timeout=5.0)
                sock.settimeout(None)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self._sock = sock
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8 << 20)
                log(f"已连接 ROS2 节点 127.0.0.1:{self.port}")
                return
            except OSError:
                if time.time() > deadline:
                    raise TimeoutError(
                        f"连接 127.0.0.1:{self.port} 超时（{self.cfg['collector']['connect_timeout_sec']}s）"
                    ) from None
                self._stop.wait(0.5)

    def _serve(self) -> None:
        self._connect()
        assert self._sock is not None
        # 同一条 TCP 连接上的反方向：采集器只写不读，控制指令（cmd 帧）另起线程读。
        # 即使运动通道启动失败（motion=None）也要读——不读就会让指令在 socket 缓冲里积压。
        if self.motion_enabled:
            self._cmd_thread = threading.Thread(target=self._cmd_loop, name="cmd_in", daemon=True)
            self._cmd_thread.start()
        while not self._stop.is_set():
            frame = None
            with self._q_lock:
                if self._cloud_q:
                    frame = self._cloud_q.popleft()
                elif self._small_q:
                    frame = self._small_q.popleft()
            if frame is None:
                self._q_event.wait(0.05)
                self._q_event.clear()
                continue
            header, payload = frame
            try:
                self._sock.sendall(protocol.pack(header, payload))
            except OSError as exc:
                log(f"连接断开（{exc}），退出")
                return


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="A2W DDS → 本机 TCP 采集器（Python 3.10 环境）")
    parser.add_argument("--config", required=True, help="a2w_bridge.json 路径")
    parser.add_argument("--port", type=int, default=None, help="覆盖配置里的 TCP 端口")
    parser.add_argument("--iface", default=None, help="覆盖配置里的网卡")
    parser.add_argument("--print-config", action="store_true", help="打印生效配置后退出")
    args = parser.parse_args(argv)

    try:
        cfg = load_config(args.config)
    except ConfigError as exc:
        log(str(exc))
        return 2
    if args.iface:
        cfg["iface"] = args.iface
    if args.print_config:
        print(json.dumps({k: v for k, v in cfg.items() if not k.startswith("_")}, indent=2, ensure_ascii=False))
        return 0

    def _sig(_signum, _frame):
        raise KeyboardInterrupt

    for sig in (signal.SIGINT, signal.SIGTERM):
        with contextlib.suppress(ValueError):
            signal.signal(sig, _sig)

    collector = Collector(cfg, port=args.port)
    return collector.run()


if __name__ == "__main__":
    sys.exit(main())
