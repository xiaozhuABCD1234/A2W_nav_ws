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


def log(text: str) -> None:
    """写 stderr（由 ROS2 节点转发进 ROS 日志）。"""
    print(f"[collector] {text}", file=sys.stderr, flush=True)


# ---------------------------------------------------------------------------
# 点云解析 / 滤波
# ---------------------------------------------------------------------------
def cloud_to_array(msg: Any, fields: list[str]) -> tuple[np.ndarray, list[str]]:
    """``sensor_msgs/PointCloud2`` -> ``(N, K)`` float32 数组（按 fields 顺序取列）。"""
    by_name = {f.name: f for f in msg.fields}
    names = [n for n in fields if n in by_name]
    for axis in ("x", "y", "z"):
        if axis not in names:
            raise ValueError(f"点云缺少字段 {axis!r}（现有: {list(by_name)}）")

    try:
        point_step = int(msg.point_step)
        width = int(msg.width)
        height = int(msg.height)
        row_step = int(msg.row_step)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"点云字段 point_step/width/height/row_step 非法: {exc}") from None
    total = width * height
    if total == 0 or point_step == 0:
        return np.zeros((0, len(names)), dtype=np.float32), names

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
    out = np.empty((arr.shape[0], len(names)), dtype=np.float32)
    for i, name in enumerate(names):
        field = by_name[name]
        try:
            dt, size = _FIELD_DTYPE[int(field.datatype)]
            offset = int(field.offset)
        except (KeyError, TypeError, ValueError):
            raise ValueError(
                f"字段 {name!r} 的 datatype/offset 非法: {field.datatype!r}@{field.offset!r}"
            ) from None
        out[:, i] = arr[:, offset : offset + size].copy().view(endian + dt).reshape(-1).astype(np.float32)

    if out.size:
        out = out[np.isfinite(out).all(axis=1)]
    return out, names


def crop_range(points: np.ndarray, max_range: float) -> np.ndarray:
    if max_range <= 0 or points.shape[0] == 0:
        return points
    return points[np.linalg.norm(points[:, :3], axis=1) <= max_range]


def voxel_downsample(points: np.ndarray, voxel: float) -> np.ndarray:
    """体素下采样（每个体素取一个代表点），纯 numpy：三维体素索引打包成 int64 后 1D unique。"""
    if voxel <= 0 or points.shape[0] == 0:
        return points
    keys = np.floor(points[:, :3] / voxel).astype(np.int64)
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
        self._active_cloud = self.cloud_source if self.cloud_source != "auto" else self.cloud_order[0]
        self._active_imu = None

        self._small_q: deque = deque(maxlen=2000)   # IMU/slam/status/log
        self._cloud_q: deque = deque(maxlen=8)      # 点云（大帧，只保留最近几帧）
        self._q_lock = threading.Lock()
        self._q_event = threading.Event()

        self._last_mode: dict[str, Any] | None = None   # rt/lowstate 的 mode/tick 快照
        self._last_bms: dict[str, Any] | None = None    # rt/bms_state 快照
        self._last_sport: dict[str, Any] | None = None  # rt/sportmodestate 快照（只读）
        self._sport_last_emit = 0.0                     # 输出限频用
        self._sport_prev_code: int | None = None        # 状态机跳变检测

        self._subs: list[Any] = []
        self._cloud_channels: dict[str, Any] = {}
        self._stop = threading.Event()
        self._sock: socket.socket | None = None

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
            return "multi(" + ",".join(self.cfg["pointcloud"]["multi_topics"]) + ")"
        return self._active_cloud

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
                self._cloud_msg[key] = msg             # 只存引用，解码在解码线程
                self._cloud_gen[key] = self._cloud_gen.get(key, 0) + 1
                self._touch(f"cloud:{key}")

        return handler

    # --------------------------------------------------------------- 队列
    def _enqueue_small(self, header: dict[str, Any], payload: bytes = b"") -> None:
        with self._q_lock:
            self._small_q.append((header, payload))
        self._q_event.set()

    def _enqueue_cloud(self, header: dict[str, Any], payload: bytes) -> None:
        with self._q_lock:
            self._cloud_q.append((header, payload))
        self._q_event.set()

    # --------------------------------------------------------- 解码 / 看门狗
    def _decode_loop(self) -> None:
        last_gen: dict[str, int] = {}
        while not self._stop.wait(0.03):
            with self._lock:
                keys = list(self._cloud_msg)
                jobs = []
                for key in keys:
                    gen = self._cloud_gen.get(key, 0)
                    if gen != last_gen.get(key):
                        last_gen[key] = gen
                        jobs.append((key, gen, self._cloud_msg[key]))
            for key, gen, msg in jobs:
                try:
                    header, payload = self._decode_cloud(key, msg)
                except Exception as exc:  # noqa: BLE001 —— 单帧解码失败不该拖垮采集
                    log(f"点云解码失败({key}): {exc}")
                    continue
                # 解码期间又来了新帧：这张旧帧直接丢弃（KEEP_LAST(1) 语义）
                if self._cloud_gen.get(key, 0) != gen:
                    continue
                self._enqueue_cloud(header, payload)

    def _decode_cloud(self, key: str, msg: Any) -> tuple[dict[str, Any], bytes]:
        t0 = time.time()
        points, names = cloud_to_array(msg, self.cloud_fields)
        points = crop_range(points, self.cloud_max_range)
        points = voxel_downsample(points, self.cloud_voxel)
        if self.cloud_max_points > 0 and points.shape[0] > self.cloud_max_points:
            idx = np.random.choice(points.shape[0], size=self.cloud_max_points, replace=False)
            idx.sort()
            points = points[idx]
        payload = np.ascontiguousarray(points, dtype="<f4").tobytes()
        try:  # numpy shape 恒为整数；防御性转换（JSON 序列化需要 Python int）
            n_pts = int(points.shape[0])
        except (TypeError, ValueError):
            n_pts = 0
        header = {
            "t": "cloud",
            "k": key,
            "ts": time.time(),
            "fields": names,
            "points": n_pts,
            "decode_ms": round((time.time() - t0) * 1000.0, 2),
            "hdr_stamp": [
                safe_int(getattr(msg.header.stamp, "sec", 0)),
                safe_int(getattr(msg.header.stamp, "nanosec", 0)),
            ],
            "hdr_frame_id": str(getattr(msg.header, "frame_id", "")),
        }
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
            log(
                f"状态: 运控={sp} mode_machine={frame.get('mode_machine')} 关节={jd} 电池={bat} | "
                f"点云源={self._active_cloud if self.cloud_enabled else 'off'} "
                f"IMU源={self._active_imu if self.imu_enabled else 'off'} | {rates}"
            )

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
