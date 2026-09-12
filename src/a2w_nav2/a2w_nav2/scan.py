#!/usr/bin/env python3
"""前雷达点云 → 2D ``LaserScan``（干净带）+ 过滤点云（宽带 − 自车体）。

为什么需要这个节点（而不是直接用 pointcloud_to_laserscan）
=========================================================

本机实测（2026-09，A2W 实机，静止）：

* **前雷达只有前向 180°**：``/a2w/points`` 的方位覆盖是 270°→0°→90°，后半圈
  **一个点都没有**（被自家机身挡住 —— 雷达装在机身前端内部）。所以 2D 扫描天生
  只有 180°，导航要按“后方盲区”约束配参数。
* **机身占地面以上 0.014~0.224 m**（``base_link.STL`` z∈[−0.080,+0.130] + 实测
  离地 0.094 m），而雷达在 0.185 m —— **雷达水平面正处在机身高度区间内**，
  所以带内会混进大量自车体/地面回波（r<0.5 m 占 19.8%）：
  实测自车体回波在 **z ≥ +0.30 m（离地）后为 0 点**，天花板在 2.2 m。
* 雷达离地高度**随姿态变**（待机 0.185 m，站起来会更高）→ 高度带必须以
  **实时实测的机身高度 h** 为基准，不能写死绝对值。

因此本节点做三件事，都是 ``pointcloud_to_laserscan`` 做不到的：

1. **以 h 为基准的高度带**：``h`` 从 TF ``base_footprint → base_link`` 实时取
   （也就是 ``a2w_base_footprint`` 节点量的那个离地高度），带 = ``[h+低偏移, h+高偏移]``；
2. **自车体几何掩膜**：过滤云用它去掉机身/腿/轮子（按 footprint 盒 + 高度上限）；
3. **两路输出**：干净带 → ``LaserScan``（AMCL/代价地图），宽带掩膜后 → ``PointCloud2``
   （voxel 层，能看见低于扫描带的障碍）。

数据流::

    /a2w/points (a2w/lidar, 前向 180°)
        │  TF: base_footprint ← a2w/lidar（桥的静态外参 + base_footprint→base_link 动态）
        ├─ 带 [h+0.21, h+1.00]  ──▶ /scan              LaserScan（180°，AMCL + 代价地图）
        └─ 带 [0.10, h+1.00] −自车体盒──▶ /a2w/points_nav PointCloud2（voxel 层用）

用法::

    ros2 run a2w_nav2 a2w_scan
    ros2 run a2w_nav2 a2w_scan --ros-args -p band_low_above_body:=0.25
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

# 实测常数（A2W 实机 2026-09，见模块 docstring 与 a2w_bridge/README.md）
BODY_Z_ABOVE = 0.130       # 机身上表面在 base_link 系里的 z（mesh 实测）
SELF_MASK_Z_MARGIN = 0.05  # 自车体掩膜在机身上表面之上再留的余量
FAR_FROM_SELF = 0.60       # r 超过它就不再可能是自车体（实测自车体回波都在 r<0.6）
WARN_PERIOD_SEC = 5.0


def _as_float(value: Any) -> float | None:
    """尽力把值转成 float；转不了返回 None（不抛异常）。"""
    try:
        return float(value)
    except (TypeError, ValueError, OverflowError):  # noqa: BLE001 —— 防御
        return None


def _num(value: Any, default: float) -> float:
    """防御式取数：坏值（None/字符串/NaN/±Inf）一律给 default，绝不抛异常。

    参数来自命令行/launch/YAML，TF/点云来自网络，一律按不可信输入处理
    （与 a2w_bridge/config.py、motion.py 的 _num 同规矩）。
    """
    num = _as_float(value)
    if num is None or not math.isfinite(num):
        fallback = _as_float(default)
        return fallback if fallback is not None else 0.0
    return num


def _to_int(value: Any, default: int = 0) -> int:
    """防御式取整（同上；计数/束数用）。"""
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):  # noqa: BLE001 —— 防御
        return int(default)


def _flag(value: Any, default: bool) -> bool:
    """防御式取布尔：坏值一律给 default。"""
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        low = value.strip().lower()
        if low in ("true", "1", "yes", "on"):
            return True
        if low in ("false", "0", "no", "off"):
            return False
    return bool(default)


def _text(value: Any, default: str) -> str:
    """防御式取字符串：空值一律给 default。"""
    try:
        s = str(value).strip()
    except (TypeError, ValueError):
        return default
    return s or default


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def cloud_to_columns(msg: PointCloud2) -> dict[str, np.ndarray]:
    """把 PointCloud2 解成 {字段名: 一维数组}（按 offset 取，不假设 point_step）。

    只解本节点要用的列：x/y/z（f4）+（有的话）intensity。
    """
    offs = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    if n == 0 or msg.point_step < 12:
        return {}
    raw = np.frombuffer(msg.data, dtype=np.uint8, count=n * msg.point_step).reshape(n, msg.point_step)
    out: dict[str, np.ndarray] = {}
    for name in ("x", "y", "z", "intensity"):
        if name in offs and offs[name] + 4 <= msg.point_step:
            o = offs[name]
            out[name] = raw[:, o:o + 4].copy().view(np.float32).reshape(-1).astype(np.float64)
    return out


class A2WScan(Node):
    def _par(self, name: str, default: Any) -> Any:
        """取一个参数的值（声明即带默认值，launch/YAML/命令行都能覆盖）。"""
        return self.declare_parameter(name, default).value

    def __init__(self) -> None:
        super().__init__("a2w_scan")

        # ── 参数（默认值 = 实机实测推荐值；见 config/a2w_scan.yaml 的注释）──
        self.cloud_topic = _text(self._par("cloud_topic", "/a2w/points"), "/a2w/points")
        self.scan_topic = _text(self._par("scan_topic", "scan"), "scan")
        self.cloud_out_topic = _text(self._par("cloud_out_topic", "a2w/points_nav"), "a2w/points_nav")
        self.publish_cloud_enabled = _flag(self._par("publish_cloud", True), True)
        self.target_frame = _text(self._par("target_frame", "base_footprint"), "base_footprint")
        self.body_frame = _text(self._par("body_frame", "base_link"), "base_link")
        # 扫描带（相对实测机身高度 h 的偏移）
        self.band_low = _num(self._par("band_low_above_body", 0.21), 0.21)     # 实测 h+0.21 以上无自车体回波
        self.band_high = _num(self._par("band_high_above_body", 1.00), 1.00)   # 天花板 2.2 m，留足余量
        # 过滤云（离地绝对高度 + 机身以上偏移）
        self.cloud_min_height = _num(self._par("cloud_min_height", 0.10), 0.10)  # 高于地面 0.10 m：丢地面
        self.cloud_high = _num(self._par("cloud_high_above_body", 1.00), 1.00)
        # 自车体掩膜盒（base_footprint 系；默认 = a2w_base_footprint 实测足迹）
        self.mask_x_min = _num(self._par("self_mask_x_min", -0.358), -0.358)
        self.mask_x_max = _num(self._par("self_mask_x_max", 0.417), 0.417)
        self.mask_y = _num(self._par("self_mask_y", 0.341), 0.341)
        self.mask_z_above_body = _num(self._par("self_mask_z_above_body", BODY_Z_ABOVE), BODY_Z_ABOVE)
        self.mask_margin = _num(self._par("self_mask_margin", SELF_MASK_Z_MARGIN), SELF_MASK_Z_MARGIN)
        # 扫描几何
        self.angle_min = _num(self._par("angle_min", -math.pi / 2), -math.pi / 2)
        self.angle_max = _num(self._par("angle_max", math.pi / 2), math.pi / 2)
        self.angle_increment = _num(self._par("angle_increment", 0.5 * math.pi / 180.0), 0.5 * math.pi / 180.0)
        self.range_min = _num(self._par("range_min", 0.25), 0.25)
        self.range_max = _num(self._par("range_max", 12.0), 12.0)
        self.status_period = _num(self._par("status_period", 5.0), 5.0)

        if self.angle_max <= self.angle_min:
            raise ValueError(f"angle_max({self.angle_max}) 必须大于 angle_min({self.angle_min})")
        if self.angle_increment <= 0.0:
            raise ValueError("angle_increment 必须为正")
        if self.range_max <= self.range_min:
            raise ValueError("range_max 必须大于 range_min")

        # ── TF ──
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # ── 收发 ──
        qos_in = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        qos_out = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub_scan = self.create_publisher(LaserScan, self.scan_topic, qos_out)
        self.pub_cloud = (
            self.create_publisher(PointCloud2, self.cloud_out_topic, qos_out)
            if self.publish_cloud_enabled else None
        )
        self.sub = self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, qos_in)

        # ── 统计/诊断 ──
        self.n_frames = 0
        self.n_skipped = 0
        self.last_warn = 0.0
        self.acc = {"in": 0, "band": 0, "beams": 0, "cloud": 0, "h": []}

        self.get_logger().info(
            f"扫描带 = [h+{self.band_low:.2f}, h+{self.band_high:.2f}] m（h=实测机身离地高）| "
            f"角度 [{math.degrees(self.angle_min):+.0f}°,{math.degrees(self.angle_max):+.0f}°] "
            f"@{math.degrees(self.angle_increment):.2f}° → {self.n_beams} 束 | "
            f"量程 [{self.range_min},{self.range_max}] m | 输出 {self.scan_topic}"
            + (f" + {self.cloud_out_topic}（自车体盒 x∈[{self.mask_x_min:.3f},{self.mask_x_max:.3f}] "
               f"y∈±{self.mask_y:.3f}）" if self.pub_cloud is not None else "（过滤云关）")
        )

    @property
    def n_beams(self) -> int:
        # 参数在 __init__ 里已校验（increment>0、max>min），这里仍按防御式取整
        return max(1, _to_int(round((self.angle_max - self.angle_min) / self.angle_increment), 1) + 1)

    # ------------------------------------------------------------------
    def _tf_lookup(self, target: str, source: str):
        """按“最新可用”取 TF（本链路里的边都是近静态或高频，用最新值最稳）。

        返回 (xyz, R) 或 None。
        """
        try:
            if not self.buffer.can_transform(target, source, Time()):
                return None
            tf = self.buffer.lookup_transform(target, source, Time())
        except Exception as e:  # noqa: BLE001 —— tf2 的异常类型不定，统一当“拿不到”
            self.get_logger().debug(f"TF {target} ← {source} 查询失败: {e}", throttle_duration_sec=5.0)
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), quat_to_matrix(q.x, q.y, q.z, q.w)

    def _warn(self, msg: str) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_warn > WARN_PERIOD_SEC:
            self.last_warn = now
            self.get_logger().warning(msg)

    def on_cloud(self, msg: PointCloud2) -> None:
        src = msg.header.frame_id or "a2w/lidar"

        # 1) 点云 → base_footprint
        tr = self._tf_lookup(self.target_frame, src)
        if tr is None:
            self.n_skipped += 1
            self._warn(f"等 TF {self.target_frame} ← {src}（要 a2w_base_footprint/odom_tf + 桥）")
            return
        xyz0, rot = tr
        cols = cloud_to_columns(msg)
        if not {"x", "y", "z"} <= cols.keys():
            self.n_skipped += 1
            self._warn(f"点云缺 x/y/z 字段（实到 {sorted(cols)}）")
            return
        pts = np.stack([cols["x"], cols["y"], cols["z"]], axis=1)
        ok = np.isfinite(pts).all(axis=1) & ~(pts == 0).all(axis=1)
        pts = pts[ok] @ rot.T + xyz0
        # intensity 按同一个 ok 掩码对齐，后面再按各自的带内掩码取（别把两个掩码混用）
        inten = cols["intensity"][ok] if "intensity" in cols else None

        # 2) 机身离地高度 h（= TF base_footprint → base_link 的 z）
        bl = self._tf_lookup(self.target_frame, self.body_frame)
        if bl is None:
            self.n_skipped += 1
            self._warn(f"等 TF {self.target_frame} ← {self.body_frame}（要 a2w_base_footprint 节点）")
            return
        h = _num(bl[0][2], 0.0)

        # 3) 干净带 → 扫描
        z = pts[:, 2]
        r = np.hypot(pts[:, 0], pts[:, 1])
        m_scan = (z >= h + self.band_low) & (z <= h + self.band_high)
        m_scan &= (r >= self.range_min) & (r <= self.range_max)
        self.publish_scan(pts[m_scan], msg.header.stamp)

        # 4) 宽带 − 自车体盒 → 过滤云（voxel 层用）
        if self.pub_cloud is not None:
            m_cloud = (z >= self.cloud_min_height) & (z <= h + self.cloud_high)
            inside = (
                (pts[:, 0] >= self.mask_x_min - self.mask_margin)
                & (pts[:, 0] <= self.mask_x_max + self.mask_margin)
                & (np.abs(pts[:, 1]) <= self.mask_y + self.mask_margin)
                & (z <= h + self.mask_z_above_body + self.mask_margin)
            )
            # 自车体只可能在近处：远处点即便落在盒内也保留（避免挡住贴着车身的真障碍）
            m_cloud &= ~(inside & (r <= FAR_FROM_SELF))
            self.publish_cloud_msg(
                pts[m_cloud],
                inten[m_cloud] if inten is not None else None,
                msg.header.stamp,
            )
            self.acc["cloud"] += _to_int(m_cloud.sum())

        self.n_frames += 1
        self.acc["in"] += _to_int(len(pts))
        self.acc["band"] += _to_int(m_scan.sum())
        self.acc["h"].append(h)
        self.log_status()

    # ------------------------------------------------------------------
    def publish_scan(self, pts: np.ndarray, stamp: Any) -> None:
        n = self.n_beams
        ranges = np.full(n, np.inf, dtype=np.float32)
        if len(pts):
            r = np.hypot(pts[:, 0], pts[:, 1])
            th = np.arctan2(pts[:, 1], pts[:, 0])
            idx = np.floor((th - self.angle_min) / self.angle_increment).astype(np.int64)
            keep = (idx >= 0) & (idx < n)
            idx, r = idx[keep], r[keep]
            if len(idx):
                # 同一束里取最近（挡在前面的物体优先）
                np.minimum.at(ranges, idx, r.astype(np.float32))
        msg = LaserScan()
        msg.header = Header(stamp=stamp, frame_id=self.target_frame)
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges.tolist()
        self.pub_scan.publish(msg)
        self.acc["beams"] += _to_int(np.isfinite(ranges).sum())

    def publish_cloud_msg(self, pts: np.ndarray, inten: np.ndarray | None, stamp: Any) -> None:
        pub = self.pub_cloud
        if pub is None:  # publish_cloud=false 时不会走到这里，兜底防御
            return
        n = len(pts)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        buf = [pts.astype(np.float32)]
        if inten is not None:
            fields.append(PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1))
            buf.append(inten.astype(np.float32).reshape(-1, 1))
        arr = np.hstack(buf)
        msg = PointCloud2()
        msg.header = Header(stamp=stamp, frame_id=self.target_frame)
        msg.height = 1
        msg.width = n
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = arr.shape[1] * 4
        msg.row_step = msg.point_step * n
        msg.is_dense = False
        msg.data = arr.tobytes()
        pub.publish(msg)

    # ------------------------------------------------------------------
    def log_status(self) -> None:
        every = max(1, _to_int(self.status_period * 8, 1)) if self.status_period > 0 else 0
        if not every or self.n_frames % every != 0:
            return
        n = max(1, self.n_frames)
        hs = self.acc["h"][-50:]
        h_mean = _as_float(np.mean(hs)) if hs else None
        h_txt = f"{h_mean:.3f}" if h_mean is not None else "nan"
        self.get_logger().info(
            f"已出 {self.n_frames} 帧（跳过 {self.n_skipped}）| 平均 输入 {self.acc['in']/n:.0f} 点 → "
            f"扫描带 {self.acc['band']/n:.0f} 点 / {self.acc['beams']/n:.0f} 束有回波 "
            f"(共 {self.n_beams} 束, {self.acc['beams']/n/self.n_beams*100:.0f}%) | "
            f"过滤云 {self.acc['cloud']/n:.0f} 点 | h={h_txt} m → 带="
            + (f"[{h_mean + self.band_low:.2f},{h_mean + self.band_high:.2f}] m" if h_mean is not None else "?")
        )


def main() -> None:
    rclpy.init()
    node = A2WScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
