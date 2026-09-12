#!/usr/bin/env python3
"""发布动态 ``base_footprint`` TF 与 2D 足迹（Nav2 定位/代价地图的坐标系基础）。

为什么需要这个节点
==================

A2W 是轮足狗，``base_link`` 离地高度**随姿态变**（实测：待机停放 ~0.10 m，
站立行走更高；四轮接地点在 base_link 系里的 x/y 也随之变）。``base_footprint``
是 REP-105 里 ``map → odom → base_footprint → base_link`` 链中的“地面投影坐标
系”，要求它**始终贴在当前地面的正下方** —— 所以不能像普通底盘那样写死在
URDF 里（静态 z 会错），必须按 TF 实时量离地高度再发动态变换。

输入（靠 TF 树，不订阅话题）：

- ``base_link → *_Link4``（URDF + 关节角，由 a2w_joint_display 的
  robot_state_publisher 提供）
- 桥的静态外参 TF（base_link → 雷达/IMU，本节点用不到，但保证树的完整性）

输出：

- TF：``base_footprint → base_link``，z = 实测离地高度（base_footprint 贴地）
- 话题 ``a2w/footprint``（geometry_msgs/PolygonStamped，base_footprint 系，
  逆时针 4 点矩形）= **2D 足迹**：
  - x 方向：机身 mesh 实测边界（base_link.STL bbox 前后悬空都算，保守）
  - y 方向：**实时**四轮接地点外侧 + 胎宽 + 余量（姿态变了足迹自动跟着变）

用法::

    # 与关节显示一起（默认已带 footprint:=true）：
    ros2 launch a2w_bridge a2w_joint_display.launch.py
    # 单独跑：
    ros2 run a2w_bridge a2w_base_footprint
    # 只打印一次当前足迹 + Nav2 footprint 参数片段（无 TF 时输出 URDF 零位参考值）：
    ros2 run a2w_bridge a2w_base_footprint --once --print-nav2

足迹几何来源（2026-09 由 a2w_description/urdf 与 meshes/*.STL 实测量得）：

- 机身 base_link.STL bbox：x ∈ [−0.328, +0.387]（长 0.715 m）、y ∈ ±0.143
- 轮胎 Link4.STL：半径 0.09486 m（mesh 尺寸 0.1897），轮胎 y 面距轮心 0.074 m
  （胎宽 0.050 m）→ 足迹 y 外扩量 = 轮接地点 |y| + 0.074
- 轮足（零位/直腿）接地点：x=±0.259、y=±0.203 → 零位静态足迹
  x∈[−0.358, +0.417]、y∈±0.307（含 0.03 m 余量），与 ``--print-nav2``
  的参考值一致。**改 URDF/换轮胎后按上面方法重量**，或让节点在 TF 在位时
  自动用实时的四轮接地点（推荐，实际跑导航时就这样）。
"""

import argparse
import math
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from a2w_bridge.base_height import WHEELS, WHEEL_RADIUS

# ---------------------------------------------------------------------------
# 机身/轮胎常数（由 meshes/*.STL bbox 实测，见模块 docstring）
# ---------------------------------------------------------------------------
# 机身前后边界（base_link 系，含前后悬空/保险杠）—— 足迹 x 方向用它（静态）
BODY_X_MIN = -0.328
BODY_X_MAX = 0.387
# 轮胎 y 面到轮心的距离（胎宽 0.050 → 外侧面 0.074）—— 足迹 y 外扩量（静态）
TIRE_HALF_WIDTH = 0.074
# 足迹整体安全余量（m）：腿部往返摆动等动态形变
DEFAULT_INFLATE = 0.03
# 默认话题 / TF 帧名
DEFAULT_TOPIC = "a2w/footprint"
DEFAULT_PARENT = "base_footprint"
# 退出时打印 Nav2 footprint 参数片段的开关
WARN_PERIOD_SEC = 5.0


def _rpy_to_R(r: float, p: float, y: float):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _axis_rot_R(ax, th: float):
    """绕单位轴 ax 旋转 th 的旋转矩阵（Rodrigues）。"""
    x, y, z = ax
    c, s = math.cos(th), math.sin(th)
    C = 1.0 - c
    return [
        [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
    ]


def _mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mat_apply(m, v):
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))


def zero_pose_wheel_contacts(urdf_path: str):
    """URDF 全关节 0° 时四个轮心在 base_link 系的 (x, y, z)。

    返回 dict[link 名] = (x, y, z)；URDF 缺轮子 link 时该项缺失。仅用于
    “没有 TF/机器人在线”时的静态参考值（--once --print-nav2）。
    """
    root = ET.parse(urdf_path).getroot()

    def vec3(tag: ET.Element | None, attr: str = "xyz", default=(0.0, 0.0, 0.0)):
        """URDF 的 xyz/rpy 是 <origin> 的属性（不是子元素）。"""
        if tag is None:
            return default
        try:
            return tuple(float(v) for v in tag.attrib[attr].split())
        except (KeyError, ValueError):
            return default

    tree: dict[str, tuple] = {}
    for j in root.iter("joint"):
        jt = j.attrib["type"]
        child_el = j.find("child")
        parent_el = j.find("parent")
        if child_el is None or parent_el is None:
            continue
        child = child_el.attrib["link"]
        parent = parent_el.attrib["link"]
        origin = j.find("origin")
        xyz = vec3(origin)
        rpy = vec3(origin, "rpy")
        axis = j.find("axis")
        if axis is not None:
            try:
                ax = tuple(float(v) for v in axis.attrib["xyz"].split())
            except (KeyError, ValueError):
                ax = (1.0, 0.0, 0.0)
        else:
            ax = (1.0, 0.0, 0.0)
        tree[child] = (parent, xyz, rpy, ax, jt)

    pos: dict[str, tuple] = {"base_link": (0.0, 0.0, 0.0)}
    rot: dict[str, list] = {"base_link": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}
    pending = [c for c in tree if tree[c][0] == "base_link"]
    while pending:
        child = pending.pop(0)
        parent, xyz, rpy, ax, jt = tree[child]
        if parent not in pos:
            pending.append(child)
            continue
        r = _mat_mul(rot[parent], _rpy_to_R(*rpy))
        if jt != "fixed":
            r = _mat_mul(r, _axis_rot_R(ax, 0.0))
        pos[child] = tuple(pos[parent][i] + sum(rot[parent][i][k] * xyz[k] for k in range(3)) for i in range(3))
        rot[child] = r
        for c2, (p2, *_ ) in tree.items():
            if p2 == child and c2 not in pos:
                pending.append(c2)
    return {w: pos[w] for w in WHEELS if w in pos}


def footprint_rect(body_x_min, body_x_max, max_wheel_y, inflate):
    """逆时针 4 点矩形足迹（base_footprint 系，z=0）。"""
    x0, x1 = body_x_min - inflate, body_x_max + inflate
    y = max_wheel_y + inflate
    return [
        (x0, -y), (x1, -y), (x1, y), (x0, y),
    ]


class BaseFootprint(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("base_footprint")
        self.args = args
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_footprint = self.create_publisher(PolygonStamped, args.topic, 10)
        self.n_warned = 0
        self.n_warned_time = self.get_clock().now()
        self.last_measure = None  # (monotonic, height, max_wheel_y)
        self.last_log_time = None  # 状态行限频用
        self.done = False
        self.create_timer(1.0 / max(args.rate, 0.05), self.tick)

    # ---- 测量：四轮接地点 → 离地高度 + y 方向外沿 ----
    def _measure(self):
        """返回 (base_link 离地高度, 轮接地点最大 |y|)；TF 不全返回 None。"""
        contacts = []
        for w in WHEELS:
            if not self.buffer.can_transform("base_link", w, Time()):
                return None
            tf = self.buffer.lookup_transform("base_link", w, Time())
            q = tf.transform.rotation
            # 轮轴（link 局部 y 轴）在父系 z 上的分量 = R[2][1]，同 base_height
            axis_z = 2.0 * (q.y * q.z + q.w * q.x)
            droop = self.args.wheel_radius * math.sqrt(max(0.0, 1.0 - axis_z ** 2))
            t = tf.transform.translation
            contacts.append((t.x, t.y, t.z - droop))
        ground = min(c[2] for c in contacts)
        max_y = max(abs(c[1]) for c in contacts)
        return -ground, max_y

    def _warn(self, msg: str) -> None:
        now = self.get_clock().now()
        cold = (now - self.n_warned_time).nanoseconds > WARN_PERIOD_SEC * 1e9
        if self.n_warned == 0 or cold:
            self.n_warned += 1
            self.n_warned_time = now
            self.get_logger().warning(msg)

    def tick(self) -> None:
        m = self._measure()
        if m is None:
            self._warn(
                "等 TF（base_link → *_Link4）—— 需要 joint display 的 URDF/关节："
                "ros2 launch a2w_bridge a2w_joint_display.launch.py"
            )
            return
        height, max_wheel_y = m
        now = time.monotonic()
        self.last_measure = (now, height, max_wheel_y)

        # ── 发布 TF：base_footprint → base_link（z = 离地高度）──
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.args.parent
        tf.child_frame_id = "base_link"
        tf.transform.translation.z = height
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf)

        # ── 发布 2D 足迹（base_footprint 系，含机身悬空与胎宽/余量）──
        poly = PolygonStamped()
        poly.header.stamp = tf.header.stamp
        poly.header.frame_id = self.args.parent
        for x, y in footprint_rect(
            self.args.body_x_min, self.args.body_x_max,
            max_wheel_y + self.args.tire_half_width,
            self.args.inflate,
        ):
            p = Point32()
            p.x, p.y, p.z = x, y, 0.0
            poly.polygon.points.append(p)
        self.pub_footprint.publish(poly)

        self._log_status(height, max_wheel_y)

        if self.args.once:
            self.done = True

    def _log_status(self, height: float, max_wheel_y: float) -> None:
        if self.args.once:
            pass  # --once 的结果由 main 打印
        # 限频：默认 2 秒一行
        now = self.get_clock().now()
        if self.last_log_time is not None:
            age = (now - self.last_log_time).nanoseconds / 1e9
            if age < self.args.status_period:
                return
        self.last_log_time = now
        ext = footprint_rect(
            self.args.body_x_min, self.args.body_x_max,
            max_wheel_y + self.args.tire_half_width, self.args.inflate,
        )
        self.get_logger().info(
            f"base_footprint 离地 {height:.3f} m | 足迹 x∈[{ext[0][0]:+.3f},{ext[1][0]:+.3f}] "
            f"y∈±{ext[1][1]:.3f}（轮外沿 |y|={max_wheel_y + self.args.tire_half_width:.3f}）"
        )

    def nav2_yaml(self, height: float | None, max_wheel_y: float | None) -> str:
        """Nav2 params 可直接粘贴的 footprint 片段。"""
        if height is None:
            # 无 TF：用 URDF 零位参考值（轮心 y=±0.203 → 外沿 0.277 → 余量 0.03）
            max_wheel_y = self.args.fallback_wheel_y
        pts = footprint_rect(
            self.args.body_x_min, self.args.body_x_max,
            max_wheel_y + self.args.tire_half_width, self.args.inflate,
        )
        s = ",".join(f"[{x:.3f},{y:.3f}]" for x, y in pts)
        return (
            "# a2w_base_footprint 输出的足迹（base_footprint 系，逆时针，m）\n"
            f"footprint: [{s}]\n"
            "# 参考：inflation_radius 建议 ≥ 足迹半宽 + 0.1，见 a2w_bridge README\n"
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="发布动态 base_footprint TF 与 2D 足迹（Nav2 坐标系基础）"
    )
    parser.add_argument("--rate", type=float, default=10.0, help="发布频率 Hz（默认 10）")
    parser.add_argument("--parent", default=DEFAULT_PARENT, help="TF 父帧（默认 base_footprint）")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="足迹话题（默认 a2w/footprint）")
    parser.add_argument(
        "--body-x-min", type=float, default=BODY_X_MIN,
        help="机身前/后边界（base_link 系 x，默认由 base_link.STL 实测）",
    )
    parser.add_argument("--body-x-max", type=float, default=BODY_X_MAX, help="机身前端边界")
    parser.add_argument(
        "--tire-half-width", type=float, default=TIRE_HALF_WIDTH,
        help="轮胎 y 面到轮心距离（默认 0.074，由 Link4.STL 实测）",
    )
    parser.add_argument(
        "--fallback-wheel-y", type=float, default=0.203,
        help="无 TF 时的零位轮接地点 |y|（默认 0.203，URDF 零位 FK）",
    )
    parser.add_argument("--inflate", type=float, default=DEFAULT_INFLATE, help="足迹安全余量 m（默认 0.03）")
    parser.add_argument("--wheel-radius", type=float, default=WHEEL_RADIUS, help="轮半径 m")
    parser.add_argument("--status-period", type=float, default=2.0, help="状态行最小间隔 s")
    parser.add_argument("--urdf", default="", help="URDF 路径（--print-nav2 无 TF 时用零位 FK 取参考值）")
    parser.add_argument("--once", action="store_true", help="量一次就退出（不等）")
    parser.add_argument(
        "--print-nav2", action="store_true",
        help="与 --once 一起：打印 Nav2 footprint 参数片段（有 TF 用实时值，否则用零位参考）",
    )
    parser.add_argument("--timeout", type=float, default=10.0, help="--once 等 TF 的上限秒数")
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init(args=sys.argv)
    node = BaseFootprint(args)
    try:
        if args.once:
            deadline = time.monotonic() + args.timeout
            while rclpy.ok() and not node.done and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            height, max_y = None, None
            if node.last_measure is not None:
                _, height, max_y = node.last_measure
                node.get_logger().info(
                    f"实测：base_footprint 离地 {height:.3f} m，轮外沿 |y|={max_y + args.tire_half_width:.3f} m"
                )
            else:
                node.get_logger().warning(
                    "超时没拿到 TF —— 输出 URDF 零位参考值（机器人不在线/关节显示没跑）"
                )
            if args.print_nav2:
                # 无 TF 时优先用 URDF 零位 FK 的轮接地点（避免硬编码过期）
                if node.last_measure is None and args.urdf:
                    try:
                        contacts = zero_pose_wheel_contacts(args.urdf)
                        vals = [c for c in contacts.values() if c is not None]
                        if vals:
                            max_y = max(abs(c[1]) for c in vals)
                            node.get_logger().info(
                                f"URDF 零位 FK：轮接地点 |y|={max_y:.3f} m（{len(vals)} 轮）"
                            )
                    except (ET.ParseError, OSError) as e:
                        node.get_logger().warning(f"URDF 解析失败（{e}），用零位参考值")
                print("\n" + node.nav2_yaml(height, max_y))
            sys.exit(0 if node.last_measure is not None else 1)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl-C，退出")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()