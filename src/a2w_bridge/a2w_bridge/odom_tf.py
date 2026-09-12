#!/usr/bin/env python3
"""把 LIO 里程计接进机器人 TF 树：发布 ``camera_init → base_footprint``（REP-105 的 odom 边）。

为什么需要它
============

机器人相关的 TF 现在是**两棵互不相连的子树**：

.. code-block:: text

    # 子树 1：LIO（point_lio 发的，10 Hz）
    camera_init ──▶ body                    # body = 前雷达系（LIO 外参是单位阵）

    # 子树 2：机器人（桥的静态外参 + a2w_base_footprint 的动态高度）
    base_footprint ──▶ base_link ──▶ a2w/lidar ──▶ a2w/lidar_rear
                               ├──▶ a2w/imu
                               └──▶ （URDF 各腿，由 robot_state_publisher 发）

后果：RViz 里机器人和点云/轨迹（``/cloud_registered``、``/path`` 都在 ``camera_init`` 系）
放不进同一棵树；Nav2 也拿不到 REP-105 要求的 ``odom → base_footprint``
（**Nav2 的 odom 就是这里的 camera_init**）。

本节点只补**一条边**，整棵树立刻连通：

.. code-block:: text

    camera_init ──(LIO)──────────▶ body
         │
         └──(本节点)──▶ base_footprint ──▶ base_link ──▶ a2w/lidar ──▶ a2w/lidar_rear
                                                     ├──▶ a2w/imu
                                                     └──▶ （URDF 各腿）

数学
====

``body`` 与 ``a2w/lidar`` 是**同一个物理坐标系** —— 喂给 LIO 的点云与 IMU 都是前雷达那一路
（``pointcloud.frame_id = a2w/lidar``、``imu.source = lidar_front``），LIO 的外参是单位阵。
于是：

.. code-block:: text

    T(camera_init→base_link) = T(camera_init→body) · T(body→base_link)
    T(body→base_link)        = inverse( T(base_link→a2w/lidar) )    # 桥按标定 JSON 发的静态外参

**为什么不能把 body 直接当 base_link 用**：前雷达相对 base_link 有标定外参
``xyz=[0.33767, 0, 0.08134]、rpy=[90°, 0, 90°]`` —— 差一个平移 *加* 一个 90°/90° 的安装旋转。
不补偿的话机器人会平移错 0.34 m、航向也会转错，所以必须做上面这次复合。

压平成 2D（``base_footprint`` 是地面投影系）
============================================

* ``x, y``：base_link 的横纵位置（竖直投影到地面，REP-105 的定义）；
* ``z``：**地面** —— ``z = z_base_link − 离地高``。离地高由 ``a2w_base_footprint`` 发的
  ``base_footprint → base_link`` 实时量出来（狗蹲下/站立都不影响这个关系）；
  拿不到那条边就退回 ``z = 0``（= LIO 原点所在的水平面，比真实地面高约 0.3 m，
  只影响 RViz 里的观感，不影响 Nav2 —— 见 ``--z-mode``）；
* ``roll/pitch`` 归零、只留 ``yaw``：2D 代价地图要的是平面位姿，机身俯仰是噪声。
  （这一步与 ``a2w_base_footprint`` 的约定一致：那条边也是纯 z 偏移、不带旋转，
  所以压平后 ``camera_init → base_footprint → base_link`` 复合回去与 LIO 的位姿
  在 x/y/z 上一致，只丢掉 roll/pitch。）

用法::

    # 与 LIO 链路一起（a2w_lio.launch.py 默认已带 odom_tf:=true）：
    ros2 launch a2w_bridge a2w_lio.launch.py
    # 单独跑：
    ros2 run a2w_bridge a2w_odom_tf
    # 量一次、打印复合过程与自检后就退出（排查 TF 用；无 LIO 时退出码 1）：
    ros2 run a2w_bridge a2w_odom_tf --once

验证（RViz）：Fixed Frame 改 ``base_footprint``，同时能看到机器人和
``/cloud_registered``、``/path`` —— 说明两棵树已经连上。

⚠️ 本节点**只读 TF、只发这一条边**：不订阅话题、不碰机器人 DDS（与桥的只读性、
ROS 域隔离都不冲突）。整条边只应由**一个**实例发布 —— 起两遍会让 TF 抖动。
"""

import argparse
import math
import sys
import time
from typing import Any

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

# ---------------------------------------------------------------------------
# 默认帧名（与全工作区一致，改标定/换雷达只用参数覆盖，不用改代码）
# ---------------------------------------------------------------------------
DEFAULT_ODOM_FRAME = "camera_init"      # point_lio 的 odom_header_frame_id
DEFAULT_BODY_FRAME = "body"             # point_lio 的 odom_child_frame_id（= 前雷达系）
DEFAULT_LIDAR_FRAME = "a2w/lidar"       # 桥的 pointcloud.frame_id（body 的物理同名系）
DEFAULT_ROBOT_FRAME = "base_link"       # 机器人本体根（桥静态外参的父帧）
DEFAULT_FOOTPRINT_FRAME = "base_footprint"  # a2w_base_footprint 发的子帧
# 告警限频（秒）：等 TF 属于常态（LIO 还没起），别刷屏
WARN_PERIOD_SEC = 5.0


# ---------------------------------------------------------------------------
# 小矩阵/四元数工具（标准库，方便单测与 --once 自检）
# ---------------------------------------------------------------------------
def _quat_to_R(qx: float, qy: float, qz: float, qw: float):
    """四元数 (x,y,z,w) → 3×3 旋转矩阵（先归一化，坏值退回单位阵）。"""
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    s = 2.0 / n
    xx, yy, zz = qx * qx * s, qy * qy * s, qz * qz * s
    xy, xz, yz = qx * qy * s, qx * qz * s, qy * qz * s
    wx, wy, wz = qw * qx * s, qw * qy * s, qw * qz * s
    return [
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ]


def _mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mat_vec(m, v):
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))


def _transpose(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def _compose(R1, t1, R2, t2):
    """T1 ∘ T2（都是「父系下的子系位姿」语义：p_父 = R·p_子 + t）。"""
    v = _mat_vec(R1, t2)
    return _mat_mul(R1, R2), (t1[0] + v[0], t1[1] + v[1], t1[2] + v[2])


def _invert(R, t):
    """T 的逆（旋转取转置）。"""
    Rt = _transpose(R)
    v = _mat_vec(Rt, t)
    return Rt, (-v[0], -v[1], -v[2])


def _yaw_of(R) -> float:
    """旋转矩阵的偏航角（ZYX 分解；俯仰接近 ±90° 时 atan2 的退化由调用方忽略）。"""
    return math.atan2(R[1][0], R[0][0])


def _yaw_to_quat(yaw: float):
    """只含偏航的四元数 (x, y, z, w)。"""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _rpy_of(R):
    """旋转矩阵 → (roll, pitch, yaw)，仅 --once 报告用。"""
    sy = max(-1.0, min(1.0, -R[2][0]))
    pitch = math.asin(sy)
    if abs(sy) < 1.0 - 1e-9:
        return math.atan2(R[2][1], R[2][2]), pitch, math.atan2(R[1][0], R[0][0])
    return 0.0, pitch, math.atan2(-R[0][1], R[1][1])


class OdomTf(Node):
    """``camera_init → base_footprint``（odom → base_footprint）发布者。"""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("a2w_odom_tf")
        self.args = args
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.warned: dict[str, float] = {}          # key → 上次告警的 monotonic 时刻
        self.n_pub = 0
        self.last: dict[str, Any] | None = None     # 最近一次的完整结果（--once 报告用）
        self.last_stamp: Time | None = None         # 已发布的 LIO 时间戳（去重）
        self.last_status: Time | None = None
        self.create_timer(1.0 / max(args.rate, 0.05), self.tick)

    # ---------------------------------------------------------------- 读 TF
    def _lookup(self, target: str, source: str):
        """target→source 的 (stamp, R, t)；不在树里返回 None（不抛异常）。"""
        if not self.buffer.can_transform(target, source, Time()):
            return None
        tf = self.buffer.lookup_transform(target, source, Time())
        q = tf.transform.rotation
        tr = tf.transform.translation
        return (
            Time.from_msg(tf.header.stamp),
            _quat_to_R(q.x, q.y, q.z, q.w),
            (tr.x, tr.y, tr.z),
        )

    def _warn_throttled(self, key: str, msg: str) -> None:
        now = time.monotonic()
        last = self.warned.get(key)
        if last is None or now - last >= WARN_PERIOD_SEC:
            self.warned[key] = now
            self.get_logger().warning(msg)

    def _odom_to_body(self):
        """LIO 的 TF；顺带做「断流保护」：太旧就当没有（免得发一个定格假位姿）。"""
        got = self._lookup(self.args.odom_frame, self.args.body_frame)
        if got is None:
            self._warn_throttled(
                "lio",
                f"等 LIO 的 TF {self.args.odom_frame} → {self.args.body_frame} —— "
                "需要 point_lio 在跑：ros2 launch a2w_bridge a2w_lio.launch.py",
            )
            return None
        stamp, R, t = got
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if age > self.args.stale_sec:
            self._warn_throttled(
                "stale",
                f"LIO 的 TF 已停 {age:.1f} s（>{self.args.stale_sec} s）—— 暂停发布，"
                "恢复后自动继续",
            )
            return None
        return stamp, R, t

    def _body_to_robot(self):
        """``body → base_link`` = inverse(``base_link → a2w/lidar``)（body ≡ 前雷达系）。"""
        got = self._lookup(self.args.robot_frame, self.args.lidar_frame)
        if got is None:
            self._warn_throttled(
                "extrinsic",
                f"等静态外参 {self.args.robot_frame} → {self.args.lidar_frame} —— "
                "它由桥按标定 JSON 发布：ros2 launch a2w_bridge a2w_bridge.launch.py",
            )
            return None
        _, R, t = got
        return _invert(R, t)

    def _ground_height(self) -> tuple[float, bool]:
        """离地高 = ``base_footprint → base_link`` 的 z（a2w_base_footprint 实时量的）。

        返回 (高度, 是否是实测值)；拿不到就退回 0（--z-mode zero 时也一样）。
        """
        if self.args.z_mode == "zero":
            return 0.0, False
        got = self._lookup(self.args.footprint_frame, self.args.robot_frame)
        if got is None:
            self._warn_throttled(
                "height",
                f"拿不到 {self.args.footprint_frame} → {self.args.robot_frame}（离地高）——"
                " 暂用 z=0（RViz 里机器人会浮在 LIO 原点那个水平面上，Nav2 无影响）；"
                "要实测高度就一起跑：ros2 launch a2w_bridge a2w_joint_display.launch.py",
            )
            return 0.0, False
        _, R_h, t_h = got
        roll, pitch, _ = _rpy_of(R_h)
        tilt = abs(roll) + abs(pitch)
        if tilt > 1e-6:
            # 约定是纯 z 偏移（不带旋转）；带了旋转说明那条边被别的节点改过，提示一次
            self._warn_throttled(
                "height_rot",
                f"{self.args.footprint_frame} → {self.args.robot_frame} 带旋转"
                f"（roll/pitch≈{tilt:.3f} rad），仍只取 z 当离地高",
            )
        return t_h[2], True

    # ---------------------------------------------------------------- 主循环
    def tick(self) -> None:
        got = self._odom_to_body()
        if got is None:
            return
        stamp, R_ob, t_ob = got
        # 同一个时间戳只发一次：定时器只负责轮询（默认 20 Hz），发布间隔跟 LIO 帧走（≈10 Hz）。
        # 否则同一条时间戳会发两条，而两轮的离地高可能不同 → 同一时刻两个不同的 z。
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        body_to_robot = self._body_to_robot()
        if body_to_robot is None:
            return
        R_br, t_br = body_to_robot

        # T(camera_init → base_link) = T(camera_init→body) ∘ T(body→base_link)
        R, t = _compose(R_ob, t_ob, R_br, t_br)
        yaw = _yaw_of(R)
        ground, measured = self._ground_height()
        z = t[2] - ground if self.args.z_mode == "ground" else 0.0

        # ── 发布：camera_init → base_footprint（时间戳沿用 LIO 那一帧）──
        msg = TransformStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.args.odom_frame
        msg.child_frame_id = self.args.footprint_frame
        msg.transform.translation.x = t[0]
        msg.transform.translation.y = t[1]
        msg.transform.translation.z = z
        msg.transform.rotation.x, msg.transform.rotation.y, \
            msg.transform.rotation.z, msg.transform.rotation.w = _yaw_to_quat(yaw)
        self.tf_broadcaster.sendTransform(msg)
        self.n_pub += 1

        self.last = {
            "stamp": stamp,
            "base_link": (t[0], t[1], t[2]),
            "rpy": _rpy_of(R),
            "footprint": (t[0], t[1], z),
            "yaw": yaw,
            "ground": ground,
            "ground_measured": measured,
            "lidar_xyz": t_br,
        }
        self._log_status()

    def _log_status(self) -> None:
        r = self.last
        if r is None:
            return
        now = self.get_clock().now()
        if self.last_status is not None:
            age = (now - self.last_status).nanoseconds / 1e9
            if age < self.args.status_period:
                return
        self.last_status = now
        tilt = math.degrees(r["rpy"][0]) + math.degrees(r["rpy"][1])
        self.get_logger().info(
            f"odom TF: {self.args.odom_frame} → {self.args.footprint_frame} "
            f"x={r['footprint'][0]:+.3f} y={r['footprint'][1]:+.3f} z={r['footprint'][2]:+.3f} "
            f"yaw={math.degrees(r['yaw']):+.1f}°（离地高 "
            f"{r['ground']:.3f} m{'实测' if r['ground_measured'] else '缺省'}；"
            f"机身俯仰 {tilt:+.1f}° 已压平；已发 {self.n_pub} 条）"
        )

    # ---------------------------------------------------------------- 报告
    def report(self) -> str:
        """--once 的详细输出：复合过程 + 自检（把发布出去的边复合回去对不对得上）。"""
        a = self.args
        if self.last is None:
            return (
                "没拿到完整的 TF，无法计算。需要同时有：\n"
                f"  1) LIO 的 {a.odom_frame} → {a.body_frame}"
                "（point_lio：ros2 launch a2w_bridge a2w_lio.launch.py）\n"
                f"  2) 桥的静态外参 {a.robot_frame} → {a.lidar_frame}"
                "（ros2 launch a2w_bridge a2w_bridge.launch.py）\n"
                f"  3) 可选：{a.footprint_frame} → {a.robot_frame} 离地高"
                "（a2w_joint_display.launch.py 里的 a2w_base_footprint）\n"
            )
        r = self.last
        bx, by, bz = r["base_link"]
        roll, pitch, yaw = r["rpy"]
        # 自检：T(odom→footprint) ∘ T(footprint→base_link) 应该回到 base_link 的 x/y/z
        R_f = [[math.cos(r["yaw"]), -math.sin(r["yaw"]), 0.0],
               [math.sin(r["yaw"]), math.cos(r["yaw"]), 0.0],
               [0.0, 0.0, 1.0]]
        back = _compose(R_f, r["footprint"], [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]],
                        (0.0, 0.0, r["ground"]))[1]
        err = max(abs(back[0] - bx), abs(back[1] - by), abs(back[2] - bz))
        return (
            f"帧：odom={a.odom_frame}  body={a.body_frame}  lidar={a.lidar_frame}  "
            f"robot={a.robot_frame}  footprint={a.footprint_frame}\n"
            f"LIO 位姿（{a.body_frame} 在 {a.odom_frame} 系）→ base_link：\n"
            f"  T(body→base_link) 平移 = ({r['lidar_xyz'][0]:+.5f}, {r['lidar_xyz'][1]:+.5f}, "
            f"{r['lidar_xyz'][2]:+.5f})（= −桥的 base_link→{a.lidar_frame} 平移）\n"
            f"  base_link 位姿 = x={bx:+.4f} y={by:+.4f} z={bz:+.4f} "
            f"rpy=({math.degrees(roll):+.2f}°, {math.degrees(pitch):+.2f}°, {math.degrees(yaw):+.2f}°)\n"
            f"离地高（{a.footprint_frame}→{a.robot_frame} 的 z）= {r['ground']:.4f} m"
            f"{'（实测）' if r['ground_measured'] else '（缺省 0：那条边不在树里）'}\n"
            f"→ 发布 {a.odom_frame} → {a.footprint_frame}："
            f"x={r['footprint'][0]:+.4f} y={r['footprint'][1]:+.4f} z={r['footprint'][2]:+.4f} "
            f"yaw={math.degrees(r['yaw']):+.2f}°（roll/pitch 已归零）\n"
            f"自检：把发布的边复合回去 x/y/z 最大误差 {err:.6f} m"
            f"{'✓' if err < 1e-6 else '✗'}\n"
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="发布 camera_init → base_footprint（把 LIO 里程计接进机器人 TF 树）"
    )
    parser.add_argument("--odom-frame", default=DEFAULT_ODOM_FRAME, help="LIO 的世界系（默认 camera_init）")
    parser.add_argument("--body-frame", default=DEFAULT_BODY_FRAME, help="LIO 的机体系（默认 body）")
    parser.add_argument("--lidar-frame", default=DEFAULT_LIDAR_FRAME, help="桥的点云系（默认 a2w/lidar）")
    parser.add_argument("--robot-frame", default=DEFAULT_ROBOT_FRAME, help="机器人本体根帧（默认 base_link）")
    parser.add_argument(
        "--footprint-frame", default=DEFAULT_FOOTPRINT_FRAME,
        help="要发布的子帧（默认 base_footprint，REP-105）",
    )
    parser.add_argument(
        "--rate", type=float, default=20.0,
        help="轮询频率 Hz（默认 20）；LIO 每来一帧新位姿才发一条",
    )
    parser.add_argument(
        "--stale-sec", type=float, default=1.0,
        help="LIO 的 TF 超过这么久没更新就暂停发布（默认 1.0 s）",
    )
    parser.add_argument(
        "--z-mode", choices=("ground", "zero"), default="ground",
        help="ground=z 取实地地面（用 base_footprint→base_link 的离地高，默认）；"
             "zero=z 恒为 0（= LIO 原点水平面）",
    )
    parser.add_argument("--status-period", type=float, default=5.0, help="状态行最小间隔 s（默认 5）")
    parser.add_argument("--once", action="store_true", help="算一次、打印报告就退出（无 LIO 时退出码 1）")
    parser.add_argument("--timeout", type=float, default=10.0, help="--once 等 TF 的上限秒数")
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init(args=sys.argv)
    node = OdomTf(args)
    try:
        if args.once:
            deadline = time.monotonic() + args.timeout
            while rclpy.ok() and node.last is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            print(node.report())
            sys.exit(0 if node.last is not None else 1)
        node.get_logger().info(
            f"发布 {args.odom_frame} → {args.footprint_frame}（body={args.body_frame} ≡ "
            f"{args.lidar_frame}，z-mode={args.z_mode}）"
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl-C，退出")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
