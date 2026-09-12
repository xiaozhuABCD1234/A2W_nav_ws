#!/usr/bin/env python3
"""按 TF 量坐标系离地高度（默认 `base_link`）。

原理：URDF 里四个轮子的 Mesh 最低点 = 轮心 − 半径（半径 0.09486 m，本轮于 link 原点、
绕 link 的 y 轴转）。四轮最低点在 `base_link` 系里的 z 就是地面（平地假设），
取最低的那个反号即 `base_link` 离地高度；其它坐标系（雷达/IMU）再做一次 TF 换算即可。

依赖 TF：`base_link → *_Link4`（URDF + 关节角，由 joint display 发）+ 桥的静态外参 TF。

用法：
  ros2 run a2w_bridge a2w_base_height                     # 每秒打印一次 base_link 离地高度
  ros2 run a2w_bridge a2w_base_height --frame a2w/lidar    # 量雷达离地高度
  ros2 run a2w_bridge a2w_base_height --once               # 只打一行，便于脚本取值
  ros2 run a2w_bridge a2w_base_height --ground-frame a2w/ground
        # 另发一条动态 TF base_link → a2w/ground（z = −离地高度），
        # RViz 里把 Fixed Frame 设成 a2w/ground，机器人就会「站」在网格地面上
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

# 四个轮子的 link（URDF: a2w_description/urdf/a2w_description.urdf）
WHEELS = ("left_front_Link4", "left_hind_Link4", "right_front_Link4", "right_hind_Link4")
# 轮子 mesh 半径（meshes/*_Link4.STL 的 bbox: z ∈ [−0.09486, +0.09486]）
WHEEL_RADIUS = 0.09486
# 提醒“还没等到 TF”的最小间隔
WARN_PERIOD_SEC = 5.0


def _quat_local_y_z(q) -> float:
    """轮轴（link 的 y 轴）在父坐标系里的 z 分量 = R[2][1]。"""
    return 2.0 * (q.y * q.z + q.w * q.x)


class BaseHeight(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("base_height")
        self.args = args
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self) if args.ground_frame else None
        self.n_warned = 0
        self.n_warned_time = self.get_clock().now()
        self.done = False
        self.create_timer(1.0 / max(args.rate, 0.05), self.tick)

    # ---- 单个轮子的最低点在 base_link 系里的 z（查不到就 None）----
    def _wheel_lowest_z(self, link: str) -> float | None:
        if not self.buffer.can_transform("base_link", link, Time()):
            return None
        tf = self.buffer.lookup_transform("base_link", link, Time())
        axis_z = _quat_local_y_z(tf.transform.rotation)
        # 轮子是圆柱：轴水平时最低点 = 轮心 − r；轴有倾角时径向在 z 上的投影为 r·√(1−axis_z²)
        droop = self.args.wheel_radius * math.sqrt(max(0.0, 1.0 - axis_z**2))
        return tf.transform.translation.z - droop

    # ---- 任意坐标系相对 base_link 的 z 偏移（外参来自桥的静态 TF）----
    def _frame_offset_z(self, frame: str) -> float | None:
        if frame == "base_link":
            return 0.0
        if not self.buffer.can_transform("base_link", frame, Time()):
            return None
        return self.buffer.lookup_transform("base_link", frame, Time()).transform.translation.z

    def _warn_waiting_tf(self) -> None:
        # 节流提醒：except/布尔逻辑别写在一起，这里单独一个方法
        now = self.get_clock().now()
        cold = (now - self.n_warned_time).nanoseconds > WARN_PERIOD_SEC * 1e9
        if self.n_warned == 0 or cold:
            self.n_warned += 1
            self.n_warned_time = now
            self.get_logger().warning(
                "等 TF（base_link → *_Link4）—— 需要先跑起 URDF/关节："
                "ros2 launch a2w_bridge a2w_joint_display.launch.py"
            )

    def tick(self) -> None:
        heels = [(w, self._wheel_lowest_z(w)) for w in WHEELS]
        contacts = [z for _, z in heels if z is not None]
        if len(contacts) != len(WHEELS):
            missing = ", ".join(w for w, z in heels if z is None)
            self._warn_waiting_tf()
            if self.n_warned == 1:
                self.get_logger().warning(f"缺这些轮子：{missing}")
            return

        ground = min(contacts)  # 地面在 base_link 系里的 z（取最低轮 → 平地/单轮悬空都能用）
        base_h = -ground
        offset = self._frame_offset_z(self.args.frame)
        height_txt = "?（该坐标系不在 TF 里）" if offset is None else f"{base_h + offset:+.3f} m"

        spread = max(contacts) - min(contacts)
        line = (
            f"{self.args.frame} 离地 {height_txt}"
            f" | base_link 离地 {base_h:.3f} m"
            f" | 四轮接地点 z=[{' '.join(f'{c:+.3f}' for c in contacts)}]"
        )
        if spread > 0.02:
            line += f" ⚠️ 四轮差 {spread * 1000:.0f} mm（不在平地或某轮没落地）"
        self.get_logger().info(line)

        if self.broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = "base_link"
            tf.child_frame_id = self.args.ground_frame
            tf.transform.translation.z = -base_h
            tf.transform.rotation.w = 1.0
            self.broadcaster.sendTransform(tf)

        if self.args.once:
            self.done = True


def main() -> None:
    parser = argparse.ArgumentParser(description="按 TF 量坐标系离地高度（默认 base_link）")
    parser.add_argument("--frame", default="base_link", help="要量的坐标系（默认 base_link）")
    parser.add_argument("--rate", type=float, default=1.0, help="打印频率 Hz（默认 1）")
    parser.add_argument("--wheel-radius", type=float, default=WHEEL_RADIUS, help="轮子半径 m")
    parser.add_argument("--once", action="store_true", help="只打印一行就退出")
    parser.add_argument("--timeout", type=float, default=10.0, help="--once 模式下等 TF 的上限秒数")
    parser.add_argument("--ground-frame", default="", help="另发一条 base_link→该坐标系的动态 TF（地面）")
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init(args=sys.argv)
    node = BaseHeight(args)
    try:
        if args.once:
            # 单次模式：手动 pump 到拿到一次结果（或超时）
            deadline = time.monotonic() + args.timeout
            while rclpy.ok() and not node.done and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not node.done:
                node.get_logger().error(f"{args.timeout:.0f} s 内没拿到 TF（关节/URDF 没跑？）")
                sys.exit(1)
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
