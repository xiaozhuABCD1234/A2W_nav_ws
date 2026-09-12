"""只读关节转发：``/a2w/joint_states``（SDK 命名）→ ``/joint_states``（URDF 命名）。

数据流（全程只读，不发任何控制指令）::

    rt/lowstate ──采集器(只 subscribe)──▶ /a2w/joint_states ──本节点──▶ /joint_states
                                                                       └─▶ robot_state_publisher ─▶ /tf ─▶ RViz

为什么需要这一个节点：

* 桥发布的关节名是 SDK 命名（``FR_hip`` / ``FL_wheel`` …），而 ``a2w_description``
  的 URDF 里叫 ``right_front_joint1`` / ``left_hind_joint4`` …，映射表见
  ``dds_topics.A2W_URDF_JOINT_NAMES``；
* 桥跟随 ``rt/lowstate`` 以 ~1.1 kHz 发布，``robot_state_publisher`` 不需要这么快，
  这里按 ``display.rate_hz``（默认 50 Hz）限频；超过 ``display.stale_sec`` 没有新数据
  就停止发布，免得 RViz 里定格一个假姿态；
* 万一某个关节与 URDF 转向相反，把 SDK 关节名加进 ``display.flip`` 反号即可，
  不用改 URDF、也不用改代码。

用法::

    ros2 launch a2w_bridge a2w_joint_display.launch.py        # 推荐（含 RViz）
    ros2 run a2w_bridge joint_relay --ros-args -p rate_hz:=30.0 -p flip:="FR_thigh,FL_thigh"
"""

from __future__ import annotations

import sys
import threading
import time
from typing import Any, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from .config import ConfigError, default_config_path, load_config
from .dds_topics import A2W_JOINT_NAMES, A2W_URDF_JOINT_NAMES


def _f(value: Any) -> float:
    """任何东西 → float（NaN/None/字符串都不抛异常）。"""
    try:
        out = float(value)
    except (TypeError, ValueError):
        return 0.0
    return out if out == out else 0.0  # NaN → 0.0


def _at(seq: Sequence[Any], index: int) -> Any:
    try:
        return seq[index]
    except (IndexError, TypeError):
        return 0.0


def name_list(value: Any) -> list[str]:
    """把参数规范成关节名列表：支持字符串数组，也支持 ``"A,B"`` 这种逗号/空格分隔字符串。"""
    if value is None:
        return []
    if isinstance(value, str):
        return [x for x in value.replace(",", " ").split() if x]
    try:
        return [str(x).strip() for x in value if str(x).strip()]
    except TypeError:
        return []


class JointRelay(Node):
    """SDK 命名关节角 → URDF 命名关节角（只转发，不下发控制）。"""

    def __init__(self) -> None:
        super().__init__("a2w_joint_relay")

        self.declare_parameter("config", "")
        self.declare_parameter("input_topic", "")
        self.declare_parameter("output_topic", "")
        self.declare_parameter("rate_hz", 0.0)
        self.declare_parameter("stale_sec", 0.0)
        self.declare_parameter("frame_id", "")
        # flip 用逗号分隔字符串：JSON 里是数组，launch/命令行传的是字符串，字符串是两者公分母
        self.declare_parameter("flip", "")

        cfg_path = self.get_parameter("config").value or default_config_path()
        try:
            cfg = load_config(cfg_path)
        except ConfigError as exc:
            self.get_logger().fatal(str(exc))
            raise SystemExit(2) from exc
        disp = cfg["display"]

        # 参数优先级：命令行/launch 覆盖 > JSON 配置 > 内置默认
        self.input_topic = str(self.get_parameter("input_topic").value or disp["input_topic"])
        self.output_topic = str(self.get_parameter("output_topic").value or disp["output_topic"])
        self.rate_hz = _f(self.get_parameter("rate_hz").value) or _f(disp["rate_hz"]) or 50.0
        self.stale_sec = _f(self.get_parameter("stale_sec").value) or _f(disp["stale_sec"]) or 2.0
        self.frame_id = str(self.get_parameter("frame_id").value or disp.get("frame_id", ""))

        self.flip = set(name_list(self.get_parameter("flip").value) or name_list(disp.get("flip")))
        unknown = sorted(self.flip - set(A2W_JOINT_NAMES))
        if unknown:
            self.get_logger().warning(
                f"display.flip 里有不认识的关节名 {unknown}；可用名: {', '.join(A2W_JOINT_NAMES)}"
            )
            self.flip -= set(unknown)

        self._latest: JointState | None = None
        self._latest_mono = 0.0
        self._lock = threading.Lock()
        self._unmapped_warned: set[str] = set()
        self._published = False
        self._stale_warned = False

        sub_qos = QoSProfile(  # BEST_EFFORT 订阅能同时匹配 reliable / best_effort 的发布者
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._sub = self.create_subscription(JointState, self.input_topic, self._on_joints, sub_qos)
        self._pub = self.create_publisher(JointState, self.output_topic, 10)
        self._timer = self.create_timer(1.0 / self.rate_hz, self._on_timer)

        mapped = sum(1 for n in A2W_JOINT_NAMES if n in A2W_URDF_JOINT_NAMES)
        self.get_logger().info(
            f"关节转发就绪: {self.input_topic} → {self.output_topic} @ {self.rate_hz:g} Hz，"
            f"映射 {mapped}/{len(A2W_JOINT_NAMES)} 个关节（{len(A2W_URDF_JOINT_NAMES)} 项对照表）"
        )
        if self.flip:
            self.get_logger().info(f"已反号的关节: {', '.join(sorted(self.flip))}")
        self.get_logger().info(
            f"数据超过 {self.stale_sec:g} s 没更新就暂停发布；没数据时先启动桥: "
            "ros2 launch a2w_bridge a2w_bridge.launch.py"
        )

    # ------------------------------------------------------------------ 回调
    def _on_joints(self, msg: JointState) -> None:
        with self._lock:
            self._latest = msg
            self._latest_mono = time.monotonic()
        if self._stale_warned:
            self._stale_warned = False
            self.get_logger().info(f"{self.input_topic} 恢复")

    def _on_timer(self) -> None:
        with self._lock:
            latest, stamp_mono = self._latest, self._latest_mono
        age = 1e9 if latest is None else time.monotonic() - stamp_mono
        if latest is None or age > self.stale_sec:
            if not self._stale_warned:
                self._stale_warned = True
                self.get_logger().warning(
                    f"已 {self.stale_sec:g} s 没收到 {self.input_topic}"
                    + (f"（最后一条 {age:.1f} s 前）" if latest is not None else "（还没收到过）")
                    + "，暂停发布 " + self.output_topic
                    + "；确认桥在运行且本终端在同一 ROS 域（先 source "
                    "src/a2w_bridge/scripts/a2w_env.sh）"
                )
            return

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id or latest.header.frame_id
        for i, sdk_name in enumerate(latest.name):
            urdf_name = A2W_URDF_JOINT_NAMES.get(sdk_name)
            if urdf_name is None:
                if sdk_name not in self._unmapped_warned:
                    self._unmapped_warned.add(sdk_name)
                    self.get_logger().warning(
                        f"{self.input_topic} 里有未映射的关节 {sdk_name!r}，已跳过"
                        "（新关节请更新 dds_topics.A2W_URDF_JOINT_NAMES）"
                    )
                continue
            scale = -1.0 if sdk_name in self.flip else 1.0
            out.name.append(urdf_name)
            out.position.append(_f(_at(latest.position, i)) * scale)
            out.velocity.append(_f(_at(latest.velocity, i)) * scale)
            out.effort.append(_f(_at(latest.effort, i)) * scale)

        if not out.name:
            return
        self._pub.publish(out)
        if not self._published:
            self._published = True
            self.get_logger().info(
                f"首个 /joint_states 已发布: {len(out.name)} 个关节，"
                f"例如 {out.name[0]}={out.position[0]:+.4f}；RViz 里定点观察姿势是否与实际一致，"
                "腿方向反了就把对应 SDK 关节名加到配置的 display.flip"
            )


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node: JointRelay | None = None
    try:
        node = JointRelay()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001 —— 生命周期内异常要能看到完整日志
        print(f"[a2w_joint_relay] 致命错误: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
