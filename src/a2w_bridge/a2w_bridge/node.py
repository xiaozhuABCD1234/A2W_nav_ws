#!/usr/bin/env python3
"""A2W → ROS2 桥接节点（rclpy，跑在 ROS2 的 Python 环境里）。

职责：
  1. 读 JSON 配置（``config`` 参数，默认 ``share/a2w_bridge/config/a2w_bridge.json``）；
  2. 拉起采集器子进程（Python 3.10 + unitree_sdk2py，见 ``collector.py``），采集器崩溃自动重启；
  3. 接收采集器的本机 TCP 帧，转成**标准 ROS2 消息**发布：

     - 点云          → ``sensor_msgs/PointCloud2``（x/y/z[/intensity]）
     - IMU           → ``sensor_msgs/Imu``（lidar imu / lowstate imu）
     - 关节状态      → ``sensor_msgs/JointState``（A2W 16 关节：腿序×髋/大腿/小腿/轮）
     - 电池          → ``sensor_msgs/BatteryState``（rt/bms_state）
     - slam 广播     → ``std_msgs/String``（原始 JSON 透传）
     - 全局占栅格    → ``nav_msgs/OccupancyGrid``
     - 机器人状态    → ``std_msgs/String``（JSON：machine 模式 / 关节新鲜度 / 电池；
       不再报桥自身健康——点云源、IMU 源、频率等只在日志里）
     - 静态 TF       → 按配置发布 base → lidar / imu 等

时间戳统一用**采集端墙钟**（所有设备 / 话题同源，天然一致），ROS2 侧照抄成
``header.stamp``；机器人自带时间戳（hdr_stamp）只进状态日志，不做时钟源。

帧数据视为**不可信输入**：本模块所有 int()/float() 均走防御转换（_i/_f），
单帧坏数据丢帧+记日志，而不是把采集器连接打掉。
"""

from __future__ import annotations

import contextlib
import json
import math
import os
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped, Vector3
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import BatteryState, Imu, JointState, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

from . import protocol
from .config import ConfigError, describe, find_ws_root, load_config, resolve_python, resolve_sdk_path
from .dds_topics import CLOUD_TOPICS, IMU_TOPICS

HEADER_FIELDS = ("x", "y", "z", "intensity")


def _f(value: Any, default: float = 0.0) -> float:
    """防御性 float 转换（帧数据不可信）。"""
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _i(value: Any, default: int = 0) -> int:
    """防御性 int 转换（帧数据不可信）。"""
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def ros_time(ts: float) -> RosTime:
    sec = _i(ts)
    try:
        nsec = int(round((ts - sec) * 1e9))
    except (TypeError, ValueError):
        nsec = 0
    return RosTime(sec=sec, nanosec=nsec)


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    """RPY（弧度）→ 四元数（静态 TF 用，标准 ZYX 顺序）。"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def quaternion(x, y, z, w) -> Quaternion:
    return Quaternion(x=_f(x), y=_f(y), z=_f(z), w=_f(w, 1.0))


def vector3(x, y, z) -> Vector3:
    return Vector3(x=_f(x), y=_f(y), z=_f(z))


class A2WBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("a2w_bridge")

        self.declare_parameter("config", "")
        self.declare_parameter("iface", "")
        cfg_param = self.get_parameter("config").value
        cfg_path = cfg_param or str(
            Path(get_package_share_directory("a2w_bridge")) / "config" / "a2w_bridge.json"
        )
        try:
            self.cfg = load_config(cfg_path)
        except ConfigError as exc:
            self.get_logger().fatal(str(exc))
            raise SystemExit(2) from exc

        iface = self.get_parameter("iface").value
        if iface:
            self.cfg["iface"] = iface
        self.get_logger().info(f"配置加载完成: {describe(self.cfg)}")

        ws_root = find_ws_root()
        try:
            self.collector_python = resolve_python(self.cfg, ws_root)
            self.sdk_path = resolve_sdk_path(self.cfg, ws_root)
        except ConfigError as exc:
            self.get_logger().fatal(str(exc))
            raise SystemExit(2) from exc

        self.port = self.cfg["collector"]["port"]  # config.py 已规范化成 int
        self._stop = threading.Event()
        self._proc: subprocess.Popen | None = None
        self._server: socket.socket | None = None
        self._server_thread: threading.Thread | None = None
        self._proc_lock = threading.Lock()
        self._restart_idx = 0

        self._make_publishers()
        self._broadcast_static_tf()

    # ------------------------------------------------------------ 发布器
    def _make_publishers(self) -> None:
        self._pub: dict[str, Any] = {}
        pc = self.cfg["pointcloud"]
        if pc.get("enabled", True):
            if pc["mode"] == "multi":
                for key, topic in pc.get("multi_topics", {}).items():
                    self._pub[f"cloud:{key}"] = self.create_publisher(PointCloud2, topic, 2)
            else:
                # single 模式（含 auto 轮换）：所有候选源都发到同一个 topic
                for key in CLOUD_TOPICS:
                    self._pub[f"cloud:{key}"] = self.create_publisher(PointCloud2, pc["topic"], 2)

        imu = self.cfg["imu"]
        if imu.get("enabled", True):
            if imu["source"] == "all":
                for key, topic in imu.get("topics", {}).items():
                    self._pub[f"imu:{key}"] = self.create_publisher(Imu, topic, 100)
            elif imu["source"] == "auto":
                # auto 降级：三个候选都发到同一个 topic，消费端无感
                for key in IMU_TOPICS:
                    self._pub[f"imu:{key}"] = self.create_publisher(Imu, imu["topic"], 100)
            else:
                self._pub[f"imu:{imu['source']}"] = self.create_publisher(Imu, imu["topic"], 100)

        sensors = self.cfg["sensors"]
        if sensors["slam_info"].get("enabled", True):
            self._pub["slam_info"] = self.create_publisher(String, sensors["slam_info"]["topic"], 20)
        if sensors["slam_key_info"].get("enabled", True):
            self._pub["slam_key_info"] = self.create_publisher(String, sensors["slam_key_info"]["topic"], 20)
        if sensors["global_map"].get("enabled", False):
            self._pub["global_map"] = self.create_publisher(OccupancyGrid, sensors["global_map"]["topic"], 2)

        joints = self.cfg["joints"]
        if joints.get("enabled", True):
            self._pub["joints"] = self.create_publisher(JointState, joints["topic"], 100)

        battery = self.cfg["battery"]
        if battery.get("enabled", True):
            self._pub["battery"] = self.create_publisher(BatteryState, battery["topic"], 10)

        sport = self.cfg["sport_state"]
        if sport.get("enabled", True):
            self._pub["sport"] = self.create_publisher(String, sport["topic"], 20)

        self._pub["status"] = self.create_publisher(String, self.cfg["status_topic"], 5)

    def _broadcast_static_tf(self) -> None:
        tf_cfg = self.cfg.get("tf", {})
        if not tf_cfg.get("enabled", True):
            return
        broadcaster = StaticTransformBroadcaster(self)
        transforms = []
        now = ros_time(time.time())
        for item in tf_cfg.get("transforms", []):
            try:
                parent, child = str(item["parent"]), str(item["child"])
                xyz = [float(v) for v in item.get("xyz", [0, 0, 0])][:3] + [0.0] * (3 - len(item.get("xyz", [])))
                rpy = [float(v) for v in item.get("rpy", [0, 0, 0])][:3] + [0.0] * (3 - len(item.get("rpy", [])))
            except (KeyError, TypeError, ValueError):
                self.get_logger().warning(f"tf.transforms 配置项非法，已跳过: {item}")
                continue
            q = _quat_from_rpy(*rpy)
            msg = TransformStamped()
            msg.header.stamp = now
            msg.header.frame_id = parent
            msg.child_frame_id = child
            msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = xyz
            msg.transform.rotation = q
            transforms.append(msg)
            self.get_logger().info(f"静态 TF: {parent} → {child} xyz={xyz} rpy={rpy}")
        if transforms:
            broadcaster.sendTransform(transforms)

    # ------------------------------------------------------------ 采集器进程
    @property
    def _collector_script(self) -> Path:
        return Path(__file__).resolve().parent / "collector.py"

    def _spawn_collector(self) -> subprocess.Popen | None:
        if self._stop.is_set():
            return None
        env = dict(os.environ)
        env["PYTHONPATH"] = str(Path(self.sdk_path)) + os.pathsep + env.get("PYTHONPATH", "")
        try:
            proc = subprocess.Popen(
                [
                    self.collector_python,
                    str(self._collector_script),
                    "--config",
                    self.cfg["_config_path"],
                    "--port",
                    str(self.port),
                ],
                env=env,
                stderr=subprocess.PIPE,
                text=True,
            )
        except OSError as exc:
            self.get_logger().error(f"启动采集器失败: {exc}")
            return None
        self._proc = proc
        thread = threading.Thread(target=self._forward_stderr, args=(proc,), daemon=True, name="col_stderr")
        thread.start()
        self.get_logger().info(f"采集器已启动 (pid={proc.pid}, python={self.collector_python})")
        return proc

    def _forward_stderr(self, proc: subprocess.Popen) -> None:
        assert proc.stderr is not None
        for line in proc.stderr:
            line = line.rstrip("\n")
            if line:
                self.get_logger().info(f"[collector] {line}")

    def _supervise(self) -> None:
        """盯采集器：挂了就按退避节奏重启。"""
        backoff = self.cfg["collector"]["restart_backoff_sec"]  # config.py 已规范化成 float 数组
        while not self._stop.wait(0.5):
            with self._proc_lock:
                proc = self._proc
            if proc is None:
                if not self._spawn_collector():
                    self._stop.wait(2.0)
                continue
            code = proc.poll()
            if code is not None:
                delay = backoff[min(self._restart_idx, len(backoff) - 1)]
                self._restart_idx += 1
                self.get_logger().warning(f"采集器退出 (code={code})，{delay}s 后重启")
                self._stop.wait(delay)
                with self._proc_lock:
                    self._proc = None

    # ------------------------------------------------------------ 网络服务
    def _serve(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("127.0.0.1", self.port))
        server.listen(1)
        server.settimeout(0.5)
        self._server = server
        self.get_logger().info(f"等待采集器连接: tcp://127.0.0.1:{self.port}")
        while not self._stop.is_set():
            try:
                conn, _ = server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.get_logger().info("采集器已连接")
            self._restart_idx = 0  # 联通了，重启计数清零
            try:
                self._pump(conn)
            except ConnectionError:
                self.get_logger().warning("采集器连接断开")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"帧处理异常: {exc}")
            finally:
                with contextlib.suppress(OSError):
                    conn.close()

    def _pump(self, conn: socket.socket) -> None:
        reader = protocol.FrameReader(conn)
        while not self._stop.is_set():
            header, payload = reader.read_frame()
            self._dispatch(header, payload)

    # ------------------------------------------------------------ 帧分发
    def _dispatch(self, header: dict[str, Any], payload: bytes) -> None:
        kind = header.get("t")
        if kind == "cloud":
            self._on_cloud(header, payload)
        elif kind == "imu":
            self._on_imu(header)
        elif kind == "slam":
            self._on_slam(header)
        elif kind == "grid":
            self._on_grid(header, payload)
        elif kind == "joints":
            self._on_joints(header)
        elif kind == "battery":
            self._on_battery(header)
        elif kind == "sport":
            self._on_sport(header)
        elif kind == "status":
            self._on_status(header)
        elif kind == "log":
            level = str(header.get("level", "info"))
            text = str(header.get("text", ""))
            if level == "warn":
                self.get_logger().warning(text)
            elif level == "error":
                self.get_logger().error(text)
            else:
                self.get_logger().info(text)

    def _frame_id(self, header: dict[str, Any], configured: str) -> str:
        """非空配置优先；配置为空则跟机器人消息自带 frame_id。"""
        return configured or str(header.get("frame_id", "") or header.get("hdr_frame_id", "") or "")

    def _on_cloud(self, header: dict[str, Any], payload: bytes) -> None:
        key = str(header.get("k", ""))
        pub = self._pub.get(f"cloud:{key}")
        if pub is None:
            return
        fields = header.get("fields") or ["x", "y", "z"]
        n = _i(header.get("points", 0))
        pts = len(payload) // 4 // len(fields)
        msg = PointCloud2()
        msg.header.stamp = ros_time(_f(header.get("ts")))
        msg.header.frame_id = self.cfg["pointcloud"]["frame_id"]
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(
                name=name,
                offset=i * 4,
                datatype=PointField.FLOAT32,
                count=1,
            )
            for i, name in enumerate(fields)
        ]
        msg.is_bigendian = False
        msg.point_step = 4 * len(fields)
        msg.row_step = msg.point_step * msg.width
        msg.data = payload
        msg.is_dense = True
        pub.publish(msg)

    def _on_imu(self, header: dict[str, Any]) -> None:
        key = str(header.get("k", ""))
        pub = self._pub.get(f"imu:{key}")
        if pub is None:
            return
        cfg = self.cfg["imu"]
        msg = Imu()
        msg.header.stamp = ros_time(_f(header.get("ts")))
        # 每路源各自的坐标系（结构外参）：前雷达 IMU 在 a2w/lidar，**后雷达 IMU 在
        # a2w/lidar_rear**（机器人把所有话题都标成 hesai_lidar，后 IMU 那个标是错的，
        # 照搬会让后 IMU 的朝向差 180°），本体 IMU 在 a2w/imu。
        frames = cfg.get("frames") or {}
        msg.header.frame_id = self._frame_id(header, str(frames.get(key, "") or cfg["frame_id"]))

        quat = header.get("quat") or [0.0, 0.0, 0.0, 1.0]
        msg.orientation = quaternion(*quat[:4])
        msg.angular_velocity = vector3(*(header.get("gyro") or [0.0, 0.0, 0.0])[:3])
        msg.linear_acceleration = vector3(*(header.get("accel") or [0.0, 0.0, 0.0])[:3])
        # ROS2 惯例：协方差缺失时对角线填 -1 表示“未知”
        for attr in (
            "orientation_covariance",
            "angular_velocity_covariance",
            "linear_acceleration_covariance",
        ):
            values = header.get(attr) or []
            cov = [_f(v, -1.0) for v in values] if len(values) >= 9 else [-1.0 if i % 4 == 0 else 0.0 for i in range(9)]
            setattr(msg, attr, cov)
        pub.publish(msg)

    def _on_joints(self, header: dict[str, Any]) -> None:
        """关节帧 → ``sensor_msgs/JointState``（16 关节，随 rt/lowstate 节奏）。"""
        pub = self._pub.get("joints")
        if pub is None:
            return
        cfg = self.cfg["joints"]
        names = list(cfg["names"])
        n = len(names)
        q = [_f(v) for v in (header.get("q") or [])]
        dq = [_f(v) for v in (header.get("dq") or [])]
        tau = [_f(v) for v in (header.get("tau") or [])]
        msg = JointState()
        msg.header.stamp = ros_time(_f(header.get("ts")))
        msg.header.frame_id = cfg.get("frame_id", "a2w/base")
        msg.name = names
        msg.position = q[:n] + [0.0] * (n - len(q))
        msg.velocity = dq[:n] + [0.0] * (n - len(dq))
        msg.effort = tau[:n] + [0.0] * (n - len(tau))
        pub.publish(msg)

    def _on_battery(self, header: dict[str, Any]) -> None:
        """电池帧（采集器已把 mV/mA 换成 V/A）→ ``sensor_msgs/BatteryState``。"""
        pub = self._pub.get("battery")
        if pub is None:
            return
        msg = BatteryState()
        msg.header.stamp = ros_time(_f(header.get("ts")))
        msg.voltage = _f(header.get("voltage"))
        msg.current = _f(header.get("current"))
        # 消息合同：percentage 是 0~1（BMS soc 是 0~100%）
        msg.percentage = max(0.0, min(1.0, _f(header.get("soc")) / 100.0))
        temps = [_f(v) for v in (header.get("temperature") or [])]
        msg.temperature = temps[0] if temps else math.nan
        msg.cell_temperature = temps
        msg.cell_voltage = [_f(v) for v in (header.get("cell_vol") or [])]
        msg.present = msg.voltage > 0.0
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.location = "body_bms"
        pub.publish(msg)

    def _on_sport(self, header: dict[str, Any]) -> None:
        """运控状态帧（只读）→ ``std_msgs/String``(JSON)：error_code/状态名/高度/速度等。

        `error_code` 取自官方文档的状态机表，**1001 = 阻尼（软急停）**。
        状态跳变时采集器会额外发一条 log 帧（1001 为 warn），转发到 ROS 日志。
        """
        pub = self._pub.get("sport")
        if pub is None:
            return
        msg = String()
        public = {k: v for k, v in header.items() if k != "payload"}
        msg.data = json.dumps(public, ensure_ascii=False)
        pub.publish(msg)

    def _on_slam(self, header: dict[str, Any]) -> None:
        key = str(header.get("k", ""))
        pub = self._pub.get(key)
        if pub is None:
            return
        msg = String()
        msg.data = str(header.get("data", ""))
        pub.publish(msg)

    def _on_grid(self, header: dict[str, Any], payload: bytes) -> None:
        pub = self._pub.get("global_map")
        if pub is None:
            return
        cfg = self.cfg["sensors"]["global_map"]
        msg = OccupancyGrid()
        msg.header.stamp = ros_time(_f(header.get("ts")))
        msg.header.frame_id = self._frame_id(header, cfg.get("frame_id", ""))
        msg.info.resolution = _f(header.get("resolution", 0.05) or 0.05, 0.05)
        msg.info.width = _i(header.get("width", 0) or 0)
        msg.info.height = _i(header.get("height", 0) or 0)
        origin = header.get("origin_position") or [0.0, 0.0, 0.0]
        oq = header.get("origin_orientation") or [0.0, 0.0, 0.0, 1.0]
        msg.info.origin = Pose(
            position=Point(x=origin[0], y=origin[1], z=origin[2]),
            orientation=quaternion(oq[0], oq[1], oq[2], oq[3]),
        )
        # 机器人侧是 uint8（255=未知），ROS2 侧是 int8（-1=未知）→ 显式换算
        msg.data = [v - 256 if v > 127 else v for v in payload]
        pub.publish(msg)

    def _on_status(self, header: dict[str, Any]) -> None:
        """机器人状态帧（JSON，默认 5 s）：machine 模式 / 关节新鲜度 / 电池。"""
        pub = self._pub.get("status")
        if pub is None:
            return
        msg = String()
        # 去掉传输层字段（payload 是 TCP 帧头内部信息，不属于机器人状态）
        public = {k: v for k, v in header.items() if k != "payload"}
        msg.data = json.dumps(public, ensure_ascii=False)
        pub.publish(msg)

        joints = header.get("joints")
        if isinstance(joints, dict):
            fresh = joints.get("fresh_sec", -1.0)
            jline = "无数据" if fresh < 0 else f"{joints.get('count')} 关节, 距上帧 {fresh}s"
        else:
            jline = "关闭"
        battery = header.get("battery")
        bline = f"{battery.get('voltage')}V/{battery.get('soc')}%" if isinstance(battery, dict) else "无数据"
        sport = header.get("sport")
        sline = sport.get("name") if isinstance(sport, dict) else "无数据"
        self.get_logger().info(
            f"机器人状态: 运控={sline} mode_machine={header.get('mode_machine')} mode_pr={header.get('mode_pr')} "
            f"| 关节: {jline} | 电池: {bline}"
        )

    # ------------------------------------------------------------ 生命周期
    def start(self) -> None:
        self._server_thread = threading.Thread(target=self._serve, daemon=True, name="tcp_server")
        self._server_thread.start()
        threading.Thread(target=self._supervise, daemon=True, name="supervise").start()

    def stop(self) -> None:
        self.get_logger().info("正在关闭…")
        self._stop.set()
        with self._proc_lock:
            if self._proc is not None and self._proc.poll() is None:
                self._proc.terminate()
                try:
                    self._proc.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    self._proc.kill()
        if self._server is not None:
            with contextlib.suppress(OSError):
                self._server.close()
            self._proc = None


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node: A2WBridgeNode | None = None
    try:
        node = A2WBridgeNode()
        node.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001 —— 生命周期内异常要能看到完整日志
        print(f"[a2w_bridge] 致命错误: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.stop()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())