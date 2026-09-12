"""``/cmd_vel`` → ``sport_client.Move(vx, vy, vyaw)`` 的运动通道（配置见 JSON 的 ``motion`` 段）。

数据流::

    ROS2 侧 node.py（py3.14）                     采集器侧 collector.py（py3.10 + unitree_sdk2py）
    /cmd_vel → 限幅/死区 → 限频 ────TCP 帧────▶ MotionController ──RPC──▶ sport_client.Move(vx,vy,vyaw)
                                                                └──────▶ sport_client.StopMove()
                                                                └──────▶ Damp()/preflight（可选）

**本模块是全桥唯一会向机器人下发控制指令的地方**（其余部分只读订阅）。因此安全设计全部
写在这里，任何一条都不建议为了"调通"而关掉：

1. **默认关闭**：``motion.enabled=false`` 时两端都完全不动作（不订阅 /cmd_vel、不建 RPC 客户端）。
2. **状态门**：``rt/sportmodestate`` 不新鲜、或 ``error_code`` 落在 ``block_codes``
   （默认 ``[1001]`` = 阻尼/软急停）时**拒绝下发**，并退化为 ``StopMove()`` ——
   遥控器一按 L2+B 软急停，这一层立刻把自主速度指令掐掉。
3. **看门狗**：``stale_sec`` 内没有新的 cmd_vel（手柄断连 / 上游崩溃 / 话题没人发）→ ``StopMove()``。
4. **限幅**：vx/vy/vyaw 按 ``limits`` 夹住（默认走路低速档 ±0.8/±0.5/±2.0），两端各夹一次。
5. **非阻塞**：``Move`` 是 request-response RPC（默认超时 1 s），调用期间不发新指令，
   避免 RPC 抖动把控制循环拖垮；错误率限流打日志。
6. **退出即停**：``close()`` 先 ``StopMove()`` 再关通道（含采集器被 terminate / 崩溃重启）。
7. **试运行**：``dry_run=true`` 时**不创建** SportClient，只在日志里打印"本该下发什么" ——
   实机上验证整条链路（手柄 → /cmd_vel → 帧 → 采集器）而不让机器人动一下。

本模块顶层只用标准库，SDK 在 :meth:`MotionController.start` 里惰性导入，
因此 ROS2 侧（Python 3.14，装不了 cyclonedds 0.10.2）也能安全 import 限幅函数。
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable

# 采集器侧日志回调：log(text)
LogFn = Callable[[str], None]
# 运控状态提供者：返回 (error_code, 采样时刻) 或 None（从未收到）
StateFn = Callable[[], "tuple[int, float] | None"]
# SportClient 工厂（测试注入假客户端用；返回对象需有 Move/StopMove/Damp/BalanceStand 等方法）
ClientFactory = Callable[[], Any]

# 限幅默认值 = 官方《A2W轮足运动服务接口》走路·低速档 [±0.8, ±0.5, ±2.0]
DEFAULT_LIMITS = {"vx": 0.8, "vy": 0.5, "vyaw": 2.0}
# 死区：手柄/上游的小抖动不变成指令（避免机器人原地"嗡嗡"动）
DEFAULT_DEADBAND = {"linear": 0.02, "angular": 0.05}


def _num(value: Any, default: float = 0.0) -> float:
    """防御式取数：坏值（None/字符串/NaN/±Inf）一律给 default，绝不抛异常。

    配置来自 JSON、目标值来自 TCP 帧 —— 两者都按不可信输入处理（与 node.py 的 _f/_i 同规矩）。
    """
    try:
        num = float(value)
    except (TypeError, ValueError):
        return default
    if math.isnan(num) or math.isinf(num):
        return default
    return num


def _int(value: Any, default: int = 0) -> int:
    """防御式取整数（同 :func:`_num`）。"""
    num = _num(value, default)
    try:
        return int(num)
    except (TypeError, ValueError, OverflowError):  # noqa: BLE001 —— 理论上不可达，防御
        return default


def _clamp(value: float, limit: float) -> float:
    """对称夹紧（limit <= 0 表示不限）。"""
    if limit <= 0:
        return value
    return max(-limit, min(limit, value))


def _dead(value: float, zone: float) -> float:
    """死区：|value| < zone 归零，否则原样返回。"""
    return 0.0 if abs(value) < zone else value


def shape_cmd(
    vx: float,
    vy: float,
    vyaw: float,
    limits: dict[str, Any] | None = None,
    deadband: dict[str, Any] | None = None,
) -> tuple[float, float, float]:
    """限幅 + 死区（node 与 collector 两端各调一次；坏值一律归零，不抛异常）。

    :param vx: 机体坐标系 x 速度（前进为正，m/s）
    :param vy: 机体坐标系 y 速度（左移为正，m/s）
    :param vyaw: 机体坐标系 yaw 角速度（逆时针为正，rad/s）
    """
    lim = {**DEFAULT_LIMITS, **(limits or {})}
    dead = {**DEFAULT_DEADBAND, **(deadband or {})}
    out: list[float] = []
    for value, limit in ((vx, lim.get("vx")), (vy, lim.get("vy")), (vyaw, lim.get("vyaw"))):
        out.append(_clamp(_num(value), _num(limit)))
    return (
        _dead(out[0], _num(dead.get("linear"), DEFAULT_DEADBAND["linear"])),
        _dead(out[1], _num(dead.get("linear"), DEFAULT_DEADBAND["linear"])),
        _dead(out[2], _num(dead.get("angular"), DEFAULT_DEADBAND["angular"])),
    )


class MotionController:
    """采集器侧的运动执行器：把 TCP 帧里的速度目标变成 ``sport_client.Move()``。

    线程模型：:meth:`start` 起一个**工作线程**按 ``rate_hz`` 下发；:meth:`submit` 由
    socket 读线程调用（只写目标值，不阻塞）。
    """

    def __init__(
        self,
        cfg: dict[str, Any],
        log: LogFn,
        state_provider: StateFn | None = None,
        client_factory: ClientFactory | None = None,
    ) -> None:
        self.log = log
        self.cfg = cfg
        self.dry_run = bool(cfg.get("dry_run", False))
        self.rate_hz = _num(cfg.get("rate_hz"), 20.0) or 20.0
        self.stale_sec = _num(cfg.get("stale_sec"), 0.5) or 0.5
        self.state_stale_sec = _num(cfg.get("state_stale_sec"), 1.0) or 1.0
        self.require_state = bool(cfg.get("require_state", True))
        self.block_codes = {_int(c) for c in (cfg.get("block_codes") or [1001])}
        self.timeout_sec = _num(cfg.get("timeout_sec"), 0.3) or 0.3
        self.stop_on_exit = bool(cfg.get("stop_on_exit", True))
        self.preflight = [str(name) for name in (cfg.get("preflight") or [])]
        self.limits = {**DEFAULT_LIMITS, **(cfg.get("limits") or {})}
        self.deadband = {**DEFAULT_DEADBAND, **(cfg.get("deadband") or {})}
        self._state_provider: StateFn = state_provider or (lambda: None)
        self._client_factory = client_factory

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._client: Any = None

        # 目标与状态（_lock 保护）
        self._target: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._target_ts = 0.0            # 目标到达墙钟（0 = 从未收到）
        self._stop_requested = False     # 显式 stop 帧（比超时更早生效）
        self._moving = False             # 最近一次是否真的下发过 Move
        self._blocked: str | None = None  # 当前被拒绝的原因（None = 没被拦）
        self._sent = 0                   # Move 调用次数（含试运行）
        self._errors = 0                 # 非 0 返回码次数
        self._last_code: int | None = None
        self._preflight_done = False
        self._last_error_log = 0.0

    # ------------------------------------------------------------- 生命周期
    def start(self) -> None:
        """建 SportClient（试运行除外）并起工作线程。失败向上抛，由调用方决定是否降级。"""
        if not self.dry_run:
            self._client = (self._client_factory or self._make_client)()
        mode = "试运行（不下发）" if self.dry_run else "已连接 sport 服务"
        self.log(
            f"运动通道: {mode}，限幅 vx≤{self.limits['vx']} vy≤{self.limits['vy']} "
            f"vyaw≤{self.limits['vyaw']}，{self.rate_hz:g}Hz，"
            f"看门狗 {self.stale_sec:g}s，状态门 {'开' if self.require_state else '关'}"
            f"{'' if self.require_state else '⚠️'}"
        )
        self._thread = threading.Thread(target=self._loop, name="motion", daemon=True)
        self._thread.start()

    def _make_client(self) -> Any:
        """建 A2/A2W 的 sport RPC 客户端（惰性导入：只有采集器（py3.10）能建）。"""
        from unitree_sdk2py.a2.sport.sport_client import (  # pyright: ignore[reportMissingImports]
            SportClient,
        )

        client = SportClient()
        client.SetTimeout(self.timeout_sec)
        client.Init()
        return client

    def close(self) -> None:
        """退出即停：先 StopMove（带一小段等待），再停线程。"""
        if self._thread is None and self._client is None:
            return
        moving = False
        with self._lock:
            moving = self._moving
        if self.stop_on_exit and (moving or self._sent):
            self._stop_move("桥关闭")
        self._stop.set()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=1.0)

    # --------------------------------------------------------------- 命令入口
    def submit(self, vx: float, vy: float, vyaw: float, ts: float | None = None) -> None:
        """记录一个新的速度目标（由 cmd 帧读线程调用，非阻塞）。"""
        with self._lock:
            self._target = shape_cmd(vx, vy, vyaw, self.limits, self.deadband)
            self._target_ts = time.time() if ts is None else _num(ts, time.time())
            self._stop_requested = False

    def request_stop(self, reason: str = "显式停止") -> None:
        """上游要求停止（stop 帧）：立刻 StopMove，并进入"无目标"态。"""
        with self._lock:
            self._stop_requested = True
        self._stop_move(reason)

    def status(self) -> dict[str, Any]:
        """给状态帧 / 日志用的快照（不含 RPC 调用，调用方随时可读）。"""
        with self._lock:
            now = time.time()
            age = (now - self._target_ts) if self._target_ts > 0 else -1.0
            return {
                "enabled": True,
                "dry_run": self.dry_run,
                "moving": self._moving,
                "target": [round(v, 3) for v in self._target],
                "age_sec": round(age, 3),
                "blocked": self._blocked,
                "sent": self._sent,
                "errors": self._errors,
                "last_code": self._last_code,
            }

    # --------------------------------------------------------------- 内部
    def _state(self) -> tuple[int, float] | None:
        try:
            return self._state_provider()
        except Exception:  # noqa: BLE001 —— 状态读取失败按"无状态"处理（会更保守）
            return None

    def _gate(self) -> str | None:
        """状态门：返回拒绝原因；None = 允许下发。"""
        if not self.require_state:
            return None
        state = self._state()
        if state is None:
            return "还没有运控状态（rt/sportmodestate 无数据）"
        code, ts = state
        if time.time() - ts > self.state_stale_sec:
            return f"运控状态不新鲜（{time.time() - ts:.1f}s 无更新）"
        if code in self.block_codes:
            return f"运控处于 {code}（阻尼/软急停），拒绝下发速度指令"
        return None

    def _call(self, name: str, *args: Any) -> int:
        """调用客户端方法并统一记错误码；返回 0 = 成功。"""
        if self._client is None:
            return 0  # 试运行：不调用，按成功处理
        method = getattr(self._client, name, None)
        if method is None:
            self.log(f"运动通道: SDK 客户端没有 {name}() —— 跳过")
            return -1
        try:
            code = int(method(*args))
        except Exception as exc:  # noqa: BLE001 —— RPC 抖动不能打死采集器
            code = -1
            self.log(f"运动通道: {name}() 异常: {exc}")
        with self._lock:
            self._sent += 1
            self._last_code = code
            if code != 0:
                self._errors += 1
        if code != 0:
            now = time.time()
            with self._lock:
                quiet = (now - self._last_error_log) >= 1.0
                if quiet:
                    self._last_error_log = now
                errs = self._errors
            if quiet:
                self.log(f"运动通道: {name}() 返回 {code}（累计 {errs} 次失败）")
        return code

    def _stop_move(self, reason: str) -> None:
        """停止一次（幂等：只有当前在动才真调 StopMove）。"""
        with self._lock:
            was_moving = self._moving
            self._moving = False
            self._target = (0.0, 0.0, 0.0)
            self._target_ts = 0.0
        if was_moving or self._sent:
            self._call("StopMove")
        self.log(f"运动通道: 停止（{reason}）")

    def _enter_move(self) -> None:
        """首次下发前的可选预动作（preflight，例如 ["BalanceStand"]）。"""
        if self._preflight_done:
            return
        self._preflight_done = True
        for name in self.preflight:
            if name:
                self._call(name)
                self.log(f"运动通道: 预动作 {name}() 已执行")

    def _loop(self) -> None:
        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.05
        while not self._stop.is_set():
            tick = time.time()
            with self._lock:
                target = self._target
                ts = self._target_ts
                stop_requested = self._stop_requested
            now = time.time()

            if stop_requested or ts <= 0 or (now - ts) > self.stale_sec:
                reason = "上游要求停止" if stop_requested else (
                    "无 cmd_vel 目标" if ts <= 0 else f"cmd_vel 超时 {now - ts:.2f}s"
                )
                with self._lock:
                    was_moving, blocked = self._moving, self._blocked
                    self._blocked = None
                if was_moving:
                    self._stop_move(reason)
                elif blocked is None:
                    # 静默态：只更新日志里的"已被拒绝"原因，不刷屏
                    with self._lock:
                        self._blocked = None
                if stop_requested:
                    with self._lock:
                        self._stop_requested = False
                        self._target_ts = 0.0
            else:
                blocked = self._gate()
                with self._lock:
                    prev_blocked = self._blocked
                    self._blocked = blocked
                if blocked is not None:
                    if prev_blocked != blocked:
                        self.log(f"运动通道: {blocked} → StopMove")
                    if self._moving:
                        self._stop_move(blocked)
                else:
                    if prev_blocked is not None:
                        self.log(f"运动通道: 恢复下发（原拒绝原因: {prev_blocked}）")
                    self._enter_move()
                    if self.dry_run:
                        self.log(
                            f"运动通道[试运行]: 本应 Move(vx={target[0]:.3f}, vy={target[1]:.3f}, "
                            f"vyaw={target[2]:.3f})"
                        )
                        with self._lock:
                            self._sent += 1
                            self._moving = True
                            self._last_code = 0
                    else:
                        with self._lock:
                            self._moving = True
                        self._call("Move", target[0], target[1], target[2])

            sleep = period - (time.time() - tick)
            if sleep > 0:
                self._stop.wait(sleep)
