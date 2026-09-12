#!/usr/bin/env python3
"""运动通道自检（``/cmd_vel`` → ``sport_client.Move``）——**不连机器人、不连 ROS2**。

用假 SportClient 验证 ``a2w_bridge.motion.MotionController`` 的四条安全线，任何一条
失效都会在这个脚本里红掉：

  1. 状态门：运控状态是 1001（阻尼/软急停）或没有状态 → 一条 Move 都不发；
  2. 限幅：目标速度超过 limits → 夹到上限；
  3. 看门狗：``stale_sec`` 内没有新目标 → StopMove；
  4. 退出即停：``close()`` → StopMove。

跑法（任意 Python ≥ 3.8，采集器 venv 或系统 python 都行）::

    python3 src/a2w_bridge/scripts/motion_selftest.py

想验证"实机链路通不通"但不让机器人动，见 README「运动通道」一节的 dry_run 说明。
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from a2w_bridge.motion import MotionController, shape_cmd  # noqa: E402

FAILED: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"{'✓' if ok else '✗'} {name}{(' — ' + detail) if detail else ''}")
    if not ok:
        FAILED.append(name)


class FakeClient:
    """假 SportClient：记录调用，绝不发 DDS。"""

    def __init__(self) -> None:
        self.calls: list[tuple[str, tuple[Any, ...]]] = []

    def Move(self, vx: float, vy: float, vyaw: float) -> int:
        self.calls.append(("Move", (round(vx, 4), round(vy, 4), round(vyaw, 4))))
        return 0

    def StopMove(self) -> int:
        self.calls.append(("StopMove", ()))
        return 0

    def BalanceStand(self) -> int:
        self.calls.append(("BalanceStand", ()))
        return 0

    def moves(self) -> list[tuple[Any, ...]]:
        return [args for name, args in self.calls if name == "Move"]

    def stops(self) -> int:
        return sum(1 for name, _ in self.calls if name == "StopMove")


def make(limits: dict | None = None, **cfg: Any) -> tuple[MotionController, FakeClient, list[str]]:
    logs: list[str] = []
    client = FakeClient()
    base = {
        "rate_hz": 50.0,          # 自检要快
        "stale_sec": 0.15,
        "state_stale_sec": 0.5,
        "require_state": True,
        "block_codes": [1001],
        "limits": limits or {"vx": 0.8, "vy": 0.5, "vyaw": 2.0},
        "deadband": {"linear": 0.02, "angular": 0.05},
    }
    base.update(cfg)
    ctl = MotionController(base, logs.append, client_factory=lambda: client)
    return ctl, client, logs


def test_shape() -> None:
    vx, vy, vyaw = shape_cmd(5.0, -3.0, 9.0)
    check("限幅: 超限目标被夹到走路低速档", (vx, vy, vyaw) == (0.8, -0.5, 2.0), f"{vx},{vy},{vyaw}")
    vx, vy, vyaw = shape_cmd(0.005, 0.01, 0.01)
    check("死区: 抖动归零", (vx, vy, vyaw) == (0.0, 0.0, 0.0), f"{vx},{vy},{vyaw}")
    try:
        nan, inf = float("nan"), float("inf")
    except ValueError:  # noqa: BLE001 —— 不可能发生，仅避开静态检查
        nan, inf = 0.0, 0.0
    vx, vy, vyaw = shape_cmd(nan, inf, "垃圾")  # type: ignore[arg-type]
    check("坏值: NaN/Inf/字符串一律归零", (vx, vy, vyaw) == (0.0, 0.0, 0.0), f"{vx},{vy},{vyaw}")


def test_healthy_state_moves() -> None:
    ctl, client, _ = make()
    ctl._state_provider = lambda: (1015, time.time())  # 常规行走
    ctl.start()
    try:
        ctl.submit(0.5, 0.0, 0.0)
        time.sleep(0.25)
        check("状态正常: 下发 Move", len(client.moves()) > 0, f"{len(client.moves())} 次")
        check("状态正常: 数值原样（未超限）", client.moves()[:1] == [(0.5, 0.0, 0.0)], str(client.moves()[:1]))
    finally:
        ctl.close()


def test_damping_gate() -> None:
    ctl, client, logs = make()
    ctl._state_provider = lambda: (1001, time.time())  # 阻尼/软急停
    ctl.start()
    try:
        ctl.submit(0.5, 0.0, 0.0)
        time.sleep(0.25)
        check("阻尼门: 一条 Move 都不发", client.moves() == [], f"{len(client.moves())} 次")
        check("阻尼门: 日志说明原因", any("1001" in text for text in logs), "; ".join(logs[-2:]))
    finally:
        ctl.close()


def test_state_missing_and_stale() -> None:
    ctl, client, _ = make()
    ctl._state_provider = lambda: None
    ctl.start()
    try:
        ctl.submit(0.5, 0.0, 0.0)
        time.sleep(0.2)
        check("无状态: 拒绝下发", client.moves() == [], f"{len(client.moves())} 次")
    finally:
        ctl.close()

    ctl, client, _ = make(state_stale_sec=0.05)
    ctl._state_provider = lambda: (1015, time.time() - 5.0)  # 状态很旧
    ctl.start()
    try:
        ctl.submit(0.5, 0.0, 0.0)
        time.sleep(0.2)
        check("状态不新鲜: 拒绝下发", client.moves() == [], f"{len(client.moves())} 次")
    finally:
        ctl.close()


def test_watchdog_and_explicit_stop() -> None:
    ctl, client, _ = make(stale_sec=0.1)
    ctl._state_provider = lambda: (1015, time.time())
    ctl.start()
    try:
        ctl.submit(0.5, 0.0, 0.0)
        time.sleep(0.15)
        before = len(client.moves())
        time.sleep(0.25)  # 超过 stale_sec 不再 submit
        check("看门狗: 超时后 StopMove", client.stops() >= 1, f"stops={client.stops()}")
        check("看门狗: 超时后不再 Move", len(client.moves()) == before, f"{before} → {len(client.moves())}")
        ctl.submit(0.5, 0.0, 0.0)
        time.sleep(0.15)
        stops_before = client.stops()
        ctl.request_stop("测试")           # 等价于收到 stop 帧
        check("显式停止: 立刻 StopMove", client.stops() > stops_before, f"{stops_before} → {client.stops()}")
    finally:
        ctl.close()


def test_close_and_dry_run() -> None:
    ctl, client, _ = make()
    ctl._state_provider = lambda: (1015, time.time())
    ctl.start()
    ctl.submit(0.5, 0.0, 0.0)
    time.sleep(0.15)
    ctl.close()
    check("退出即停: close() → StopMove", client.stops() >= 1, f"stops={client.stops()}")

    logs: list[str] = []
    dry = MotionController(
        {"rate_hz": 50.0, "stale_sec": 0.5, "dry_run": True, "limits": {"vx": 0.8}},
        logs.append,
        state_provider=lambda: (1015, time.time()),
    )
    check("试运行: 不创建 SportClient", dry._client is None)
    dry.start()
    try:
        dry.submit(0.3, 0.0, 0.0)
        time.sleep(0.15)
        check("试运行: 只打日志", any("本应 Move" in text for text in logs), "; ".join(logs[-1:]))
    finally:
        dry.close()

    logs2: list[str] = []
    pre = MotionController(
        {"rate_hz": 50.0, "stale_sec": 0.5, "preflight": ["BalanceStand"]},
        logs2.append,
        state_provider=lambda: (1015, time.time()),
        client_factory=lambda: client,
    )
    client.calls.clear()
    pre.start()
    try:
        pre.submit(0.3, 0.0, 0.0)
        time.sleep(0.15)
        names = [name for name, _ in client.calls]
        check("预动作: 首次下发前调 BalanceStand", names[:1] == ["BalanceStand"], str(names[:3]))
    finally:
        pre.close()


def main() -> int:
    print("运动通道自检（假 SportClient，不连机器人 / 不连 ROS2）\n")
    for fn in (
        test_shape,
        test_healthy_state_moves,
        test_damping_gate,
        test_state_missing_and_stale,
        test_watchdog_and_explicit_stop,
        test_close_and_dry_run,
    ):
        fn()
    print()
    if FAILED:
        print(f"❌ 失败 {len(FAILED)} 项: {FAILED}")
        return 1
    print("✅ 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
