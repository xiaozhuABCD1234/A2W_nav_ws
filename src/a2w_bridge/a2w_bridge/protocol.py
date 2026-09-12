"""采集器 ↔ ROS2 节点之间的本机 TCP 帧协议（纯标准库，两端 Python 版本不同也能用）。

帧格式::

    ┌──────────────┬──────────────────────┬────────────────────────┐
    │ uint32 BE len│ JSON header (len 字节)│ binary payload (可选)   │
    └──────────────┴──────────────────────┴────────────────────────┘

- ``len`` 只描述 JSON header 的字节数；payload 长度写在 header 的 ``payload`` 字段里。
- 数值型消息（IMU / 里程计 / 状态 JSON）只用 header，不带 payload。
- 点云走二进制 payload（float32 交错数组），避免 JSON 序列化 2 MB 点云的巨大开销。

header 公共字段：

======== ==================================================================
``t``    帧类型: ``cloud`` / ``imu`` / ``slam`` / ``grid`` / ``status`` / ``log``
``k``    来源键: ``fused`` / ``front`` / ``rear`` / ``lidar_front`` / ``lowstate`` ...
``ts``   采集端墙钟时间（float 秒，time.time()），ROS2 侧据此打 header.stamp
======== ==================================================================
"""

from __future__ import annotations

import json
import struct
from typing import Any

_LEN = struct.Struct(">I")
_RECV_CHUNK = 1 << 20  # 1 MB


def pack(header: dict[str, Any], payload: bytes = b"") -> bytes:
    """把 header(+可选二进制 payload) 打包成一帧字节串。"""
    header = dict(header)
    header["payload"] = len(payload)
    head = json.dumps(header, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    if payload:
        return _LEN.pack(len(head)) + head + payload
    return _LEN.pack(len(head)) + head


class FrameReader:
    """从（阻塞）socket 流中按帧读取；帧被拆包/粘包都能正确还原。"""

    def __init__(self, sock, chunk: int = _RECV_CHUNK) -> None:
        self._sock = sock
        self._chunk = chunk
        self._buf = bytearray()

    def _take(self, n: int) -> bytes:
        while len(self._buf) < n:
            data = self._sock.recv(self._chunk)
            if not data:
                raise ConnectionError("对端已关闭连接")
            self._buf += data
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out

    def read_frame(self) -> tuple[dict[str, Any], bytes]:
        """阻塞读一帧；对端关闭时抛 ``ConnectionError``。"""
        (head_len,) = _LEN.unpack(self._take(4))
        if head_len == 0 or head_len > (1 << 24):
            raise ConnectionError(f"帧头长度非法: {head_len}")
        try:
            header = json.loads(self._take(head_len).decode("utf-8"))
            payload_len = int(header.get("payload", 0) or 0)
        except (ValueError, json.JSONDecodeError, UnicodeDecodeError) as exc:
            raise ConnectionError(f"帧头解析失败: {exc}") from None
        payload = self._take(payload_len) if payload_len else b""
        return header, payload
