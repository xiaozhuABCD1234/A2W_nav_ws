"""launch 文件共用的小工具（标准库 + ament_index + launch）。

两个启动器都要做同样几件事，逻辑只写一份，避免漂移：

* ``a2w_bridge.launch.py``      —— 起桥接节点（采集器 + ROS 发布）
* ``a2w_joint_display.launch.py`` —— 起 robot_state_publisher + 关节转发 + RViz

共用内容：

1. 读安装目录里的 JSON 配置拿默认值（``ros.isolate`` / ``ros.domain_id`` / ``display.*``）；
2. 给节点进程设「ROS 域 + FastDDS 只走回环」的隔离环境
   （原因见 README「与机器人控制网隔离」：同域会把 DDS 发现组播打进机器人控制网）；
3. 读 a2w_description 里的 URDF 文本（顺手修正历史 mesh 包名）。
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

# 显示用 URDF 里可能残留的历史 mesh 包名（SolidWorks 导出名）→ 本仓库包名
_MESH_PKG_ALIASES = ("X2-0807urdf(轮式)", "X2-0807urdf", "x2_0807urdf")
MESH_PACKAGE = "a2w_description"


def package_share(package: str) -> Path | None:
    """包的 share 目录；不在 ROS 环境里时返回 None（launch 阶段尽量别抛异常）。"""
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(package))
    except Exception:  # noqa: BLE001 —— 未 source ROS 时退回 None，由调用方兜底
        return None


def load_json(path: str | Path) -> dict[str, Any]:
    """读 JSON 对象；读不到或格式不对就返回空 dict（默认值由调用方给）。"""
    try:
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return raw if isinstance(raw, dict) else {}


def _num(value: Any, default: float) -> float:
    """JSON 里的数字容错（字符串/None/NaN 都不抛异常）。"""
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if out == out and out > 0 else default


def config_defaults(config_path: str | Path | None = None) -> dict[str, Any]:
    """JSON 配置里的 ros/display 默认值（读不到就回安全默认：隔离开、域 42）。"""
    from .config import default_config_path

    path = Path(config_path) if config_path else Path(default_config_path())
    raw = load_json(path)
    ros = raw.get("ros") or {}
    disp = raw.get("display") or {}
    isolate = bool(ros.get("isolate", True))
    try:
        domain = int(ros.get("domain_id", 42))
    except (TypeError, ValueError):
        domain = 42
    if isolate and domain == 0:  # 域 0 正是机器人所在域，会触发软急停，禁止
        domain = 42
    return {
        "config_path": str(path),
        "isolate": isolate,
        "domain_id": domain,
        "profile_name": str(ros.get("fastdds_profile", "fastdds_iso.xml") or ""),
        "display": {
            "enabled": bool(disp.get("enabled", True)),
            "input_topic": str(disp.get("input_topic", "a2w/joint_states") or "a2w/joint_states"),
            "output_topic": str(disp.get("output_topic", "joint_states") or "joint_states"),
            "rate_hz": _num(disp.get("rate_hz"), 50.0),
            "stale_sec": _num(disp.get("stale_sec"), 2.0),
            "flip": [str(x) for x in (disp.get("flip") or [])],
            "urdf": str(disp.get("urdf", "") or ""),
            "rviz_config": str(disp.get("rviz_config", "") or ""),
        },
    }


def fastdds_profile(profile_name: str) -> Path | None:
    """share/a2w_bridge/config/<profile_name>（回环专用 FastDDS profile）。"""
    share = package_share("a2w_bridge")
    if share is None or not profile_name:
        return None
    path = share / "config" / profile_name
    return path if path.is_file() else None


def isolation_actions(cfg: dict[str, Any], isolate: Any, domain_id: Any) -> list[Any]:
    """隔离用的 launch actions：ROS_DOMAIN_ID + FASTDDS_DEFAULT_PROFILES_FILE。

    ``isolate`` / ``domain_id`` 传 ``LaunchConfiguration``（或普通字符串）。
    找不到 profile 时只做域隔离并打印警告（域隔离本身已能拦住跨域发现包）。
    """
    from launch.actions import LogInfo, SetEnvironmentVariable
    from launch.conditions import IfCondition

    actions: list[Any] = [
        SetEnvironmentVariable("ROS_DOMAIN_ID", domain_id, condition=IfCondition(isolate)),
    ]
    profile = fastdds_profile(str(cfg.get("profile_name", "")))
    if profile is not None:
        actions.append(
            SetEnvironmentVariable(
                "FASTDDS_DEFAULT_PROFILES_FILE", str(profile), condition=IfCondition(isolate)
            )
        )
        note = f"FastDDS 只走回环（{profile.name}）"
    else:
        note = "⚠️ 没找到 FastDDS 回环 profile，只做了域隔离"
    actions.append(
        LogInfo(
            msg=["[a2w] ROS 已隔离: ROS_DOMAIN_ID=", domain_id, f"，{note}。", " 自己看 /a2w/* 话题前先 source ", "src/a2w_bridge/scripts/a2w_env.sh"],
            condition=IfCondition(isolate),
        )
    )
    return actions


def default_urdf() -> Path | None:
    """a2w_description 里的 URDF 路径。"""
    share = package_share(MESH_PACKAGE)
    if share is None:
        return None
    path = share / "urdf" / "a2w_description.urdf"
    return path if path.is_file() else None


def default_rviz_config() -> Path | None:
    """a2w_description 自带的 RViz 配置（Grid + TF + RobotModel，Fixed Frame=base_link）。"""
    share = package_share(MESH_PACKAGE)
    if share is None:
        return None
    path = share / "urdf.rviz"
    return path if path.is_file() else None


def read_robot_description(urdf_path: str | Path) -> tuple[str, int]:
    """读 URDF 文本，并修正历史 mesh 包名；返回 ``(文本, 修正处数)``。"""
    text = Path(urdf_path).read_text(encoding="utf-8")
    fixed = 0
    for alias in _MESH_PKG_ALIASES:
        needle = f"package://{alias}/"
        count = text.count(needle)
        if count:
            text = text.replace(needle, f"package://{MESH_PACKAGE}/")
            fixed += count
    return text, fixed


if __name__ == "__main__":
    # 给 launch 文件当「cat 兼 mesh 包名修正」用：
    #   python3 -m a2w_bridge.launch_common <urdf 路径>   →  stdout 输出可直接喂给
    #   robot_state_publisher 的 robot_description
    try:
        _text, _fixed = read_robot_description(sys.argv[1])
    except (IndexError, OSError) as exc:
        print(f"[a2w] 读不到 URDF: {exc}", file=sys.stderr)
        raise SystemExit(2)
    if _fixed:
        print(
            f"[a2w] URDF 里有 {_fixed} 处历史 mesh 包名（SolidWorks 导出名），"
            f"已改成 package://{MESH_PACKAGE}/",
            file=sys.stderr,
        )
    sys.stdout.write(_text)
