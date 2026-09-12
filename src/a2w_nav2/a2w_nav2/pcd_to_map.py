#!/usr/bin/env python3
"""PCD → 2D 占据栅格（``.pgm`` + ``.yaml``），给 Nav2 的 ``map_server`` + AMCL 用。

为什么自己写（而不是 octomap_server）
====================================

本机 ROS 2 Lyrical 的 apt 里**没有 octomap_server**（`ros-lyrical-octomap-server`
不存在），而且这里要的也不是"3D 占据树" —— 只是把 Point-LIO 的建图点云按一个
**离地高度带**压成 2D 栅格。带的高度基准跟 ``a2w_scan`` 保持一致：

* 机身占地面以上 0.014~0.224 m（实测）→ 带的下沿取 **离地 0.30 m**，避开自车体/地面；
* 天花板在 2.2 m → 上沿取 **离地 1.00 m**，避开天花板与顶上杂物。

算法（三态，直接对应 map_server 的 trinary 语义）
================================================

对每个分辨率格子统计落点（都只看 **x,y**，高度只用来选带）：

=================  ==================================================
占据 (0 / 黑)      **带内**落点数 ≥ ``--min-hits``（默认 2）
空闲 (254 / 白)    有任意高度的落点，但带内 0 点 → 说明这格"看得见地面/别处"，
                   即带内没有障碍
未知 (205 / 灰)    整格一个点都没有
=================  ==================================================

地平面不用手填：默认从点云 z 直方图里自动找（最下面的主峰），``--floor-z`` 可覆盖。
输出的 yaml 里 ``origin`` 就是栅格左下角在**建图时 LIO 世界系**里的坐标 ——
所以 map_server 的 ``frame_id: map`` 与建图起点对齐，AMCL 的 ``set_initial_pose``
用 (0,0,0) 就对应"把狗放在建图起点、朝向一致"。

用法::

    ros2 run a2w_nav2 a2w_pcd_to_map PCD/scans.pcd -o maps/a2w_map
    ros2 run a2w_nav2 a2w_pcd_to_map PCD/scans_*.pcd -o maps/a2w_map --z-min 0.30 --z-max 1.00
    # 先看看带选得对不对（只打印统计与预览，不写文件）
    ros2 run a2w_nav2 a2w_pcd_to_map PCD/scans.pcd --dry-run
"""

from __future__ import annotations

import argparse
import math
import os
import struct
from typing import Any
import sys

import numpy as np

# 与 a2w_scan 的扫描带保持一致（离地高度，m）
DEFAULT_Z_MIN = 0.30
DEFAULT_Z_MAX = 1.00
DEFAULT_RESOLUTION = 0.05
DEFAULT_MIN_HITS = 2
DEFAULT_PAD = 1.0

# PGM 三态取值（与 map_server 的 trinary 语义一致）
PIX_OCCUPIED = 0
PIX_UNKNOWN = 205
PIX_FREE = 254

# PCD 字段类型 → numpy dtype（type: F=float, I=有符号, U=无符号；size=字节数）
_PCD_TYPES = {
    ("F", 4): "f4", ("F", 8): "f8",
    ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4", ("I", 8): "i8",
    ("U", 1): "u1", ("U", 2): "u2", ("U", 4): "u4", ("U", 8): "u8",
}


def _to_int(value: Any, default: int = 0) -> int:
    """防御式取整（PCD 头部/统计值都用它；坏值给 default，绝不抛异常）。"""
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):  # noqa: BLE001 —— 防御
        return default


def _to_float(value: Any, default: float = 0.0) -> float:
    """防御式取浮点（同上）。"""
    try:
        return float(value)
    except (TypeError, ValueError, OverflowError):  # noqa: BLE001 —— 防御
        return default


class PcdError(RuntimeError):
    """PCD 读取失败（缺字段/格式不支持等）。"""


def read_pcd(path: str) -> np.ndarray:
    """读 PCD，返回 (N,3) float64 的 x/y/z。

    支持 ``ascii`` 与 ``binary``（Point-LIO 用 ``writeBinary`` 写盘）；
    ``binary_compressed``（LZF）不支持 —— 会明确报错让用户转换。
    """
    try:
        with open(path, "rb") as f:
            header: dict[str, list[str]] = {}
            while True:
                raw = f.readline()
                if not raw:
                    raise PcdError(f"{path}: 文件在 DATA 行之前就结束了")
                line = raw.decode("ascii", errors="replace").strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                key = parts[0].upper()
                if key == "DATA":
                    data_kind = (parts[1].lower() if len(parts) > 1 else "ascii")
                    break
                header[key] = parts[1:]
            payload = f.read()
    except OSError as e:
        raise PcdError(f"{path}: 打不开（{e}）") from e

    fields = header.get("FIELDS", [])
    if not {"x", "y", "z"} <= set(fields):
        raise PcdError(f"{path}: 缺 x/y/z 字段（实到 {fields}）")
    sizes = [_to_int(s, -1) for s in header.get("SIZE", [])]
    types = [t.upper() for t in header.get("TYPE", [])]
    counts = [_to_int(c, 1) for c in header.get("COUNT", ["1"] * len(fields))]
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise PcdError(f"{path}: 头部 FIELDS/SIZE/TYPE/COUNT 长度不一致")

    if data_kind == "binary_compressed":
        raise PcdError(
            f"{path}: binary_compressed（LZF）暂不支持 —— 用 pcl_convert_pcd_ascii_binary "
            f"或 Python 另存为 ascii/binary 再来"
        )

    names: list[str] = []
    formats: list[str] = []
    for name, size, typ, cnt in zip(fields, sizes, types, counts):
        fmt = _PCD_TYPES.get((typ, size))
        if fmt is None:
            raise PcdError(f"{path}: 不认识的字段类型 {name} type={typ} size={size}")
        names.extend([name] * cnt)
        formats.extend([fmt] * cnt)

    if data_kind == "ascii":
        try:
            table = np.loadtxt(payload.decode("ascii", errors="replace").splitlines())
        except ValueError as e:
            raise PcdError(f"{path}: ASCII 数据解析失败（{e}）") from e
        table = np.atleast_2d(table)
        if table.shape[1] < len(names):
            raise PcdError(f"{path}: ASCII 列数 {table.shape[1]} 少于头部字段数 {len(names)}")
        col = {n: table[:, i] for i, n in enumerate(names)}
    elif data_kind == "binary":
        dtype = np.dtype([(n, f) for n, f in zip(names, formats)])
        need = dtype.itemsize
        n_pts = len(payload) // need
        table = np.frombuffer(payload[: n_pts * need], dtype=dtype)
        col = {n: np.asarray(table[n], dtype=np.float64) for n in ("x", "y", "z")}
    else:
        raise PcdError(f"{path}: 不支持的数据格式 {data_kind!r}（只支持 ascii / binary）")

    pts = np.stack([np.asarray(col["x"], dtype=np.float64),
                    np.asarray(col["y"], dtype=np.float64),
                    np.asarray(col["z"], dtype=np.float64)], axis=1)
    finite = np.isfinite(pts).all(axis=1)
    return pts[finite]


def detect_floor_z(z: np.ndarray) -> float:
    """自动找地平面：z 最低处起 0.02 m 一档，取最下面的主峰（超过 3% 的第一峰）。"""
    lo = _to_float(np.min(z))
    hi = lo + 0.60
    hist, edges = np.histogram(z, bins=np.arange(lo, hi + 1e-9, 0.02))
    if hist.size == 0 or hist.sum() == 0:
        return lo
    thr = max(1.0, 0.03 * hist.sum())
    for i, c in enumerate(hist):
        if c >= thr:
            # 用该档附近 ±0.05 m 的中位数细化
            near = z[(z >= edges[i] - 0.05) & (z <= edges[i] + 0.07)]
            if near.size:
                return _to_float(np.median(near))
            return _to_float((edges[i] + edges[i + 1]) / 2.0)
    return lo


def build_grid(points: np.ndarray, floor_z: float, args: argparse.Namespace):
    """返回 (grid, x0, y0, origin) ；grid 是 (rows, cols) 的 uint8 三态图（row0 = 顶部）。"""
    band = (points[:, 2] >= floor_z + args.z_min) & (points[:, 2] <= floor_z + args.z_max)
    print(f"高度带（离地）[{args.z_min:.2f}, {args.z_max:.2f}] m → 绝对 z "
          f"[{floor_z + args.z_min:+.3f}, {floor_z + args.z_max:+.3f}]；带内点 "
          f"{_to_int(band.sum())} / {len(points)}（{band.mean() * 100:.1f}%）")
    if band.sum() == 0:
        raise PcdError("带内一个点都没有 —— 高度带选错了（用 --floor-z/--z-min/--z-max 调）")

    lo = points[:, :2].min(axis=0) - args.pad
    hi = points[:, :2].max(axis=0) + args.pad
    res = args.resolution
    n_x = _to_int(math.ceil((hi[0] - lo[0]) / res), 1)
    n_y = _to_int(math.ceil((hi[1] - lo[1]) / res), 1)
    print(f"栅格 {n_x}×{n_y}（{n_x * res:.1f}×{n_y * res:.1f} m）@ {res} m/格；"
          f"左下角 ({lo[0]:+.2f}, {lo[1]:+.2f})")

    def keys(sel: np.ndarray) -> np.ndarray:
        ix = np.floor((points[sel, 0] - lo[0]) / res).astype(np.int64)
        iy = np.floor((points[sel, 1] - lo[1]) / res).astype(np.int64)
        np.clip(ix, 0, n_x - 1, out=ix)
        np.clip(iy, 0, n_y - 1, out=iy)
        return ix * n_y + iy

    seen_key, _ = np.unique(keys(np.ones(len(points), dtype=bool)), return_counts=True)
    band_key, band_cnt = np.unique(keys(band), return_counts=True)

    grid = np.full(n_x * n_y, PIX_UNKNOWN, dtype=np.uint8)
    grid[seen_key] = PIX_FREE                     # 看得见 → 带内无遮挡
    grid[band_key[band_cnt >= args.min_hits]] = PIX_OCCUPIED   # 带内够多命中 → 障碍

    n_occ = _to_int((grid == PIX_OCCUPIED).sum())
    n_free = _to_int((grid == PIX_FREE).sum())
    n_unk = _to_int((grid == PIX_UNKNOWN).sum())
    tot = max(1, grid.size)
    print(f"占据 {n_occ} 格（{n_occ / tot * 100:.1f}%）| 空闲 {n_free}（{n_free / tot * 100:.1f}%）"
          f"| 未知 {n_unk}（{n_unk / tot * 100:.1f}%）")
    if args.min_hits > 1:
        dropped = _to_int(((band_cnt > 0) & (band_cnt < args.min_hits)).sum())
        print(f"被 --min-hits={args.min_hits} 丢掉的稀疏格: {dropped}（去噪用）")

    # 栅格 → 图像：图像 row0 在顶部 = y 最大处，所以 y 要翻过来
    grid2d = grid.reshape(n_x, n_y).T[::-1, :]
    return grid2d, _to_float(lo[0]), _to_float(lo[1])


def write_pgm(path: str, grid: np.ndarray) -> None:
    rows, cols = (_to_int(grid.shape[0], 1), _to_int(grid.shape[1], 1))
    try:
        with open(path, "wb") as f:
            f.write(f"P5\n{cols} {rows}\n255\n".encode("ascii"))
            f.write(grid.tobytes())
    except OSError as e:
        raise PcdError(f"{path}: 写不进去（{e}）") from e


def write_yaml(path: str, image_name: str, res: float, x0: float, y0: float,
               floor_z: float, args: argparse.Namespace) -> None:
    # free_thresh 必须用经典的 0.196（不是 map_saver 默认的 0.25）：
    # map_server 的 trinary 判定是 occ=(255-v)/255 → occ>occupied_thresh 为占据、
    # occ<free_thresh 为空闲；未知用的灰度是 205 → occ=(255-205)/255=0.196078…，
    # 只有 free_thresh=0.196 时它才“不小于”阈值、落在中间 → 判为未知。
    # （实测：用 0.25 时未知格会被当成空闲，`track_unknown_space` 也救不回来。）
    text = f"""# A2W 2D 占据栅格（由 a2w_pcd_to_map 从 Point-LIO 的 PCD 投影而来）
#
# 生成参数：高度带（离地）[{args.z_min:.2f}, {args.z_max:.2f}] m，min-hits={args.min_hits}，
#           自动化地平面 z = {floor_z:+.3f}（建图时 LIO 世界系，≈ 雷达离地高度的相反数）
# 坐标系：由 map_server 的 frame_id 决定（默认 map）；origin 是栅格左下角的 (x, y, yaw)，
#         原点即【建图起点】——所以 AMCL 的 initial_pose 用 (0,0,0) 就是"放回建图起点"。
image: {image_name}
resolution: {res}
origin: [{x0:.3f}, {y0:.3f}, 0.000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
mode: trinary
"""
    try:
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
    except OSError as e:
        raise PcdError(f"{path}: 写不进去（{e}）") from e


def preview(grid: np.ndarray, max_cols: int = 100) -> None:
    """终端里打一张缩略图（占据 #，空闲 .，未知 空格）。"""
    step = max(1, grid.shape[1] // max_cols)
    chars = {PIX_OCCUPIED: "#", PIX_FREE: ".", PIX_UNKNOWN: " "}
    print(f"\n预览（每字符 {step} 格；# 障碍，. 空闲，空格 未知）")
    for row in grid[::max(1, step * 2)]:
        print("  " + "".join(chars.get(_to_int(v, -1), "?") for v in row[::step]))


def parse_args(argv: list[str]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Point-LIO 的 PCD → 2D 占据栅格（pgm + yaml），给 nav2 map_server/AMCL 用",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("pcd", nargs="+", help="输入的 PCD（可多个，会合并；支持 ascii/binary）")
    ap.add_argument("-o", "--out", default="maps/a2w_map", help="输出前缀（写 <前缀>.pgm/.yaml）")
    ap.add_argument("--resolution", type=float, default=DEFAULT_RESOLUTION, help="栅格分辨率 m")
    ap.add_argument("--z-min", type=float, default=DEFAULT_Z_MIN, help="高度带下沿（离地 m）")
    ap.add_argument("--z-max", type=float, default=DEFAULT_Z_MAX, help="高度带上沿（离地 m）")
    ap.add_argument("--floor-z", type=float, default=None,
                    help="地平面在点云里的 z（不给就自动找最下面的主峰）")
    ap.add_argument("--min-hits", type=int, default=DEFAULT_MIN_HITS, help="判为占据的最少点数")
    ap.add_argument("--pad", type=float, default=DEFAULT_PAD, help="四周留的未知边（m）")
    ap.add_argument("--dry-run", action="store_true", help="只统计+预览，不写文件")
    return ap.parse_args(argv)


def main() -> None:
    args = parse_args(sys.argv[1:])
    if args.resolution <= 0 or args.z_max <= args.z_min or args.min_hits < 1 or args.pad < 0:
        print("参数不合法：resolution>0、z-max>z-min、min-hits≥1、pad≥0", file=sys.stderr)
        raise SystemExit(2)

    clouds = []
    for path in args.pcd:
        pts = read_pcd(path)
        print(f"{path}: {len(pts)} 点")
        clouds.append(pts)
    points = np.vstack(clouds)
    print(f"合并后 {len(points)} 点；z 范围 [{points[:, 2].min():+.3f}, {points[:, 2].max():+.3f}]")

    floor_z = args.floor_z
    if floor_z is None:
        floor_z = detect_floor_z(points[:, 2])
        print(f"自动找地平面：z = {floor_z:+.3f} m（≈ 雷达建图时的离地高度）；"
              f"可用 --floor-z 覆盖")
    else:
        print(f"用你给的 --floor-z = {floor_z:+.3f} m")

    grid, x0, y0 = build_grid(points, floor_z, args)
    preview(grid)

    if args.dry_run:
        print("\n--dry-run：没有写文件")
        return
    pgm = f"{args.out}.pgm"
    yml = f"{args.out}.yaml"
    try:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    except OSError as e:
        print(f"建目录失败：{e}", file=sys.stderr)
        raise SystemExit(1) from e
    write_pgm(pgm, grid)
    write_yaml(yml, os.path.basename(pgm), args.resolution, x0, y0, floor_z, args)
    print(f"\n已写 {pgm} + {yml}\n"
          f"起导航：ros2 launch a2w_nav2 a2w_nav2.launch.py map:={os.path.abspath(yml)}")


if __name__ == "__main__":
    main()
