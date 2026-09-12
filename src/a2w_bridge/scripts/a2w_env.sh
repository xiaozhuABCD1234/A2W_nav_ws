#!/usr/bin/env bash
# A2W 桥接环境隔离脚本（**用 source 加载，不要直接执行**）
#
#   source src/a2w_bridge/scripts/a2w_env.sh              # 用包内默认配置
#   source src/a2w_bridge/scripts/a2w_env.sh /path/my.json # 用指定配置
#
# 作用：让本机 ROS2(FastDDS) 与机器人的控制网彻底隔离。
#
# 为什么必须这么做（实测 + 官方文档）：
#   * A2W 的 192.168.123.0/24（交换机1）是官方文档写明的 "DDS控制信号" 局域网；
#   * 本机 ROS2 默认 FastDDS 域 0，会把发现组播 239.255.0.1（含类型对象）打到
#     那张网卡上；机器人侧是 CycloneDDS 0.10.2，跨实现类型对象会让它出问题，
#     运控随即落到 阻尼 = 软急停（官方 error_code 1001）；
#   * 实测：不隔离时 `ros2 topic echo` 会让机器人网卡加入 239.255.0.1；
#     用本脚本（换域 + FastDDS 只走回环）后该网卡上不再有任何 DDS 组播。
#
# 注意：所有需要看到 /a2w/* 话题的本机 ROS2 工具（rviz2、point_lio、ros2 CLI…）
#       都要先 source 本脚本，否则它们不在同一 ROS 域里，互相看不见。
#       机器人数据不经过 ROS2 DDS（采集器 CycloneDDS → TCP → 节点），隔离不影响桥接。

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "请用 source 加载: source ${BASH_SOURCE[0]} [config.json]" >&2
  exit 1
fi

_a2w_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1) 配置文件：命令行参数 > 包内 config/（安装后 share/a2w_bridge/config/）
_a2w_cfg="${1:-}"
if [[ -z "$_a2w_cfg" ]]; then
  for _cand in \
    "$_a2w_here/../config/a2w_bridge.json" \
    "$_a2w_here/../../share/a2w_bridge/config/a2w_bridge.json"; do
    if [[ -f "$_cand" ]]; then
      _a2w_cfg="$_cand"
      break
    fi
  done
fi

# 2) FastDDS 隔离 profile（与配置同目录）
_a2w_profile=""
if [[ -n "$_a2w_cfg" ]]; then
  _a2w_profile="$(cd "$(dirname "$_a2w_cfg")" && pwd)/fastdds_iso.xml"
fi

# 3) ROS 域：从 JSON 的 ros.domain_id 读（缺省 42；域 0 就是机器人所在域，不安全）
_a2w_domain="42"
_a2w_isolate="true"
if [[ -n "$_a2w_cfg" && -f "$_a2w_cfg" ]] && command -v python3 >/dev/null 2>&1; then
  _a2w_json_out="$(
    python3 - "$_a2w_cfg" <<'PY' 2>/dev/null || true
import json, sys
try:
    ros = (json.load(open(sys.argv[1], encoding="utf-8")) or {}).get("ros", {}) or {}
    print(int(ros.get("domain_id", 42)), str(bool(ros.get("isolate", True))).lower())
except Exception:
    pass
PY
  )"
  if [[ -n "$_a2w_json_out" ]]; then
    _a2w_domain="${_a2w_json_out%% *}"
    _a2w_isolate="${_a2w_json_out##* }"
  fi
fi

if [[ "$_a2w_isolate" != "true" ]]; then
  echo "[a2w] 配置里 ros.isolate=false：不做 DDS 隔离（有触发机器狗软急停的风险，慎用）" >&2
  unset _a2w_here _a2w_cfg _a2w_profile _a2w_json_out _a2w_isolate
  return 0
fi

export ROS_DOMAIN_ID="$_a2w_domain"
if [[ -n "$_a2w_profile" && -f "$_a2w_profile" ]]; then
  export FASTDDS_DEFAULT_PROFILES_FILE="$_a2w_profile"
  export FASTRTPS_DEFAULT_PROFILES_FILE="$_a2w_profile"
  echo "[a2w] ROS 已隔离: ROS_DOMAIN_ID=$ROS_DOMAIN_ID, FastDDS 只走回环"
  echo "      配置=$_a2w_cfg"
else
  echo "[a2w] 警告: 找不到 FastDDS 隔离 profile（$_a2w_profile），只设了 ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >&2
fi

unset _a2w_here _a2w_cfg _a2w_profile _a2w_json_out _a2w_isolate
