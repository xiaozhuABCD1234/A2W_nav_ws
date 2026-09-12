#!/usr/bin/env bash
# A2W 桥接环境隔离脚本（**用 source 加载，不要直接执行**）
#
#   source src/a2w_bridge/scripts/a2w_env.sh              # 自动找包内配置
#   source src/a2w_bridge/scripts/a2w_env.sh /path/my.json # 指定配置
#   A2W_CONFIG=/path/my.json source .../a2w_env.sh         # 或用环境变量指定
#
# 兼容 bash 与 zsh（zsh 下没有 BASH_SOURCE，脚本会自己适配）。
#
# 环境变量：
#   ROS_DOMAIN_ID                       隔离用的 ROS 域（取配置 ros.domain_id，缺省 42）
#   FASTDDS_DEFAULT_PROFILES_FILE        FastDDS 3.x 的 profile 变量（本脚本默认设这个）
#   A2W_LEGACY_FASTRTPS=1                旧发行版（Humble / FastDDS 2.x）才需要，
#                                        额外设已废弃的 FASTRTPS_DEFAULT_PROFILES_FILE；
#                                        FastDDS 3.x 设了它会报 deprecation WARN，所以默认不设
#
# 作用：让本机 ROS2(FastDDS) 与机器人的控制网彻底隔离。
#
# 为什么必须这么做（实测 + 官方文档）：
#   * A2W 的 192.168.123.0/24（交换机1）是官方文档写明的 "DDS控制信号" 局域网；
#   * 本机 ROS2 默认 FastDDS 域 0，会把发现组播 239.255.0.1（含类型对象）打到
#     那张网卡上；机器人侧是 CycloneDDS 0.10.2，跨实现类型对象会让它出问题，
#     运控随即落到 阻尼 = 软急停（官方 error_code 1001）；
#   * 实测：不隔离时 `ros2 topic echo` 会让机器人网卡加入 239.255.0.1；
#     本脚本（换域 + FastDDS 只走回环）之后该网卡不再有任何 ROS2 的 DDS 组播。
#
# 注意：所有需要看到 /a2w/* 话题的本机 ROS2 工具（rviz2、point_lio、ros2 CLI…）
#       都要先 source 本脚本，否则它们不在同一 ROS 域里，互相看不见。
#       机器人数据不经过 ROS2 DDS（采集器 CycloneDDS → TCP → 节点），隔离不影响桥接。

# ---- 定位脚本自身（bash 用 BASH_SOURCE；zsh 的 $0 就是脚本路径）--------
if [[ -n "${BASH_SOURCE:-}" ]]; then
  _a2w_src="${BASH_SOURCE[0]}"
else
  _a2w_src="$0"
fi
_a2w_dir="$(cd "$(dirname "$_a2w_src")" 2>/dev/null && pwd)"

# ---- 是否被 source（直接执行时给提示）---------------------------------
_a2w_sourced=0
if [[ -n "${BASH_SOURCE:-}" ]]; then
  [[ "${BASH_SOURCE[0]}" != "$0" ]] && _a2w_sourced=1
elif [[ -n "${ZSH_VERSION:-}" ]]; then
  case "${ZSH_EVAL_CONTEXT:-}" in *:file* | file) _a2w_sourced=1 ;; esac
fi
if [[ "$_a2w_sourced" != "1" ]]; then
  echo "请用 source 加载: source $_a2w_src [config.json]" >&2
  exit 1
fi

# ---- 1) 找配置文件：A2W_CONFIG > 参数 > 包内 config/ > install share/ > 当前目录 --
_a2w_cfg="${A2W_CONFIG:-${1:-}}"
if [[ -z "$_a2w_cfg" ]]; then
  for _cand in \
    "$_a2w_dir/../config/a2w_bridge.json" \
    "$_a2w_dir/../../share/a2w_bridge/config/a2w_bridge.json" \
    "$PWD/install/a2w_bridge/share/a2w_bridge/config/a2w_bridge.json" \
    "$PWD/src/a2w_bridge/config/a2w_bridge.json"; do
    if [[ -f "$_cand" ]]; then
      _a2w_cfg="$_cand"
      break
    fi
  done
fi

# ---- 2) 找 FastDDS 隔离 profile：配置同目录 > install share ---------------
_a2w_profile=""
if [[ -n "$_a2w_cfg" ]]; then
  _a2w_profile="$(cd "$(dirname "$_a2w_cfg")" && pwd)/fastdds_iso.xml"
fi
for _cand in \
  "$_a2w_profile" \
  "$_a2w_dir/../config/fastdds_iso.xml" \
  "$PWD/install/a2w_bridge/share/a2w_bridge/config/fastdds_iso.xml" \
  "$PWD/src/a2w_bridge/config/fastdds_iso.xml"; do
  if [[ -n "$_cand" && -f "$_cand" ]]; then
    _a2w_profile="$_cand"
    break
  fi
done

# ---- 3) 读 JSON 的 ros.domain_id / ros.isolate（缺省 42 / true）-----------
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

# ---- 4) 导出环境 ------------------------------------------------------------
if [[ "$_a2w_isolate" != "true" ]]; then
  echo "[a2w] 配置里 ros.isolate=false：不做 DDS 隔离（有触发机器狗软急停的风险，慎用）" >&2
  unset _a2w_src _a2w_dir _a2w_cfg _a2w_profile _a2w_json_out _a2w_isolate _a2w_sourced
  return 0
fi
export ROS_DOMAIN_ID="$_a2w_domain"
if [[ -n "$_a2w_profile" && -f "$_a2w_profile" ]]; then
  export FASTDDS_DEFAULT_PROFILES_FILE="$_a2w_profile"
  # 旧版 FastDDS（2.x，如 Humble）只认 FASTRTPS_*；3.x 认它但会打 deprecation WARN。
  if [[ "${A2W_LEGACY_FASTRTPS:-0}" == "1" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$_a2w_profile"
  else
    unset FASTRTPS_DEFAULT_PROFILES_FILE
  fi
  echo "[a2w] ROS 已隔离: ROS_DOMAIN_ID=$ROS_DOMAIN_ID, FastDDS 只走回环"
  echo "      配置=$_a2w_cfg"
  echo "      profile=$_a2w_profile"
else
  echo "[a2w] 警告: 找不到 FastDDS 隔离 profile，只设了 ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >&2
  echo "      请检查 config/fastdds_iso.xml 是否存在，或用 A2W_CONFIG=<json> 指定配置" >&2
fi

unset _a2w_src _a2w_dir _a2w_cfg _a2w_profile _a2w_json_out _a2w_isolate _a2w_sourced
