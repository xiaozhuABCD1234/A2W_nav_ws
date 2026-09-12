#!/usr/bin/env bash
# 创建采集器专用 Python 3.10 虚拟环境。
#
# 为什么必须 3.10：unitree_sdk2py 依赖 cyclonedds==0.10.2（PyPI 只提供 cp37~cp310 wheel），
# 而本机 ROS2（Lyrical）的 rclpy 跑在 Python 3.14 上，两者无法共存一个解释器，
# 所以采集器单独用这个 3.10 venv（见 README「架构」）。
#
# 依赖：uv（https://docs.astral.sh/uv/）。unitree_sdk2py 本身不装，
# 运行期通过配置里的 collector.sdk_path（PYTHONPATH）指向官方/上游 SDK 源码。
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VENV="${A2W_COLLECTOR_VENV:-$WS_ROOT/.venv-collector}"

if command -v uv >/dev/null 2>&1; then
    echo "[setup] 使用 uv 创建 $VENV (Python 3.10)"
    uv venv --python 3.10 "$VENV"
    uv pip install --python "$VENV/bin/python" "cyclonedds==0.10.2" numpy
else
    echo "[setup] 没有 uv，尝试系统 python3.10" >&2
    command -v python3.10 >/dev/null 2>&1 || {
        echo "[setup] 也没有 python3.10。请安装 uv（curl -LsSf https://astral.sh/uv/install.sh | sh）" >&2
        exit 1
    }
    python3.10 -m venv "$VENV"
    "$VENV/bin/pip" install "cyclonedds==0.10.2" numpy
fi

echo "[setup] 完成。验证："
"$VENV/bin/python" -c "import cyclonedds, numpy; print('  cyclonedds', cyclonedds.version, 'numpy', numpy.__version__)"