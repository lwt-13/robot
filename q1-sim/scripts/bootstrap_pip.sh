#!/usr/bin/env bash
# 在 WSL 里免 sudo 引导 pip 并安装 MuJoCo 仿真内核。
# 背景：Ubuntu 默认 python3 没有 pip，且 python3-venv 未装导致 ensurepip 不可用，
#       所以这里用 --without-pip 建 venv，再从清华 PyPI 镜像直接取 pip 的 wheel 来运行 pip。
# 幂等：可反复执行。
set -uo pipefail

MIRROR="${PIP_MIRROR:-https://pypi.tuna.tsinghua.edu.cn/simple}"
VENV="$HOME/q1_sim/venv"
TOOLS="$HOME/q1_tools"
WHEEL="$TOOLS/pip.whl"

mkdir -p "$TOOLS"
echo "== 1. 准备 venv =="
if [ ! -x "$VENV/bin/python" ]; then
  python3 -m venv --without-pip "$VENV" && echo "已创建 venv: $VENV"
else
  echo "venv 已存在: $VENV"
fi

echo "== 2. 从镜像获取 pip wheel =="
# 注意：镜像的 simple 页面里版本顺序不可靠（曾取到 pip 9.0.3，在 Python 3.10 下直接 ImportError），
# 所以优先用 JSON API 拿「最新版本」的文件列表，失败再退回 simple 页面 + 版本排序。
if [ ! -s "$WHEEL" ] || ! PYTHONPATH="$WHEEL" "$VENV/bin/python" -m pip --version >/dev/null 2>&1; then
  rm -f "$WHEEL"
  URL="$(curl -sS --max-time 90 --retry 3 "${MIRROR%/simple}/pypi/pip/json" 2>/dev/null \
    | python3 -c "
import json, sys
try:
    d = json.load(sys.stdin)
except Exception:
    sys.exit(1)
cands = [u for u in d.get('urls', []) if u.get('filename', '').endswith('py3-none-any.whl')]
print(cands[0]['url'] if cands else '')
" 2>/dev/null)"

  if [ -z "$URL" ]; then
    echo "JSON API 不可用，退回 simple 页面解析（按版本排序）"
    HTML="$(curl -sS --max-time 90 --retry 3 "$MIRROR/pip/")" || { echo "[FAIL] 无法访问 $MIRROR/pip/"; exit 1; }
    FN="$(printf '%s' "$HTML" | grep -oE 'pip-[0-9][^"/]*py3-none-any\.whl' | sort -uV | tail -1)"
    [ -n "$FN" ] || { echo "[FAIL] 镜像页里没找到 pip 的 wheel 链接"; exit 1; }
    HREF="$(printf '%s' "$HTML" | grep -oE 'href="[^"]*'"$FN"'[^"]*"' | head -1 | sed 's/^href="//; s/"$//; s/#.*$//')"
    URL="$(python3 -c "import urllib.parse,sys; print(urllib.parse.urljoin(sys.argv[1], sys.argv[2]))" "$MIRROR/pip/" "$HREF")"
  fi

  case "$URL" in
    https://*) : ;;
    *) echo "[FAIL] 解析出的下载地址不可信: $URL"; exit 1 ;;
  esac
  echo "下载: $URL"
  curl -sSL --retry 3 --retry-delay 2 --max-time 300 -o "$WHEEL" "$URL" || { echo "[FAIL] pip wheel 下载失败"; exit 1; }
  echo "校验 wheel 可用性:"
  PYTHONPATH="$WHEEL" "$VENV/bin/python" -m pip --version || { echo "[FAIL] 该 wheel 在 Python 3.10 下不可用"; rm -f "$WHEEL"; exit 1; }
else
  echo "pip wheel 已就绪: $WHEEL"
fi
ls -l "$WHEEL"

echo "== 3. 安装 pip、MuJoCo 与验证/抓屏用的小工具 =="
PYTHONPATH="$WHEEL" "$VENV/bin/python" -m pip install --no-warn-script-location -i "$MIRROR" pip \
  || { echo "[FAIL] pip 安装失败"; exit 1; }
# mujoco     : 仿真内核
# PyYAML     : 改 RViz2 配置（做渲染对照实验）
# mss/python-xlib/Pillow : WSLg 下抓 RViz2 窗口截图（系统里没有任何截图工具，装它们要 sudo）
"$VENV/bin/python" -m pip install --no-warn-script-location -i "$MIRROR" \
  mujoco PyYAML mss python-xlib Pillow \
  || { echo "[FAIL] 依赖安装失败"; exit 1; }

echo "== 4. 验证 =="
"$VENV/bin/python" - <<'PY'
import mujoco, numpy, sys
print("python :", sys.version.split()[0])
print("mujoco :", mujoco.__version__)
print("numpy  :", numpy.__version__)
for mod in ("yaml", "mss", "Xlib", "PIL"):
    try:
        __import__(mod)
        print(f"{mod:<7}: OK")
    except ImportError:
        print(f"{mod:<7}: 缺失（只影响部分验证脚本）")
PY
echo "== 完成：pip 与 MuJoCo 就绪 =="