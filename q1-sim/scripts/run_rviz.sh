#!/usr/bin/env bash
# 启动 RViz2 查看 Q1 模型（需要先跑 install-ros2.sh 与 build_ws.sh）。
set -uo pipefail
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$HOME/q1_ws/install/setup.bash" || { echo "[FAIL] 请先运行 build_ws.sh"; exit 1; }

echo "启动 RViz2（关上窗口即退出）。fixed frame = pelvis，可用左侧关节滑块拖动。"
exec ros2 launch q1_description display.launch.py "$@"