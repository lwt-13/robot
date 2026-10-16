#!/usr/bin/env bash
# 自动验证 ROS2 侧是否真的可用：显示链路每个环节 + RViz2 能否加载配置。
# 需要在 build_ws.sh 之后运行。不需要 sudo。
#
# 注意：不要用 `ros2 topic echo` 判断 /robot_description 是否发布——
# 它会把 28 KB 的 URDF 截断成 154 字节，导致假失败。检查交给 check_display.py。
#
# 用法:  bash verify_ros2.sh
set -uo pipefail
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$HOME/q1_ws/install/setup.bash" || { echo "[FAIL] 请先运行 build_ws.sh"; exit 1; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MUJOCO_DIR="$(cd "$SCRIPT_DIR/../mujoco" && pwd)"
RSP_LOG=/tmp/q1-rsp.log
RVIZ_LOG=/tmp/q1-rviz.log
# venv 里有 mujoco，且经 setup.bash 的 PYTHONPATH 也能看到 rclpy
PY=python3
[ -x "$HOME/q1_sim/venv/bin/python" ] && PY="$HOME/q1_sim/venv/bin/python"
fail=0

cleanup() { pkill -f 'display.launch.py' >/dev/null 2>&1; sleep 1; }
trap cleanup EXIT

echo "===== 1. 显示链路（robot_state_publisher + joint_state_publisher）====="
cleanup
ros2 launch q1_description display.launch.py rviz:=false gui:=false > "$RSP_LOG" 2>&1 &
sleep 8
if grep -qiE '\[ERROR\]|unable to initialize|Traceback' "$RSP_LOG"; then
  echo "  [FAIL] 启动日志里有错误:"
  grep -iE '\[ERROR\]|unable to initialize|Traceback' "$RSP_LOG" | head -8
  fail=1
else
  echo "  [PASS] 启动日志无错误"
fi

"$PY" "$SCRIPT_DIR/../ros2/check_display.py" --mujoco-dir "$MUJOCO_DIR" || fail=1
cleanup

echo
echo "===== 2. RViz2 能否加载配置 ====="
RVIZ_CFG="$HOME/q1_ws/install/q1_description/share/q1_description/rviz/q1.rviz"
if [ -f "$RVIZ_CFG" ]; then
  for pat in 'RobotModel' 'robot_description' 'pelvis'; do
    if grep -q "$pat" "$RVIZ_CFG"; then
      echo "  [PASS] 配置里包含 $pat"
    else
      echo "  [FAIL] 配置里缺少 $pat"; fail=1
    fi
  done
  echo "  固定坐标系: $(grep -m1 -E '^[[:space:]]*Fixed Frame:' "$RVIZ_CFG" | sed 's/^[[:space:]]*//')"
else
  echo "  [FAIL] 找不到 RViz2 配置 $RVIZ_CFG"; fail=1
fi

rosa=$(command -v rviz2)
timeout 45 "$rosa" -d "$RVIZ_CFG" > "$RVIZ_LOG" 2>&1 &
RVIZ_PID=$!
sleep 12
if grep -qiE '\[ERROR\]|Failed to|Could not' "$RVIZ_LOG"; then
  echo "  [WARN] RViz2 日志里有错误/警告，节选:"
  grep -iE '\[ERROR\]|Failed to|Could not' "$RVIZ_LOG" | head -8
else
  echo "  [PASS] RViz2 日志无错误"
fi
if kill -0 "$RVIZ_PID" 2>/dev/null; then
  echo "  [PASS] RViz2 进程存活（配置已加载，未崩溃）"
else
  echo "  [FAIL] RViz2 进程已退出，配置可能不合法"; fail=1
fi
echo "  提示: RViz2 的窗口通过 WSLg 直接显示在你的 Windows 桌面上"
rm -f /tmp/q1-rviz-shot.* 2>/dev/null
for tool in import xwd scrot gnome-screenshot grim ffmpeg; do
  command -v "$tool" >/dev/null 2>&1 && echo "  可用的截图工具: $tool"
done

kill "$RVIZ_PID" 2>/dev/null
wait "$RVIZ_PID" 2>/dev/null

echo
echo "===== 结论 ====="
if [ "$fail" -eq 0 ]; then
  echo "[PASS] ROS2 显示链路验证通过"
else
  echo "[FAIL] 有检查项未通过，见 $RSP_LOG 与 $RVIZ_LOG"
fi
exit "$fail"