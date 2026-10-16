#!/usr/bin/env bash
# 编译 colcon 工作区：aimdk_msgs（SDK 消息）+ q1_description（Q1 模型描述）。
# 需要先跑过 install-ros2.sh。不需要 sudo，幂等。
set -uo pipefail
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash || { echo "[FAIL] 找不到 /opt/ros/humble，请先执行 sudo bash install-ros2.sh"; exit 1; }

WS="$HOME/q1_ws"
cd "$WS" || { echo "[FAIL] 工作区不存在: $WS"; exit 1; }

echo "== 工作区包 =="
ls -1 src

echo "== colcon build =="
colcon build \
  --symlink-install \
  --packages-select aimdk_msgs q1_description \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_cohesion+ \
  2>&1 | tail -25
build_rc=${PIPESTATUS[0]}
echo "colcon 退出码: $build_rc"
if [ "$build_rc" -ne 0 ]; then
  echo "[FAIL] 编译失败。若是 aimdk_msgs 报缺少 rosidl 依赖，请确认 install-ros2.sh 装全了 rosidl-default-generators/runtime。"
  exit "$build_rc"
fi

echo "== 加载并验证 =="
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

echo "-- 包 --"
ros2 pkg prefix aimdk_msgs
ros2 pkg prefix q1_description

echo "-- 消息/服务类型数量 --"
echo "msg : $(ros2 interface list | grep -c 'aimdk_msgs/msg/')"
echo "srv : $(ros2 interface list | grep -c 'aimdk_msgs/srv/')"

echo "-- 关键类型抽查 --"
ros2 interface show aimdk_msgs/msg/JointCommand || true
ros2 interface show aimdk_msgs/srv/PlayTts || true

echo "-- URDF 是否可被描述包解析 --"
python3 -c "
import os
from ament_index_python.packages import get_package_share_directory
p = os.path.join(get_package_share_directory('q1_description'), 'urdf', 'q1_v3.urdf')
print('URDF:', p, os.path.getsize(p), 'bytes')
" || echo "[WARN] 无法定位 q1_description 的 urdf"

echo "== 编译与验证完成 =="