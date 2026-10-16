#!/usr/bin/env bash
# 安装 ROS2 Humble（Jammy）+ 编译工具 + MuJoCo 渲染依赖。
# 必须用 root 执行：  sudo bash install-ros2.sh
#
# 为什么用清华镜像：本机 packages.ros.org 的 TLS 校验失败（curl exit 60），
# 而 mirrors.tuna.tsinghua.edu.cn / mirrors.aliyun.com 均可达。
# 公钥获取见 fetch-ros-key.sh：raw.githubusercontent.com 不可达、镜像站 ros.key 为 404，
# 因此改用 keyserver.ubuntu.com 并强制校验完整指纹。
# 幂等：可反复执行；日志同时输出到终端与 /tmp/q1-install-ros2.log
set -uo pipefail

JAMMY_MIRROR="${ROS_MIRROR:-https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu}"
PIP_MIRROR="${PIP_MIRROR:-https://pypi.tuna.tsinghua.edu.cn/simple}"
LOG=/tmp/q1-install-ros2.log
# 避免任何交互式提问把无人值守的安装挂住（needrestart 在 22.04 上也会弹问）
export DEBIAN_FRONTEND=noninteractive
export NEEDRESTART_MODE=a
exec > >(tee -a "$LOG") 2>&1

echo "===== Q1 ROS2 Humble 安装 $(date -Is) ====="
if [ "$(id -u)" -ne 0 ]; then
  echo "[FAIL] 请用 sudo 运行： sudo bash $0"
  exit 1
fi
if [ -n "${SUDO_USER:-}" ]; then
  TARGET_USER="$SUDO_USER"
else
  TARGET_USER="$(logname 2>/dev/null || echo "$USER")"
fi
echo "目标用户: $TARGET_USER"

echo "== 1. 把 Ubuntu 归档源换成国内镜像 =="
# 实测：archive.ubuntu.com 约 2 MB/s，清华镜像约 18.7 MB/s（快约 9 倍）。
# ros-humble-desktop 的依赖闭包是 1343 个包、约 760 MB，换源能省下大量等待时间。
UBUNTU_MIRROR="${UBUNTU_MIRROR:-https://mirrors.tuna.tsinghua.edu.cn/ubuntu}"
if [ "${SKIP_UBUNTU_MIRROR:-0}" = "1" ]; then
  echo "按 SKIP_UBUNTU_MIRROR=1 跳过，继续使用原源"
else
  SRC=/etc/apt/sources.list
  if [ -f "$SRC" ] && grep -qE 'https?://(archive\.ubuntu\.com|security\.ubuntu\.com|[a-z]{2}\.archive\.ubuntu\.com)/ubuntu' "$SRC"; then
    [ -f "${SRC}.q1-backup" ] || cp -a "$SRC" "${SRC}.q1-backup"
    sed -i -E "s#https?://(archive\.ubuntu\.com|security\.ubuntu\.com|[a-z]{2}\.archive\.ubuntu\.com)/ubuntu#${UBUNTU_MIRROR}#g" "$SRC"
    echo "已替换为 $UBUNTU_MIRROR"
    echo "原文件备份: ${SRC}.q1-backup"
    echo "还原命令  : sudo cp ${SRC}.q1-backup ${SRC} && sudo apt-get update"
    grep -E '^deb ' "$SRC" | head -6
  else
    echo "未发现官方 archive/security 源（可能已换过源），跳过"
  fi
fi

echo "== 2. 基础包 =="
apt-get update -qq || { echo "[FAIL] apt-get update 失败"; exit 1; }
apt-get install -y --no-install-recommends curl gnupg lsb-release ca-certificates \
  || { echo "[FAIL] 基础包安装失败"; exit 1; }

echo "== 3. 配置 ROS2 源（镜像：$JAMMY_MIRROR）=="
KEYRING=/usr/share/keyrings/ros-archive-keyring.gpg
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 公钥获取交给独立脚本：它会从 keyserver 或 GitHub 代理取，并**强制校验完整指纹**。
# 本机 raw.githubusercontent.com 不可达、镜像站 ros.key 为 404，所以这一步是最容易失败的环节。
if bash "$SCRIPT_DIR/fetch-ros-key.sh" "$KEYRING"; then
  KEYSPEC="signed-by=$KEYRING"
elif [ "${ALLOW_TRUSTED:-0}" = "1" ]; then
  echo "[WARN] 未能取得并校验 ROS 公钥；因 ALLOW_TRUSTED=1，改用 trusted=yes（跳过包签名校验，安全性降低）"
  KEYSPEC="trusted=yes"
else
  echo "[FAIL] 无法获取并校验 ROS 签名公钥，已中止（不使用未校验的源）"
  echo "       如果只求先跑通、接受跳过签名校验，可执行："
  echo "         ALLOW_TRUSTED=1 sudo -E bash $0"
  exit 1
fi

echo "deb [arch=$(dpkg --print-architecture) $KEYSPEC] $JAMMY_MIRROR jammy main" \
  > /etc/apt/sources.list.d/ros2.list
cat /etc/apt/sources.list.d/ros2.list

echo "== 4. 安装 ROS2 Humble 与工具链（预计下载约 760 MB / 1343 个包）=="
apt-get update -qq || { echo "[FAIL] 加源后 apt-get update 失败，检查镜像连通性"; exit 1; }
apt-get install -y \
  ros-humble-desktop \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  python3-colcon-common-extensions \
  python3-dev python3-pip python3-venv python3-yaml \
  build-essential cmake git \
  || { echo "[FAIL] ROS2 安装失败"; exit 1; }

echo "== 5. MuJoCo 渲染依赖（RViz2/查看器需要）=="
# 注意：jammy 上的 GLEW 包名是 libglew2.2（libglew2.1 在 22.04 里不存在，已核对索引）
apt-get install -y --no-install-recommends \
  libgl1 libgl1-mesa-dri libegl1 libglu1-mesa libglew2.2 libosmesa6 libglfw3 mesa-utils \
  || echo "[WARN] 部分渲染依赖安装失败，MuJoCo 无头模式（MUJOCO_GL=egl）仍可用"

echo "== 6. 配置 pip 镜像（写入 /etc/pip.conf）=="
cat > /etc/pip.conf <<EOF
[global]
index-url = $PIP_MIRROR
trusted-host = pypi.tuna.tsinghua.edu.cn
EOF
cat /etc/pip.conf

echo "== 7. 为用户配置 ROS2 环境变量 =="
BASHRC="/home/$TARGET_USER/.bashrc"
if [ -f "$BASHRC" ] && ! grep -q 'ros/humble/setup.bash' "$BASHRC"; then
  {
    echo ''
    echo '# ROS2 Humble'
    echo 'source /opt/ros/humble/setup.bash'
  } >> "$BASHRC"
  chown "$TARGET_USER":"$TARGET_USER" "$BASHRC" 2>/dev/null || true
  echo "已写入 $BASHRC"
fi

echo "== 8. 验证 =="
# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO"
if [ "$ROS_DISTRO" != "humble" ]; then
  echo "[FAIL] ROS_DISTRO 不是 humble"
  exit 1
fi
echo "已安装包数量: $(ros2 pkg list 2>/dev/null | wc -l)"
for c in colcon cmake g++ rviz2 ros2; do
  printf '%-8s ' "$c"; command -v "$c" || echo "MISSING"
done

echo "===== 安装完成，日志：$LOG ====="
echo "下一步（不需要 sudo）："
echo "  bash /mnt/e/robot/q1-sim/scripts/stage_ros2_pkg.sh"
echo "  bash /mnt/e/robot/q1-sim/scripts/build_ws.sh"