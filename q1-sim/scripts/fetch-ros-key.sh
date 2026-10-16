#!/usr/bin/env bash
# 取得 ROS 2 的 APT 签名公钥，并**强制校验完整指纹**后写入目标 keyring。
#
# 为什么需要这个脚本：本机 raw.githubusercontent.com 不可达（HTTP 000），
# 各镜像站的 /ros/ros.key 路径是 404，官方的 packages.ros.org 又存在 TLS 校验失败。
# 但 keyserver.ubuntu.com 可用，且 GitHub 代理可用。
#
# 安全设计：无论公钥从哪里来，都要求指纹与 ROS 2 官方签名密钥完全一致：
#   C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654  (Open Robotics <info@osrfoundation.org>)
# 指纹不匹配就拒绝安装该公钥 —— 因此经由第三方代理传输也无法被替换成别的密钥。
#
# 用法:  bash fetch-ros-key.sh [目标路径，默认 /usr/share/keyrings/ros-archive-keyring.gpg]
# 退出码: 0 = 已写入并校验通过；1 = 全部来源失败
set -uo pipefail

DEST="${1:-/usr/share/keyrings/ros-archive-keyring.gpg}"
ROS_KEY_ID="C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"
ROS_KEY_FPR="C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"
KEYSERVERS=(
  "hkps://keyserver.ubuntu.com"
  "hkps://keys.openpgp.org"
  "hkp://keyserver.ubuntu.com:80"
)
PROXIES=(
  "https://ghproxy.net/"
  "https://ghfast.top/"
  "https://gh-proxy.com/"
)
KEY_URL="https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"

command -v gpg >/dev/null 2>&1 || { echo "[FAIL] 缺少 gpg"; exit 1; }
command -v curl >/dev/null 2>&1 || { echo "[FAIL] 缺少 curl"; exit 1; }

fpr_of_keyring() {
  # $1 = 一个被当作 GNUPGHOME 的目录
  GNUPGHOME="$1" gpg --batch --with-colons --fingerprint 2>/dev/null \
    | awk -F: '/^fpr:/ {print $10; exit}'
}

verify_and_install_file() {
  local file="$1" tmp fpr
  tmp="$(mktemp -d)"
  if ! GNUPGHOME="$tmp" gpg --batch --quiet --import "$file" >/dev/null 2>&1; then
    rm -rf "$tmp"; echo "  不是有效的 GPG 公钥"; return 1
  fi
  fpr="$(fpr_of_keyring "$tmp")"
  if [ "$fpr" != "$ROS_KEY_FPR" ]; then
    rm -rf "$tmp"; echo "  指纹不匹配：得到 '${fpr:-空}'，期望 '$ROS_KEY_FPR'"; return 1
  fi
  GNUPGHOME="$tmp" gpg --batch --export "$ROS_KEY_ID" > "$DEST" 2>/dev/null
  rm -rf "$tmp"
  echo "  指纹校验通过：$fpr"
  return 0
}

echo "== 目标 keyring: $DEST"
mkdir -p "$(dirname "$DEST")" 2>/dev/null || true

# 已经装好且指纹正确则直接复用
if [ -s "$DEST" ]; then
  tmp="$(mktemp -d)"
  if GNUPGHOME="$tmp" gpg --batch --quiet --import "$DEST" >/dev/null 2>&1 \
     && [ "$(fpr_of_keyring "$tmp")" = "$ROS_KEY_FPR" ]; then
    rm -rf "$tmp"
    echo "[PASS] 已存在且指纹正确，无需重新获取"
    exit 0
  fi
  rm -rf "$tmp"
  echo "  已存在的 keyring 指纹不对或不可读，重新获取"
fi

echo "-- 来源 1: keyserver（权威）--"
for ks in "${KEYSERVERS[@]}"; do
  tmp="$(mktemp -d)"
  if GNUPGHOME="$tmp" timeout 60 gpg --batch --keyserver "$ks" \
       --keyserver-options timeout=30 --recv-keys "$ROS_KEY_ID" >/dev/null 2>&1; then
    fpr="$(fpr_of_keyring "$tmp")"
    if [ "$fpr" = "$ROS_KEY_FPR" ]; then
      GNUPGHOME="$tmp" gpg --batch --export "$ROS_KEY_ID" > "$DEST" 2>/dev/null
      rm -rf "$tmp"
      echo "[PASS] 从 $ks 取得并校验通过：$fpr"
      exit 0
    fi
    echo "  [$ks] 指纹不匹配：${fpr:-空}"
  else
    echo "  [$ks] 不可用"
  fi
  rm -rf "$tmp"
done

echo "-- 来源 2: GitHub 代理（仍强制指纹校验）--"
for proxy in "${PROXIES[@]}"; do
  tmpfile="$(mktemp)"
  if curl -sSL --max-time 60 --retry 2 --retry-delay 2 "${proxy}${KEY_URL}" -o "$tmpfile" 2>/dev/null \
     && [ -s "$tmpfile" ]; then
    echo "  [$proxy] 已下载 $(stat -c%s "$tmpfile") 字节"
    if verify_and_install_file "$tmpfile"; then
      rm -f "$tmpfile"
      echo "[PASS] 经 $proxy 取得并校验通过"
      exit 0
    fi
  else
    echo "  [$proxy] 不可用"
  fi
  rm -f "$tmpfile"
done

echo "[FAIL] 所有来源都拿不到指纹正确的 ROS 2 公钥"
echo "       期望指纹: $ROS_KEY_FPR"
exit 1