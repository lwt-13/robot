#!/usr/bin/env bash
# 把上游 q1-urdf 暂存成 ROS2 ament 包 ~/q1_ws/src/q1_description。
# 不需要 sudo，幂等。
#
# 上游包是 ROS1 catkin 的（包名 Q1-V2.0-URDF-0601），且 launch 文件引用了不存在的
# urdf/Q1-V2.0-URDF-0601.urdf 与缺失的 urdf.rviz。这里只做必要修正，不动模型本体：
#   1) 改用 ament 构建
#   2) mesh 路径 ../meshes/ -> package://q1_description/meshes/
#   3) 去掉 URDF 里的 <mujoco> 扩展标签（urdfdom 不认识它）
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_SRC="$HERE/../ros2/q1_description"
URDF_SRC="${Q1_URDF_SRC:-$HOME/q1_sim/q1_urdf}"
DEST="$HOME/q1_ws/src/q1_description"

echo "== 源包: $PKG_SRC"
echo "== 模型源: $URDF_SRC"
echo "== 目标: $DEST"

[ -d "$PKG_SRC" ] || { echo "[FAIL] 找不到移植包源: $PKG_SRC"; exit 1; }
[ -d "$URDF_SRC/meshes" ] || { echo "[FAIL] 找不到网格目录: $URDF_SRC/meshes"; exit 1; }

rm -rf "$DEST"
mkdir -p "$DEST/urdf" "$DEST/launch" "$DEST/rviz"

cp "$PKG_SRC/package.xml" "$PKG_SRC/CMakeLists.txt" "$DEST/"
cp "$PKG_SRC"/launch/* "$DEST/launch/"
cp "$PKG_SRC"/rviz/* "$DEST/rviz/"
cp -r "$URDF_SRC/meshes" "$DEST/meshes"
if [ -d "$URDF_SRC/config" ]; then cp -r "$URDF_SRC/config" "$DEST/config"; fi
echo "已复制 package.xml / CMakeLists.txt / launch / rviz / meshes"

echo "== 转换 URDF =="
Q1_URDF_SRC="$URDF_SRC" Q1_PKG_DEST="$DEST" python3 - <<'PY'
import os
import pathlib
import re

src = pathlib.Path(os.environ["Q1_URDF_SRC"])
dest = pathlib.Path(os.environ["Q1_PKG_DEST"])

for path in sorted((src / "urdf").glob("*.urdf")):
    text = path.read_text(encoding="utf-8")
    n_mujoco = len(re.findall(r"<mujoco>.*?</mujoco>", text, flags=re.S))
    text = re.sub(r"<mujoco>.*?</mujoco>", "", text, flags=re.S)
    n_mesh = text.count('"../meshes/')
    text = text.replace('"../meshes/', '"package://q1_description/meshes/')
    (dest / "urdf" / path.name).write_text(text, encoding="utf-8")
    print(f"  {path.name}: 移除 mujoco 标签 {n_mujoco} 处, 重写 mesh 路径 {n_mesh} 处")

print("== 校验：URDF 引用的网格是否齐全 ==")
referenced, missing = set(), set()
for urdf in sorted((dest / "urdf").glob("*.urdf")):
    for name in re.findall(r'filename="package://q1_description/meshes/([^"]+)"', urdf.read_text(encoding="utf-8")):
        referenced.add(name)
        if not (dest / "meshes" / name).is_file():
            missing.add(name)
print(f"  引用的网格种类: {len(referenced)}")
if missing:
    print(f"  [WARN] 缺失网格 {len(missing)} 个（URDF 会显示不全）: {sorted(missing)}")
else:
    print("  [PASS] 引用的网格全部存在")

leftover = [p.name for p in (dest / "urdf").glob("*.urdf")
            if "../meshes/" in p.read_text(encoding="utf-8")]
print(f"  残留相对路径的文件: {leftover if leftover else '无'}")
PY

echo "== 结果 =="
find "$DEST" -maxdepth 1 | sort
echo "URDF:"; ls -1 "$DEST/urdf"
echo "网格数量: $(find "$DEST/meshes" -type f | wc -l)"
echo "== 暂存完成 =="