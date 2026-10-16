#!/usr/bin/env bash
# 证明「模型确实被 RViz2 渲染出来了」，而不是只证明「RViz2 进程没崩」。
#
# 做法：同一份配置下抓两次图——一次正常（RobotModel 显示启用），
# 一次把 RobotModel 显示禁用（其他一切不变）。两张图的差异就应当正好是机器人本身。
# 若禁用后画面几乎不变，说明模型其实没画出来。
#
# 为什么需要这个：WSLg 下窗口内容只能靠抓屏，而我无法直接看图，
# 所以用像素统计代替肉眼。不需要 sudo。
set -uo pipefail
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$HOME/q1_ws/install/setup.bash" || { echo "[FAIL] 请先运行 build_ws.sh"; exit 1; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY=python3
[ -x "$HOME/q1_sim/venv/bin/python" ] && PY="$HOME/q1_sim/venv/bin/python"
CFG="$HOME/q1_ws/install/q1_description/share/q1_description/rviz/q1.rviz"
WORK=/tmp/q1-proof
mkdir -p "$WORK"

stop_all() { pkill -f 'display.launch.py' >/dev/null 2>&1; pkill -f rviz2 >/dev/null 2>&1; }
trap stop_all EXIT

echo "===== 0. 生成「禁用 RobotModel」的对照配置 ====="
# 关键：必须做**文本级最小修改**，只翻转 RobotModel 那一行的 Enabled。
# 如果用 yaml 读进来再 dump，整个文件会被重新格式化（实测差 155 行），
# 连摄像机视角都会变，两张图自然不同 —— 那样的对照毫无意义，会得出假结论。
rm -f "$WORK/disabled.rviz"
"$PY" - "$CFG" "$WORK/disabled.rviz" <<'PY' || exit 1
import re
import sys

src, dst = sys.argv[1], sys.argv[2]
with open(src, encoding="utf-8") as fh:
    lines = fh.read().splitlines(keepends=True)

# RViz2 里 RobotModel 的类名是 rviz_default_plugins/RobotModel（不是 rviz/RobotModel）；
# 配置里这行写作 "    - Class: rviz_default_plugins/RobotModel"，行首还有个 "- "，别忘了。
start = next((i for i, ln in enumerate(lines)
              if ln.strip().endswith("Class: rviz_default_plugins/RobotModel")), None)
if start is None:
    raise SystemExit("[FAIL] 配置里没有 RobotModel 显示")

indent = len(lines[start]) - len(lines[start].lstrip())
end = start + 1
while end < len(lines):
    stripped = lines[end].strip()
    if stripped.startswith("- ") and (len(lines[end]) - len(lines[end].lstrip())) <= indent:
        break
    end += 1

target = next((i for i in range(start, end) if lines[i].strip().startswith("Enabled:")), None)
if target is None:
    raise SystemExit("[FAIL] RobotModel 块里没有 Enabled 行")

lines[target] = re.sub(r"Enabled:.*", "Enabled: false", lines[target])
with open(dst, "w", encoding="utf-8") as fh:
    fh.write("".join(lines))
print(f"  已把第 {target + 1} 行的 RobotModel Enabled 改为 false")
PY
# 必须显式确认文件是新生成的：只检查「文件存在」会放进上一轮的残留文件，让对照失效
[ -f "$WORK/disabled.rviz" ] || { echo "[FAIL] 对照配置没有生成"; exit 1; }
grep -q 'Class: rviz_default_plugins/RobotModel' "$WORK/disabled.rviz" \
  || { echo "[FAIL] 生成的对照配置不含 RobotModel"; exit 1; }

echo "  两次配置的实际差异（应当只有 RobotModel 的 Enabled 一行）:"
diff "$CFG" "$WORK/disabled.rviz" | sed 's/^/    /'
CHANGED=$(diff "$CFG" "$WORK/disabled.rviz" | grep -cE '^[<>]')
if [ "$CHANGED" -ne 2 ]; then
  echo "  [FAIL] 两次配置差了 $CHANGED 行，对照不成立（应当只差一行的改动）"
  exit 1
fi
echo "  [PASS] 配置只差 Enabled 一行，对照成立"

capture() {  # $1=配置 $2=输出png $3=标签
  stop_all; sleep 1
  timeout 75 ros2 launch q1_description display.launch.py \
    gui:=false rviz:=true rviz_config:="$1" > "$WORK/log_$3.txt" 2>&1 &
  sleep 20
  if ! pgrep -f rviz2 >/dev/null; then
    echo "  [FAIL] RViz2 没起来，见 $WORK/log_$3.txt"
    return 1
  fi
  "$PY" "$SCRIPT_DIR/../ros2/shot_window.py" --out "$2" | tail -2
  stop_all; sleep 2
}

echo
echo "===== 1. 抓图：RobotModel 启用 ====="
capture "$CFG" "$WORK/on.png" on || exit 1
echo
echo "===== 2. 抓图：RobotModel 禁用（对照）====="
capture "$WORK/disabled.rviz" "$WORK/off.png" off || exit 1

echo
echo "===== 3. 像素级对比 ====="
"$PY" - "$WORK/on.png" "$WORK/off.png" "$WORK/diff.png" "$WORK/model_crop.png" <<'PY'
import sys

from PIL import Image, ImageChops

on = Image.open(sys.argv[1]).convert("RGB")
off = Image.open(sys.argv[2]).convert("RGB")
w = min(on.size[0], off.size[0])
h = min(on.size[1], off.size[1])
on = on.crop((0, 0, w, h))
off = off.crop((0, 0, w, h))


def diff_ratio(box):
    a, b = on.crop(box), off.crop(box)
    m = ImageChops.difference(a, b).convert("L").point(lambda v: 255 if v > 12 else 0)
    d = list(m.getdata())
    return sum(1 for p in d if p) / len(d), m


def fg_count(box, bg=(48, 48, 48), thr=60):
    sub = on.crop(box), off.crop(box)
    out = []
    for img in sub:
        px = img.load()
        out.append(sum(1 for y in range(img.size[1]) for x in range(img.size[0])
                       if sum(abs(c - bg[i]) for i, c in enumerate(px[x, y])) > thr))
    return out


print(f"  尺寸: {w}x{h}")

# 先验证对照本身是否有效：视口上方两角是机器人不可能出现的地方，
# 若那里也有差异，说明两次运行的视角不同，后面的对比就没有意义（会得出假结论）。
guard_boxes = {"左上视口角": (330, 120, 570, 300), "右上视口角": (1150, 120, 1390, 300)}
guard_worst = 0.0
for label, box in guard_boxes.items():
    r, _ = diff_ratio(box)
    guard_worst = max(guard_worst, r)
    print(f"  对照校验 · {label}: 差异 {r:.2%}（应接近 0）")
if guard_worst > 0.005:
    print(f"  [FAIL] 两角差异达 {guard_worst:.2%}，两次运行视角不同，对照无效")
    sys.exit(2)
print("  [PASS] 视口角落像素一致，两次运行视角相同，对照有效")

center = (600, 250, 1000, 800)
r_center, m_center = diff_ratio(center)
on_fg, off_fg = fg_count(center)
mask = ImageChops.difference(on, off).convert("L").point(lambda v: 255 if v > 12 else 0)
data = list(mask.getdata())
n = sum(1 for p in data if p)
print(f"  整窗有差异的像素: {n} / {w * h} = {n / (w * h):.2%}")
print(f"  中央区域差异: {r_center:.2%}")
print(f"  中央区域前景像素（相对背景色）: 启用 {on_fg} → 禁用 {off_fg}，净增 {on_fg - off_fg}")
mask.save(sys.argv[3])

bbox = m_center.getbbox()
if bbox:
    bx0, by0 = center[0], center[1]
    pad = 20
    box = (max(0, bx0 + bbox[0] - pad), max(0, by0 + bbox[1] - pad),
           min(w, bx0 + bbox[2] + pad), min(h, by0 + bbox[3] + pad))
    on.crop(box).save(sys.argv[4])
    print(f"  已裁出模型所在区域: {sys.argv[4]}（{box[2]-box[0]}x{box[3]-box[1]}）")

if r_center > 0.05 and (on_fg - off_fg) > 1000:
    print("  [PASS] 关掉 RobotModel 后中央区域明显变化 —— 模型确实被渲染出来了")
    sys.exit(0)
print(f"  [FAIL] 关掉 RobotModel 前后中央区域几乎没变（差异 {r_center:.2%}，"
      f"前景净增 {on_fg - off_fg}），模型可能没有画出来")
sys.exit(1)
PY
rc=$?
# 归档证据
cp -f "$WORK/on.png" "$SCRIPT_DIR/../docs/q1_rviz2.png" 2>/dev/null
cp -f "$WORK/model_crop.png" "$SCRIPT_DIR/../docs/q1_rviz2_model.png" 2>/dev/null
echo
if [ "$rc" -eq 0 ]; then
  echo "[PASS] 渲染证明通过"
  echo "  证据: docs/q1_rviz2.png（RViz2 整窗）, docs/q1_rviz2_model.png（模型区域裁剪）"
else
  echo "[FAIL] 渲染证明未通过"
fi
exit "$rc"