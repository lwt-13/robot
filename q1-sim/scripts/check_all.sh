#!/usr/bin/env bash
# 一条命令跑完全部自检：仿真内核 / 可驱动性 / 动作定义 / 挥手（三种跑法 + 出图）
#
# 用法：
#   bash /mnt/e/robot/q1-sim/scripts/check_all.sh
#
# 说明：脚本会重新生成 docs/q1_wave.gif、docs/q1_wave_track.csv 和 docs/wave_verify/。
#       交互窗口（--viewer）需要显示器、且会弹窗，不放在这里，单独跑：
#       ~/q1_sim/venv/bin/python /mnt/e/robot/q1-sim/mujoco/play_motion.py --motion wave --viewer
set -u

cd "$(dirname "$0")/../mujoco" || exit 1
PY=${Q1_PY:-$HOME/q1_sim/venv/bin/python}
DOCS=$(cd ../docs && pwd)
LOG=/tmp/q1_check.log
fail=0

run() {
    local label="$1"; shift
    timeout 1800 "$@" > "$LOG" 2>&1
    local code=$?
    local verdict
    verdict=$(grep -oE '结果: \[PASS\] 全部通过|结论: \[(PASS|FAIL)\][^=]*' "$LOG" | head -1)
    printf '  exit=%-3s %-34s %s\n' "$code" "$label" "$verdict"
    [ "$code" -eq 0 ] || fail=1
}

echo "=== 单元自检 ==="
run "轨迹原语 trajectory.py" "$PY" trajectory.py
run "动作定义 motions.py" "$PY" motions.py
run "仿真内核 test_sim_core.py" "$PY" test_sim_core.py
run "可驱动性 drive_sim.py" "$PY" drive_sim.py --seconds 3

echo
echo "=== 挥手动作（默认这条会重新生成交付物）==="
run "固定基座（出图+导出+验收）" "$PY" play_motion.py --motion wave \
    --gif "$DOCS/q1_wave.gif" \
    --verify-dir "$DOCS/wave_verify" \
    --export-csv "$DOCS/q1_wave_track.csv" \
    --json /tmp/q1_wave.json
run "自由基座（站着挥手）" "$PY" play_motion.py --motion wave --mode free \
    --json /tmp/q1_wave_free.json
run "厂商真机增益 20/3" "$PY" play_motion.py --motion wave --gains vendor \
    --json /tmp/q1_wave_vendor.json
run "CSV 回放往返" "$PY" play_motion.py --from-csv "$DOCS/q1_wave_track.csv" \
    --json /tmp/q1_wave_csv.json

echo
echo "=== 汇总（直接读回各次运行的 JSON）==="
"$PY" - <<'PY'
import json
import os

rows = [("pinned 试验台", "/tmp/q1_wave.json"),
        ("free 站着挥手", "/tmp/q1_wave_free.json"),
        ("vendor 增益 20/3", "/tmp/q1_wave_vendor.json"),
        ("csv 回放", "/tmp/q1_wave_csv.json")]
for tag, path in rows:
    if not os.path.exists(path):
        continue
    d = json.load(open(path))
    ew = d["err_wave"]
    emax = max(v[1] for v in ew.values())
    emean = sum(v[2] for v in ew.values()) / len(ew)
    # 摆动轴现在是 shoulder_roll（yaw 改成常量，用来把手臂平面转进躯干/头所在平面）
    roll = d["act_range"]["right_shoulder_roll_joint"]
    er = d.get("elbow_rise")
    fu = d.get("forearm_up")
    pf = d.get("plane_face")
    wt = d.get("wrist_travel")
    rise = f"{er[0]:+.3f}m" if er else "n/a"
    up = f"{fu[0]:+.3f}m" if fu else "n/a"
    face = f"{pf[0]:.4f}" if pf else "n/a"
    travel = f"{wt:.3f}m" if wt else "n/a"
    print(f"  {tag:18s} 摆动段误差 {emax:5.2f}°/{emean:5.2f}°  "
          f"roll 行程 {roll[1] - roll[0]:6.2f}°  肘-肩最低 {rise}  小臂比肘高 {up}  "
          f"腕部横扫 {travel}  共面度最低 {face}  饱和 {d['sat_samples']}/{d['samples']}")
PY

echo
if [ "$fail" -eq 0 ]; then
    echo "=== 全部检查通过 ==="
else
    echo "=== 有检查未通过，逐个看 $LOG ==="
fi
exit "$fail"