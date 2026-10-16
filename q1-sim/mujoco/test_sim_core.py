#!/usr/bin/env python3
"""sim_core 的自检套件（不依赖 pytest，直接 python3 test_sim_core.py）。

覆盖：
  1. 关节名映射完整性 + 未知关节必须被上报（不能静默丢弃）
  2. 指令超出关节行程时必须被裁剪
  3. stiffness/damping/effort 三个字段的语义
  4. stiffness=0 且 effort=0 时的默认刚度兜底
  5. 基座固定 + 重力：能保持姿态
  6. 输出状态字段与 SDK JointState 一致（7 个字段）
  7. URDF 与 MJCF 的关节行程/力矩上限交叉核对
  8. 固定基座不注入能量（回归：曾经出现过的虚假力矩饱和）
"""
from __future__ import annotations

import glob
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_core import JointCommand, Q1SimCore  # noqa: E402

RAD2DEG = 180.0 / np.pi
FAILURES: list[str] = []


def check(ok: bool, title: str, detail: str = "") -> bool:
    print(f"  [{'PASS' if ok else 'FAIL'}] {title}" + (f" — {detail}" if detail else ""))
    if not ok:
        FAILURES.append(title)
    return ok


def find_urdf() -> str | None:
    cands = [
        os.path.expanduser("~/q1_ws/src/q1_description/urdf/q1_v3.urdf"),
        os.path.expanduser("~/q1_sim/q1_urdf/urdf/q1_v3.urdf"),
    ]
    cands += sorted(glob.glob(os.path.expanduser("~/q1_sim/q1_urdf/urdf/*.urdf")))
    for c in cands:
        if os.path.isfile(c) and "v3" in os.path.basename(c):
            return c
    return cands[0] if cands and os.path.isfile(cands[0]) else None


# ---------------------------------------------------------------- 1, 2
def test_mapping_and_limits():
    print("\n=== 1) 关节名映射与未知关节上报 ===")
    sim = Q1SimCore(mode="pinned", gravity=False)
    check(len(sim.joint_names) == 22, "22 个 SDK 关节名全部映射到模型",
          f"实际 {len(sim.joint_names)}")
    no_act = [n for n in sim.joint_names if sim.mapping[n]["actuator_id"] is None]
    check(not no_act, "每个关节都有执行器", f"缺执行器: {no_act or '无'}")

    rep = sim.apply_commands([
        JointCommand(name="left_knee_joint", position=0.1, stiffness=100, damping=10),
        JointCommand(name="not_a_real_joint", position=0.5, stiffness=100),
    ])
    check(rep.accepted == 1 and rep.rejected == 1, "未知关节被拒绝且计入报告",
          f"accepted={rep.accepted} rejected={rep.rejected}")
    check(rep.unknown == ["not_a_real_joint"], "未知关节名被明确列出", str(rep.unknown))

    print("\n=== 2) 指令超行程必须裁剪 ===")
    entry = sim.mapping["left_knee_joint"]
    lo, hi = entry["range_rad"]
    rep = sim.apply_commands([JointCommand(name="left_knee_joint", position=hi + 5.0,
                                           stiffness=200, damping=10)])
    sim.advance(1.5)
    pos = sim.data.qpos[entry["qpos_adr"]]
    check(rep.clamped == 1 and rep.clamped_joints == ["left_knee_joint"],
          "超限指令被记为已裁剪", f"clamped={rep.clamped}")
    check(pos <= hi + 0.05, "关节没有越过行程上限",
          f"目标被裁到 {np.rad2deg(hi):.1f}deg，实到 {np.rad2deg(pos):.1f}deg")

    rep = sim.apply_commands([JointCommand(name="left_knee_joint", position=lo - 5.0,
                                           stiffness=200, damping=10)])
    sim.advance(1.5)
    pos = sim.data.qpos[entry["qpos_adr"]]
    check(pos >= lo - 0.05, "关节没有越过行程下限",
          f"目标被裁到 {np.rad2deg(lo):.1f}deg，实到 {np.rad2deg(pos):.1f}deg")
    return sim


# ---------------------------------------------------------------- 3, 4
def test_command_semantics():
    print("\n=== 3) stiffness / damping / effort 字段语义 ===")
    # 地面下移 0.30 m：站立姿态下脚是踩在地上的，而这里所有关节刚度都是 0，
    # 着地瞬间的微小穿透修正就足以把完全自由的胳膊推动一点点（实测 0.12°），
    # 会干扰"纯力矩语义"这条判据。让脚离地就没有这个干扰。
    sim = Q1SimCore(mode="pinned", gravity=False, use_defaults=False, floor_drop=0.30)
    name = "left_elbow_joint"
    entry = sim.mapping[name]
    adr = entry["qpos_adr"]

    # 只给位置、刚度 0、且关掉兜底 → 不应该动
    sim.apply_commands([JointCommand(name=name, position=1.0, stiffness=0.0, damping=0.0)],
                       use_defaults=False)
    start = sim.data.qpos[adr]
    sim.advance(0.5)
    moved_zero = abs(sim.data.qpos[adr] - start)
    check(moved_zero < 1e-6, "stiffness=0 且 effort=0 时关节保持不动（纯力矩语义）",
          f"位移 {moved_zero:.2e} rad")

    # 给刚度 → 应该朝目标走
    sim.reset()
    sim.set_stiffness(40.0, 2.0)
    sim.apply_commands([JointCommand(name=name, position=0.6, stiffness=40.0, damping=2.0)],
                       use_defaults=False)
    sim.advance(2.0)
    err = abs(sim.data.qpos[adr] - 0.6) * RAD2DEG
    check(err < 5.0, "给定刚度后能驱动到目标", f"误差 {err:.2f} deg")

    # 纯前馈力矩（刚度 0、阻尼 0）→ 关节应被推动
    sim.reset()
    sim.model.opt.gravity[:] = [0.0, 0.0, 0.0]
    before = sim.data.qpos[adr]
    sim.apply_commands([JointCommand(name=name, position=0.0, effort=8.0,
                                     stiffness=0.0, damping=0.0)], use_defaults=False)
    sim.advance(0.3)
    delta = sim.data.qpos[adr] - before
    check(abs(delta) > 1e-3, "纯前馈 effort 能推动关节（力矩通道有效）",
          f"位移 {np.rad2deg(delta):+.2f} deg")

    # 阻尼应抑制振荡
    sim.reset()
    sim.apply_commands([JointCommand(name=name, position=0.6, stiffness=60.0, damping=0.0)],
                       use_defaults=False)
    sim.advance(2.0)
    vel_nd = abs(sim.data.qvel[entry["dof_adr"]])
    sim.reset()
    sim.apply_commands([JointCommand(name=name, position=0.6, stiffness=60.0, damping=8.0)],
                       use_defaults=False)
    sim.advance(2.0)
    vel_d = abs(sim.data.qvel[entry["dof_adr"]])
    check(vel_d <= max(vel_nd, 1e-6) * 1.05, "damping 起到抑制速度的作用",
          f"无阻尼末速 {vel_nd:.4f} vs 有阻尼 {vel_d:.4f} rad/s")

    print("\n=== 4) 默认刚度兜底（只发位置、不发刚度）===")
    sim = Q1SimCore(mode="pinned", gravity=False)  # use_defaults=True
    sim.apply_commands([JointCommand(name=name, position=0.5)])  # 只给位置
    sim.advance(2.0)
    err = abs(sim.data.qpos[adr] - 0.5) * RAD2DEG
    check(err < 5.0, "只发位置也能驱动（兜底生效，避免误判成仿真坏了）",
          f"误差 {err:.2f} deg")
    # 但一旦给了 effort，就要尊重 stiffness=0
    sim = Q1SimCore(mode="pinned", gravity=False)
    sim.apply_commands([JointCommand(name=name, position=0.5, effort=1.0, stiffness=0.0)])
    check(sim._cmds[name].stiffness == 0.0,
          "给了 effort 时不再套用默认刚度（尊重纯力矩意图）",
          f"stiffness={sim._cmds[name].stiffness}")


# ---------------------------------------------------------------- 5, 6
def test_gravity_hold_and_state_fields():
    print("\n=== 5) 基座固定 + 重力：保持姿态 ===")
    sim = Q1SimCore(mode="pinned", gravity=True, gravity_comp=True)
    sim.advance(1.0)
    # 焊接约束有极小柔度，实测高度漂移约 3e-4 m，故容差取 2e-3
    check(abs(sim.pelvis_height - sim.base_height) < 2e-3, "基座被焊死，高度不变",
          f"{sim.pelvis_height:.4f} m（模型站立高度 {sim.base_height:.4f}）")
    check(sim.n_contact > 0, "站立姿态下脚是踩在地上的", f"{sim.n_contact} 个接触点")
    check(sim.tilt_deg < 1.0, "躯干保持直立", f"{sim.tilt_deg:.2f} deg")
    check(sim.finite(), "数值有限，没有发散")

    # 屈膝会改变腿长：脚踩在地上时会被地板顶住，目标根本到不了。
    # 这不是控制器的问题，是姿态本身不可达，所以要下移地面给脚留空间。
    print("\n=== 5b) 屈膝姿态（地面下移 0.30 m，脚离地）===")
    hang = Q1SimCore(mode="pinned", gravity=True, gravity_comp=True, floor_drop=0.30)
    pose = {"left_knee_joint": 20.0, "right_knee_joint": 20.0,
            "left_hip_pitch_joint": -10.0, "right_hip_pitch_joint": -10.0}
    hang.set_positions_deg(pose)
    hang.advance(3.0)
    worst = max(hang.position_error_deg().values())
    check(worst < 3.0, "重力下姿态保持（最大误差 < 3 deg）", f"最大 {worst:.2f} deg")
    check(hang.n_contact == 0, "脚确实离地了（腿部动作不会被地板挡住）",
          f"{hang.n_contact} 个接触点")
    check(hang.tilt_deg < 1.0, "屈膝时躯干仍保持直立", f"{hang.tilt_deg:.2f} deg")
    sim = hang

    print("\n=== 6) 输出状态字段与 SDK JointState 一致 ===")
    states = sim.read_states()
    expect = {"name", "position", "velocity", "effort", "coil_temp", "motor_temp", "motor_vol"}
    got = set(vars(states[0]).keys())
    check(got == expect, "7 个字段齐全", f"实际 {sorted(got)}")
    check(len(states) == len(sim.joint_names), "每个关节都输出状态",
          f"{len(states)}/{len(sim.joint_names)}")
    check(all(isinstance(s.coil_temp, int) and isinstance(s.motor_vol, int) for s in states),
          "温度/电压是整数（与 uint8 字段类型一致）")
    st = states[0]
    print(f"    示例: {st.name} pos={st.position:+.4f} eff={st.effort:+.2f} "
          f"温度={st.coil_temp}/{st.motor_temp} 电压={st.motor_vol}")


# ---------------------------------------------------------------- 7
def test_urdf_vs_mjcf():
    print("\n=== 7) URDF 与 MJCF 交叉核对（行程 + 力矩上限）===")
    urdf = find_urdf()
    if not urdf:
        check(False, "找到 URDF 文件", "未找到")
        return
    print(f"    使用 {urdf}")
    root = ET.parse(urdf).getroot()
    urdf_joints = {}
    for j in root.findall("joint"):
        lim = j.find("limit")
        if lim is None:
            continue
        urdf_joints[j.get("name")] = {
            "lower": float(lim.get("lower", "0")),
            "upper": float(lim.get("upper", "0")),
            "effort": float(lim.get("effort", "0")),
            "velocity": lim.get("velocity"),
        }

    sim = Q1SimCore(mode="pinned", gravity=False)
    both = [n for n in sim.joint_names if n in urdf_joints]
    print(f"    URDF 关节 {len(urdf_joints)} 个，与 MJCF 同名 {len(both)} 个")
    check(len(both) == len(sim.joint_names), "SDK 的 22 个关节在 URDF 里都有定义",
          f"{len(both)}/{len(sim.joint_names)}")

    range_bad, effort_bad = [], []
    for n in both:
        u = urdf_joints[n]
        e = sim.mapping[n]
        lo, hi = e["range_rad"]
        if abs(lo - u["lower"]) > 1e-3 or abs(hi - u["upper"]) > 1e-3:
            range_bad.append(f"{n}: URDF[{u['lower']:.4f},{u['upper']:.4f}] "
                             f"vs MJCF[{lo:.4f},{hi:.4f}]")
        clo, chi = e["ctrlrange"]
        if abs(chi - u["effort"]) > 1e-6 or abs(clo + u["effort"]) > 1e-6:
            effort_bad.append(f"{n}: URDF {u['effort']} vs MJCF ±{chi}")
    check(not range_bad, "22 个关节的行程在 URDF 与 MJCF 中一致",
          "全部一致" if not range_bad else f"{len(range_bad)} 处不一致")
    for b in range_bad[:6]:
        print(f"      {b}")
    check(not effort_bad, "力矩上限在 URDF 与 MJCF 中一致",
          "全部一致" if not effort_bad else f"{len(effort_bad)} 处不一致")
    for b in effort_bad[:6]:
        print(f"      {b}")

    only_urdf = [n for n in urdf_joints if n not in sim.mapping]
    check(True, "仅在 URDF 中存在、MJCF 无执行器的关节（信息项）",
          ", ".join(sorted(only_urdf)) or "无（除浮动基座外完全对应）")
    vel = sorted({urdf_joints[n]["velocity"] for n in both})
    check(True, "URDF 声明了速度上限、MJCF 没有速度约束（信息项）",
          f"URDF velocity 取值: {vel}")


# ---------------------------------------------------------------- 8
def test_pinned_is_quiet():
    """回归测试：固定基座绝不能靠"每步强行把 qpos/qvel 归位"来实现。

    早期实现就是那样写的，结果腿部出现**2 个积分步周期**的极限环：qvel 在
    -3.23 / +3.54 rad/s 之间逐拍翻转，力矩全程贴着 ±36 N·m 上限，而关节角度
    看起来几乎没动。这个假象一度让人得出错误结论："保持这个姿态需要超过电机
    上限的力矩，所以 MJCF 的力矩上限偏保守。"
    改用 equality/weld 后，同样条件下速度 ~0、力矩 ~0.2 N·m。
    """
    print("\n=== 8) 固定基座不注入能量（回归：虚假的力矩饱和）===")
    sim = Q1SimCore(mode="pinned", gravity=True, gravity_comp=True)
    sim.advance(1.0)

    actuated = {n: e for n, e in sim.mapping.items() if e and e["actuator_id"] is not None}
    max_vel = max(abs(float(sim.data.qvel[e["dof_adr"]])) for e in actuated.values())
    check(max_vel < 0.05, "无指令时关节速度接近 0（没有高频极限环）",
          f"最大 |qvel| = {max_vel:.5f} rad/s")

    max_tau = max(abs(sim.applied_torque(n)) for n in actuated)
    check(max_tau < 1.0, "保持自然站立姿态所需力矩接近 0（不是「需要超限力矩」）",
          f"最大 |tau| = {max_tau:.3f} N·m")

    saturated = []
    for name, entry in actuated.items():
        lo, hi = entry["ctrlrange"]
        if abs(abs(sim.applied_torque(name)) - max(abs(lo), abs(hi))) < 1e-6:
            saturated.append(name)
    check(not saturated, "无指令时没有任何关节贴着力矩上限", f"饱和: {saturated or '无'}")

    hang = Q1SimCore(mode="pinned", gravity=True, gravity_comp=True, floor_drop=0.30)
    hang.advance(1.0)
    check(abs(hang.pelvis_height - hang.base_height) < 2e-3, "下移地面不影响基座高度",
          f"{hang.pelvis_height:.4f} m")
    check(hang.n_contact == 0, "下移地面后脚离地（腿部动作有空间）",
          f"{hang.n_contact} 个接触点")

    free = Q1SimCore(mode="free", gravity=True, gravity_comp=True)
    free.advance(1.0)
    check(free.finite() and free.pelvis_height > 0.30,
          "自由基座在自然姿态下处于平衡（静止站立，不是「必然倒下」）",
          f"倾角 {free.tilt_deg:.2f}° 骨盆 {free.pelvis_height:.3f} m 接触 {free.n_contact}")

    # 悬吊 + 屈膝：脚离地时没有地面接触"帮忙"锁住脚掌，最容易暴露 PD 增益自激。
    # 曾经用 kd=12 跑踝关节，脚掌惯量只有 ~0.006 kg·m^2，kd*dt/I 正好压在稳定边界上，
    # 出现 2 个积分步周期的极限环、力矩反复打到 ±22 N·m。
    crouch = {"left_knee_joint": 25.0, "right_knee_joint": 25.0,
              "left_hip_pitch_joint": 20.0, "right_hip_pitch_joint": 20.0,
              "left_ankle_pitch_joint": -12.0, "right_ankle_pitch_joint": -12.0}
    air = Q1SimCore(mode="pinned", gravity=True, gravity_comp=True, floor_drop=0.30)
    air.set_positions_deg(crouch)
    air.advance(2.5)
    act = {n: e for n, e in air.mapping.items() if e and e["actuator_id"] is not None}
    cyc = [n for n in act if abs(float(air.data.qvel[act[n]["dof_adr"]])) > 0.5]
    check(not cyc, "悬吊+屈膝下没有关节自激振荡（踝关节增益是否合适）",
          f"|qvel|>0.5 rad/s 的关节: {cyc or '无'}")
    sat2 = [n for n in act
            if abs(abs(air.applied_torque(n)) - max(abs(v) for v in act[n]["ctrlrange"])) < 1e-6]
    check(not sat2, "悬吊+屈膝下没有关节贴着力矩上限", f"饱和: {sat2 or '无'}")
    worst2 = max(air.position_error_deg().values())
    check(worst2 < 3.0, "悬吊+屈膝姿态跟踪正常", f"最大误差 {worst2:.2f} deg")


def main() -> int:
    print("=" * 68)
    print("Q1 仿真内核 sim_core 自检")
    print("=" * 68)
    test_mapping_and_limits()
    test_command_semantics()
    test_gravity_hold_and_state_fields()
    test_pinned_is_quiet()
    test_urdf_vs_mjcf()
    print("\n" + "=" * 68)
    if FAILURES:
        print(f"结果: [FAIL] {len(FAILURES)} 项未通过")
        for f in FAILURES:
            print(f"  - {f}")
        return 1
    print("结果: [PASS] 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())