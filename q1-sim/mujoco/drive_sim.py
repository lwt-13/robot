#!/usr/bin/env python3
"""驱动 Q1 MuJoCo 仿真：用 SDK 的关节名下发目标角度，PD + 重力补偿控制。

设计说明（为什么这样测）：
  Q1 是浮动基座双足机器人。没有平衡控制器时，它必然会倒下——这是正常的物理结果，
  不是仿真坏了。所以这里把"执行器能否驱动关节"和"机器人能否自己站住"分开测：

  A) 无重力 + 自由基座  -> 纯执行器跟踪精度（可判定）
  B) 有重力 + 基座固定  -> 相当于把机器人架在试验台上，验证重力补偿是否真的有效（可判定）
  C) 有重力 + 自由基座  -> 诚实展示无人平衡控制时的倒地过程（仅作参考，不参与判定）

用法:
    python drive_sim.py
    python drive_sim.py --seconds 3 --gif q1_drive.gif
"""
import argparse
import sys

import numpy as np
import mujoco

from joint_map import DEFAULT_UPSTREAM, MJCF_RELATIVE, load_model_and_mapping
from sim_core import GROUP_GAINS, build_pinned_model_xml, group_of

RAD2DEG = 180.0 / np.pi

# 一个略微屈膝屈髋的站立姿态（度）。仅用于自检，不代表真机标定值。
DEFAULT_POSE_DEG = {
    "left_knee_joint": 25.0,
    "right_knee_joint": 25.0,
    "left_hip_pitch_joint": 20.0,
    "right_hip_pitch_joint": 20.0,
    "left_ankle_pitch_joint": -12.0,
    "right_ankle_pitch_joint": -12.0,
    "left_shoulder_pitch_joint": 25.0,
    "right_shoulder_pitch_joint": 25.0,
    "left_elbow_joint": -20.0,
    "right_elbow_joint": -20.0,
}

# 增益不在这里另立一份，直接复用 sim_core 的 GROUP_GAINS（单一来源）。
# 之前这里复制了 KP/KD 常量，改 sim_core 时不同步，导致踝关节自激问题"改了没效果"。
SUSPEND_DROP = 0.30         # 试验台出图/测腿姿态时把地面下移，给脚留空间（m）
STANDING_HEIGHT_MIN = 0.35  # 判定"还站着"的骨盆高度下限（m）
TILT_MAX_DEG = 30.0         # 判定"还没倒"的躯干倾角上限


def gains_for(name: str) -> tuple[float, float]:
    return GROUP_GAINS[group_of(name)]


def target_vector(model, names, mapping, pose_deg: dict) -> np.ndarray:
    q_des = np.zeros(model.nv)
    for name in names:
        entry = mapping[name]
        if entry and entry["actuator_id"] is not None:
            q_des[entry["dof_adr"]] = np.deg2rad(pose_deg.get(name, 0.0))
    return q_des


def raw_torque(name, entry, data, q_des, gravity_comp: bool) -> float:
    kp, kd = gains_for(name)
    dof = entry["dof_adr"]
    tau = kp * (q_des[dof] - data.qpos[entry["qpos_adr"]]) - kd * data.qvel[dof]
    if gravity_comp:
        tau += data.qfrc_bias[dof]
    return tau


def tilt_deg(quat) -> float:
    """躯干 z 轴相对世界 z 轴的夹角（0=直立，180=完全翻倒）。"""
    w, x, y, z = quat
    r22 = 1.0 - 2.0 * (x * x + y * y)
    return float(np.rad2deg(np.arccos(np.clip(r22, -1.0, 1.0))))


class Control:
    """PD + 可选重力补偿，带力矩饱和统计。"""

    def __init__(self):
        self.saturated = 0
        self.samples = 0
        self.per_joint_saturated: dict[str, int] = {}
        self.settle_saturated = 0
        self.settle_samples = 0

    def mark_settle(self) -> None:
        """记录瞬态结束时的计数，之后新增的才算稳态饱和。"""
        self.settle_saturated = self.saturated
        self.settle_samples = self.samples

    def step(self, model, data, names, mapping, q_des, gravity_comp: bool) -> None:
        ctrl = np.zeros(model.nu)
        for name in names:
            entry = mapping[name]
            if entry is None or entry["actuator_id"] is None:
                continue
            want = raw_torque(name, entry, data, q_des, gravity_comp)
            lo, hi = entry["ctrlrange"]
            self.samples += 1
            if want > hi + 1e-6 or want < lo - 1e-6:
                self.saturated += 1
                self.per_joint_saturated[name] = self.per_joint_saturated.get(name, 0) + 1
            ctrl[entry["actuator_id"]] = float(np.clip(want, lo, hi))
        data.ctrl[:] = ctrl


def run_case(model, names, mapping, q_des, seconds: float, gravity: bool,
             pin_base: bool, gravity_comp: bool, pinned_model=None):
    if pin_base:
        if pinned_model is None:
            raise ValueError("pin_base=True 必须传 pinned_model（基座被焊住的模型）")
        model = pinned_model
    data = mujoco.MjData(model)
    model.opt.gravity[:] = [0.0, 0.0, -9.81] if gravity else [0.0, 0.0, 0.0]
    mujoco.mj_resetData(model, data)

    control = Control()
    steps = max(1, int(seconds / model.opt.timestep))
    settle_from = int(steps * 0.8)
    accum, n_accum = np.zeros(model.nq), 0
    heights, tilts = [], []

    for step in range(steps):
        # 基座由模型里的 weld 约束焊死，这里不再干预。早期版本每步把 qpos/qvel 归位，
        # 等于持续向腿注入能量（实测 qvel 在 ±3.2 rad/s 之间逐拍翻转、力矩全程贴上限）。
        if step == settle_from:
            control.mark_settle()
        control.step(model, data, names, mapping, q_des, gravity_comp)
        mujoco.mj_step(model, data)
        heights.append(float(data.qpos[2]))
        tilts.append(tilt_deg(data.qpos[3:7]))
        if step >= settle_from:
            accum += data.qpos
            n_accum += 1

    q_settled = accum / max(1, n_accum)
    errs = {}
    for name in names:
        entry = mapping[name]
        if entry is None or entry["actuator_id"] is None:
            continue
        errs[name] = abs(q_settled[entry["qpos_adr"]] - q_des[entry["dof_adr"]]) * RAD2DEG
    err_arr = np.array(list(errs.values())) if errs else np.zeros(1)
    worst = sorted(errs.items(), key=lambda kv: -kv[1])[:3]

    return {
        "data": data,
        "err_deg": errs,
        "max_err_deg": float(err_arr.max()),
        "mean_err_deg": float(err_arr.mean()),
        "worst": worst,
        "n_contact": int(data.ncon),
        "pelvis_z": float(data.qpos[2]),
        "min_height": float(np.min(heights)),
        "final_tilt_deg": tilts[-1],
        "max_tilt_deg": float(np.max(tilts)),
        "saturated": control.saturated,
        "samples": control.samples,
        "sat_steady": control.saturated - control.settle_saturated,
        "samples_steady": control.samples - control.settle_samples,
        "sat_joints": sorted(control.per_joint_saturated.items(), key=lambda kv: -kv[1])[:4],
        "finite": bool(np.all(np.isfinite(data.qpos))),
    }


def report(title: str, res: dict) -> None:
    print(f"  {title}")
    print(f"    稳态跟踪误差: 最大 {res['max_err_deg']:.3f} deg / 平均 {res['mean_err_deg']:.3f} deg")
    if res["worst"]:
        print("    误差最大的三个关节: " + ", ".join(f"{n}={e:.2f}deg" for n, e in res["worst"]))
    print(f"    骨盆高度: 末态 {res['pelvis_z']:.3f} m / 最低 {res['min_height']:.3f} m")
    print(f"    躯干倾角: 末态 {res['final_tilt_deg']:.1f} deg / 最大 {res['max_tilt_deg']:.1f} deg")
    print(f"    接触点 {res['n_contact']}   数值有限 {res['finite']}")
    if res["sat_joints"]:
        top = ", ".join(f"{n}×{c}" for n, c in res["sat_joints"])
        print(f"    力矩饱和(全程) {res['saturated']}/{res['samples']} 采样（最多: {top}）")
    print(f"    力矩饱和(稳态) {res['sat_steady']}/{res['samples_steady']} 采样")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default=None)
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--gif", default=None)
    args = parser.parse_args()

    model, names, mapping = load_model_and_mapping(args.model)
    # 试验台用的模型：骨盆被焊住在厂商的站立姿态上；地面下移给腿部姿态留空间
    # （脚踩在地上时，屈膝这类改变腿长的指令会被地板顶住，目标根本到不了）
    pinned_model, _, _ = load_model_and_mapping(
        build_pinned_model_xml(args.model or (DEFAULT_UPSTREAM / MJCF_RELATIVE),
                               floor_drop=SUSPEND_DROP))
    q_des = target_vector(model, names, mapping, DEFAULT_POSE_DEG)
    total_mass = float(np.sum(model.body_mass))

    print(f"关节 {len(names)}   执行器 {model.nu}   总质量 {total_mass:.2f} kg   "
          f"控制周期 {model.opt.timestep}s   仿真 {args.seconds}s")
    print()

    print("=== A) 无重力 + 自由基座：纯执行器跟踪精度 ===")
    a = run_case(model, names, mapping, q_des, args.seconds, gravity=False, pin_base=False, gravity_comp=False)
    report("结果", a)
    a_ok = a["finite"] and a["mean_err_deg"] < 2.0 and a["max_err_deg"] < 5.0
    print(f"  [{'PASS' if a_ok else 'FAIL'}] 执行器能把关节驱动到指令角度")
    print()

    print(f"=== B) 有重力 + 基座固定（试验台，地面下移 {SUSPEND_DROP} m）：重力补偿是否有效 ===")
    b = run_case(model, names, mapping, q_des, args.seconds, gravity=True, pin_base=True,
                 gravity_comp=True, pinned_model=pinned_model)
    report("结果", b)
    b_ok = b["finite"] and b["mean_err_deg"] < 2.0 and b["max_err_deg"] < 5.0
    print(f"  [{'PASS' if b_ok else 'FAIL'}] 在重力下能保持指令姿态（基座支撑）")
    print()

    print("=== C) 有重力 + 自由基座：无平衡控制器时的表现（仅参考）===")
    c = run_case(model, names, mapping, q_des, args.seconds, gravity=True, pin_base=False, gravity_comp=True)
    report("结果", c)
    stood = c["min_height"] > STANDING_HEIGHT_MIN and c["final_tilt_deg"] < TILT_MAX_DEG
    if stood:
        print("  [NOTE] 该姿态下短时间内没有倾倒")
    else:
        print("  [NOTE] 如预期地倾倒了：浮动基座双足在没有平衡控制器时必然倒下。")
        print("         骨盆高度掉到 0.35 m 以下、躯干倾角超过 30deg 即视为倒地。")
        print("         本环境只负责'关节可被驱动'，平衡/步态控制属于上层算法，不在范围内。")
    print()

    print("=== D) 力矩饱和诊断 ===")
    print(f"  MJCF 电机力矩上限: 腿 36 N·m / 踝与手臂 22 N·m")
    print(f"  稳态饱和: A {a['sat_steady']}/{a['samples_steady']}   B {b['sat_steady']}/{b['samples_steady']}")
    if b["sat_steady"] > 0:
        print("  [NOTE] 稳态仍有饱和：该姿态在这个模型里确实需要更大的关节力矩。")
    else:
        print("  [PASS] 稳态无饱和（全程统计里的饱和来自起步过渡，属正常）")

    if args.gif:
        try:
            _render_gif(pinned_model, names, mapping, q_des, args)
        except Exception as exc:  # noqa: BLE001
            print(f"\n[WARN] 动图渲染失败: {exc}")

    overall = a_ok and b_ok
    print()
    print(f"=== 结论: {'[PASS] 仿真可驱动（关节按指令运动，重力下可保持姿态）' if overall else '[FAIL] 有检查项未通过'} ===")
    return 0 if overall else 1


def _render_gif(model, names, mapping, q_des, args) -> None:
    from PIL import Image

    data = mujoco.MjData(model)
    model.opt.gravity[:] = [0.0, 0.0, -9.81]
    mujoco.mj_resetData(model, data)
    # 注意 mujoco.Renderer 的参数顺序是 (model, height, width)
    renderer = mujoco.Renderer(model, height=480, width=640)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    camera.distance, camera.azimuth, camera.elevation = 1.7, 130.0, -10.0
    camera.lookat[:] = [0.0, 0.0, 0.50]

    control = Control()
    frames = []
    steps = int(args.seconds / model.opt.timestep)
    every = max(1, steps // 60)
    for step in range(steps):
        # 基座已被 weld 焊死，不需要（也不能）每步强制归位
        control.step(model, data, names, mapping, q_des, True)
        mujoco.mj_step(model, data)
        if step % every == 0:
            renderer.update_scene(data, camera)
            frames.append(Image.fromarray(renderer.render()))
    if frames:
        frames[0].save(args.gif, save_all=True, append_images=frames[1:], duration=50, loop=0)
        print(f"\n[PASS] 动图已保存: {args.gif}（{len(frames)} 帧）")


if __name__ == "__main__":
    sys.exit(main())