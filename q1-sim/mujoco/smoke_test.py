#!/usr/bin/env python3
"""Q1 MuJoCo 冒烟测试：验证厂商 MJCF 能否加载、执行器是否可用、物理能否推进。

用法:
    python smoke_test.py [path/to/q1_v3_mc.xml]

不依赖 ROS2，只验证仿真内核。输出为纯文本报告。
"""
import sys
import os
import math

import numpy as np
import mujoco


def main() -> int:
    default = os.path.expanduser("~/q1_sim/q1_urdf/xml/q1_v3_mc.xml")
    path = sys.argv[1] if len(sys.argv) > 1 else default
    if not os.path.isfile(path):
        print(f"[FAIL] 找不到模型文件: {path}")
        return 2

    print(f"模型文件: {path}")
    print(f"文件大小: {os.path.getsize(path) / 1024:.1f} KB")
    print(f"MuJoCo 版本: {mujoco.__version__}")

    # ---------- 1. 加载模型（这一步同时验证 meshdir 相对路径与 STL 网格） ----------
    try:
        model = mujoco.MjModel.from_xml_path(path)
    except Exception as exc:  # noqa: BLE001
        print(f"[FAIL] 模型加载失败: {exc}")
        return 3

    print("\n=== 模型规模 ===")
    print(f"nq(位置自由度)={model.nq}  nv(速度自由度)={model.nv}  nu(执行器)={model.nu}")
    print(f"njnt(关节)={model.njnt}  nbody(刚体)={model.nbody}  ngeom(几何)={model.ngeom}")
    print(f"nmesh(网格)={model.nmesh}  nsensor(传感器)={model.nsensor}  nkey(关键帧)={model.nkey}")
    print(f"时间步长 timestep={model.opt.timestep}  重力={model.opt.gravity}")

    if model.nmesh == 0:
        print("[WARN] 未加载到任何网格，STL 可能没被解析")

    # ---------- 2. 执行器清单（对应 SDK 的低层关节控制） ----------
    print(f"\n=== 执行器 ({model.nu}) ===")
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or f"actuator_{i}"
        lo, hi = model.actuator_ctrlrange[i]
        joint_id = int(model.actuator_trnid[i][0])
        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or "?"
        print(f"  [{i:2d}] {name:32s} -> joint={jname:32s} ctrlrange=[{lo:7.2f}, {hi:7.2f}]")

    # ---------- 3. 关节清单与限位 ----------
    print(f"\n=== 关节 ({model.njnt}) ===")
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or f"joint_{i}"
        jtype = model.jnt_type[i]
        type_name = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}.get(int(jtype), str(jtype))
        if model.jnt_limited[i]:
            lo, hi = model.jnt_range[i]
            rng = f"[{math.degrees(lo):7.2f}, {math.degrees(hi):7.2f}] deg"
        else:
            rng = "(无限位)"
        print(f"  [{i:2d}] {name:34s} type={type_name:5s} range={rng}")

    # ---------- 4. 关键帧 ----------
    if model.nkey > 0:
        print(f"\n=== 关键帧 ({model.nkey}) ===")
        for i in range(model.nkey):
            kname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, i) or f"key_{i}"
            print(f"  [{i}] {kname}: qpos[:6]={np.round(model.key_qpos[i][:6], 4)}")

    # ---------- 5. 物理推进 ----------
    data = mujoco.MjData(model)
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
        print("\n已用关键帧 0 作为初始姿态")
    else:
        mujoco.mj_resetData(model, data)
        print("\n已用 qpos0 作为初始姿态")

    n_steps = int(2.0 / model.opt.timestep)  # 仿真 2 秒
    print(f"\n=== 推进 {n_steps} 步 (仿真 {n_steps * model.opt.timestep:.2f} 秒) ===")
    tallest = float(data.qpos[2]) if model.nq >= 3 else 0.0
    for step in range(n_steps):
        mujoco.mj_step(model, data)
        if model.nq >= 3:
            tallest = max(tallest, float(data.qpos[2]))

    finite = bool(np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel)))
    n_contact = int(data.ncon)
    print(f"qpos 有限值: {finite}")
    print(f"qpos[:7] = {np.round(data.qpos[:7], 4)}")
    print(f"qvel 最大绝对值 = {np.max(np.abs(data.qvel)):.4f}")
    print(f"当前接触点数量 = {n_contact}")
    print(f"接触力总和 = {float(np.sum(np.abs(data.qfrc_constraint))):.3f}")
    print(f"能量 qfrc_actuator 范数 = {np.linalg.norm(data.qfrc_actuator):.4f}")

    # ---------- 6. 结论 ----------
    print("\n=== 结论 ===")
    ok = True
    checks = []
    checks.append(("模型可加载", True))
    checks.append(("网格已解析", model.nmesh > 0))
    checks.append(("存在执行器", model.nu > 0))
    checks.append(("数值稳定(无 NaN/Inf)", finite))
    checks.append(("存在接触(站在地面上)", n_contact > 0))
    for label, passed in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {label}")
        ok = ok and passed

    if not ok:
        print("\n[FAIL] 冒烟测试未全部通过")
        return 1

    print("\n[PASS] MuJoCo 仿真内核可用")
    return 0


if __name__ == "__main__":
    sys.exit(main())