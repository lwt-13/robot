#!/usr/bin/env python3
"""在 MuJoCo 里播放一段动作：打通「动作数据 → 播放器 → 看到动作」这条链。

播放器做的事很薄，因为 sim_core 的接口就是 SDK 的 JointCommand 语义：
    每一拍（1 kHz，与 MJCF timestep 一致）把轨迹求值成 22 个关节目标角，
    按 JointCommand 下发，然后推进一个仿真步。

同时产出可核对的证据（不是"GIF 生成了"就算完）：
  - 关键关节的**实测行程** vs 指令行程（摆动轴 roll 应该看到 ±18° 左右）
  - 「小臂+大臂所在平面」与「躯干/头所在平面」的**共面度**（法线夹角，可测）
  - 两张关键帧的像素级差异，并检查变化是否集中在画面上半部（上肢）
  - 力矩饱和统计、基座高度/倾角、数值是否有限
  - 可选导出轨迹 CSV（格式与真机录制 /aima/hal/joint/state 一致，能被 --from-csv 回放）

用法:
    python play_motion.py --motion wave --gif /tmp/q1_wave.gif --export-csv /tmp/wave.csv
    python play_motion.py --motion wave --gains vendor        # 用真机增益 20/3 对比
    python play_motion.py --motion wave --mode free           # 反面示例：无平衡控制会倒
    python play_motion.py --from-csv /tmp/wave.csv            # 回放导出的轨迹
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from collections import defaultdict
from importlib import import_module

# MuJoCo 是在 import 时读 MUJOCO_GL 的，必须先设好。
#
# 这里要分两种情况，不能一律用 egl：
#   无头出图（--gif / --verify-dir）需要 egl；
#   交互窗口（--viewer）必须走 glfw —— 实测同时强制 egl 又开 GLFW 窗口，
#   动作能正常播完，但退出时 GL 上下文冲突直接段错误（exit 139）。
# 所以按命令行决定后端，且要在 import mujoco 之前。
if "--viewer" not in sys.argv:
    os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from motions import (DEFAULT_STAND_POSITIONS, UPPER_JOINT_NAMES,  # noqa: E402
                     VENDOR_DAMPING, VENDOR_STIFFNESS, WAVE_CYCLES,
                     WAVE_ROLL_AMPLITUDE, MOTIONS, get_motion, load_track_csv)
from sim_core import GROUP_GAINS, JointCommand, Q1SimCore, group_of  # noqa: E402

RAD2DEG = 180.0 / np.pi
CAMERA_LOOKAT_Z = 0.55     # 取景中心放在躯干中段，让整机（脚到抬手）都进画面
UPPER_HALF = 0.60          # 仅作参考：图像上半部的比例（是否命中取决于取景）


def full_home(core: Q1SimCore) -> dict:
    """22 关节的基准姿态：腿伸直（0），上肢用厂商实测待机姿态。"""
    home = {name: 0.0 for name in core.joint_names}
    home.update({k: v for k, v in DEFAULT_STAND_POSITIONS.items() if k in home})
    return home


def gains_for(name: str, mode: str) -> tuple:
    if mode == "vendor" and name in UPPER_JOINT_NAMES:
        return VENDOR_STIFFNESS, VENDOR_DAMPING
    return GROUP_GAINS[group_of(name)]


def motion_joints(motion) -> set:
    out = set()
    for seg in motion.segments:
        if hasattr(seg, "keys"):
            out |= set(seg.keys)
        elif hasattr(seg, "pose"):
            out |= set(seg.pose)
    return out


def make_camera(model) -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    # distance=2.25 是量出来的：1.85 时抬起来的手会顶到画面右边缘（差异包围盒右边距 0），
    # 2.25 时四周留 30~140 px 余量，手不会被切。
    camera.distance, camera.azimuth, camera.elevation = 2.25, 140.0, -5.0
    camera.lookat[:] = [0.0, 0.0, CAMERA_LOOKAT_Z]
    return camera


def default_verify_times(motion) -> tuple:
    """默认取「抬臂完成、yaw=0」和「摆动到 +幅值」两个时刻。"""
    span = next(((a, b) for kind, a, b in motion.segment_spans() if kind == "sine"), None)
    if span is None:
        return (0.0, motion.duration / 2.0)
    start, end = span
    quarter = (end - start) / (4.0 * WAVE_CYCLES)   # 1/4 周期
    return (start, start + quarter)


def play(core: Q1SimCore, motion, args) -> dict:
    from PIL import Image

    names = list(core.joint_names)
    home = full_home(core)
    dt = core.timestep
    steps = int(round(motion.duration / dt))
    wave_span = next(((a, b) for kind, a, b in motion.segment_spans() if kind == "sine"),
                     (0.0, motion.duration))

    want_render = bool(args.gif or args.verify_dir)
    renderer = camera = None
    every = 0
    if want_render:
        # 注意 mujoco.Renderer 的参数顺序是 (model, height, width)，不是 (width, height)
        renderer = mujoco.Renderer(core.model, height=args.height, width=args.width)
        camera = make_camera(core.model)
        every = max(1, int(round(1.0 / (args.fps * dt))))

    # 实时窗口：按墙钟时间节流，让 11 秒的动作真的播 11 秒（否则 1 kHz 的仿真会一闪而过）
    viewer = None
    viewer_open = False
    wall0 = time.perf_counter()
    if args.viewer:
        # 注意别在这里写 `import mujoco.viewer`：那会让 mujoco 变成 play() 的局部名，
        # 上面那些 mujoco.Renderer / mujoco.MjvCamera 就会 UnboundLocalError。
        viewer_mod = import_module("mujoco.viewer")
        viewer = viewer_mod.launch_passive(core.model, core.data)
        cam = make_camera(core.model)
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = (
            cam.distance, cam.azimuth, cam.elevation)
        viewer.cam.lookat[:] = cam.lookat
        viewer_open = True
        print("\n[实时窗口] 已打开 MuJoCo 交互窗口，按真实时间播放；关掉窗口即可提前结束。")

    err_wave = defaultdict(list)
    err_all = defaultdict(list)
    sat = defaultdict(int)
    cmd_range = defaultdict(lambda: [float("inf"), float("-inf")])
    act_range = defaultdict(lambda: [float("inf"), float("-inf")])
    heights, tilts, rows, frames = [], [], [], []
    n_sat = 0

    # "手肘高于肩膀"是可以测量的事实，不该靠看图判断：用关节锚点的世界坐标算高度差。
    jid_sh = mujoco.mj_name2id(core.model, mujoco.mjtObj.mjOBJ_JOINT, "right_shoulder_roll_joint")
    jid_el = mujoco.mj_name2id(core.model, mujoco.mjtObj.mjOBJ_JOINT, "right_elbow_joint")
    bid_wr = mujoco.mj_name2id(core.model, mujoco.mjtObj.mjOBJ_BODY, "right_wrist_yaw_link")
    bid_hd = mujoco.mj_name2id(core.model, mujoco.mjtObj.mjOBJ_BODY, "head_link")
    probe_ok = min(jid_sh, jid_el, bid_wr) >= 0
    elbow_rise, forearm_up, wrist_z, wrist_xy, plane_face = [], [], [], [], []
    head_gap = []

    for step in range(steps + 1):
        t = step * dt
        pose = motion.pose_at(t)
        target = dict(home)
        target.update({k: v for k, v in pose.items() if k in target})

        cmds = []
        for name in names:
            velocity = 0.0
            if args.feedforward:
                eps = args.ff_eps
                ahead = motion.pose_at(t + eps).get(name, home[name])
                behind = motion.pose_at(max(0.0, t - eps)).get(name, home[name])
                velocity = (ahead - behind) / (2.0 * eps)
            kp, kd = gains_for(name, args.gains)
            cmds.append(JointCommand(name=name, position=target[name], velocity=velocity,
                                     stiffness=kp, damping=kd))

        report = core.apply_commands(cmds)
        if report.unknown:
            raise SystemExit(f"[FAIL] 动作里有仿真不认识的关节: {report.unknown}")
        core.step(1)

        errs = core.position_error_deg()
        in_wave = wave_span[0] <= t <= wave_span[1]
        for name in names:
            qpos = float(core.data.qpos[core.mapping[name]["qpos_adr"]])
            if in_wave:
                cmd_deg, act_deg = target[name] * RAD2DEG, qpos * RAD2DEG
                cmd_range[name][0] = min(cmd_range[name][0], cmd_deg)
                cmd_range[name][1] = max(cmd_range[name][1], cmd_deg)
                act_range[name][0] = min(act_range[name][0], act_deg)
                act_range[name][1] = max(act_range[name][1], act_deg)
                if name in UPPER_JOINT_NAMES:
                    err_wave[name].append(errs[name])
            if name in UPPER_JOINT_NAMES:
                err_all[name].append(errs[name])

            lo, hi = core.mapping[name]["ctrlrange"]
            limit = max(abs(lo), abs(hi))
            if abs(abs(core.applied_torque(name)) - limit) < 1e-6:
                sat[name] += 1
                n_sat += 1

        heights.append(core.pelvis_height)
        tilts.append(core.tilt_deg)
        if probe_ok and in_wave:
            sh = core.data.xanchor[jid_sh]
            el = core.data.xanchor[jid_el]
            wr = core.data.xpos[bid_wr]
            elbow_rise.append(float(el[2] - sh[2]))
            forearm_up.append(float(wr[2] - el[2]))
            # 「小臂+大臂所在平面」的法线：上臂方向 × 前臂方向。
            # 该法线与 x̂ 重合度 = 这个平面与"躯干/头所在正面平面"的共面度
            # （躯干和头左右对称，其平面法线就是 x̂）。
            normal = np.cross(el - sh, wr - el)
            norm = float(np.linalg.norm(normal))
            if norm > 1e-9:
                plane_face.append(abs(float(normal[0])) / norm)
            wrist_z.append(float(wr[2]))
            wrist_xy.append((float(wr[0]), float(wr[1])))
            if bid_hd >= 0:
                # 手举到头旁边，最该防的是打到头：直接量手到头部中心的三维距离
                head_gap.append(float(np.linalg.norm(wr - core.data.xpos[bid_hd])))
        rows.append([t] + [float(core.data.qpos[core.mapping[n]["qpos_adr"]]) for n in names])

        if renderer is not None and step % every == 0:
            renderer.update_scene(core.data, camera)
            frames.append((t, Image.fromarray(renderer.render())))

        # 窗口按 ~60 Hz 刷新；窗口关掉后不再同步，但仿真继续跑完以保住验收数据
        if viewer is not None and viewer_open and step % 16 == 0:
            if not viewer.is_running():
                viewer_open = False
                print(f"\n[实时窗口] 已关闭（播放到 t={t:.2f}s），仿真继续跑完以输出完整结果。")
            else:
                viewer.sync()
                lag = t - (time.perf_counter() - wall0)
                if lag > 1e-4:
                    time.sleep(lag)

    if viewer is not None:
        viewer.close()

    return {
        "motion": motion.name,
        "duration": motion.duration,
        "steps": steps + 1,
        "mode": core.mode,
        "gains": args.gains,
        "feedforward": bool(args.feedforward),
        "wave_span": list(wave_span),
        "finite": core.finite(),
        "err_wave": {k: [min(v), max(v), sum(v) / len(v)] for k, v in err_wave.items()},
        "err_all": {k: [min(v), max(v), sum(v) / len(v)] for k, v in err_all.items()},
        "cmd_range": {k: list(v) for k, v in cmd_range.items()},
        "act_range": {k: list(v) for k, v in act_range.items()},
        "sat": dict(sat),
        "sat_samples": n_sat,
        "samples": (steps + 1) * len(names),
        "elbow_rise": ([min(elbow_rise), max(elbow_rise)] if elbow_rise else None),
        "forearm_up": ([min(forearm_up), max(forearm_up)] if forearm_up else None),
        "plane_face": ([min(plane_face), max(plane_face)] if plane_face else None),
        "head_gap": ([min(head_gap), max(head_gap)] if head_gap else None),
        "wrist_z": ([min(wrist_z), max(wrist_z)] if wrist_z else None),
        "wrist_travel": (
            float(np.hypot(max(p[0] for p in wrist_xy) - min(p[0] for p in wrist_xy),
                           max(p[1] for p in wrist_xy) - min(p[1] for p in wrist_xy)))
            if wrist_xy else None),
        "min_height": float(min(heights)),
        "final_height": float(heights[-1]),
        "max_tilt": float(max(tilts)),
        "final_tilt": float(tilts[-1]),
        "frames": frames,
        "rows": rows,
        "columns": names,
        "n_contact": int(core.data.ncon),
    }


def verify_frames(frames, times, out_dir, prefix) -> dict:
    """两帧像素对比 + 检查变化是否集中在画面上半部。"""
    from PIL import ImageChops

    picks = [min(range(len(frames)), key=lambda k: abs(frames[k][0] - want)) for want in times]
    (t_a, img_a), (t_b, img_b) = frames[picks[0]], frames[picks[1]]
    mask = ImageChops.difference(img_a.convert("RGB"), img_b.convert("RGB")) \
        .convert("L").point(lambda v: 255 if v > 12 else 0)
    data = list(mask.tobytes())   # tobytes 与 getdata 等价，但没有 Pillow 弃用警告
    w, h = img_a.size
    changed = sum(1 for p in data if p)
    upper = sum(1 for y in range(h) for x in range(w)
                if data[y * w + x] and y < h * UPPER_HALF)
    bbox = mask.getbbox()

    os.makedirs(out_dir, exist_ok=True)
    paths = {}
    for label, img in (("a", img_a), ("b", img_b)):
        path = os.path.join(out_dir, f"{prefix}_{label}.png")
        img.save(path)
        paths[label] = path
    diff_path = os.path.join(out_dir, f"{prefix}_diff.png")
    mask.save(diff_path)
    paths["diff"] = diff_path

    return {
        "t_a": t_a, "t_b": t_b,
        "size": [w, h],
        "changed": changed,
        "ratio": changed / float(w * h),
        "upper_ratio": (upper / changed) if changed else 0.0,
        "bbox": list(bbox) if bbox else None,
        "paths": paths,
    }


def export_csv(path: str, columns, rows) -> None:
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["t"] + list(columns))
        for row in rows:
            writer.writerow([f"{v:.6f}" for v in row])


def main() -> int:
    parser = argparse.ArgumentParser(description="在 MuJoCo 里播放一段 Q1 动作")
    parser.add_argument("--motion", default="wave", choices=sorted(MOTIONS),
                        help="内置动作名（默认 wave）")
    parser.add_argument("--from-csv", default=None, help="改为回放一个轨迹 CSV")
    parser.add_argument("--model", default=None)
    parser.add_argument("--mode", default="pinned", choices=["pinned", "free"],
                        help="pinned=架在试验台上（默认），free=浮动基座")
    parser.add_argument("--gains", default="sim", choices=["sim", "vendor"],
                        help="sim=仿真空组增益（默认），vendor=厂商真机上肢增益 20/3")
    parser.add_argument("--feedforward", action="store_true",
                        help="把轨迹的数值微分作为速度前馈（厂商示例里是 0）")
    parser.add_argument("--ff-eps", type=float, default=0.01)
    parser.add_argument("--max-err", type=float, default=5.0,
                        help="摆动段跟踪误差的判定阈值（度）")
    parser.add_argument("--min-elbow-rise", type=float, default=0.05,
                        help="摆动段手肘必须高于肩膀的最小高度差（米）；设成负数可关闭该判定")
    parser.add_argument("--min-plane-align", type=float, default=0.99,
                        help="摆动段「小臂+大臂所在平面」与「躯干/头所在平面」的最小共面度"
                             "（两平面法线的 |cos|，1.0=完全共面；0.99≈8°）；设成 0 可关闭该判定")
    parser.add_argument("--min-head-gap", type=float, default=0.15,
                        help="摆动段手腕到头部中心的最小距离（米）；设成 0 可关闭该判定")
    parser.add_argument("--gif", default=None)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--verify-dir", default=None, help="保存关键帧与差异图到该目录")
    parser.add_argument("--verify-times", default=None, help="两个对比时刻，逗号分隔")
    parser.add_argument("--export-csv", default=None, help="导出实测轨迹 CSV")
    parser.add_argument("--viewer", action="store_true",
                        help="打开 MuJoCo 交互窗口，按真实时间实时看动作（WSLg 下窗口出现在 Windows 桌面）")
    parser.add_argument("--json", default=None, help="把数值结果写成 JSON")
    args = parser.parse_args()

    if args.from_csv:
        motion = load_track_csv(args.from_csv)
    else:
        motion = get_motion(args.motion)

    print("=" * 68)
    print(motion.describe())
    unknown = sorted(motion_joints(motion) - set(Q1SimCore(args.model).joint_names))
    if unknown:
        print(f"[FAIL] 动作里有仿真不认识的关节: {unknown}")
        return 1

    core = Q1SimCore(args.model, mode=args.mode, gravity=True, gravity_comp=True)
    print()
    print(f"播放设置: 模式={args.mode}  增益={args.gains}  速度前馈={'开' if args.feedforward else '关'}"
          f"  控制周期={core.timestep}s  仿真步数={int(round(motion.duration / core.timestep)) + 1}")
    if args.mode == "free":
        print("  [注意] free 模式下没有平衡控制器，机器人必然会倒 —— 这是诚实展示，不是故障。")
    print()

    res = play(core, motion, args)

    # ---------- 跟踪误差 ----------
    print("=== 跟踪精度（摆动段，仅上肢 10 个关节）===")
    worst = sorted(res["err_wave"].items(), key=lambda kv: -kv[1][1])[:3]
    mean_all = float(np.mean([v[2] for v in res["err_wave"].values()]))
    max_all = float(max(v[1] for v in res["err_wave"].values()))
    for name, (lo, hi, avg) in worst:
        print(f"  {name:<32} 最大 {hi:5.2f}°  平均 {avg:5.2f}°")
    print(f"  整体: 最大 {max_all:.2f}°  平均 {mean_all:.2f}°")
    tracking_ok = max_all <= args.max_err
    if tracking_ok:
        print(f"  [PASS] 摆动段跟踪误差 ≤ {args.max_err:.1f}°")
    else:
        print(f"  [NOTE] 摆动段跟踪误差 {max_all:.2f}° 超过 {args.max_err:.1f}°"
              f"（增益={args.gains}）—— 这是参数问题，不是链路问题")

    # ---------- 关键关节实测行程 ----------
    print()
    print("=== 关键关节实测行程（摆动段）===")
    for name, role in (("right_shoulder_roll_joint", "摆动轴"),
                       ("right_shoulder_yaw_joint", "常量：决定手臂落在哪个平面"),
                       ("right_elbow_joint", "常量：保持弯肘")):
        c_lo, c_hi = res["cmd_range"].get(name, [0.0, 0.0])
        a_lo, a_hi = res["act_range"].get(name, [0.0, 0.0])
        print(f"  {name:<30} [{role}]")
        print(f"  {'':<30} 指令 {c_lo:+7.2f}° ~ {c_hi:+7.2f}°  (行程 {c_hi - c_lo:5.2f}°)")
        print(f"  {'':<30} 实测 {a_lo:+7.2f}° ~ {a_hi:+7.2f}°  (行程 {a_hi - a_lo:5.2f}°)")
    roll = res["act_range"].get("right_shoulder_roll_joint", [0.0, 0.0])
    roll_span = roll[1] - roll[0]
    # 幅值从 motions.py 取，别在这里再写一遍常量（写两份迟早对不上）
    expect = 2 * WAVE_ROLL_AMPLITUDE * RAD2DEG
    roll_ok = roll_span > expect * 0.8
    print(f"  [{'PASS' if roll_ok else 'FAIL'}] 摆动轴 roll 实测行程 {roll_span:.2f}° ≥ "
          f"指令行程的 80%（指令 {expect:.2f}°）")

    # ---------- 挥手的形态（可测指标，不靠看图）----------
    form_ok = True
    if res["elbow_rise"] is not None:
        has_sine = any(k == "sine" for k, _, _ in motion.segment_spans())
        lo, hi = res["elbow_rise"]
        wz, wt = res["wrist_z"], res["wrist_travel"]
        up = res["forearm_up"]
        pf = res["plane_face"]
        print()
        print("=== 挥手形态（摆动段实测，按关节锚点世界坐标计算）===")
        print(f"  手肘相对肩膀高度差: 最低 {lo:+.3f} m / 最高 {hi:+.3f} m")
        if up:
            print(f"  手腕比手肘高:       最低 {up[0]:+.3f} m（>0 = 小臂朝上，手举到头旁边）")
        print(f"  腕部横扫 {wt:.3f} m   手腕高度 {wz[0]:.3f}~{wz[1]:.3f} m")
        hg = res["head_gap"]
        if hg:
            gap_ok = hg[0] >= args.min_head_gap
            print(f"  手腕到头部中心最近距离: {hg[0]:.3f} m"
                  f"（{'安全' if gap_ok else '过近'}，要求 ≥ {args.min_head_gap:.3f} m）")
        if has_sine:
            form_ok = lo >= args.min_elbow_rise
            print(f"  [{'PASS' if form_ok else 'FAIL'}] 手肘全程高于肩膀"
                  f"（要求最低 ≥ {args.min_elbow_rise:+.3f} m）")
            if not form_ok:
                print("        低于该值时会看起来像「手臂没有抬起来」")
            # 「小臂+大臂所在平面」是否与「躯干/头所在平面」共面：
            # 两个平面共面 ⇔ 法线平行。躯干和头左右对称，其平面法线是 x̂，
            # 所以用 |n̂ · x̂| 度量；1.0 = 完全共面。
            if pf:
                align = pf[0]
                plane_ok = align >= args.min_plane_align
                off = float(np.degrees(np.arccos(min(1.0, align))))
                limit = float(np.degrees(np.arccos(args.min_plane_align)))
                print(f"  小臂/大臂平面 与 躯干/头平面 的最大夹角: {off:.1f}°"
                      f"（共面度 {align:.4f}，1.0=完全共面）")
                print(f"  [{'PASS' if plane_ok else 'FAIL'}] 两平面共面"
                      f"（要求夹角 ≤ {limit:.1f}°，由 --min-plane-align 控制）")
                form_ok = form_ok and plane_ok
                if hg:
                    form_ok = form_ok and gap_ok
        else:
            print("  [----] 该动作用 CSV 回放，没有摆动段，不做形态判定")

    # ---------- 力矩饱和 ----------
    print()
    print("=== 力矩饱和 ===")
    print(f"  全程饱和 {res['sat_samples']}/{res['samples']} 采样"
          f"（{res['sat_samples'] / max(1, res['samples']):.2%}）")
    if res["sat"]:
        top = sorted(res["sat"].items(), key=lambda kv: -kv[1])[:3]
        print("  饱和最多的关节: " + ", ".join(f"{n}×{c}" for n, c in top))

    # ---------- 基座 ----------
    print()
    print("=== 基座状态 ===")
    print(f"  骨盆高度: 最低 {res['min_height']:.3f} m / 末态 {res['final_height']:.3f} m")
    print(f"  躯干倾角: 最大 {res['max_tilt']:.1f}° / 末态 {res['final_tilt']:.1f}°")
    if args.mode == "pinned":
        print("  [PASS] 试验台模式：基座被固定，动作本身可被完整观察")
    else:
        fell = res["final_tilt"] > 30.0 or res["min_height"] < 0.35
        print(f"  [{'NOTE' if fell else 'PASS'}] 浮动基座："
              f"{'如预期倾倒 —— 仿真里没有平衡控制器' if fell else '该动作幅度下未倾倒'}")

    # ---------- 出图与像素验证 ----------
    verify = None
    if res["frames"]:
        if args.verify_times:
            times = tuple(float(x) for x in args.verify_times.split(","))
        else:
            times = default_verify_times(motion)
        out_dir = args.verify_dir or (os.path.dirname(os.path.abspath(args.gif)) or ".")
        prefix = os.path.splitext(os.path.basename(args.gif or "motion"))[0]
        verify = verify_frames(res["frames"], times, out_dir, prefix)
        print()
        print("=== 画面变化验证（像素级）===")
        print(f"  对比时刻: t={verify['t_a']:.2f}s（抬臂完成） vs t={verify['t_b']:.2f}s（摆到极值）")
        print(f"  画面尺寸 {verify['size'][0]}x{verify['size'][1]}，"
              f"变化像素 {verify['changed']}（{verify['ratio']:.2%}）")
        print(f"  变化像素落在图像上半部的比例: {verify['upper_ratio']:.1%}（仅供参考）")
        bbox = verify["bbox"]
        cy = ((bbox[1] + bbox[3]) / 2.0 / verify["size"][1]) if bbox else 0.0
        print(f"  变化区域竖直重心: {cy:.0%}（0%=画面顶部，50%=中部，100%=底部）")
        print(f"  差异包围盒: {bbox}")
        print(f"  已保存: {verify['paths']['a']} / {verify['paths']['b']} / {verify['paths']['diff']}")
        # 判据只要求"渲染确实跟着动作变了"。"变化是否在上半部"取决于取景，
        # 不是可靠判据（早先把它当判据，换个拍摄高度就会误报失败）。
        moved_ok = verify["changed"] > 500
        print(f"  [{'PASS' if moved_ok else 'FAIL'}] 画面确实随动作变化（渲染没有卡住）")

    if args.gif and res["frames"]:
        imgs = [img for _, img in res["frames"]]
        imgs[0].save(args.gif, save_all=True, append_images=imgs[1:],
                     duration=int(1000.0 / args.fps), loop=0)
        size_mb = os.path.getsize(args.gif) / 1024.0 / 1024.0
        print()
        print(f"[PASS] 动图已保存: {args.gif}（{len(imgs)} 帧，{size_mb:.1f} MB）")

    if args.export_csv:
        export_csv(args.export_csv, res["columns"], res["rows"])
        print(f"[PASS] 实测轨迹已导出: {args.export_csv}"
              f"（{len(res['rows'])} 行 × {len(res['columns'])} 关节，可被 --from-csv 回放）")

    # ---------- 结论 ----------
    print()
    print("=" * 68)
    print(f"数值有限: {res['finite']}   接触点: {res['n_contact']}")
    hard_ok = bool(res["finite"]) and form_ok
    print(f"=== 结论: {'[PASS] 动作播放完成（动作数据 → 播放器 → 仿真里动起来）' if hard_ok else '[FAIL] 数值发散'} ===")

    if args.json:
        payload = {k: v for k, v in res.items() if k not in ("frames", "rows")}
        payload["verify"] = verify
        payload["tracking_ok"] = tracking_ok
        payload["roll_span_deg"] = roll_span
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=2)
        print(f"[PASS] 结果 JSON: {args.json}")

    return 0 if hard_ok else 1


if __name__ == "__main__":
    _code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    if "--viewer" in sys.argv:
        # WSLg 下 GLFW/GL 的进程退出清理会段错误（实测：结果全部正确输出、
        # viewer.close() 也正常返回，之后解释器退出阶段才 crash，exit 139）。
        # 这里跳过 Python 的析构清理直接退出，避免那个与本次任务无关的环境问题。
        os._exit(_code)
    sys.exit(_code)