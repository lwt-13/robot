#!/usr/bin/env python3
"""Q1 仿真的纯逻辑核心（不依赖 ROS）。

设计动机：把可测试的部分和不可测试的部分分开。
- 本模块：加载 MJCF、接收「按关节名的指令」、PD/力矩控制、输出关节状态。**完全可以用 MuJoCo 离线测**。
- ROS2 桥接层（q1_sim_bridge）：只做 aimdk_msgs 消息 <-> 这里字典的转换，代码尽量薄。

指令语义完全对齐 SDK 的 aimdk_msgs/msg/JointCommand：
    string name; float64 position; velocity; effort; stiffness; damping

控制律（每个关节独立）：
    tau = stiffness * (position - q) + damping * (velocity - qd) + effort
    tau += qfrc_bias（开启重力补偿时），最后按执行器 ctrlrange 限幅

关于 mode='pinned'（固定基座/试验台）：用 MuJoCo 的 **equality/weld 约束**把骨盆焊住，
而不是"每个积分步把基座 qpos/qvel 强行归位"。后者会持续向腿注入能量，制造出 2 个积分步
周期的极限环和全程贴上限的力矩（详见 build_pinned_model_xml 的说明）。默认焊在模型自己的
站立姿态（厂商写在 pelvis 的 pos 里，脚正好着地）；要测屈膝这类改变腿长的动作，
传 floor_drop 把地面下移若干米给脚留空间。

关于 stiffness 为 0 的处理（重点）：如果客户端只给位置、不给刚度，
按上面公式 tau 恒为 0，关节不会动，容易误判成"仿真坏了"。
因此默认行为是：**当 stiffness==0 且 effort==0 时，套用该关节组的默认刚度**；
一旦客户端给了 effort（想做纯力矩/前馈控制），就完全尊重其 stiffness=0。
可用 use_defaults=False 关掉这个兜底。
"""
from __future__ import annotations

import hashlib
import os
import pathlib
import re
import tempfile
from dataclasses import dataclass

import mujoco
import numpy as np

from joint_map import DEFAULT_UPSTREAM, MJCF_RELATIVE, load_model_and_mapping

RAD2DEG = 180.0 / np.pi

# 关节组默认增益（仅用于「只给位置不给刚度」的兜底）
#
# 关于 damping 的量级：显式积分下，PD 的阻尼项要满足 kd*dt/I < 2 才不会自激
# （I 是该关节的等效惯量，dt = 1 ms）。脚掌的惯量只有 ~0.006 kg·m^2，
# kd=12 时 kd*dt/I ≈ 2，正好压在稳定边界上，实测会出现**2 个积分步周期**的
# 极限环（qvel 在 ±2.6 rad/s 之间逐拍翻转、力矩反复打到 ±22 N·m 上限），
# 而脚离地、没有地面接触"帮忙"锁住脚掌时才暴露出来。
# 临界阻尼 kd ≈ 2*sqrt(kp*I) ≈ 2.3，所以踝关节的阻尼必须明显低于髋/膝。
# 厂商真机腿增益是 20/2，其中 damping=2 与这个估算一致。
GROUP_GAINS = {
    "leg": (220.0, 12.0),    # 髋 / 膝：连杆质量大、惯量足够
    "ankle": (220.0, 3.0),   # 踝：脚掌惯量极小，kd 必须压下来
    "waist": (150.0, 8.0),
    "arm": (60.0, 3.0),      # shoulder / elbow
}


def group_of(joint_name: str) -> str:
    if "ankle" in joint_name:
        return "ankle"
    if any(k in joint_name for k in ("hip", "knee")):
        return "leg"
    if "waist" in joint_name:
        return "waist"
    return "arm"


def build_pinned_model_xml(src: str | os.PathLike, floor_drop: float = 0.0) -> pathlib.Path:
    """生成"基座被焊死"的 MJCF 副本，返回临时文件路径。

    为什么不能"每个积分步强行把基座 qpos/qvel 归位"（曾经就是这么写的，是个坑）：
    那样等于每步把基座瞬移回目标位姿，持续向腿注入能量。实测腿部会出现**2 个积分步
    周期**的极限环：qvel 在 -3.23 / +3.54 rad/s 之间逐拍翻转，力矩全程贴着 ±36 N·m
    上限，而关节角度看起来几乎没动（误差 <2.5°）。这会让人得出完全错误的结论，
    例如"保持这个姿态需要超过电机上限的力矩"——实际上什么都不需要。
    用 equality/weld 才是物理上正确的硬约束：实测同样条件下速度 ~0、力矩 ~0.2 N·m。

    焊接点取模型自己的站立姿态（厂商写在 pelvis 的 pos 里，脚正好着地），
    此时脚是踩在地面上的；屈膝等改变腿长的动作会被地板挡住，所以提供 floor_drop：
    把地面整体下移若干米，脚就离地了，腿部动作可以自由做。

    注意：不能靠 weld 的 relpose 去抬高度。只给 body1 时 relpose 是"焊点在 body1
    坐标系里的位姿"，写 relpose z=0.55 会把机器人拽到地板下面（实测骨盆掉到 0.087 m、
    13 个接触点），语义和"抬到 0.55 m"正好相反。
    """
    src = pathlib.Path(src)
    text = src.read_text(encoding="utf-8")
    if "</mujoco>" not in text:
        raise RuntimeError(f"{src} 里没有 </mujoco>，无法注入焊接约束")
    # meshdir 换成绝对路径，这样临时文件放在 /tmp 也能加载（否则相对路径会失效）
    meshdir = (src.parent / "../meshes/").resolve()
    text = re.sub(r'meshdir="[^"]*"', f'meshdir="{meshdir}/"', text, count=1)

    if floor_drop:
        m = re.search(r'<geom\s+name="floor"[^>]*>', text)
        if not m:
            raise RuntimeError(f"{src} 里找不到名为 floor 的 geom，无法下移地面")
        tag = m.group(0)
        new_tag = (re.sub(r'pos="[^"]*"', f'pos="0 0 {-float(floor_drop):.4f}"', tag)
                   if 'pos="' in tag
                   else tag.replace("<geom ", f'<geom pos="0 0 {-float(floor_drop):.4f}" ', 1))
        text = text.replace(tag, new_tag, 1)

    weld = '  <equality>\n    <weld body1="pelvis" name="pin_base"/>\n  </equality>\n'
    text = text.replace("</mujoco>", weld + "</mujoco>", 1)

    key = hashlib.sha1(
        f"{src}|{src.stat().st_mtime_ns}|{floor_drop}".encode()).hexdigest()[:12]
    out = pathlib.Path(tempfile.gettempdir()) / f"q1_pinned_{key}.xml"
    if not out.exists():
        out.write_text(text, encoding="utf-8")
    return out


@dataclass
class JointCommand:
    """对应 aimdk_msgs/msg/JointCommand 的六个字段。"""

    name: str
    position: float = 0.0   # rad
    velocity: float = 0.0   # rad/s（作为阻尼项的目标速度）
    effort: float = 0.0     # N·m（前馈力矩）
    stiffness: float = 0.0  # N·m/rad
    damping: float = 0.0    # N·m·s/rad


@dataclass
class JointStateOut:
    """对应 aimdk_msgs/msg/JointState 的七个字段。温度/电压为仿真值。"""

    name: str
    position: float
    velocity: float
    effort: float
    coil_temp: int = 0
    motor_temp: int = 0
    motor_vol: int = 0


@dataclass
class ApplyReport:
    accepted: int = 0
    rejected: int = 0
    clamped: int = 0
    unknown: list = None
    clamped_joints: list = None

    def __post_init__(self):
        self.unknown = self.unknown or []
        self.clamped_joints = self.clamped_joints or []


class Q1SimCore:
    """Q1 仿真内核。mode='pinned' 表示基座被固定在试验台上，'free' 为真实浮动基座。"""

    def __init__(self, model_path: str | os.PathLike | None = None, mode: str = "pinned",
                 gravity: bool = True, gravity_comp: bool = True, clamp_to_limits: bool = True,
                 use_defaults: bool = True, floor_drop: float = 0.0):
        if mode not in ("pinned", "free"):
            raise ValueError("mode 只能是 'pinned' 或 'free'")
        source = pathlib.Path(model_path) if model_path else (DEFAULT_UPSTREAM / MJCF_RELATIVE)
        if mode == "pinned":
            source = build_pinned_model_xml(source, floor_drop)
        self.model, self.joint_names, self.mapping = load_model_and_mapping(source)
        self.data = mujoco.MjData(self.model)
        self.mode = mode
        self.gravity_comp = gravity_comp
        self.clamp_to_limits = clamp_to_limits
        self.use_defaults = use_defaults
        self.source_path = source
        self.floor_drop = float(floor_drop)
        # 焊接在模型自己的站立姿态上（厂商写在 pelvis 的 pos 里）
        self.base_height = float(self.model.qpos0[2])
        self.model.opt.gravity[:] = [0.0, 0.0, -9.81] if gravity else [0.0, 0.0, 0.0]
        self._cmds: dict[str, JointCommand] = {}
        self._ctrl = np.zeros(self.model.nu)
        self._applied: dict[str, float] = {}
        self.reset()

    # ---------- 生命周期 ----------
    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self._cmds = {}
        for name in self.joint_names:
            adr = self.mapping[name]["qpos_adr"]
            stiff, damp = GROUP_GAINS[group_of(name)] if self.use_defaults else (0.0, 0.0)
            # 默认状态是「保持当前姿态」：没有被指令提到的关节也必须被撑住。
            # 否则零刚度下它们会在重力和力矩限幅作用下自由乱转
            # （实测会甩到 ±180°，看起来像仿真坏了，其实是没给刚度）。
            self._cmds[name] = JointCommand(
                name=name, position=float(self.data.qpos[adr]),
                stiffness=stiff, damping=damp)
        self._ctrl[:] = 0.0
        self._applied = {name: 0.0 for name in self.joint_names}

    @property
    def sim_time(self) -> float:
        return float(self.data.time)

    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)

    # ---------- 指令输入 ----------
    def apply_commands(self, commands, use_defaults: bool | None = None) -> ApplyReport:
        """接收一批 JointCommand。未知关节名不会被静默丢弃，会记进报告。"""
        report = ApplyReport()
        use_defaults = self.use_defaults if use_defaults is None else use_defaults

        for cmd in commands:
            entry = self.mapping.get(cmd.name)
            if entry is None or entry["actuator_id"] is None:
                report.rejected += 1
                if cmd.name not in report.unknown:
                    report.unknown.append(cmd.name)
                continue

            position = float(cmd.position)
            if self.clamp_to_limits and entry["range_rad"] is not None:
                lo, hi = entry["range_rad"]
                if position < lo or position > hi:
                    position = float(min(max(position, lo), hi))
                    report.clamped += 1
                    if cmd.name not in report.clamped_joints:
                        report.clamped_joints.append(cmd.name)

            stiffness = float(cmd.stiffness)
            damping = float(cmd.damping)
            if use_defaults and stiffness == 0.0 and float(cmd.effort) == 0.0:
                stiffness, default_damping = GROUP_GAINS[group_of(cmd.name)]
                if damping == 0.0:
                    damping = default_damping

            self._cmds[cmd.name] = JointCommand(
                name=cmd.name, position=position, velocity=float(cmd.velocity),
                effort=float(cmd.effort), stiffness=stiffness,
                damping=damping if damping != 0.0 else (GROUP_GAINS[group_of(cmd.name)][1] if use_defaults else 0.0),
            )
            report.accepted += 1
        return report

    def set_position(self, name: str, position_rad: float) -> ApplyReport:
        return self.apply_commands([JointCommand(name=name, position=position_rad)])

    def set_positions_deg(self, pose_deg: dict) -> ApplyReport:
        return self.apply_commands(
            [JointCommand(name=n, position=np.deg2rad(v)) for n, v in pose_deg.items()]
        )

    def set_stiffness(self, stiffness: float, damping: float) -> None:
        for cmd in self._cmds.values():
            cmd.stiffness = stiffness
            cmd.damping = damping

    # ---------- 推进 ----------
    def _compute_ctrl(self) -> None:
        ctrl = np.zeros(self.model.nu)
        for name in self.joint_names:
            entry = self.mapping[name]
            if entry is None or entry["actuator_id"] is None:
                continue
            cmd = self._cmds[name]
            dof, qadr = entry["dof_adr"], entry["qpos_adr"]
            tau = cmd.stiffness * (cmd.position - self.data.qpos[qadr]) \
                + cmd.damping * (cmd.velocity - self.data.qvel[dof]) \
                + cmd.effort
            if self.gravity_comp:
                tau += self.data.qfrc_bias[dof]
            lo, hi = entry["ctrlrange"]
            clipped = float(np.clip(tau, lo, hi))
            self._applied[name] = clipped
            ctrl[entry["actuator_id"]] = clipped
        self._ctrl = ctrl

    def step(self, n: int = 1) -> None:
        for _ in range(max(1, int(n))):
            self._compute_ctrl()
            self.data.ctrl[:] = self._ctrl
            mujoco.mj_step(self.model, self.data)

    def advance(self, seconds: float) -> int:
        steps = max(1, int(round(seconds / self.timestep)))
        self.step(steps)
        return steps

    # ---------- 状态输出 ----------
    def applied_torque(self, name: str) -> float:
        return self._applied.get(name, 0.0)

    def read_states(self) -> list[JointStateOut]:
        out = []
        for name in self.joint_names:
            entry = self.mapping[name]
            if entry is None:
                continue
            effort = self.applied_torque(name)
            lo, hi = entry["ctrlrange"] or (1.0, 1.0)
            ratio = abs(effort) / max(abs(lo), abs(hi), 1e-6)
            # 温度/电压是按当前力矩占比算出来的仿真值，不是真实热模型
            out.append(JointStateOut(
                name=name,
                position=float(self.data.qpos[entry["qpos_adr"]]),
                velocity=float(self.data.qvel[entry["dof_adr"]]),
                effort=float(effort),
                coil_temp=int(35 + 25 * ratio),
                motor_temp=int(33 + 22 * ratio),
                motor_vol=int(round(48 - 4 * ratio)),
            ))
        return out

    def position_error_deg(self) -> dict[str, float]:
        errs = {}
        for name in self.joint_names:
            entry = self.mapping[name]
            if entry is None:
                continue
            errs[name] = float(abs(
                self.data.qpos[entry["qpos_adr"]] - self._cmds[name].position) * RAD2DEG)
        return errs

    # ---------- 诊断 ----------
    @property
    def pelvis_height(self) -> float:
        return float(self.data.qpos[2])

    @property
    def tilt_deg(self) -> float:
        """躯干 z 轴相对世界 z 轴的夹角：0=直立，180=翻倒。"""
        w, x, y, z = self.data.qpos[3:7]
        return float(np.rad2deg(np.arccos(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0))))

    @property
    def n_contact(self) -> int:
        return int(self.data.ncon)

    def finite(self) -> bool:
        return bool(np.all(np.isfinite(self.data.qpos)) and np.all(np.isfinite(self.data.qvel)))


if __name__ == "__main__":
    sim = Q1SimCore()
    print(f"关节 {len(sim.joint_names)}  执行器 {sim.model.nu}  模式 {sim.mode}")
    print(f"全部关节名: {list(sim.joint_names)}")
    sim.set_positions_deg({"left_knee_joint": 20.0, "right_knee_joint": 20.0})
    sim.advance(2.0)
    print(f"仿真 {sim.sim_time:.2f}s 后骨盆高度 {sim.pelvis_height:.3f} m")
    for st in sim.read_states()[:3]:
        print(f"  {st.name}: pos={st.position:+.4f} vel={st.velocity:+.4f} "
              f"effort={st.effort:+6.2f} 温度={st.coil_temp}/{st.motor_temp} 电压={st.motor_vol}")