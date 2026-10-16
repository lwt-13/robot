#!/usr/bin/env python3
"""动作定义：复刻一段"挥手"。

来自厂商 SDK 的部分（原样抄录）：
    sdk_q1-v2d0-v0.1.2.7-R/examples/python/upper_body_control.py
      - DEFAULT_STAND_POSITIONS   : 厂商实测的上肢待机姿态（10 个关节）
      - DEFAULT_STIFFNESS/DAMPING = 20.0 / 3.0（真机上肢增益）
      - run_demo() 的动作时序      : 见文末

**抬臂姿态的三个角没有照抄厂商的 wave_pose()，是按仿真实测重新标定的。**
原因都是量出来的，不是猜的：

  1. 本模型 right_elbow 的 0° 本身就已经是"弯着"的（两连杆夹角 82°，
     肩→腕 0.206 m），**正向越大越伸直**（+85° 时 0.272 m ≈ 上臂 0.145 + 前臂 0.127，
     等于完全伸直）。厂商用的 +1.10 rad(+63°) 就是把手臂伸得笔直。
  2. shoulder_yaw 的轴是 (0,0,1)，而且它**装在上臂中点**（肩→yaw 原点 0.0736 m，
     占上臂 0.1445 m 的 51%）——所以这根轴**始终与上臂同轴**，本质是"肱骨自转"，
     不是"肩膀左右摆"。对着一条伸直的手臂绕它自身轴去拧，手基本不动：
     实测腕部横扫只有 0.030 m，而且肘还在肩下面 0.117 m。
     它是模型里**唯一**能让手左右晃的关节（腕关节在 URDF 里是 fixed）。
  3. shoulder_yaw 的行程是 ±90°，**刚好够把弯肘平面从"正中对称面"转到"正面平面"**。
     所以它在这里当**常量**用，摆动轴另外选。
  4. 摆动改用 **shoulder_roll**（绕 x 轴）：整个手臂在**同一个平面内**一起摆，
     手在画面上左右扫。若还用 yaw 当摆动轴，摆的其实是"弯肘平面的朝向"，
     小臂会甩出躯干所在平面（这就是上一版看起来不对的原因）。

四个角的取值（都是量出来的）：
    shoulder_pitch   0°   : 手臂不前后摆，纯在"正面平面"内抬。
    shoulder_yaw   -85°   : 把弯肘平面转进躯干/头所在平面（限位 ±90°，留 5° 余量；
                            实测方向取负号时小臂才朝上，正号小臂会朝下）。
    elbow          +35°   : 保持弯曲，小臂朝上、手举到头旁边。
    shoulder_roll -138°   : 抬臂基准，肘比肩高；摆动 ±18° → 行程 -156°~-120°（限位 -160°）。

标定后的仿真实测（摆动段）：肘比肩高 **+0.083~+0.137 m**，手腕比手肘高 **+0.112 m**
（小臂朝上），腕部横扫 **0.149 m**，手臂平面与躯干/头平面夹角 **≤5.2°**，
手腕到头部中心最近 **0.201 m**，全程无碰撞（这些都由 play_motion.py 打印并做硬判定）。

厂商 run_demo() 的动作序列（本文件的 build_wave 照搬）：
    1. 当前姿态 → 待机姿态   2s
    2. 待机姿态 → 抬臂姿态   2s
    3. 挥手摆动              4s   （4 秒里 1.5 个周期）
    4. 收臂回待机姿态        2s
    5. 保持                  1s
"""
from __future__ import annotations

import csv
import math
import os

from trajectory import Hold, Motion, Move, Segment, Sine, Track

VENDOR_SOURCE = "sdk_q1-v2d0-v0.1.2.7-R/examples/python/upper_body_control.py"

# 厂商示例里明确列出的 10 个"上肢自定义控制"关节（腰 2 + 双臂 4+4）
UPPER_JOINT_NAMES = [
    "waist_roll_joint",
    "waist_yaw_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
]

# 厂商实测值，单位 rad。注意 elbow ≈ 1.02 rad ≈ 58°，也就是手臂自然微屈。
_STAND_VALUES = [
    -0.01982023485863345,
    -0.00021580613966982196,
    0.017233758101518008,
    0.22511493021261703,
    -0.003530979547924497,
    1.0186741542682025,
    0.017164215358473386,
    -0.22262401216020816,
    0.004204617106736518,
    1.0185279282650144,
]

DEFAULT_STAND_POSITIONS = dict(zip(UPPER_JOINT_NAMES, _STAND_VALUES))

# 真机上肢增益（厂商示例的 DEFAULT_STIFFNESS / DEFAULT_DAMPING）
VENDOR_STIFFNESS = 20.0
VENDOR_DAMPING = 3.0

# --- 抬臂姿态：按仿真实测重新标定（见模块开头说明），不是厂商原值 ---
#   shoulder_pitch   0°：不前后摆
#   shoulder_yaw   -85°：常量，把"小臂+大臂"所在平面转进躯干/头所在平面（共面度 0.996）
#   elbow          +35°：保持弯曲，小臂朝上、手举到头旁边
WAVE_ARM_FIXED = {
    "right_shoulder_pitch_joint": math.radians(0.0),
    "right_shoulder_yaw_joint": math.radians(-85.0),
    "right_elbow_joint": math.radians(35.0),
}

# 摆动轴是 shoulder_roll（不是 yaw）：绕 x 轴摆，正面平面的朝向不变，
# 整条手臂始终留在躯干/头那个平面里。±18° 时腕部左右扫约 0.16 m。
WAVE_ROLL_CENTER = math.radians(-138.0)
WAVE_ROLL_AMPLITUDE = math.radians(18.0)
WAVE_CYCLES = 1.5

# 各段时长（秒），照搬厂商 run_demo
DUR_RISE = 2.0
DUR_EXTEND = 2.0
DUR_WAVE = 4.0
DUR_RETURN = 2.0
DUR_HOLD = 1.0


def wave_start_pose(home: dict | None = None) -> dict:
    """抬臂姿态（"挥手之前"）：待机姿态 + 右臂标定角，roll 停在摆动中心。"""
    pose = dict(home if home is not None else DEFAULT_STAND_POSITIONS)
    pose.update(WAVE_ARM_FIXED)
    pose["right_shoulder_roll_joint"] = WAVE_ROLL_CENTER
    return pose


def build_wave(home: dict | None = None, name: str = "wave") -> Motion:
    """厂商的挥手动作序列（摆动轴换成 shoulder_roll，见文件开头说明）。"""
    stand = dict(home if home is not None else DEFAULT_STAND_POSITIONS)
    start = wave_start_pose(stand)
    return Motion(
        name=name,
        segments=[
            # 厂商是从"当前姿态"插值到待机姿态；仿真里起始是各关节 0，故 start={}
            Move({}, stand, DUR_RISE),
            Move(stand, start, DUR_EXTEND),
            Sine(base=stand,
                 oscillate={"right_shoulder_roll_joint": (WAVE_ROLL_CENTER, WAVE_ROLL_AMPLITUDE)},
                 fixed=WAVE_ARM_FIXED, cycles=WAVE_CYCLES, duration=DUR_WAVE),
            Move(start, stand, DUR_RETURN),
            Hold(stand, DUR_HOLD),
        ],
        note=(f"时序/待机姿态/增益取自厂商 SDK {os.path.basename(VENDOR_SOURCE)}；"
              f"抬臂姿态按仿真实测重新标定；"
              f"摆动 {WAVE_CYCLES:g} 周期 / {DUR_WAVE:g}s ≈ {WAVE_CYCLES / DUR_WAVE:.3f} Hz，"
              f"肩 roll {WAVE_ROLL_CENTER * 180 / math.pi:.0f}°±{math.degrees(WAVE_ROLL_AMPLITUDE):.0f}°"
              f"（肩 yaw 常量 {WAVE_ARM_FIXED['right_shoulder_yaw_joint'] * 180 / math.pi:.0f}°）"),
    )


def load_track_csv(path: str, name: str | None = None) -> Motion:
    """读回放文件：第一列是时间，其余列是关节角（rad）。

    这正是真机录制会产出的格式（订阅 /aima/hal/joint/state 的 JointStateArray 存盘）。
    """
    with open(path, newline="", encoding="utf-8") as fh:
        reader = csv.reader(fh)
        header = next(reader)
        joint_names = header[1:]
        times, poses = [], []
        for row in reader:
            if not row or not row[0].strip():
                continue
            times.append(float(row[0]))
            poses.append({n: float(v) for n, v in zip(joint_names, row[1:])})
    if not times:
        raise ValueError(f"{path} 里没有数据行")
    return Motion(name=name or f"track:{os.path.basename(path)}",
                  segments=[Track(times, poses)],
                  note=f"从 {path} 回放（{len(times)} 个采样点）")


MOTIONS = {
    "wave": build_wave,
}


def get_motion(name: str, home: dict | None = None) -> Motion:
    if name not in MOTIONS:
        raise KeyError(f"未知动作 '{name}'，可选: {sorted(MOTIONS)}")
    return MOTIONS[name](home)


if __name__ == "__main__":
    motion = build_wave()
    print(motion.describe())
    print()
    print("段边界处的目标角（rad）：")
    for t in (0.0, 2.0, 4.0, 8.0, 10.0, 11.0):
        pose = motion.pose_at(t)
        print(f"  t={t:5.2f}s  right_shoulder_roll={pose['right_shoulder_roll_joint']:+.4f}  "
              f"right_shoulder_yaw={pose['right_shoulder_yaw_joint']:+.4f}  "
              f"right_elbow={pose['right_elbow_joint']:+.4f}  "
              f"waist_roll={pose['waist_roll_joint']:+.4f}")