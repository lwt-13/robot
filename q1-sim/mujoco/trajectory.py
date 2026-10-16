#!/usr/bin/env python3
"""动作轨迹：把关键帧 / 摆动段变成任意时刻的关节目标角。

这一层解决的是"跳舞缺什么"里的第一项：**动作数据**。

对齐厂商做法（SDK 的 examples/python/upper_body_control.py）：
  - 帧之间用**余弦缓动**（smooth_step），不用线性插值。线性插值会让关节速度在段
    边界突变，看起来"抽"，而且做速度前馈时会算出尖峰。
  - 摆动段用「基础姿态 + 幅值 × sin(2π × 圈数 × u)」，u 是段内**归一化**时间
    （厂商的 wave_pose 就是这么写的：4 秒里跑 1.5 个周期，t 是 0→1 而不是秒）。

用法自测:
    python trajectory.py
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field


def smooth_step(x: float) -> float:
    """厂商的余弦缓动，把 [0,1] 映射到 [0,1] 且两端速度为 0。"""
    x = min(1.0, max(0.0, float(x)))
    return 0.5 - 0.5 * math.cos(math.pi * x)


def blend(a: dict, b: dict, k: float, keys) -> dict:
    """按比例 k 在 a、b 之间插值；缺失的关节按 0 处理。"""
    return {n: a.get(n, 0.0) + (b.get(n, 0.0) - a.get(n, 0.0)) * k for n in keys}


class Segment:
    """一段动作。pose_at(t) 的 t 是**段内**时间（秒）。"""

    kind = "segment"

    def __init__(self, duration: float):
        self.duration = float(duration)

    def pose_at(self, t: float) -> dict:
        raise NotImplementedError

    def describe(self) -> str:
        return f"{self.kind} {self.duration:.2f}s"


class Hold(Segment):
    kind = "hold"

    def __init__(self, pose: dict, duration: float):
        super().__init__(duration)
        self.pose = dict(pose)

    def pose_at(self, t: float) -> dict:
        return dict(self.pose)

    def describe(self) -> str:
        return f"保持 {self.duration:.2f}s"


class Move(Segment):
    """从 start 姿态移动到 end 姿态。"""

    kind = "move"

    def __init__(self, start: dict, end: dict, duration: float, ease: str = "cos"):
        super().__init__(duration)
        if ease not in ("cos", "linear"):
            raise ValueError("ease 只能是 'cos' 或 'linear'")
        self.start = dict(start)
        self.end = dict(end)
        self.ease = ease
        self.keys = sorted(set(self.start) | set(self.end))

    def pose_at(self, t: float) -> dict:
        u = 0.0 if self.duration <= 0 else min(1.0, max(0.0, t / self.duration))
        k = smooth_step(u) if self.ease == "cos" else u
        return blend(self.start, self.end, k, self.keys)

    def describe(self) -> str:
        return f"移动 {self.duration:.2f}s（{len(self.keys)} 个关节）"


class Sine(Segment):
    """基础姿态 + 若干关节的正弦摆动（厂商 wave_pose 的语义）。

    oscillate[name] = (中心值, 幅值)  ->  pose[name] = 中心值 + 幅值 × sin(2π·圈数·u)
    fixed 里的关节在整个段内保持常量。
    """

    kind = "sine"

    def __init__(self, base: dict, oscillate: dict, cycles: float, duration: float,
                 fixed: dict | None = None, phase: float = 0.0):
        super().__init__(duration)
        self.base = dict(base)
        self.oscillate = {n: (float(c), float(a)) for n, (c, a) in oscillate.items()}
        self.fixed = dict(fixed or {})
        self.cycles = float(cycles)
        self.phase = float(phase)
        self.keys = sorted(set(self.base) | set(self.oscillate) | set(self.fixed))

    def wave_term(self, t: float) -> float:
        u = 0.0 if self.duration <= 0 else min(1.0, max(0.0, t / self.duration))
        return math.sin(2.0 * math.pi * self.cycles * u + self.phase)

    def pose_at(self, t: float) -> dict:
        pose = {n: self.base.get(n, 0.0) for n in self.keys}
        pose.update(self.fixed)
        w = self.wave_term(t)
        for name, (center, amplitude) in self.oscillate.items():
            pose[name] = center + amplitude * w
        return pose

    def describe(self) -> str:
        return (f"摆动 {self.duration:.2f}s（{self.cycles:g} 个周期，"
                f"{len(self.oscillate)} 个关节摆动）")


class Track(Segment):
    """一段采样好的轨迹（比如真机录下来的关节流），按时间线性插值回放。"""

    kind = "track"

    def __init__(self, times, poses, duration: float | None = None):
        self.times = [float(x) for x in times]
        self.poses = [dict(p) for p in poses]
        if len(self.times) != len(self.poses) or not self.times:
            raise ValueError("times 与 poses 长度必须一致且非空")
        if any(b < a for a, b in zip(self.times, self.times[1:])):
            raise ValueError("times 必须递增")
        self.keys = sorted({k for p in self.poses for k in p})
        span = self.times[-1] - self.times[0] if len(self.times) > 1 else 0.0
        super().__init__(span if duration is None else duration)

    def pose_at(self, t: float) -> dict:
        if len(self.times) == 1:
            return dict(self.poses[0])
        x = self.times[0] + min(max(float(t), 0.0), self.duration)
        i = min(max(bisect.bisect_right(self.times, x) - 1, 0), len(self.times) - 2)
        t_a, t_b = self.times[i], self.times[i + 1]
        k = 0.0 if t_b <= t_a else (x - t_a) / (t_b - t_a)
        return blend(self.poses[i], self.poses[i + 1], k, self.keys)

    def describe(self) -> str:
        return f"回放 {self.duration:.2f}s（{len(self.times)} 个采样点）"


@dataclass
class Motion:
    """一串首尾相接的动作段。pose_at(t) 的 t 是**全局**时间（秒）。"""

    name: str
    segments: list = field(default_factory=list)
    note: str = ""

    def __post_init__(self):
        if not self.segments:
            raise ValueError("Motion 至少要有一段")
        for i, seg in enumerate(self.segments):
            if seg.duration <= 0:
                raise ValueError(f"第 {i + 1} 段时长必须为正，当前 {seg.duration}")

    @property
    def duration(self) -> float:
        return float(sum(seg.duration for seg in self.segments))

    def segment_spans(self) -> list:
        """[(kind, 起始时间, 结束时间), ...]，用于按动作段单独统计。"""
        spans, acc = [], 0.0
        for seg in self.segments:
            spans.append((seg.kind, acc, acc + seg.duration))
            acc += seg.duration
        return spans

    def segment_at(self, t: float):
        """返回 (段序号, 段对象, 段内时间)。"""
        t = max(0.0, float(t))
        acc = 0.0
        last = len(self.segments) - 1
        for i, seg in enumerate(self.segments):
            if t < acc + seg.duration or i == last:
                return i, seg, min(t - acc, seg.duration)
            acc += seg.duration
        return last, self.segments[last], self.segments[last].duration

    def pose_at(self, t: float) -> dict:
        _, seg, local = self.segment_at(t)
        return seg.pose_at(local)

    def describe(self) -> str:
        lines = [f"动作「{self.name}」 总时长 {self.duration:.2f}s，{len(self.segments)} 段"]
        acc = 0.0
        for i, seg in enumerate(self.segments, 1):
            lines.append(f"  段{i}: {seg.describe():<28} {acc:6.2f}s → {acc + seg.duration:6.2f}s")
            acc += seg.duration
        if self.note:
            lines.append(f"  说明: {self.note}")
        return "\n".join(lines)


def _self_test() -> int:
    print("=== trajectory.py 自测 ===")
    ok = True

    move = Move({}, {"a": 1.0, "b": -2.0}, 2.0)
    checks = [
        ("余弦缓动端点", abs(smooth_step(0.0)) < 1e-12 and abs(smooth_step(1.0) - 1.0) < 1e-12),
        ("Move 起点为 0", abs(move.pose_at(0.0)["a"]) < 1e-12),
        ("Move 终点到位", abs(move.pose_at(2.0)["a"] - 1.0) < 1e-12),
        ("Move 中点缓动", abs(move.pose_at(1.0)["a"] - 0.5) < 1e-12),
        ("Move 越界钳位", abs(move.pose_at(99.0)["a"] - 1.0) < 1e-12),
    ]
    for label, good in checks:
        print(f"  [{'PASS' if good else 'FAIL'}] {label}")
        ok &= good

    # 厂商 wave_pose 语义：4 秒 1.5 个周期，
    #   u=1/6 -> +幅值（波峰）, u=1/2 -> -幅值（波谷）, u=1/3 -> 0（过零）
    sine = Sine(base={"yaw": 0.0, "pitch": 0.0}, oscillate={"yaw": (0.0, 0.30)},
                fixed={"pitch": -0.25}, cycles=1.5, duration=4.0)
    w_peak = sine.pose_at(4.0 / 6.0)["yaw"]
    w_trough = sine.pose_at(4.0 / 2.0)["yaw"]
    w_zero = sine.pose_at(4.0 / 3.0)["yaw"]
    sine_checks = [
        ("Sine 波峰 = +幅值", abs(w_peak - 0.30) < 1e-9),
        ("Sine 波谷 = -幅值", abs(w_trough + 0.30) < 1e-9),
        ("Sine 过零", abs(w_zero) < 1e-9),
        ("Sine fixed 生效", abs(sine.pose_at(1.3)["pitch"] + 0.25) < 1e-12),
    ]
    for label, good in sine_checks:
        print(f"  [{'PASS' if good else 'FAIL'}] {label}")
        ok &= good

    motion = Motion("t", [Move({}, {"a": 1.0}, 1.0), Hold({"a": 1.0}, 1.0),
                          Sine({"a": 0.0}, {"a": (0.0, 0.5)}, 1.0, 2.0)])
    boundary_ok = abs(motion.pose_at(1.0 - 1e-9)["a"] - motion.pose_at(1.0 + 1e-9)["a"]) < 1e-6
    print(f"  [{'PASS' if boundary_ok else 'FAIL'}] 段边界连续")
    print(f"  [{'PASS' if abs(motion.duration - 4.0) < 1e-12 else 'FAIL'}] 总时长 = 4.00s")
    ok &= boundary_ok and abs(motion.duration - 4.0) < 1e-12

    track = Track([0.0, 1.0, 2.0], [{"a": 0.0}, {"a": 2.0}, {"a": 0.0}])
    track_ok = abs(track.pose_at(0.5)["a"] - 1.0) < 1e-12 and abs(track.duration - 2.0) < 1e-12
    print(f"  [{'PASS' if track_ok else 'FAIL'}] Track 线性插值")
    ok &= track_ok

    print(motion.describe())
    print(f"=== {'[PASS] 全部通过' if ok else '[FAIL] 有检查项未通过'} ===")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(_self_test())