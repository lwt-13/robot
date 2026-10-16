#!/usr/bin/env python3
"""在 MuJoCo 里加载 Q1：无头渲染出图，或打开交互式查看器。

用法:
    # 无头渲染一张 PNG（需要 libegl1 / libgl1-mesa-dri，见 install-ros2.sh）
    MUJOCO_GL=egl python view_model.py --render /tmp/q1.png

    # 打开交互查看器（需要 WSLg，窗口会出现在 Windows 桌面上）
    python view_model.py --viewer

    # 只打印模型摘要
    python view_model.py --info
"""
import argparse
import os
import sys

import numpy as np
import mujoco


def default_model() -> str:
    return os.path.expanduser("~/q1_sim/q1_urdf/xml/q1_v3_mc.xml")


def summarize(model: "mujoco.MjModel") -> None:
    print(f"nq={model.nq} nv={model.nv} nu={model.nu} njnt={model.njnt} "
          f"nbody={model.nbody} ngeom={model.ngeom} nmesh={model.nmesh} "
          f"nsensor={model.nsensor}")
    print(f"timestep={model.opt.timestep} gravity={model.opt.gravity}")
    total_mass = float(np.sum(model.body_mass))
    print(f"总质量 = {total_mass:.2f} kg   体重(重力) = {total_mass * 9.81:.1f} N")


def apply_nominal_pose(model: "mujoco.MjModel", data: "mujoco.MjData",
                       knee_deg: float, hip_deg: float, shoulder_deg: float) -> None:
    """给一个略微屈膝屈髋的姿态，避免笔直站着看起来像刚体。"""
    wanted = {
        "left_knee_joint": knee_deg,
        "right_knee_joint": knee_deg,
        "left_hip_pitch_joint": hip_deg,
        "right_hip_pitch_joint": hip_deg,
        "left_ankle_pitch_joint": -hip_deg / 2.0,
        "right_ankle_pitch_joint": -hip_deg / 2.0,
        "left_shoulder_pitch_joint": shoulder_deg,
        "right_shoulder_pitch_joint": shoulder_deg,
        "left_elbow_joint": -shoulder_deg,
        "right_elbow_joint": -shoulder_deg,
    }
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name in wanted:
            data.qpos[model.jnt_qposadr[i]] = np.deg2rad(wanted[name])


def main() -> int:
    parser = argparse.ArgumentParser(description="Q1 MuJoCo 查看/渲染")
    parser.add_argument("--model", default=default_model())
    parser.add_argument("--render", metavar="PNG", help="渲染一张 PNG 到该路径后退出")
    parser.add_argument("--viewer", action="store_true", help="打开交互式查看器")
    parser.add_argument("--info", action="store_true", help="只打印模型摘要")
    parser.add_argument("--seconds", type=float, default=0.5, help="渲染前先仿真多久")
    parser.add_argument("--width", type=int, default=1000)
    parser.add_argument("--height", type=int, default=800)
    parser.add_argument("--knee", type=float, default=25.0, help="膝盖角度(度)")
    parser.add_argument("--hip", type=float, default=20.0, help="髋俯仰角度(度)")
    parser.add_argument("--shoulder", type=float, default=35.0, help="肩俯仰角度(度)")
    args = parser.parse_args()

    if not os.path.isfile(args.model):
        print(f"[FAIL] 模型不存在: {args.model}")
        return 2

    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)
    summarize(model)

    if args.info and not args.render and not args.viewer:
        return 0

    mujoco.mj_resetData(model, data)
    apply_nominal_pose(model, data, args.knee, args.hip, args.shoulder)
    mujoco.mj_forward(model, data)

    steps = max(1, int(args.seconds / model.opt.timestep))
    for _ in range(steps):
        mujoco.mj_step(model, data)
    print(f"已仿真 {args.seconds}s，qpos[:3]={np.round(data.qpos[:3], 4)}")

    if args.render:
        gl_backend = os.environ.get("MUJOCO_GL", "(未设置)")
        print(f"MUJOCO_GL = {gl_backend}")
        try:
            renderer = mujoco.Renderer(model, args.height, args.width)
        except Exception as exc:  # noqa: BLE001
            print(f"[FAIL] 无法创建渲染器: {exc}")
            print("      无头渲染需要 MUJOCO_GL=egl|osmesa，以及 libegl1 / libgl1-mesa-dri / libosmesa6")
            return 3
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, camera)
        camera.distance = 1.6
        camera.azimuth = 130.0
        camera.elevation = -12.0
        camera.lookat[:] = [0.0, 0.0, 0.55]
        renderer.update_scene(data, camera)
        pixels = renderer.render()
        try:
            from PIL import Image
        except ImportError:
            print("[FAIL] 缺少 pillow，请先 pip install pillow")
            return 4
        Image.fromarray(pixels).save(args.render)
        print(f"[PASS] 已渲染: {args.render}")
        return 0

    if args.viewer:
        # 注意：这里必须用 from ... import，不能用 import mujoco.viewer，
        # 否则函数内的 import 语句会让 `mujoco` 变成局部名，前面所有 mujoco.xxx 调用
        # 都会抛 UnboundLocalError。
        try:
            from mujoco import viewer as mujoco_viewer
        except Exception as exc:  # noqa: BLE001
            print(f"[FAIL] 无法导入查看器: {exc}")
            return 3
        print("打开查看器（关闭窗口即退出）...需要 WSLg，窗口会出现在 Windows 桌面")
        mujoco_viewer.launch(model, data)
        return 0

    return 0


if __name__ == "__main__":
    sys.exit(main())