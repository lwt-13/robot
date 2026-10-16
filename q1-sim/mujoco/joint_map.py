#!/usr/bin/env python3
"""对齐 SDK / URDF / MJCF 三方的关节命名，并提供「关节名 -> 索引」映射。

为什么要这个模块：SDK 的 `aimdk_msgs/msg/JointCommandArray` 是**按关节名**下发的
（`string name` + position/velocity/effort/stiffness/damping），而 MuJoCo 需要关节的
qpos/dof/actuator 索引。三方名字若不一致就必须做转换，这里把这件事验证并固化下来。

已验证结论：厂商 `config/joint_names_Q1-V2.0-URDF-0601.yaml` 里的 22 个关节名，
与 `q1_v3_mc.xml` 的关节名和顺序完全一致（yaml 索引 0 是空串，对应浮动基座 free joint）。
"""
import os
import pathlib
import re

import mujoco

DEFAULT_UPSTREAM = pathlib.Path(os.environ.get("Q1_URDF_SRC", pathlib.Path.home() / "q1_sim" / "q1_urdf"))
YAML_RELATIVE = "config/joint_names_Q1-V2.0-URDF-0601.yaml"
MJCF_RELATIVE = "xml/q1_v3_mc.xml"


def load_sdk_joint_names(yaml_path: pathlib.Path | None = None) -> list[str]:
    """从厂商 yaml 里读出 SDK 侧的关节名列表（去掉开头的空串）。"""
    path = yaml_path or _find_yaml()
    text = path.read_text(encoding="utf-8")
    names = re.findall(r"'([^']*)'", text)
    names = [n for n in names if n]
    if not names:
        raise RuntimeError(f"没能从 {path} 解析出关节名")
    return names


def _find_yaml() -> pathlib.Path:
    candidates = [
        DEFAULT_UPSTREAM / YAML_RELATIVE,
        pathlib.Path.home() / "q1_ws" / "src" / "q1_description" / "config" / pathlib.Path(YAML_RELATIVE).name,
    ]
    for cand in candidates:
        if cand.is_file():
            return cand
    raise FileNotFoundError(f"找不到关节名 yaml，尝试过: {[str(c) for c in candidates]}")


def mjcf_joint_names(model: "mujoco.MjModel", skip_free: bool = True) -> list[str]:
    """按 MJCF 里的顺序取关节名，默认跳过 free joint（浮动基座）。"""
    out = []
    for jid in range(model.njnt):
        if skip_free and model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_FREE:
            continue
        out.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid) or f"joint_{jid}")
    return out


def build_mapping(model: "mujoco.MjModel", names: list[str] | None = None) -> dict:
    """返回 {关节名: {'joint_id','qpos_adr','dof_adr','actuator_id','range_deg','ctrlrange'}}。"""
    names = names or mjcf_joint_names(model)

    act_of_joint = {}
    for aid in range(model.nu):
        jid = int(model.actuator_trnid[aid][0])
        act_of_joint.setdefault(jid, aid)

    mapping = {}
    for name in names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            mapping[name] = None
            continue
        entry = {
            "joint_id": jid,
            "qpos_adr": int(model.jnt_qposadr[jid]),
            "dof_adr": int(model.jnt_dofadr[jid]),
            "actuator_id": act_of_joint.get(jid),
            "range_rad": None,
            "ctrlrange": None,
        }
        if model.jnt_limited[jid]:
            lo, hi = model.jnt_range[jid]
            entry["range_rad"] = (float(lo), float(hi))
        aid = entry["actuator_id"]
        if aid is not None:
            entry["ctrlrange"] = tuple(float(v) for v in model.actuator_ctrlrange[aid])
        mapping[name] = entry
    return mapping


def load_model_and_mapping(mjcf_path: str | os.PathLike | None = None):
    """一步拿到 (model, sdk_names, mapping)，并做一致性校验。"""
    path = pathlib.Path(mjcf_path) if mjcf_path else (DEFAULT_UPSTREAM / MJCF_RELATIVE)
    model = mujoco.MjModel.from_xml_path(str(path))
    sdk_names = load_sdk_joint_names()
    mjcf_names = mjcf_joint_names(model)

    if sdk_names != mjcf_names:
        only_sdk = [n for n in sdk_names if n not in mjcf_names]
        only_mjcf = [n for n in mjcf_names if n not in sdk_names]
        raise RuntimeError(
            "SDK 关节名与 MJCF 关节名不一致：\n"
            f"  顺序是否相同: {sdk_names == mjcf_names}\n"
            f"  仅 SDK 有: {only_sdk}\n"
            f"  仅 MJCF 有: {only_mjcf}"
        )
    return model, sdk_names, build_mapping(model, sdk_names)


def main() -> int:
    model, sdk_names, mapping = load_model_and_mapping()
    print(f"MJCF: {DEFAULT_UPSTREAM / MJCF_RELATIVE}")
    print(f"关节名 yaml: {_find_yaml()}")
    print(f"关节数: {len(sdk_names)}   执行器数: {model.nu}")
    print("\n序号 关节名                              执行器  关节限位(deg)            力矩限位")
    for idx, name in enumerate(sdk_names):
        e = mapping[name]
        if e is None:
            print(f"{idx:>4} {name:<34} [未在 MJCF 中找到]")
            continue
        rng = e["range_rad"]
        rng_s = f"[{rng[0] * 57.29578:7.1f},{rng[1] * 57.29578:7.1f}]" if rng else "(无限位)"
        ctrl = e["ctrlrange"]
        ctrl_s = f"[{ctrl[0]:6.1f},{ctrl[1]:6.1f}]" if ctrl else "无执行器"
        print(f"{idx:>4} {name:<34} {str(e['actuator_id']):>6}  {rng_s:<24} {ctrl_s}")

    unactuated = [n for n, e in mapping.items() if e and e["actuator_id"] is None]
    print(f"\n无执行器的关节: {unactuated if unactuated else '无（22 个关节全部可驱动）'}")
    print("[PASS] SDK 关节名与 MJCF 完全对齐，可按名字直接驱动仿真")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())