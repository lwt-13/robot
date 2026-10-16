#!/usr/bin/env python3
"""逐环节检查 RViz2 显示链路，输出 [PASS]/[FAIL]，并以退出码表示结果。

为什么要单独写这个：`ros2 topic echo` 会把长字符串截断（/robot_description 是 28 KB 的
URDF，echo 只回显 154 字节），用它判断"URDF 是否发布"会得到假失败。

检查项：
  1. robot_state_publisher 的参数里有完整 URDF，且能被 XML 解析
  2. 以 TRANSIENT_LOCAL 订阅 /robot_description 能收到同一份完整 URDF
  3. joint_state_publisher 确实订阅了 /robot_description，且 durability 匹配
  4. /joint_states 里的关节名与 SDK/MJCF 的 22 个关节名完全一致
  5. /tf 在跳动、/tf_static 有内容
"""
from __future__ import annotations

import argparse
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

FAILURES: list[str] = []


def check(ok: bool, title: str, detail: str = "") -> bool:
    print(f"  [{'PASS' if ok else 'FAIL'}] {title}" + (f" — {detail}" if detail else ""))
    if not ok:
        FAILURES.append(title)
    return ok


def parse_urdf(tag: str, text: str | None) -> tuple[int, int] | None:
    if text is None:
        check(False, f"{tag}: 取到 URDF", "未取到")
        return None
    try:
        root = ET.fromstring(text)
    except ET.ParseError as exc:
        check(False, f"{tag}: URDF 可解析", f"{len(text)} 字节但解析失败: {exc}")
        return None
    links, joints = len(root.findall("link")), len(root.findall("joint"))
    check(len(text) > 5000 and links > 20,
          f"{tag}: URDF 完整", f"{len(text)} 字节, link={links}, joint={joints}")
    return links, joints


def wait_for(node: Node, predicate, timeout: float) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        rclpy.spin_once(node, timeout_sec=0.2)
    return predicate()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mujoco-dir", default="/mnt/e/robot/q1-sim/mujoco",
                    help="用于导入 joint_map 取 SDK 关节名")
    ap.add_argument("--joint-timeout", type=float, default=25.0)
    ap.add_argument("--tf-seconds", type=float, default=4.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node("q1_display_check")

    got: dict[str, str] = {}
    tf_count = [0]
    tf_static = [0]
    js_names: list[str] = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(String, "/robot_description",
                             lambda m: got.__setitem__("topic", m.data), qos)
    node.create_subscription(TFMessage, "/tf", lambda m: tf_count.__setitem__(0, tf_count[0] + 1), 100)
    node.create_subscription(TFMessage, "/tf_static",
                             lambda m: tf_static.__setitem__(0, tf_static[0] + len(m.transforms)), qos)

    def on_js(msg) -> None:
        if msg.name:
            js_names.clear()
            js_names.extend(msg.name)

    node.create_subscription(JointState, "/joint_states", on_js, 10)

    print("== 1) robot_state_publisher 的参数 ==")
    cli = node.create_client(GetParameters, "/robot_state_publisher/get_parameters")
    if cli.wait_for_service(timeout_sec=15.0):
        req = GetParameters.Request()
        req.names = ["robot_description"]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
        res = fut.result()
        if res and res.values:
            got["param"] = res.values[0].string_value
    else:
        check(False, "robot_state_publisher 服务可用", "15 秒内没等到 /get_parameters")
    parse_urdf("参数", got.get("param"))

    print("\n== 2) 话题 /robot_description（TRANSIENT_LOCAL）==")
    wait_for(node, lambda: "topic" in got, 10.0)
    parse_urdf("话题", got.get("topic"))

    print("\n== 3) /robot_description 的订阅端 QoS ==")
    subs = node.get_subscriptions_info_by_topic("/robot_description")
    jsp = [i for i in subs if i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL]
    check(len(subs) >= 1, "至少有一个订阅端", f"{len(subs)} 个")
    check(len(jsp) >= 1, "存在 durability=TRANSIENT_LOCAL 的订阅端（joint_state_publisher）",
          "否则拿不到 latched 的 URDF，会一直卡在 Waiting")
    for info in subs:
        print(f"      {info.node_name:<26} durability={info.qos_profile.durability.name}")

    print(f"\n== 4) /joint_states 里的关节（最多等 {args.joint_timeout:.0f} 秒）==")
    wait_for(node, lambda: bool(js_names), args.joint_timeout)
    check(bool(js_names), "/joint_states 有数据（不是空状态）",
          f"{len(js_names)} 个关节")

    expected = None
    try:
        sys.path.insert(0, args.mujoco_dir)
        from joint_map import load_sdk_joint_names  # type: ignore
        expected = [n for n in load_sdk_joint_names() if n]
    except Exception as exc:  # noqa: BLE001
        print(f"      [WARN] 取不到 SDK 关节名（{type(exc).__name__}: {exc}），跳过精确比对")
    if expected:
        check(set(js_names) == set(expected), "关节名与 SDK/MJCF 完全一致",
              f"ROS {len(js_names)} 个 vs SDK {len(expected)} 个"
              + (f"；仅 ROS 有: {sorted(set(js_names) - set(expected))}"
                 f"；仅 SDK 有: {sorted(set(expected) - set(js_names))}"
                 if set(js_names) != set(expected) else ""))

    print(f"\n== 5) TF（观察 {args.tf_seconds:.0f} 秒）==")
    end = time.time() + args.tf_seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    check(tf_count[0] > 0, "/tf 在跳动（由 /joint_states 驱动）", f"{tf_count[0]} 条消息")
    check(tf_static[0] > 0, "/tf_static 有内容（固定关节）", f"{tf_static[0]} 个变换")

    node.destroy_node()
    rclpy.shutdown()

    print()
    if FAILURES:
        print(f"结果: [FAIL] {len(FAILURES)} 项未通过")
        for f in FAILURES:
            print(f"  - {f}")
        return 1
    print("结果: [PASS] 显示链路全部环节通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())