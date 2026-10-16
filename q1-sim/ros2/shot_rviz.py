#!/usr/bin/env python3
"""抓取当前显示（WSLg 的 X11 显示）保存为 PNG，用于给 RViz2 留下视觉证据。

为什么用 mss 而不是 xwd/import：这台机器上没有任何截图工具，装它们需要 sudo，
而 mss 是纯 Python（走 ctypes 调 libX11），pip 装上即可用。

用法:  python3 shot_rviz.py [输出路径] [--wait 秒]
"""
from __future__ import annotations

import argparse
import os
import sys
import time

DEFAULT_OUT = "/mnt/e/robot/q1-sim/docs/q1_rviz2.png"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out", nargs="?", default=DEFAULT_OUT)
    ap.add_argument("--wait", type=float, default=0.0, help="抓屏前等待秒数")
    args = ap.parse_args()

    if args.wait > 0:
        time.sleep(args.wait)

    display = os.environ.get("DISPLAY")
    print(f"DISPLAY={display}  WAYLAND_DISPLAY={os.environ.get('WAYLAND_DISPLAY')}")
    if not display:
        print("[FAIL] 没有 DISPLAY，无法抓 X11 画面")
        return 1

    try:
        import mss
        import mss.tools
    except ImportError:
        print("[FAIL] 未安装 mss：pip install mss")
        return 1

    try:
        with mss.mss() as sct:
            monitors = sct.monitors
            print(f"显示器数量: {len(monitors) - 1}，全部区域 {monitors[0]}")
            shot = sct.grab(monitors[0])
            mss.tools.to_png(shot.rgb, shot.size, output=args.out)
    except Exception as exc:  # noqa: BLE001
        print(f"[FAIL] 抓屏失败: {type(exc).__name__}: {exc}")
        return 1

    size = os.path.getsize(args.out)
    print(f"[PASS] 已保存 {args.out}（{shot.size[0]}x{shot.size[1]}, {size} 字节）")

    # 程序化判断画面不是空白：我无法直接看图，所以用像素统计代替
    try:
        from PIL import Image
        with Image.open(args.out) as im:
            rgb = im.convert("RGB")
            colors = rgb.getcolors(maxcolors=1 << 24) or []
            colors.sort(reverse=True)
            total = rgb.size[0] * rgb.size[1]
            top_fraction = colors[0][0] / total if colors else 0.0
            print(f"  不同颜色数: {len(colors)}，最常见颜色占比: {top_fraction:.1%}")
            if len(colors) > 100 and top_fraction < 0.98:
                print("  [PASS] 画面有内容（不是纯色空白）")
            else:
                print("  [WARN] 画面接近纯色，可能是空窗口或抓屏拿不到合成结果")
    except ImportError:
        print("  [WARN] 没有 PIL，跳过画面内容判断")
    return 0


if __name__ == "__main__":
    sys.exit(main())