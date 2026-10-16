#!/usr/bin/env python3
"""直接抓取指定 X11 窗口（而不是 root window），用于在 WSLg 下给 RViz2 留证据。

背景：WSLg 用 Weston 合成窗口，XWayland 的 root window 是空的，
所以 mss 抓整屏只能得到纯色。但窗口自身可能仍有可读内容，
因此这里遍历窗口树、找出尺寸合适的候选窗口并逐个尝试抓取。

用法:  python3 shot_window.py [--match RViz] [--out 路径] [--list]
"""
from __future__ import annotations

import argparse
import os
import sys

DEFAULT_OUT = "/mnt/e/robot/q1-sim/docs/q1_rviz2.png"


def interesting(name: str | None, w: int, h: int) -> bool:
    if w < 300 or h < 300:
        return False
    if name and any(k in name for k in ("RViz", "rviz", "Ogre", "Qt")):
        return True
    return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--match", default="RViz")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--list", action="store_true", help="只列出窗口树")
    args = ap.parse_args()

    if not os.environ.get("DISPLAY"):
        print("[FAIL] 没有 DISPLAY")
        return 1
    try:
        from Xlib import X, display
        from PIL import Image
    except ImportError as exc:
        print(f"[FAIL] 缺少依赖: {exc}（需要 python-xlib 与 Pillow）")
        return 1

    d = display.Display()
    root = d.screen().root
    candidates = []

    def walk(win, depth: int = 0) -> None:
        try:
            name = win.get_wm_name()
            geom = win.get_geometry()
        except Exception:  # noqa: BLE001
            return
        if args.list and (name or depth < 2):
            print(f"{'  ' * depth}{hex(win.id)} {str(name)[:40]:<42} {geom.width}x{geom.height}")
        if interesting(name, geom.width, geom.height):
            candidates.append((win, name, geom.width, geom.height))
        for child in win.query_tree().children:
            walk(child, depth + 1)

    walk(root)
    print(f"候选窗口: {[(n, f'{w}x{h}') for _, n, w, h in candidates] or '无'}")

    best = None
    for win, name, w, h in candidates:
        try:
            raw = win.get_image(0, 0, w, h, X.ZPixmap, 0xFFFFFFFF)
            img = Image.frombytes("RGB", (w, h), raw.data, "raw", "BGRX")
        except Exception as exc:  # noqa: BLE001
            print(f"  {name}: 抓取失败 ({type(exc).__name__})")
            continue
        colors = img.getcolors(maxcolors=1 << 24) or []
        colors.sort(reverse=True)
        frac = colors[0][0] / (w * h) if colors else 1.0
        print(f"  {name}: {w}x{h}, 颜色数={len(colors)}, 最常见占比={frac:.1%}")
        if best is None or len(colors) > best[0]:
            best = (len(colors), img, name)

    if best is None:
        print("[FAIL] 没有抓到任何窗口内容")
        return 1
    n_colors, img, name = best
    img.save(args.out)
    print(f"[PASS] 已保存 {args.out}（来自窗口 {name}，{img.size[0]}x{img.size[1]}，"
          f"颜色数={n_colors}，{os.path.getsize(args.out)} 字节）")
    return 0 if n_colors > 100 else 2


if __name__ == "__main__":
    sys.exit(main())