#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
N10 系列 2D 雷达 —— 原始点版（不分块）
· 一圈的点全部保留，直接从原始点算墙距 → 不会再有 nan / 空洞
· 扇区内的点在图上高亮显示 → 方便你确认 LEFT_DEG / RIGHT_DEG 对不对
· 加了时间平滑（连续几圈取中位数）→ 输出更稳
"""
import argparse
from collections import deque
import numpy as np
import serial
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

PROFILES = {
    "N10":  dict(baud=230400, header=b"\xA5\x5A\x3A", frame=55,  n=16, end_off=52),
    "N10P": dict(baud=460800, header=b"\xA5\x5A\x6C", frame=105, n=32, end_off=102),
}

# ---------- 左右墙角度（⚠️ 实测确认 0° 朝向后改这里）----------
LEFT_DEG, RIGHT_DEG = 90, 270
SECTOR_HALF = 20        # 扇区半宽（度）
SMOOTH_N    = 5         # 时间平滑窗口（用最近几圈的数据）


def parse_frame(d, n, end_off):
    """一帧 → [(角度°, 距离mm), ...]"""
    start = (d[2]*256 + d[3]) / 100.0
    end   = (d[end_off]*256 + d[end_off+1]) / 100.0
    if end < start:
        end += 360.0
    step = (end - start) / (n - 1) if n > 1 else 0.0
    return [((start + i*step) % 360.0, d[4+i*3]*256 + d[4+i*3+1]) for i in range(n)]


def sector_points(buf, center, half=SECTOR_HALF):
    """筛出扇区内的全部原始点（不分块、不丢点）"""
    return [(a, dd) for a, dd in buf
            if abs((a - center + 180) % 360 - 180) <= half and dd > 0]


def sector_median(pts):
    """原始点的中位数（想更保守可改成 25 分位数）"""
    if not pts:
        return float("nan")
    vals = sorted(dd for _, dd in pts)
    return float(vals[len(vals) // 2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port",  default="/dev/wheeltec_lidar")
    ap.add_argument("--model", default="N10", choices=list(PROFILES))
    args = ap.parse_args()

    p = PROFILES[args.model]
    ser = serial.Serial(args.port, p["baud"], timeout=1)
    print(f"✅ {args.port} @ {p['baud']} ({args.model})   Ctrl+C 退出\n")

    buf, prev_ang = [], None
    hist_L, hist_R = deque(maxlen=SMOOTH_N), deque(maxlen=SMOOTH_N)

    plt.ion()
    fig, ax = plt.subplots(subplot_kw=dict(projection="polar"), figsize=(7, 7))

    try:
        while True:
            # 同步帧头
            if ser.read(1) != p["header"][0:1]: continue
            if ser.read(1) != p["header"][1:2]: continue
            if ser.read(1) != p["header"][2:3]: continue
            d = ser.read(p["frame"])
            if len(d) != p["frame"]: continue

            pts = parse_frame(d, p["n"], p["end_off"])
            buf.extend([(a, dd) for a, dd in pts if dd > 0])

            # 转完一圈？
            cur = pts[0][0]
            wrapped = prev_ang is not None and cur < prev_ang - 100
            prev_ang = cur
            if not wrapped or len(buf) < 50:
                continue

            # ---- 从原始点算墙距 ----
            Lpts = sector_points(buf, LEFT_DEG)
            Rpts = sector_points(buf, RIGHT_DEG)
            L = sector_median(Lpts)
            R = sector_median(Rpts)

            if not np.isnan(L): hist_L.append(L)
            if not np.isnan(R): hist_R.append(R)
            Lf = float(np.median(hist_L)) if hist_L else float("nan")
            Rf = float(np.median(hist_R)) if hist_R else float("nan")
            err = (Lf - Rf) / 1000.0 if not (np.isnan(Lf) or np.isnan(Rf)) else float("nan")

            print(f"点 {len(buf):4d} | 左 {len(Lpts):3d}点 {L/1000:5.2f}m | "
                  f"右 {len(Rpts):3d}点 {R/1000:5.2f}m | "
                  f"平滑: 左 {Lf/1000:5.2f} 右 {Rf/1000:5.2f} 误差 {err:+.2f}m")

            # ---- 画图 ----
            ax.clear(); ax.set_ylim(0, 6000)
            ax.plot(np.deg2rad([a for a, _ in buf]), [dd for _, dd in buf], ".", ms=3)
            if Lpts:
                ax.plot(np.deg2rad([a for a, _ in Lpts]), [dd for _, dd in Lpts], "g.", ms=6)
            if Rpts:
                ax.plot(np.deg2rad([a for a, _ in Rpts]), [dd for _, dd in Rpts], "r.", ms=6)
            fig.canvas.draw(); fig.canvas.flush_events()

            buf = []

    except KeyboardInterrupt:
        print("\n退出")
    finally:
        ser.close()


if __name__ == "__main__":
    main()