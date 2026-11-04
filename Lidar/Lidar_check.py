#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
雷达静态检查：不连飞控、不解锁、不开电机

只回答三个问题：
  1. 有数据吗？
  2. front / back 跟着真实墙距变吗？
  3. e_lat / yaw_err 的符号对不对？

用法：
  python Lidar_check.py
  然后按下面的"三个测试"用手推/转飞机，看终端数字
"""
import math
import time

from Lidar_source import make_lidar, RealLidar

PORT  = "/dev/wheeltec_lidar"
MODEL = "N10"

print(f"⏳ 打开雷达 {PORT} ({MODEL}) ...")
lidar = make_lidar(False, port=PORT, model=MODEL)
print("✅ 已打开，放地上别动，等 2 秒\n")

print(f"{'front':>8} {'back':>8} {'e_lat':>10} {'yaw_err':>10}")
print("-" * 42)

try:
    while True:
        s = lidar.read()
        if not s.ok:
            print("❌ 无有效数据：检查端口 / 波特率 / 雷达是否上电")
            time.sleep(1.0)
            continue

        if math.isnan(s.yaw_err):
            ye = "       nan"
        else:
            ye = f"{math.degrees(s.yaw_err):+9.1f}°"

        print(f"{s.front:8.3f} {s.back:8.3f} {s.e_lat*100:+9.1f}cm {ye}")
        time.sleep(0.3)

except KeyboardInterrupt:
    print("\n⏹️  结束")
finally:
    if isinstance(lidar, RealLidar):
        lidar.stop()