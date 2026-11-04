#!/usr/bin/env python3
"""真雷达地面校准：手持飞机，看数据对不对（不飞！）"""
import math
import time
from Lidar_source import RealLidar

PORT  = "/dev/wheeltec_lidar"
MODEL = "N10"

lidar = RealLidar(PORT, MODEL)

print("=" * 60)
print("手持飞机，机头对墙、举到合适高度，按下面做：")
print("  1) 摆正不动        -> yaw_err ≈ 0，d_f ≈ d_b")
print("  2) 俯视顺时针转20° -> yaw_err 应该变【负】")
print("  3) 俯视逆时针转20° -> yaw_err 应该变【正】")
print("  4) 向机头方向平移   -> e_lat  应该变【负】")
print("  5) 向后退           -> e_lat  应该变【正】")
print("=" * 60)

try:
    while True:
        s = lidar.read()
        if not s.ok:
            print("⚠️  数据无效（没同时测到两面墙，检查角度/位置）")
        else:
            print(f"d_f={s.front:5.2f}m  d_b={s.back:5.2f}m  "
                  f"e_lat={s.e_lat*100:+6.1f}cm  "
                  f"yaw_err={math.degrees(s.yaw_err):+6.1f}°")
        time.sleep(0.3)
except KeyboardInterrupt:
    lidar.stop()
    print("\n退出")