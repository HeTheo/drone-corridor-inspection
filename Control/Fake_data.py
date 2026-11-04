import json, math, socket, time

# ★★★ 这两个数必须和 HTML 里的 WALL_N / TARGET_DIST 完全一致 ★★★
WALL_N = 1.245      # HTML 里的 WALL_N
FRONT  = 1.65       # HTML 里的 TARGET_DIST（= Python 的 FRONT_TARGET_M）

HOST, PORT = "127.0.0.1", 9999
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"WALL_N={WALL_N}  FRONT={FRONT}  →  north = {WALL_N - FRONT:+.3f}")
print("开始发送假数据…（Ctrl+C 停止）")

t0 = time.time(); k = 0
while True:
    t = time.time() - t0
    along = min(12.0, 0.43 * t)                  # 0.43 m/s（= V_SIDE）
    front = FRONT + 0.05 * math.sin(t * 1.5)     # 贴墙定距 ±5cm 抖动
    yaw   = 3.0 * math.sin(t * 0.7)              # 航向 ±3°

    sock.sendto(json.dumps({
        "type":  "pos",
        "north": round(WALL_N - front, 3),       # ★ 关键：由 front 反算
        "east":  round(along, 3),
        "down":  -1.20,
        "yaw":   round(yaw, 1),
        "hyd":   k,
    }).encode(), (HOST, PORT))

    if k < 3 and along >= 5.0 * (k + 1):
        sock.sendto(json.dumps({
            "type": "det", "east": round(5.0 * (k + 1), 3), "index": k,
            "status": "ok",
            "counts": {"Fire_hydrant": 1, "Valve": 1},
            "missing": [],
        }).encode(), (HOST, PORT))
        k += 1
        print(f"  假装识别到第 {k} 个消防栓 @ east={5.0*k:.1f}m")

    time.sleep(0.1)