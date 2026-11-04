#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
走廊巡检任务 v1

状态机：
    climb   -> 爬升到高度
    cruise  -> 侧飞（高度环工作；横向不纠，靠视觉触发停下）
    align   -> 停侧飞，雷达把 front 调到 1.60，并锁航向
    inspect -> 完全静止，读视觉，判定柜内完整度
    resume  -> 冷却后继续侧飞
    done    -> 降落

设计原则：雷达只在【静止】阶段用（避开巡航时的延迟/振荡/门板干扰）
"""
import asyncio
import math
import multiprocessing as mp
import queue
import threading
import time

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Vision"))

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

from Lidar_source import RealLidar, make_lidar
from Vision_worker import start_vision_process
from collections import deque

# ==================== 配置 ====================
ADDRESS     = "serial:///dev/pixhawk:57600"
LIDAR_PORT  = "/dev/wheeltec_lidar"
LIDAR_MODEL = "N10"

# ---- ⭐ 测试模式 ----
DRY_RUN = True     # True = 不解锁，只验证状态机（手拿飞机走） False = 真飞

# ---- 高度 ----
HOVER_ALT_DOWN    = -1.45
CLIMB_RATE        = -0.35
TAKEOFF_TIMEOUT_S = 30.0
STABILIZE_S       = 3.0

ALT_HOLD_ENABLE = True
ALT_TARGET_M    = 1.30
KP_ALT, KI_ALT, KD_ALT = 0.50, 0.10, 0.0
I_LIMIT_ALT     = 0.30
MAX_DOWN_CMD    = 0.20
ALT_MIN_M       = 0.80

# ---- 巡航 ----
CRUISE_SPEED   = 0.20      # m/s
CRUISE_DIR     = +1.0      # +1 右
CRUISE_MAX_S   = 90.0      # 总巡航时长保护

# ---- 视觉触发 ----
HYDRANT_WINDOW_N = 6       # ⭐ 滑动窗口：最近 6 个视觉帧里
HYDRANT_HITS     = 3       # ⭐ 有 3 帧看到就算触发（不必连续）
INSPECT_S        = 2.5     # ⭐ inspect 累积时长
VISION_STALE_S    = 1.0    # 视觉数据超过这个时间视为失效
RESUME_COOLDOWN_S = 8.0    # 复检后多少秒内不再触发

# ---- 精调 ----
FRONT_TARGET_M = 1.60
ALIGN_TOL_M    = 0.08
ALIGN_TIMEOUT_S = 8.0
KP_LAT, KI_LAT, KD_LAT = 0.35, 0.04, 0.0
MAX_FORWARD    = 0.18
LIDAR_MIN_ALT_M = 0.50
FRONT_MIN_SAFE  = 0.50

# ---- 静止时锁航向 ----
KP_YAW, KI_YAW, KD_YAW = 3.0, 0.0, 0.0
MAX_YAWRATE    = 10.0
YAW_DEADBAND_DEG = 1.5

# ---- 柜内完整度判定 ----
REQUIRED_CLASSES = ["Valve", "Fire_hose", "Fire_nozzle", "Rubber_hose"]

STOP_HOVER_S = 3.0

# ==================== 共享状态 ====================
cmd     = {"forward": 0.0, "right": 0.0, "down": 0.0, "yawspeed": 0.0}
current = {"north": 0.0, "east": 0.0, "down": 0.0,
           "vn": 0.0, "ve": 0.0, "vd": 0.0, "yaw": 0.0, "mode": "UNKNOWN"}
stop_flag = False
PHASE = ["init"]


class PID:
    def __init__(self, kp, ki, kd, i_limit, out_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.out_limit = i_limit, out_limit
        self.i, self.prev_e = 0.0, None
    def reset(self):
        self.i = 0.0; self.prev_e = None
    def update(self, e, dt):
        if not math.isfinite(e):
            e = 0.0
        d = 0.0 if self.prev_e is None else (e - self.prev_e) / dt
        self.prev_e = e
        raw = self.kp*e + self.ki*self.i + self.kd*d
        if not (abs(raw) >= self.out_limit and raw * e > 0):
            self.i = max(-self.i_limit, min(self.i_limit, self.i + e*dt))
        out = self.kp*e + self.ki*self.i + self.kd*d
        return max(-self.out_limit, min(self.out_limit, out))


# ==================== 后台任务 ====================
def _kb_watch():
    global stop_flag
    input()
    stop_flag = True
    print("\n⏸️  收到回车，平稳停止")


async def send_velocity(drone, stop_event):
    while not stop_event.is_set():
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(
            cmd["forward"], cmd["right"], cmd["down"], cmd["yawspeed"]))
        await asyncio.sleep(0.1)


async def alt_hold_task(stop_event):
    dt = 0.05
    pid = PID(KP_ALT, KI_ALT, KD_ALT, I_LIMIT_ALT, MAX_DOWN_CMD)
    while not stop_event.is_set():
        if (not ALT_HOLD_ENABLE) or PHASE[0] == "climb":
            pid.reset()
        else:
            alt = -current["down"]
            if alt < ALT_MIN_M:
                pid.reset(); cmd["down"] = -MAX_DOWN_CMD
            else:
                cmd["down"] = pid.update(alt - ALT_TARGET_M, dt)
        await asyncio.sleep(dt)


async def track_state(drone):
    async def pos():
        async for pv in drone.telemetry.position_velocity_ned():
            current["north"] = pv.position.north_m
            current["east"]  = pv.position.east_m
            current["down"]  = pv.position.down_m
            current["vn"] = pv.velocity.north_m_s
            current["ve"] = pv.velocity.east_m_s
            current["vd"] = pv.velocity.down_m_s
    async def att():
        async for a in drone.telemetry.attitude_euler():
            current["yaw"] = a.yaw_deg
    async def mode():
        async for m in drone.telemetry.flight_mode():
            current["mode"] = str(m)
    await asyncio.gather(pos(), att(), mode())


_VIS_CACHE = {"v": None}

def poll_vision(q):
    """取最新视觉结果。
       ⚠️ 队列为空时返回【上一次的有效结果】（只要没过期），
          避免 20Hz 轮询把 15fps 的数据判成"无视觉"
    """
    while True:
        try:
            _VIS_CACHE["v"] = q.get_nowait()
        except queue.Empty:
            break

    v = _VIS_CACHE["v"]
    if v is None:
        return None
    if (time.time() - v["ts"]) > VISION_STALE_S:
        return None
    return v


# ==================== 主流程 ====================
async def fly(vq, vstop):
    drone = System()
    await drone.connect(system_address=ADDRESS)

    async def wait_conn():
        async for st in drone.core.connection_state():
            if st.is_connected:
                return
    try:
        await asyncio.wait_for(wait_conn(), timeout=15)
        print("✅ 已连接飞控")
    except asyncio.TimeoutError:
        print("❌ 15 秒没连上"); return

    async for h in drone.telemetry.health():
        if h.is_local_position_ok:
            print("✅ 位置 OK"); break

    # ---- 雷达 ----
    lidar = make_lidar(False, port=LIDAR_PORT, model=LIDAR_MODEL)
    t = time.monotonic()
    while not math.isfinite(lidar.read(0.05).front):
        if time.monotonic() - t > 10:
            print("❌ 雷达没数据"); lidar.stop(); return
        time.sleep(0.1)
    print(f"✅ 雷达就绪 front={lidar.read(0.05).front:.3f}m")

    # ---- 视觉子进程已在 __main__ 启动 ----
    print("⏳ 等视觉子进程就绪 ...")
    time.sleep(2.0)

    print(f"⚙️  巡航 {CRUISE_SPEED:.2f}m/s 高度 {ALT_TARGET_M:.2f}m "
          f"| 精调 front→{FRONT_TARGET_M:.2f}m | 需确认 {HYDRANT_CONFIRM_N} 帧")

    threading.Thread(target=_kb_watch, daemon=True).start()

    stop_event = asyncio.Event()
    send_task = None

    if DRY_RUN:
        print("🧪 ===== DRY RUN：不解锁 / 不进 Offboard / 不发指令 =====")
        print("   手拿飞机在走廊里走，机头朝墙，高度 >0.6m")
        print("   观察状态机是否按 cruise → align → inspect → resume 切换")
    else:
        send_task = asyncio.ensure_future(send_velocity(drone, stop_event))
        await asyncio.sleep(1.0)
        print("🔓 解锁")
        try:
            await drone.action.arm()
        except Exception as e:
            print(f"❌ 解锁失败: {e}")
            stop_event.set(); await send_task; lidar.stop(); return
        print("🚁 进入 Offboard")
        try:
            await drone.offboard.start()
        except OffboardError as e:
            print(f"❌ Offboard 启动失败: {e._result.result}")
            stop_event.set(); await send_task
            await drone.action.disarm(); lidar.stop(); return

    state_task = asyncio.ensure_future(track_state(drone))
    alt_task   = asyncio.ensure_future(alt_hold_task(stop_event))

    pid_lat = PID(KP_LAT, KI_LAT, KD_LAT, 1.0, MAX_FORWARD)
    pid_yaw = PID(KP_YAW, KI_YAW, KD_YAW, 10.0, MAX_YAWRATE)
    dt = 0.05
    tick = 0
    state = "cruise" if DRY_RUN else "climb"
    t_state = time.monotonic()
    t_cruise0 = time.monotonic()
    hyd_hist      = deque(maxlen=HYDRANT_WINDOW_N)
    last_vis_n    = -1
    inspect_seen  = {}
    cooldown_until = 0.0
    inspected     = []

    try:
        while not stop_flag:
            alt = -current["down"]
            s = lidar.read()
            front = s.front if math.isfinite(s.front) else float("nan")
            vis = poll_vision(vq)
            # ⭐ 只在拿到【新视觉帧】时更新，空轮询不打断计数
            new_frame = bool(vis and vis["n"] != last_vis_n)
            if new_frame:
                last_vis_n = vis["n"]

            # ============ 状态切换 ============
            now = time.monotonic()

            if state == "climb":
                if current["down"] <= HOVER_ALT_DOWN + 0.10:
                    state = "stabilize"; t_state = now
                    print(f"🛫 到高度 {alt:.2f}m，稳住 {STABILIZE_S:.0f}s")

            elif state == "stabilize":
                if now - t_state > STABILIZE_S:
                    state = "cruise"; t_state = now; t_cruise0 = now
                    print("➡️  开始巡航")

            elif state == "cruise":
                if new_frame:
                    hit = (now > cooldown_until
                           and vis["counts"].get("Fire_hydrant", 0) > 0)
                    hyd_hist.append(hit)
                    if sum(hyd_hist) >= HYDRANT_HITS and now > cooldown_until:
                        state = "align"; t_state = now
                        print(f"🎯 最近 {len(hyd_hist)} 帧里有 "
                              f"{sum(hyd_hist)} 帧看到消防栓 → 精调中...")
                        hyd_hist.clear()
                if now - t_cruise0 > CRUISE_MAX_S:
                    print("⚠️ 巡航超时"); state = "done"; t_state = now

            elif state == "align":
                e = (front - FRONT_TARGET_M) if math.isfinite(front) else float("nan")
                done = math.isfinite(e) and abs(e) < ALIGN_TOL_M
                if done or (now - t_state > ALIGN_TIMEOUT_S):
                    print(f"   {'✅' if done else '⚠️ 超时'} 精调结束 "
                          f"front={front:.3f}m e={e*100:+.1f}cm")
                    inspect_seen = {}
                    state = "inspect"; t_state = now

            elif state == "inspect":
                # ⭐ 累积窗口内所有帧的检出（并集），比单帧稳得多
                if new_frame:
                    for k, c in vis["counts"].items():
                        if c > 0:
                            inspect_seen[k] = max(inspect_seen.get(k, 0), c)

                if now - t_state > INSPECT_S:
                    cnt = dict(inspect_seen)
                    missing = [c for c in REQUIRED_CLASSES if cnt.get(c, 0) == 0]
                    ok = (len(missing) == 0) and bool(cnt)
                    rec = {"t": round(now - t_cruise0, 1),
                           "front": round(front, 2),
                           "counts": cnt,
                           "complete": ok, "missing": missing}
                    inspected.append(rec)
                    print(f"   📋 累积 {INSPECT_S:.1f}s 检出: {cnt}")
                    print(f"   {'✅ 柜内齐全' if ok else '❌ 缺: ' + ','.join(missing)}")
                    inspect_seen = {}
                    state = "resume"; t_state = now
                    cooldown_until = now + RESUME_COOLDOWN_S

            elif state == "resume":
                if now - t_state > 1.0:
                    state = "cruise"; t_state = now
                    hyd_hist.clear()      # ⭐ 清空，避免残留命中

            elif state == "done":
                break

            # ============ 输出指令 ============
            if state == "climb":
                cmd["forward"] = cmd["right"] = cmd["yawspeed"] = 0.0
                cmd["down"] = (0.0 if current["down"] <= HOVER_ALT_DOWN + 0.10
                               else CLIMB_RATE)
            elif state == "cruise":
                cmd["forward"] = cmd["yawspeed"] = 0.0
                cmd["right"] = CRUISE_SPEED * CRUISE_DIR
            elif state == "align":
                # 横向：front -> 目标
                e = front - FRONT_TARGET_M
                ok = (math.isfinite(e) and abs(e) < 0.80
                      and front > FRONT_MIN_SAFE and alt > LIDAR_MIN_ALT_M)
                cmd["forward"] = pid_lat.update(e, dt) if ok else 0.0
                if not ok: pid_lat.reset()
                # 航向锁
                if math.isfinite(s.yaw_err) and alt > LIDAR_MIN_ALT_M:
                    ey = math.degrees(s.yaw_err)
                    if abs(ey) < YAW_DEADBAND_DEG: ey = 0.0
                    cmd["yawspeed"] = pid_yaw.update(ey, dt)
                else:
                    pid_yaw.reset(); cmd["yawspeed"] = 0.0
                cmd["right"] = 0.0
            else:   # inspect / resume
                cmd["forward"] = cmd["right"] = 0.0
                if state == "inspect" and math.isfinite(s.yaw_err):
                    ey = math.degrees(s.yaw_err)
                    if abs(ey) < YAW_DEADBAND_DEG: ey = 0.0
                    cmd["yawspeed"] = pid_yaw.update(ey, dt)
                else:
                    cmd["yawspeed"] = 0.0

            # ============ 打印 ============
            tick += 1
            if tick % 10 == 0:
                if vis:
                    hc = max([d[1] for d in vis["dets"]
                              if d[0] == "Fire_hydrant"] or [0.0])
                    vtxt = (f"{vis['counts']}@{vis['fps']}fps "
                            f"hyd={hc:.2f} win={sum(hyd_hist)}/{len(hyd_hist)}")
                else:
                    vtxt = "无视觉"
                print(f"  [{state:9s}] alt={alt:4.2f} d={cmd['down']:+.3f} "
                      f"fwd={cmd['forward']:+.3f} r={cmd['right']:+.2f} "
                      f"f={front:5.3f} ye={math.degrees(s.yaw_err):+5.1f}° "
                      f"| {vtxt}")
            await asyncio.sleep(dt)

    except Exception as e:
        print(f"\n❌ 异常: {type(e).__name__}: {e}")
    finally:
        cmd["forward"] = cmd["right"] = cmd["down"] = cmd["yawspeed"] = 0.0
        await asyncio.sleep(0.3)

        stop_event.set()
        if send_task is not None:
            try: await drone.offboard.stop()
            except Exception: pass
            try: await send_task
            except Exception: pass
            print("🛬 降落")
            mode_now = current.get("mode", "")
            if "OFFBOARD" in mode_now.upper():
                try:
                    await drone.action.land(); print("✅ 已发送降落")
                except Exception as e:
                    print(f"⚠️ land 失败: {type(e).__name__}")
            else:
                print(f"⚠️ 模式已是 {mode_now} → 跳过自动降落")
        else:
            print("🧪 DRY RUN 结束（从未解锁）")
        alt_task.cancel(); state_task.cancel()

        lidar.stop()

        print("\n📊 巡检记录：")
        for i, r in enumerate(inspected, 1):
            print(f"   #{i} t={r['t']}s front={r['front']}m "
                  f"complete={r['complete']} {r['counts']}")
        if not inspected:
            print("   （没有触发任何检测点）")
        print("✅ 完成")


if __name__ == "__main__":
    # ⭐ 先起视觉子进程（此时还没连 mavsdk，fork 安全）
    vq = mp.Queue()
    vstop = mp.Event()
    vproc = mp.Process(target=start_vision_process, args=(vq, vstop), daemon=True)
    vproc.start()
    print(f"[Main] 视觉子进程 pid={vproc.pid}")
    time.sleep(1.0)

    try:
        asyncio.run(fly(vq, vstop))
    except Exception as e:
        print(f"\n❌ 顶层异常: {type(e).__name__}: {e}")
        print("⚠️ 若飞机还在空中，立刻用遥控器切到 Position/Stabilized 接管！")
    finally:
        vstop.set()
        vproc.join(timeout=2.0)
        if vproc.is_alive():
            vproc.terminate()
            print("[Main] 视觉子进程已强制结束")