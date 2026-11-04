#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
侧飞 + 高度外环（无雷达、无日志）

流程：起飞 → 定高 → 稳住 → 侧飞 → 停 → 悬停 → 降落

核心：脚本侧【高度外环 PID】
    为什么需要：悬停时推力够（不超过 MPC_THR_HOVER），
                但平移时额外耗功，PX4 速度环补不上 → 一直掉。
    做法：读 alt（测距真值）→ PID → 直接改 down 指令。
    位置：独立 20Hz 任务 alt_hold_task，只在非爬升阶段接管。

参数在配置区，改 V_SIDE 试不同侧飞速度。
"""
import asyncio
import math
import threading
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

# ==================== 配置 ====================
ADDRESS = "serial:///dev/pixhawk:57600"

# ---- 高度 ----
HOVER_ALT_DOWN    = -1.45   # 命令爬到 1.45m（到 1.35m 就停）
CLIMB_RATE        = -0.35
TAKEOFF_TIMEOUT_S = 30.0
STABILIZE_S       = 3.0

# ---- ⭐ 高度外环 PID ----
ALT_HOLD_ENABLE = True
ALT_TARGET_M    = 1.30      # 想保持的高度
KP_ALT          = 0.50      # 1m 误差 -> 0.5 m/s 修正
KI_ALT          = 0.10      # 小积分，消残余稳态误差
KD_ALT          = 0.00
I_LIMIT_ALT     = 0.30      # 积分上限（输出最多贡献 KI*0.30 = 0.03 m/s）
MAX_DOWN_CMD    = 0.20      # down 指令限幅 m/s
ALT_MIN_M       = 0.80      # ⭐ 低于这个高度 → 无条件全力爬升

# ---- 侧飞 ----
V_SIDE      = 0.20          # ⭐ 慢飞试：0.20（原来 0.50）
SIDE_TIME_S = 10.0          # 10s × 0.20 = 2.0m
SIDE_DIR    = +1.0          # +1 向右，-1 向左

STOP_HOVER_S = 8.0

# ==================== 共享状态 ====================
cmd     = {"forward": 0.0, "right": 0.0, "down": 0.0, "yawspeed": 0.0}
current = {"north": 0.0, "east": 0.0, "down": 0.0,
           "vn": 0.0, "ve": 0.0, "vd": 0.0,
           "yaw": 0.0, "mode": "UNKNOWN"}

stop_flag = False
PHASE = ["init"]        # climb / hold / side / stop


# ============================================================
#  PID（条件积分抗饱和）
# ============================================================
class PID:
    def __init__(self, kp, ki, kd, i_limit, out_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.out_limit = i_limit, out_limit
        self.i, self.prev_e = 0.0, None

    def reset(self):
        self.i = 0.0
        self.prev_e = None

    def update(self, e, dt):
        if not math.isfinite(e):
            e = 0.0

        d = 0.0 if self.prev_e is None else (e - self.prev_e) / dt
        self.prev_e = e

        raw = self.kp*e + self.ki*self.i + self.kd*d
        saturated = abs(raw) >= self.out_limit
        if not (saturated and raw * e > 0):
            self.i = max(-self.i_limit, min(self.i_limit, self.i + e*dt))

        out = self.kp*e + self.ki*self.i + self.kd*d
        return max(-self.out_limit, min(self.out_limit, out))


# ============================================================
#  后台任务
# ============================================================
def _kb_watch():
    global stop_flag
    input()
    stop_flag = True
    print("\n⏸️  收到回车，平稳停止")


async def send_velocity(drone, stop_event):
    """10Hz 点流"""
    while not stop_event.is_set():
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(
            cmd["forward"], cmd["right"], cmd["down"], cmd["yawspeed"]))
        await asyncio.sleep(0.1)


async def alt_hold_task(stop_event):
    """⭐ 高度外环 20Hz
       e = alt - ALT_TARGET_M ：>0 太高 → down 为正(下降)
       爬升阶段不接管（climb 自己管 down）
    """
    dt = 0.05
    pid = PID(KP_ALT, KI_ALT, KD_ALT, I_LIMIT_ALT, MAX_DOWN_CMD)

    while not stop_event.is_set():
        if (not ALT_HOLD_ENABLE) or PHASE[0] == "climb":
            pid.reset()
        else:
            alt = -current["down"]
            if alt < ALT_MIN_M:                 # ⭐ 太低：无条件全力爬
                pid.reset()
                cmd["down"] = -MAX_DOWN_CMD
            else:
                e = alt - ALT_TARGET_M
                cmd["down"] = pid.update(e, dt)
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


# ============================================================
#  主流程
# ============================================================
async def fly():
    drone = System()
    print(f"⏳ 连接 {ADDRESS} ...")
    await drone.connect(system_address=ADDRESS)

    async def wait_conn():
        async for st in drone.core.connection_state():
            if st.is_connected:
                return
    try:
        await asyncio.wait_for(wait_conn(), timeout=15)
        print("✅ 已连接飞控")
    except asyncio.TimeoutError:
        print("❌ 15 秒没连上：查 ADDRESS / 端口 / 是否关了 QGC")
        return

    async for h in drone.telemetry.health():
        if h.is_local_position_ok:
            print("✅ 位置 OK")
            break

    print(f"⚙️  爬升 {CLIMB_RATE:+.2f} → 停 {-HOVER_ALT_DOWN-0.10:.2f}m")
    print(f"⚙️  ⭐ 高度外环: 目标 {ALT_TARGET_M:.2f}m  "
          f"KP={KP_ALT} KI={KI_ALT} 限幅={MAX_DOWN_CMD}m/s  "
          f"下限={ALT_MIN_M:.2f}m")
    print(f"⚙️  侧飞 {V_SIDE:.2f}m/s × {SIDE_TIME_S:.0f}s "
          f"({V_SIDE*SIDE_TIME_S:.2f}m) 方向 {'右' if SIDE_DIR > 0 else '左'}")
    print(f"⚠️  确认侧飞方向 {V_SIDE*SIDE_TIME_S + 1.5:.1f}m 内无障碍")

    threading.Thread(target=_kb_watch, daemon=True).start()

    stop_event = asyncio.Event()
    send_task = asyncio.ensure_future(send_velocity(drone, stop_event))
    await asyncio.sleep(1.0)

    print("🔓 解锁")
    try:
        await drone.action.arm()
    except Exception as e:
        print(f"❌ 解锁失败: {e}")
        stop_event.set(); await send_task
        return

    print("🚁 进入 Offboard")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"❌ Offboard 启动失败: {e._result.result}")
        stop_event.set(); await send_task; await drone.action.disarm()
        return

    state_task = asyncio.ensure_future(track_state(drone))
    alt_task   = asyncio.ensure_future(alt_hold_task(stop_event))
    dt = 0.05
    tick = 0
    alt_min_seen = 99.0

    try:
        # ========== 阶段 1：爬升（高度外环暂不接管）==========
        PHASE[0] = "climb"
        print("🛫 起飞中...")
        t0 = time.monotonic()
        t_prev = t0
        alt_prev = -current["down"]
        while not stop_flag:
            alt = -current["down"]
            cmd["forward"] = cmd["right"] = cmd["yawspeed"] = 0.0
            cmd["down"] = (0.0 if current["down"] <= HOVER_ALT_DOWN + 0.10
                           else CLIMB_RATE)

            tick += 1
            if tick % 10 == 0:
                now = time.monotonic()
                d_el = max(1e-3, now - t_prev)
                v_act = (alt - alt_prev) / d_el
                t_prev, alt_prev = now, alt
                flag = ""
                if alt > 0.50 and CLIMB_RATE < 0 and v_act < abs(CLIMB_RATE)*0.6:
                    flag = "  ⚠️爬升速率不足"
                print(f"  高度={alt:+5.2f}m 命令{CLIMB_RATE:+.2f} "
                      f"实际{v_act:+.3f}{flag}")

            if current["down"] <= HOVER_ALT_DOWN + 0.10:
                break
            if time.monotonic() - t0 > TAKEOFF_TIMEOUT_S:
                print("⚠️ 起飞超时")
                break
            await asyncio.sleep(dt)

        print(f"   爬升耗时 {time.monotonic()-t0:.1f}s，高度 {alt:+.2f}m")

        # ========== 阶段 2：稳住（⭐ 高度外环开始接管）==========
        PHASE[0] = "hold"
        print(f"⏸️  悬停稳住 {STABILIZE_S:.0f}s（高度外环已接管）...")
        t_s = time.monotonic()
        while not stop_flag and time.monotonic() - t_s < STABILIZE_S:
            tick += 1
            if tick % 10 == 0:
                alt = -current["down"]
                alt_min_seen = min(alt_min_seen, alt)
                print(f"  稳 alt={alt:4.2f}m vz={-current['vd']:+.3f} "
                      f"d={cmd['down']:+.3f}")
            await asyncio.sleep(dt)

        # ========== 阶段 3：侧飞 ==========
        PHASE[0] = "side"
        print(f"➡️  开始侧飞 {V_SIDE:.2f}m/s × {SIDE_TIME_S:.0f}s ...")
        t_sd = time.monotonic()
        while not stop_flag and time.monotonic() - t_sd < SIDE_TIME_S:
            el = time.monotonic() - t_sd
            cmd["forward"] = cmd["yawspeed"] = 0.0
            cmd["right"] = V_SIDE * SIDE_DIR
            # ⚠️ 不碰 cmd["down"]，交给 alt_hold_task

            alt = -current["down"]
            alt_min_seen = min(alt_min_seen, alt)

            tick += 1
            if tick % 10 == 0:
                print(f"  侧飞{el:5.1f}s(走{V_SIDE*el:4.2f}m) "
                      f"alt={alt:4.2f}m vz={-current['vd']:+.3f} "
                      f"d={cmd['down']:+.3f} r={cmd['right']:+.2f} "
                      f"yaw={current['yaw']:+6.1f}° "
                      f"n={current['north']:+.2f} e={current['east']:+.2f}")
            await asyncio.sleep(dt)

        # ========== 阶段 4：停 + 悬停 ==========
        PHASE[0] = "stop"
        print(f"⏹️  停止侧飞，悬停 {STOP_HOVER_S:.0f}s ...")
        cmd["right"] = 0.0
        t_st = time.monotonic()
        while not stop_flag and time.monotonic() - t_st < STOP_HOVER_S:
            cmd["forward"] = cmd["yawspeed"] = 0.0
            cmd["right"] = 0.0
            alt = -current["down"]
            alt_min_seen = min(alt_min_seen, alt)

            tick += 1
            if tick % 10 == 0:
                print(f"  悬停{time.monotonic()-t_st:5.1f}s alt={alt:4.2f}m "
                      f"vz={-current['vd']:+.3f} d={cmd['down']:+.3f}")
            await asyncio.sleep(dt)

    except Exception as e:
        print(f"\n❌ 飞行阶段异常: {type(e).__name__}: {e}")
        print("   → 立即收尾")
    finally:
        cmd["forward"] = cmd["right"] = cmd["down"] = cmd["yawspeed"] = 0.0
        await asyncio.sleep(0.3)

        print("🛬 降落")
        try:
            await drone.offboard.stop()
        except Exception as e:
            print(f"⚠️ offboard.stop 失败: {type(e).__name__}")
        stop_event.set()
        try:
            await send_task
        except Exception:
            pass
        alt_task.cancel()
        state_task.cancel()

        mode_now = current.get("mode", "")
        if "OFFBOARD" not in mode_now.upper():
            print(f"⚠️ 模式已是 {mode_now}（你接管了）→ 跳过自动降落")
        else:
            try:
                await drone.action.land()
                print("✅ 已发送降落")
            except Exception as e:
                print(f"⚠️ land 失败: {type(e).__name__}")

        print(f"\n📊 全程最低高度 {alt_min_seen:.2f}m"
              f"（目标 {ALT_TARGET_M:.2f}m，低了 "
              f"{(ALT_TARGET_M-alt_min_seen)*100:+.0f}cm）")
        print("✅ 完成")


if __name__ == "__main__":
    try:
        asyncio.run(fly())
    except Exception as e:
        print(f"\n❌ 顶层异常: {type(e).__name__}: {e}")
        print("⚠️ 若飞机还在空中，立刻用遥控器切到 Position/Stabilized 接管！")