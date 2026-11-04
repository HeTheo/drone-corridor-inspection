#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
侧飞 + 雷达横向环 + 高度外环（v3 + v4 纯合并）

阶段：
    climb -> 爬升（高度环不接管，雷达不开）
    hold  -> 稳住 + 等横向到位（两环同时工作）
    side  -> 侧飞（两环同时工作）
    stop  -> 停 + 悬停

分工（互不干扰）：
    雷达环  -> cmd["forward"] / cmd["yawspeed"]   （主循环 20Hz）
    高度环  -> cmd["down"]                        （独立 20Hz 任务，非 climb 阶段才接管）
    cmd["right"] 只由 side 阶段写
"""
import asyncio
import math
import threading
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

from Lidar_source import RealLidar, make_lidar

# ==================== 配置 ====================
ADDRESS     = "serial:///dev/pixhawk:57600"
LIDAR_PORT  = "/dev/wheeltec_lidar"
LIDAR_MODEL = "N10"

WALL_MODE = "dual"          # "off" / "single" / "dual"

# ---- 走廊与雷达目标（v3）----
W_NOMINAL_M       = 2.49
W_TOL_M           = 0.15
BACK_STABLE_M     = 0.06
USE_BACK_ESTIMATE = True
FRONT_TARGET_M    = 1.60
E_LAT_TARGET_M    = 0.37

# ---- 侧飞 ----
V_SIDE      = 0.30          # 已验证值，熟练后可到 0.30
SIDE_TIME_S = 20.0
SIDE_DIR    = +1.0

# ---- 高度（v4）----
HOVER_ALT_DOWN    = -1.45
CLIMB_RATE        = -0.35
TAKEOFF_TIMEOUT_S = 30.0
STABILIZE_S       = 3.0
CLIMB_LAT_ON_ALT  = 0.60

ALT_HOLD_ENABLE = True
ALT_TARGET_M    = 1.30
KP_ALT, KI_ALT, KD_ALT = 0.50, 0.10, 0.00
I_LIMIT_ALT     = 0.30
MAX_DOWN_CMD    = 0.20
ALT_MIN_M       = 0.80

# ---- 到位门（v3）----
READY_ELAT_M    = 0.08
READY_TIMEOUT_S = 12.0
STOP_HOVER_S    = 8.0

# ---- 雷达横向 PID（v3）----
KP_LAT, KI_LAT, KD_LAT = 0.35, 0.04, 0.0
MAX_FORWARD     = 0.22
WALL_ABORT_M    = 0.80
LIDAR_MIN_ALT_M = 0.50
FRONT_MIN_SAFE  = 0.40
SIDE_STOP_FRONT = 0.60

USE_YAW_LOOP     = True
KP_YAW, KI_YAW, KD_YAW = 3.0, 0.0, 0.0
MAX_YAWRATE      = 10.0
YAW_DEADBAND_DEG = 1.5
YAW_ABORT_DEG    = 25.0

MAX_JUMP_M     = 0.25
MAX_BAD_CNT    = 8
LOST_TIMEOUT_S = 1.0

# ==================== 共享状态 ====================
cmd     = {"forward": 0.0, "right": 0.0, "down": 0.0, "yawspeed": 0.0}
current = {"north": 0.0, "east": 0.0, "down": 0.0,
           "vn": 0.0, "ve": 0.0, "vd": 0.0,
           "yaw": 0.0, "mode": "UNKNOWN"}

stop_flag = False
CTRL_ON = (WALL_MODE != "off")
PHASE = ["init"]        # init / climb / hold / side / stop


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


class FrontGuard:
    """v3 原版：任何方向单次跳变 > MAX_JUMP_M 都先拒"""
    def __init__(self, max_jump=MAX_JUMP_M, max_bad=MAX_BAD_CNT):
        self.prev, self.raw, self.bad = None, None, 0
        self.max_jump, self.max_bad = max_jump, max_bad

    def update(self, f):
        if not math.isfinite(f):
            return None, False
        if f == self.raw:
            return self.prev, False
        self.raw = f
        if self.prev is None:
            self.prev = f
            return f, False
        if abs(f - self.prev) > self.max_jump:
            self.bad += 1
            if self.bad < self.max_bad:
                return self.prev, True
        self.prev, self.bad = f, 0
        return f, False


class WallController:
    def __init__(self):
        self.pid_lat = PID(KP_LAT, KI_LAT, KD_LAT, 1.0, MAX_FORWARD)
        self.pid_yaw = PID(KP_YAW, KI_YAW, KD_YAW, 10.0, MAX_YAWRATE)
        self.guard   = FrontGuard()
        self.f_min = self.f_max = self.f_last = float("nan")
        self.e = float("nan")
        self.rej = 0
        self.lost_since = None
        self.good_front = None
        self.good_back  = None
        self.w_state = "unknown"
        self.w_dev = float("nan")
        self.n_bad = 0
        self.n_est = 0
        self.front_used = float("nan")

    def _sanitize(self, f, b):
        if not math.isfinite(f):
            return float("nan"), "nofront", float("nan")
        if not math.isfinite(b) or W_NOMINAL_M <= 0:
            self.good_front = f
            return f, "nogap", float("nan")

        w = f + b
        dev = w - W_NOMINAL_M
        if abs(dev) <= W_TOL_M:
            self.good_front, self.good_back = f, b
            return f, "ok", dev

        self.n_bad += 1
        if USE_BACK_ESTIMATE and self.good_back is not None:
            df = f - self.good_front
            db = b - self.good_back
            if abs(df) > abs(db) + BACK_STABLE_M:
                self.n_est += 1
                return W_NOMINAL_M - b, "est", dev
        return float("nan"), "bad", dev

    def update(self, s, alt, enable, dt):
        f_ok, rejected = self.guard.update(s.front)
        if rejected:
            self.rej += 1

        if f_ok is None:
            if self.lost_since is None:
                self.lost_since = time.monotonic()
            lost = (time.monotonic() - self.lost_since) > LOST_TIMEOUT_S
        else:
            self.lost_since = None
            lost = False

        f_use, self.w_state, self.w_dev = self._sanitize(f_ok, s.back)

        ok = False
        self.e = float("nan")
        if enable and math.isfinite(f_use):
            self.front_used = f_use
            if math.isnan(self.f_min):
                self.f_min = self.f_max = f_use
            self.f_min = min(self.f_min, f_use)
            self.f_max = max(self.f_max, f_use)
            self.f_last = f_use

            if WALL_MODE == "dual" and math.isfinite(s.back):
                self.e = (f_use - s.back) / 2.0 - E_LAT_TARGET_M
                ok = (abs(self.e) < WALL_ABORT_M
                      and alt > LIDAR_MIN_ALT_M)
            else:
                self.e = f_use - FRONT_TARGET_M
                ok = (abs(self.e) < WALL_ABORT_M
                      and f_use > FRONT_MIN_SAFE
                      and alt > LIDAR_MIN_ALT_M)
        elif self.w_state == "bad":
            self.pid_lat.reset()
            ok = False

        fwd = self.pid_lat.update(self.e, dt) if ok else 0.0
        if not ok:
            self.pid_lat.reset()

        if (enable and USE_YAW_LOOP and math.isfinite(s.yaw_err)
                and alt > LIDAR_MIN_ALT_M):
            e_yaw_deg = math.degrees(s.yaw_err)
            if abs(e_yaw_deg) < YAW_DEADBAND_DEG:
                e_yaw_deg = 0.0
            if abs(e_yaw_deg) < YAW_ABORT_DEG:
                yawrate = self.pid_yaw.update(e_yaw_deg, dt)
            else:
                self.pid_yaw.reset()
                yawrate = 0.0
        else:
            self.pid_yaw.reset()
            yawrate = 0.0

        return fwd, yawrate, f_ok, lost


# ============================================================
#  后台任务
# ============================================================
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
    """高度外环 20Hz —— 只写 cmd["down"]，爬升阶段不接管"""
    dt = 0.05
    pid = PID(KP_ALT, KI_ALT, KD_ALT, I_LIMIT_ALT, MAX_DOWN_CMD)
    while not stop_event.is_set():
        if (not ALT_HOLD_ENABLE) or PHASE[0] == "climb":
            pid.reset()
        else:
            alt = -current["down"]
            if alt < ALT_MIN_M:
                pid.reset()
                cmd["down"] = -MAX_DOWN_CMD
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


def open_lidar():
    print(f"⏳ 打开雷达 {LIDAR_PORT} ({LIDAR_MODEL}) ...")
    lidar = make_lidar(False, port=LIDAR_PORT, model=LIDAR_MODEL)
    t = time.monotonic()
    while True:
        s = lidar.read(0.05)
        ready = s.ok if WALL_MODE == "dual" else math.isfinite(s.front)
        if ready:
            w = (s.front + s.back) if math.isfinite(s.back) else float("nan")
            dev = w - W_NOMINAL_M if math.isfinite(w) else float("nan")
            print(f"✅ 雷达就绪: front={s.front:.3f}m back={s.back:.3f}m "
                  f"W={w:.3f}m (偏差 {dev*100:+.1f}cm)")
            return lidar
        if time.monotonic() - t > 10:
            print("❌ 10 秒没拿到雷达数据，退出")
            if isinstance(lidar, RealLidar):
                lidar.stop()
            raise SystemExit(1)
        time.sleep(0.1)


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
        print("❌ 15 秒没连上"); return

    async for h in drone.telemetry.health():
        if h.is_local_position_ok:
            print("✅ 位置 OK"); break

    lidar = open_lidar() if CTRL_ON else None

    print(f"⚙️  WALL_MODE={WALL_MODE}  走廊 W={W_NOMINAL_M:.2f}m "
          f"| 横向目标 e_lat={E_LAT_TARGET_M:+.2f} 或 front={FRONT_TARGET_M:.2f}m")
    print(f"⚙️  高度环 目标 {ALT_TARGET_M:.2f}m  KP={KP_ALT} KI={KI_ALT}  "
          f"限幅={MAX_DOWN_CMD}  下限={ALT_MIN_M:.2f}")
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
        if lidar and isinstance(lidar, RealLidar): lidar.stop()
        return

    print("🚁 进入 Offboard")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"❌ Offboard 启动失败: {e._result.result}")
        stop_event.set(); await send_task; await drone.action.disarm()
        if lidar and isinstance(lidar, RealLidar): lidar.stop()
        return

    state_task = asyncio.ensure_future(track_state(drone))
    alt_task   = asyncio.ensure_future(alt_hold_task(stop_event))
    ctrl = WallController()
    dt = 0.05
    tick = 0
    side_abort = ""
    alt_min_seen = 99.0

    try:
        # ========== 阶段 1：爬升（高度环不接管，雷达 >=0.6m 才开）==========
        PHASE[0] = "climb"
        print("🛫 起飞中...")
        t0 = time.monotonic()
        t_prev = t0
        alt_prev = -current["down"]
        while not stop_flag:
            alt = -current["down"]
            s = lidar.read(dt) if lidar else None

            if s is not None and CTRL_ON and alt > CLIMB_LAT_ON_ALT:
                fwd, yr, _, _ = ctrl.update(s, alt, True, dt)
                cmd["forward"], cmd["yawspeed"] = fwd, yr
            else:
                ctrl.pid_lat.reset(); ctrl.pid_yaw.reset()
                cmd["forward"] = cmd["yawspeed"] = 0.0
            cmd["right"] = 0.0
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
                      f"实际{v_act:+.3f} fwd={cmd['forward']:+.3f}{flag}")

            if current["down"] <= HOVER_ALT_DOWN + 0.10:
                break
            if time.monotonic() - t0 > TAKEOFF_TIMEOUT_S:
                print("⚠️ 起飞超时"); break
            await asyncio.sleep(dt)

        print(f"   爬升耗时 {time.monotonic()-t0:.1f}s，高度 {alt:+.2f}m")

        # ========== 阶段 2：稳住（高度环接管）==========
        PHASE[0] = "hold"
        print(f"⏸️  悬停稳住 {STABILIZE_S:.0f}s（高度环已接管）...")
        t_s = time.monotonic()
        while not stop_flag and time.monotonic() - t_s < STABILIZE_S:
            if lidar and CTRL_ON:
                alt = -current["down"]
                s = lidar.read(dt)
                fwd, yr, _, _ = ctrl.update(s, alt, True, dt)
                cmd["forward"], cmd["yawspeed"] = fwd, yr
            alt_min_seen = min(alt_min_seen, -current["down"])
            await asyncio.sleep(dt)

        # ========== 阶段 2b：等横向到位 ==========
        if lidar and CTRL_ON:
            print(f"⏳ 等横向到位 |e| < {READY_ELAT_M*100:.0f}cm ...")
            t_r = time.monotonic()
            ready = False
            while not stop_flag and time.monotonic() - t_r < READY_TIMEOUT_S:
                alt = -current["down"]
                s = lidar.read(dt)
                fwd, yr, _, _ = ctrl.update(s, alt, True, dt)
                cmd["forward"], cmd["yawspeed"] = fwd, yr
                if math.isfinite(ctrl.e) and abs(ctrl.e) < READY_ELAT_M:
                    ready = True; break
                await asyncio.sleep(dt)
            print(f"   {'✅ 到位' if ready else '⚠️ 超时'}: "
                  f"e={ctrl.e*100:+.1f}cm front_use={ctrl.front_used:.3f}m "
                  f"用时 {time.monotonic()-t_r:.1f}s")

        # ========== 阶段 3：侧飞（两环同时工作）==========
        PHASE[0] = "side"
        print(f"➡️  开始侧飞 {V_SIDE:.2f}m/s × {SIDE_TIME_S:.0f}s ...")
        t_sd = time.monotonic()
        while not stop_flag and time.monotonic() - t_sd < SIDE_TIME_S:
            alt = -current["down"]
            el = time.monotonic() - t_sd
            alt_min_seen = min(alt_min_seen, alt)

            if lidar and CTRL_ON:
                s = lidar.read(dt)
                fwd, yr, f_ok, lost = ctrl.update(s, alt, True, dt)
                cmd["forward"], cmd["yawspeed"] = fwd, yr

                if lost:
                    side_abort = "雷达丢墙"
                elif math.isfinite(s.front) and s.front < SIDE_STOP_FRONT:
                    side_abort = f"离墙过近 {s.front:.2f}m"

                cmd["right"] = 0.0 if side_abort else V_SIDE * SIDE_DIR
            else:
                cmd["forward"] = cmd["yawspeed"] = 0.0
                cmd["right"] = V_SIDE * SIDE_DIR

            # ⚠️ 不要写 cmd["down"]，交给 alt_hold_task

            tick += 1
            if tick % 10 == 0 and lidar and CTRL_ON:
                w = (s.front + s.back) if math.isfinite(s.back) else float("nan")
                ey = (math.degrees(s.yaw_err)
                      if math.isfinite(s.yaw_err) else float("nan"))
                print(f"  侧飞{el:5.1f}s(走{V_SIDE*el:4.2f}m) "
                      f"f={s.front:5.3f} b={s.back:5.3f} W={w:5.3f}[{ctrl.w_state}] "
                      f"fu={ctrl.front_used:5.3f} e={ctrl.e*100:+6.1f}cm "
                      f"fwd={cmd['forward']:+.3f} r={cmd['right']:+.2f} "
                      f"ye={ey:+5.1f}° yr={cmd['yawspeed']:+6.2f} "
                      f"alt={alt:4.2f}m vz={-current['vd']:+.3f} d={cmd['down']:+.3f} "
                      f"yaw={current['yaw']:+6.1f}°"
                      + (f"  [{side_abort}]" if side_abort else ""))
            await asyncio.sleep(dt)

        if side_abort:
            print(f"🛑 侧飞提前停止：{side_abort}")

        # ========== 阶段 4：停 + 悬停（两环继续工作）==========
        PHASE[0] = "stop"
        print(f"⏹️  停止侧飞，悬停 {STOP_HOVER_S:.0f}s ...")
        t_st = time.monotonic()
        while not stop_flag and time.monotonic() - t_st < STOP_HOVER_S:
            if lidar and CTRL_ON:
                alt = -current["down"]
                s = lidar.read(dt)
                fwd, yr, _, _ = ctrl.update(s, alt, True, dt)
                cmd["forward"], cmd["yawspeed"] = fwd, yr
            cmd["right"] = 0.0
            alt_min_seen = min(alt_min_seen, -current["down"])
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

        # ⭐ 关键：真正发送降落指令（原代码漏了）
        mode_now = current.get("mode", "")
        if "OFFBOARD" not in mode_now.upper():
            print(f"⚠️ 模式已是 {mode_now}（你接管了）→ 跳过自动降落")
        else:
            try:
                await drone.action.land()
                print("✅ 已发送降落")
            except Exception as e:
                print(f"⚠️ land 失败: {type(e).__name__}")

        if lidar and isinstance(lidar, RealLidar):
            lidar.stop()

        print("\n📊 本次数据：")
        if math.isfinite(ctrl.f_last):
            print(f"   front(可信) 结束 {ctrl.f_last:.3f}m  "
                  f"e_lat 目标 {E_LAT_TARGET_M:+.2f}m")
            print(f"   front 范围 {ctrl.f_min:.3f} ~ {ctrl.f_max:.3f}m")
            print(f"   跳变被拒 {ctrl.rej} 次 | 宽度异常 {ctrl.n_bad} 次"
                  f"（反推 {ctrl.n_est} 次）")
        if alt_min_seen < 90:
            print(f"   全程最低高度 {alt_min_seen:.2f}m"
                  f"（目标 {ALT_TARGET_M:.2f}m，低了 "
                  f"{(ALT_TARGET_M-alt_min_seen)*100:+.0f}cm）")
        print("✅ 完成")


if __name__ == "__main__":
    try:
        asyncio.run(fly())
    except Exception as e:
        print(f"\n❌ 顶层异常: {type(e).__name__}: {e}")
        print("⚠️ 若飞机还在空中，立刻用遥控器切到 Position/Stabilized 接管！")