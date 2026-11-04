#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
侧飞 + 雷达纠偏测试（v2）

流程：起飞 → 定高 → 等横向到位 → 侧飞 → 停 → 悬停 → 降落

WALL_MODE 三档：
    "off"    —— 不开雷达环，只验证 right 通道方向
    "single" —— 控制【front → FRONT_TARGET_M】(到前墙距离)  ⭐ 你的任务用这个
    "dual"   —— 控制【e_lat → E_LAT_TARGET_M】(偏离中线量)

⚠️ 注意：single 和 dual 控制的量不同！
    single: e = front - FRONT_TARGET_M      （只要前墙，摄像头要的就是这个）
    dual:   e = e_lat - E_LAT_TARGET_M      （要求前后墙都有效）

v2 改动：
    · 目标改成 1.60m（摄像头标定）
    · KP_LAT 0.30 → 0.45，MAX_FORWARD 0.15 → 0.22（提高响应）
    · 侧飞前先等 |e| < READY_ELAT_M（不再带着 -31cm 起跑）
    · 爬升段打印【命令速率 vs 实际速率】，暴露推力不足
    · 已移除积分分离（带外冻结会撞墙）
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

WALL_MODE = "single"        # ⭐ "off" / "single" / "dual"

# ---- 目标 ----
FRONT_TARGET_M = 1.60       # ⭐ single 模式：到 front 墙的目标距离（你实测标定值）
E_LAT_TARGET_M = 0.37       # ⭐ dual   模式：偏离中线量 = (1.60-0.86)/2
BACK_TARGET_M  = 0.86       # 仅用于打印（走廊 2.49m 时 back 应约等于这个）

# ---- 侧飞 ----
V_SIDE      = 0.50          # m/s
SIDE_TIME_S = 12.0           # 6s × 0.50 = 3.00m
SIDE_DIR    = +1.0          # +1 向右，-1 向左

# ---- 高度 ----
HOVER_ALT_DOWN    = -1.35   # 命令 1.60m → 停 1.50m → 侧飞沉到 ~1.30m
CLIMB_RATE        = -0.35
TAKEOFF_TIMEOUT_S = 30.0    # ⚠️ 实际爬升只有 0.11m/s，给足时间
STABILIZE_S       = 3.0
CLIMB_LAT_ON_ALT  = 0.60    # 爬到 0.6m 起开横向环

# ---- 到位门（侧飞前）----
READY_ELAT_M   = 0.12       # |e| < 12cm 才开始侧飞
READY_TIMEOUT_S = 10.0      # 最多等 10s

STOP_HOVER_S = 4.0

# ---- PID ----
KP_LAT, KI_LAT, KD_LAT = 0.45, 0.04, 0.0
MAX_FORWARD     = 0.22      # ⚠️ 和 KP 一起加，否则一加 KP 就顶上限
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
current = {"north": 0.0, "east": 0.0, "down": 0.0, "yaw": 0.0, "mode": "UNKNOWN"}

stop_flag = False
CTRL_ON = (WALL_MODE != "off")


def _target_text():
    if WALL_MODE == "single":
        return f"front → {FRONT_TARGET_M:.2f}m"
    if WALL_MODE == "dual":
        return f"e_lat → {E_LAT_TARGET_M:+.2f}m（≈front {FRONT_TARGET_M:.2f}m）"
    return "无雷达环"


# ============================================================
#  PID（条件积分抗饱和；不使用积分分离）
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

    def target(self):
        return FRONT_TARGET_M if WALL_MODE == "single" else E_LAT_TARGET_M

    def update(self, s, alt, enable, dt):
        """返回 (fwd, yawrate, f_ok, lost)"""
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

        # ---- 横向 ----
        ok = False
        if enable and f_ok is not None:
            if math.isnan(self.f_min):
                self.f_min = self.f_max = f_ok
            self.f_min = min(self.f_min, f_ok)
            self.f_max = max(self.f_max, f_ok)
            self.f_last = f_ok

            if WALL_MODE == "dual":
                if s.ok:
                    self.e = s.e_lat - E_LAT_TARGET_M
                    ok = (abs(self.e) < WALL_ABORT_M
                          and alt > LIDAR_MIN_ALT_M)
                else:
                    self.e = float("nan")
                    ok = False
            else:   # single：直接控制到前墙距离
                self.e = f_ok - FRONT_TARGET_M
                ok = (abs(self.e) < WALL_ABORT_M
                      and f_ok > FRONT_MIN_SAFE
                      and alt > LIDAR_MIN_ALT_M)

        if ok:
            fwd = self.pid_lat.update(self.e, dt)
        else:
            self.pid_lat.reset()
            fwd = 0.0

        # ---- 航向（度 → deg/s）----
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


async def track_state(drone):
    async def pos():
        async for pv in drone.telemetry.position_velocity_ned():
            current["north"] = pv.position.north_m
            current["east"]  = pv.position.east_m
            current["down"]  = pv.position.down_m
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
            w = s.front + s.back if math.isfinite(s.back) else float("nan")
            print(f"✅ 雷达就绪: front={s.front:.3f}m  back={s.back:.3f}m  "
                  f"e_lat={s.e_lat*100:+.1f}cm  走廊宽={w:.3f}m")
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
        print("❌ 15 秒没连上：查 ADDRESS / 端口 / 是否关了 QGC")
        return

    async for h in drone.telemetry.health():
        if h.is_local_position_ok:
            print("✅ 位置 OK")
            break

    lidar = open_lidar() if CTRL_ON else None

    print(f"⚙️  WALL_MODE={WALL_MODE}  目标 {_target_text()}")
    print(f"⚙️  悬停 {-HOVER_ALT_DOWN:.2f}m→停 {-HOVER_ALT_DOWN-0.10:.2f}m  "
          f"侧飞 {V_SIDE:.2f}m/s × {SIDE_TIME_S:.0f}s "
          f"({V_SIDE*SIDE_TIME_S:.2f}m) 方向 {'右' if SIDE_DIR > 0 else '左'}")
    if CTRL_ON:
        print(f"⚙️  PID KP={KP_LAT} KI={KI_LAT} MAX_FWD={MAX_FORWARD} "
              f"| {CLIMB_LAT_ON_ALT:.1f}m 起开环 | 到位门 {READY_ELAT_M*100:.0f}cm")
        print(f"⚙️  航向 KP_YAW={KP_YAW} MAX_YAWRATE={MAX_YAWRATE}°/s "
              f"死区 {YAW_DEADBAND_DEG}°")
    print(f"⚠️  确认侧飞方向 {V_SIDE*SIDE_TIME_S + 1.0:.1f}m 内无障碍，"
          f"墙长 > {V_SIDE*SIDE_TIME_S + 0.5:.1f}m")

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
    ctrl = WallController()
    dt = 0.05
    tick = 0
    side_abort = ""

    try:
        # ========== 阶段 1：爬升 ==========
        print("🛫 起飞中（爬升段同时打印实际速率）...")
        t0 = time.monotonic()
        t_prev = t0
        alt_prev = -current["down"]
        while not stop_flag:
            alt = -current["down"]
            s = lidar.read(dt) if lidar else None
            if s is not None and CTRL_ON and alt > CLIMB_LAT_ON_ALT:
                fwd, yr, f_ok, _ = ctrl.update(s, alt, True, dt)
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
                flag = "  ⚠️推力不足" if (CLIMB_RATE < 0 and v_act < abs(CLIMB_RATE)*0.6) else ""
                print(f"  高度={alt:+5.2f}m  命令 {CLIMB_RATE:+.2f} 实际 {v_act:+.3f} m/s"
                      f"  fwd={cmd['forward']:+.3f}{flag}")

            if current["down"] <= HOVER_ALT_DOWN + 0.10:
                break
            if time.monotonic() - t0 > TAKEOFF_TIMEOUT_S:
                print("⚠️ 起飞超时")
                break
            await asyncio.sleep(dt)

        cmd["down"] = 0.0
        print(f"   爬升总耗时 {time.monotonic()-t0:.1f}s，当前高度 {alt:+.2f}m")

        # ========== 阶段 2：稳住 ==========
        print(f"⏸️  悬停稳住 {STABILIZE_S:.0f}s ...")
        t_s = time.monotonic()
        while not stop_flag and time.monotonic() - t_s < STABILIZE_S:
            if lidar and CTRL_ON:
                alt = -current["down"]
                s = lidar.read(dt)
                fwd, yr, _, _ = ctrl.update(s, alt, True, dt)
                cmd["forward"], cmd["yawspeed"] = fwd, yr
            await asyncio.sleep(dt)

        # ========== 阶段 2b：等横向到位（不再带着 -31cm 起跑）==========
        if lidar and CTRL_ON:
            print(f"⏳ 等横向到位：|e| < {READY_ELAT_M*100:.0f}cm（最多 {READY_TIMEOUT_S:.0f}s）...")
            t_r = time.monotonic()
            ready = False
            while not stop_flag and time.monotonic() - t_r < READY_TIMEOUT_S:
                alt = -current["down"]
                s = lidar.read(dt)
                fwd, yr, _, _ = ctrl.update(s, alt, True, dt)
                cmd["forward"], cmd["yawspeed"] = fwd, yr
                if math.isfinite(ctrl.e) and abs(ctrl.e) < READY_ELAT_M:
                    ready = True
                    break
                await asyncio.sleep(dt)
            print(f"   {'✅ 到位' if ready else '⚠️ 超时未到位'}: "
                  f"e={ctrl.e*100:+.1f}cm  front={ctrl.f_last:.3f}m  "
                  f"用时 {time.monotonic()-t_r:.1f}s")

        # ========== 阶段 3：侧飞 ==========
        print(f"➡️  开始侧飞 {V_SIDE:.2f}m/s × {SIDE_TIME_S:.0f}s ...")
        t_sd = time.monotonic()
        while not stop_flag and time.monotonic() - t_sd < SIDE_TIME_S:
            alt = -current["down"]
            el = time.monotonic() - t_sd

            if lidar and CTRL_ON:
                s = lidar.read(dt)
                fwd, yr, f_ok, lost = ctrl.update(s, alt, True, dt)
                cmd["forward"], cmd["yawspeed"] = fwd, yr

                if lost:
                    side_abort = "雷达丢墙"
                elif f_ok is not None and f_ok < SIDE_STOP_FRONT:
                    side_abort = f"离墙过近 {f_ok:.2f}m"

                cmd["right"] = 0.0 if side_abort else V_SIDE * SIDE_DIR
            else:
                cmd["forward"] = cmd["yawspeed"] = 0.0
                cmd["right"] = V_SIDE * SIDE_DIR

            cmd["down"] = 0.0

            tick += 1
            if tick % 10 == 0:
                fr = f"{ctrl.f_last:5.3f}m" if lidar and CTRL_ON else "  ---  "
                ey = (math.degrees(s.yaw_err)
                      if (lidar and CTRL_ON and math.isfinite(s.yaw_err))
                      else float("nan"))
                print(f"  侧飞 {el:5.1f}s (走 {V_SIDE*el:4.2f}m) | "
                      f"front={fr} e={ctrl.e*100:+6.1f}cm "
                      f"fwd={cmd['forward']:+.3f} right={cmd['right']:+.2f} "
                      f"yaw_err={ey:+6.1f}° yawrate={cmd['yawspeed']:+6.2f} "
                      f"alt={alt:4.2f}m n={current['north']:+.2f} "
                      f"e={current['east']:+.2f} yaw={current['yaw']:+6.1f}°"
                      + (f"  [{side_abort}]" if side_abort else ""))
            await asyncio.sleep(dt)

        if side_abort:
            print(f"🛑 侧飞提前停止：{side_abort}")

        # ========== 阶段 4：停 + 稳住 ==========
        print(f"⏹️  停止侧飞，悬停 {STOP_HOVER_S:.0f}s ...")
        t_st = time.monotonic()
        while not stop_flag and time.monotonic() - t_st < STOP_HOVER_S:
            if lidar and CTRL_ON:
                alt = -current["down"]
                s = lidar.read(dt)
                fwd, yr, _, _ = ctrl.update(s, alt, True, dt)
                cmd["forward"], cmd["yawspeed"] = fwd, yr
            cmd["right"] = 0.0
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

        if lidar and isinstance(lidar, RealLidar):
            lidar.stop()

        if math.isfinite(ctrl.f_last):
            tgt = ctrl.target()
            print("\n📊 本次数据：")
            if WALL_MODE == "single":
                print(f"   front 结束 {ctrl.f_last:.3f}m（目标 {FRONT_TARGET_M:.2f}m）"
                      f"  误差 {(ctrl.f_last - FRONT_TARGET_M)*100:+.1f}cm")
            else:
                print(f"   e_lat 目标 {E_LAT_TARGET_M:+.2f}m，front 结束 {ctrl.f_last:.3f}m")
            print(f"   front 范围 {ctrl.f_min:.3f} ~ {ctrl.f_max:.3f}m")
            print(f"   跳变被拒 {ctrl.rej} 次")
        print("✅ 完成")


if __name__ == "__main__":
    try:
        asyncio.run(fly())
    except Exception as e:
        print(f"\n❌ 顶层异常: {type(e).__name__}: {e}")
        print("⚠️ 若飞机还在空中，立刻用遥控器切到 Position/Stabilized 接管！")