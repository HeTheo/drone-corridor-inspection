#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
侧飞 + 雷达纠偏测试（v3 · 抗局部结构干扰）

核心新增：走廊宽度一致性判定
    W = front + back
    · |W - W_NOMINAL| <= W_TOL          → 正常，用 front 闭环
    · 超差 且 back 稳定                 → front 被干扰，用 W_NOMINAL - back 反推
    · 超差 且 back 也变了               → 两面都不可信，冻结横向输出（直飞通过）

v3 其他改动：
    · 每行打印 back / W / 状态 / vz（判断"真掉"还是"估计漂"）
    · 爬升诊断只在 alt>0.5m 后生效（消除起飞阶段误报）
    · 到位门收紧到 8cm
    · 保持 v2 的 PID（条件积分抗饱和，不用积分分离）
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

WALL_MODE = "off"        # "off" / "single"(控 front) / "dual"(控 e_lat)

# ---- 走廊与目标 ----
W_NOMINAL_M    = 2.49       # ⭐ 走廊宽度标定值（1.63 + 0.86）
W_TOL_M        = 0.15       # ⭐ W 偏差超过这个 → 判为局部结构干扰
BACK_STABLE_M  = 0.06       # ⭐ back 变化小于这个 → 认为 back 稳定可信
USE_BACK_ESTIMATE = True    # ⭐ W 超差时用 back 反推 front

FRONT_TARGET_M = 1.60       # single 模式目标
E_LAT_TARGET_M = 0.37       # dual   模式目标 = (1.60-0.86)/2

# ---- 侧飞 ----
V_SIDE      = 0.50
SIDE_TIME_S = 0.0
SIDE_DIR    = +1.0

# ---- 高度 ----
HOVER_ALT_DOWN    = -1.45
CLIMB_RATE        = -0.35
TAKEOFF_TIMEOUT_S = 30.0
STABILIZE_S       = 3.0
CLIMB_LAT_ON_ALT  = 0.60

# ---- 到位门 ----
READY_ELAT_M    = 0.08      # ⭐ 从 12cm 收紧到 8cm
READY_TIMEOUT_S = 12.0

STOP_HOVER_S = 25.0

# ---- PID ----
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


def _target_text():
    if WALL_MODE == "single":
        return f"front → {FRONT_TARGET_M:.2f}m"
    if WALL_MODE == "dual":
        return f"e_lat → {E_LAT_TARGET_M:+.2f}m"
    return "无雷达环"


# ============================================================
#  PID
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
        # 宽度一致性相关
        self.good_front = None
        self.good_back  = None
        self.w_state = "unknown"
        self.w_dev = float("nan")
        self.n_bad = 0
        self.n_est = 0
        self.front_used = float("nan")

    def target(self):
        return FRONT_TARGET_M if WALL_MODE == "single" else E_LAT_TARGET_M

    # ---------- ⭐ 核心：宽度一致性判定 ----------
    def _sanitize(self, f, b):
        """返回 (可用的 front, 状态, W偏差)
        状态: ok / est / bad / nogap / nofront
        """
        if not math.isfinite(f):
            return float("nan"), "nofront", float("nan")

        # 没有 back（单墙场景）→ 无法校验，只能信 front
        if not math.isfinite(b) or W_NOMINAL_M <= 0:
            self.good_front = f
            return f, "nogap", float("nan")

        w = f + b
        dev = w - W_NOMINAL_M

        if abs(dev) <= W_TOL_M:
            self.good_front, self.good_back = f, b
            return f, "ok", dev

        # ---- W 超差：局部结构干扰 ----
        self.n_bad += 1
        if USE_BACK_ESTIMATE and self.good_back is not None:
            df = f - self.good_front
            db = b - self.good_back
            # front 变化远大于 back → front 是假的
            if abs(df) > abs(db) + BACK_STABLE_M:
                return W_NOMINAL_M - b, "est", dev

        # 两面都不可信 → 冻结
        return float("nan"), "bad", dev

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

        f_use, self.w_state, self.w_dev = self._sanitize(f_ok, s.back)

        # ---- 横向 ----
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
            # 两面都不可信：冻结横向（保持 0，直飞通过）
            self.pid_lat.reset()
            ok = False

        if ok:
            fwd = self.pid_lat.update(self.e, dt)
        else:
            self.pid_lat.reset()
            fwd = 0.0

        # ---- 航向 ----
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
            if math.isfinite(w) and abs(dev) > W_TOL_M:
                print(f"⚠️ 起点 W 偏差超 {W_TOL_M*100:.0f}cm："
                      f"要么 W_NOMINAL_M 填错，要么起飞点附近有结构干扰")
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
    print(f"⚙️  走廊 W={W_NOMINAL_M:.2f}m ± {W_TOL_M*100:.0f}cm "
          f"| back 稳定阈 {BACK_STABLE_M*100:.0f}cm | 反推={USE_BACK_ESTIMATE}")
    print(f"⚙️  悬停 {-HOVER_ALT_DOWN:.2f}m→停 {-HOVER_ALT_DOWN-0.10:.2f}m  "
          f"侧飞 {V_SIDE:.2f}m/s × {SIDE_TIME_S:.0f}s "
          f"({V_SIDE*SIDE_TIME_S:.2f}m) 方向 {'右' if SIDE_DIR > 0 else '左'}")
    if CTRL_ON:
        print(f"⚙️  PID KP={KP_LAT} KI={KI_LAT} MAX_FWD={MAX_FORWARD} "
              f"| 开环高度 {CLIMB_LAT_ON_ALT:.1f}m | 到位门 {READY_ELAT_M*100:.0f}cm")
        print(f"⚙️  航向 KP_YAW={KP_YAW} MAX_YAWRATE={MAX_YAWRATE}°/s")
    print(f"⚠️  确认侧飞方向 {V_SIDE*SIDE_TIME_S + 1.0:.1f}m 内无障碍")

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

    def _w_txt():
        if ctrl.w_state == "ok":
            return "ok "
        if ctrl.w_state == "est":
            return "EST"
        if ctrl.w_state == "bad":
            return "BAD"
        if ctrl.w_state == "nogap":
            return "n/b"
        return "---"

    try:
        # ========== 阶段 1：爬升 ==========
        print("🛫 起飞中...")
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
                # ⭐ 只在离地后判断（起飞前在地上，速度当然是 0）
                flag = ""
                if alt > 0.50 and CLIMB_RATE < 0 and v_act < abs(CLIMB_RATE) * 0.6:
                    flag = "  ⚠️爬升速率不足"
                print(f"  高度={alt:+5.2f}m 命令{CLIMB_RATE:+.2f} 实际{v_act:+.3f} "
                      f"fwd={cmd['forward']:+.3f}{flag}")

            if current["down"] <= HOVER_ALT_DOWN + 0.10:
                break
            if time.monotonic() - t0 > TAKEOFF_TIMEOUT_S:
                print("⚠️ 起飞超时")
                break
            await asyncio.sleep(dt)

        cmd["down"] = 0.0
        print(f"   爬升耗时 {time.monotonic()-t0:.1f}s，高度 {alt:+.2f}m")

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
                    ready = True
                    break
                await asyncio.sleep(dt)
            print(f"   {'✅ 到位' if ready else '⚠️ 超时'}: e={ctrl.e*100:+.1f}cm "
                  f"front_use={ctrl.front_used:.3f}m 用时 {time.monotonic()-t_r:.1f}s")

        # ========== 阶段 3：侧飞 ==========
        print(f"➡️  开始侧飞 {V_SIDE:.2f}m/s × {SIDE_TIME_S:.0f}s ...")
        print("   （W 列: ok=正常  EST=front被干扰/用back反推  "
              "BAD=两面都干扰/冻结横向  n/b=无back）")
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
                elif math.isfinite(s.front) and s.front < SIDE_STOP_FRONT:
                    side_abort = f"离墙过近 {s.front:.2f}m"

                cmd["right"] = 0.0 if side_abort else V_SIDE * SIDE_DIR
            else:
                cmd["forward"] = cmd["yawspeed"] = 0.0
                cmd["right"] = V_SIDE * SIDE_DIR

            cmd["down"] = 0.0

            tick += 1
            if tick % 10 == 0:
                w = (s.front + s.back) if math.isfinite(s.back) else float("nan")
                ey = (math.degrees(s.yaw_err)
                      if math.isfinite(s.yaw_err) else float("nan"))
                print(f"  侧飞{el:5.1f}s(走{V_SIDE*el:4.2f}m) "
                      f"f={s.front:5.3f} b={s.back:5.3f} W={w:5.3f}[{_w_txt()}] "
                      f"fu={ctrl.front_used:5.3f} e={ctrl.e*100:+6.1f}cm "
                      f"fwd={cmd['forward']:+.3f} r={cmd['right']:+.2f} "
                      f"ye={ey:+5.1f}° yr={cmd['yawspeed']:+6.2f} "
                      f"alt={alt:4.2f}m vz={-current['vd']:+.3f} "
                      f"yaw={current['yaw']:+6.1f}°"
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
            print("\n📊 本次数据：")
            if WALL_MODE == "single":
                print(f"   front(可信) 结束 {ctrl.f_last:.3f}m "
                      f"（目标 {FRONT_TARGET_M:.2f}m） 误差 "
                      f"{(ctrl.f_last - FRONT_TARGET_M)*100:+.1f}cm")
            else:
                print(f"   front(可信) 结束 {ctrl.f_last:.3f}m，"
                      f"e_lat 目标 {E_LAT_TARGET_M:+.2f}m")
            print(f"   front 范围 {ctrl.f_min:.3f} ~ {ctrl.f_max:.3f}m")
            print(f"   跳变被拒 {ctrl.rej} 次")
            print(f"   宽度异常帧 {ctrl.n_bad} 次（其中 back 反推 {ctrl.n_est} 次）")
        print("✅ 完成")


if __name__ == "__main__":
    try:
        asyncio.run(fly())
    except Exception as e:
        print(f"\n❌ 顶层异常: {type(e).__name__}: {e}")
        print("⚠️ 若飞机还在空中，立刻用遥控器切到 Position/Stabilized 接管！")