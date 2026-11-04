#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
侧飞 + 雷达横向环 + 高度外环 + 视觉触发悬停（v5）

⭐ 本文件 = Fly_alt_safe.py 的完整副本 + 视觉部分。
   Fly_alt_safe.py 保持不动，两个脚本可以分别单独跑，方便对照。

相对 Fly_alt_safe.py 的改动（只有这些）：
1. 新增视觉子进程（multiprocessing），在连 mavsdk 之前 fork，避免子进程继承 gRPC socket。
2. 起飞前等视觉出第一帧，20s 超时则取消起飞。
3. side 阶段按"最近 6 帧里 3 帧看到 Fire_hydrant"触发悬停。
4. 新增 inspect 阶段：右侧速度斜坡降到 0 → 原地悬停 INSPECT_S=3.0s
   → 累积各帧检出（取并集）→ 判定柜内 4 类是否齐全 → 冷却后继续侧飞。
5. 悬停期间横向/航向全部锁死（方案 A：不闭环），只靠 PX4 速度保持；
   打印 dN/dE/front/back，用来判断"会不会往前飘"。
6. 侧飞行程计时扣除悬停时间（否则会少飞 3s×V_SIDE）。
7. dist 的更新集中在 guard_task 里（单写入者），不再每个阶段各写一遍，
   避免以后加阶段忘记更新导致 guard_task 拿着过期距离。
8. finally 里一并 cancel guard_task，并通知视觉子进程退出。

视觉判定口径：
- 触发类：Fire_hydrant
- 必检类：Valve / Fire_hose / Fire_nozzle / Rubber_hose
- 忽略类：Fire_extinguisher（先不判，也不显示）

注意：
- 没有改 WallController 的控制符号和阈值。
- 没有改 Lidar_source 的接口和标定常量。
- 摄像头 /dev/video0 是独占的，跑本脚本时不要同时开 Yolo_test.py。
- 真机前必须台架拆桨 + 系留验证。
"""

import asyncio
import math
import multiprocessing as mp
import os
import queue
import sys
import threading
import time
from collections import deque

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

from Lidar_source import RealLidar, make_lidar

# ---- 视觉子进程：Pi 上目录是 Vision/，工作区里是 vision/，两个都试 ----
_HERE = os.path.dirname(os.path.abspath(__file__))
for _cand in ("Vision", "vision"):
    _p = os.path.normpath(os.path.join(_HERE, "..", _cand))
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from Vision_worker import start_vision_process
except Exception as _e:                       # cv2 缺失 / 目录名不对
    start_vision_process = None
    print(f"⚠️ 无法导入 Vision_worker: {_e}")

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
FRONT_TARGET_M    = 1.70
E_LAT_TARGET_M    = 0.455

# ---- 侧飞 ----
V_SIDE      = 0.35          # 已验证值，熟练后可到 0.30
SIDE_TIME_S = 20.0
SIDE_DIR    = +1.0

# ---- 高度（v4）----
# 注意：climb 的停止阈值是 HOVER_ALT_DOWN + 0.10 = -1.25（高度 1.25m）；
# 进入 hold 后 ALT_TARGET_M = 1.30m。
HOVER_ALT_DOWN    = -1.35
CLIMB_RATE        = -0.35
TAKEOFF_TIMEOUT_S = 15.0
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
KP_LAT, KI_LAT, KD_LAT = 0.25, 0.02, 0.0
MAX_FORWARD     = 0.2
WALL_ABORT_M    = 0.30
LIDAR_MIN_ALT_M = 0.70
FRONT_MIN_SAFE  = 0.70
SIDE_STOP_FRONT = 1.25
BACK_MIN_SAFE   = 0.65
E_LAT_TOL_TAKEOFF = 0.15

USE_YAW_LOOP     = True
KP_YAW, KI_YAW, KD_YAW = 3.0, 0.0, 0.0
MAX_YAWRATE      = 10.0
YAW_DEADBAND_DEG = 1.5
YAW_ABORT_DEG    = 25.0

MAX_JUMP_M     = 0.25
MAX_BAD_CNT    = 8
LOST_TIMEOUT_S = 1.0

# ---- 新增：安全/时序参数 ----
CONTROL_PERIOD_S   = 0.05      # 控制环 20Hz
SEND_PERIOD_S      = 0.05      # setpoint 20Hz；57600 若丢包/饱和，可改回 0.10
MAX_DT_S           = 0.20      # 单次 dt 上限，防止长时间阻塞后 PID 爆炸
HEALTH_TIMEOUT_S   = 30.0      # 等 local position OK 的超时
SEND_STOP_TIMEOUT_S = 2.0      # 收尾时等 send_velocity 退出的超时
CMD_TIMEOUT_S      = 5.0       # offboard.stop / land 的最长等待

# 到位置超时 / 侧飞中止 是否直接收尾。
# True = 安全优先，直接停止并尝试降落；False = 保留原来的继续流程行为。
ABORT_ON_READY_TIMEOUT = True
ABORT_ON_SIDE_ABORT    = True

# 这些模式说明飞手已经接管，脚本不应再发控制/不应自动降落。
PILOT_MODES = ("POSCTL", "ALTCTL", "STABILIZED", "MANUAL", "ACRO", "RATTITUDE")

# ---- 视觉（v5：看到消防栓 → 悬停 3s → 查柜内 4 件）----
VIS_ENABLE        = True
TARGET_CLASS      = "Fire_hydrant"
REQUIRED_CLASSES  = ("Valve", "Fire_hose", "Fire_nozzle", "Rubber_hose")
IGNORE_CLASSES    = ("Fire_extinguisher",)   # 先不判，也不显示

INSPECT_S         = 3.0     # 固定悬停时长
DECEL_S           = 0.4     # 触发后右侧速度斜坡降到 0 的时间
HYDRANT_WINDOW_N  = 6       # 滑动窗口：最近 6 个视觉帧里
HYDRANT_HITS      = 3       # 有 3 帧看到就算触发（不必连续）
VISION_STALE_S    = 1.0     # 视觉数据超过这么久算失效
RESUME_COOLDOWN_S = 8.0     # 复检后冷却，防止同一个柜子反复触发
VISION_START_TIMEOUT_S = 20.0   # 起飞前等视觉就绪

# ==================== 共享状态 ====================
cmd     = {"forward": 0.0, "right": 0.0, "down": 0.0, "yawspeed": 0.0}
current = {"north": 0.0, "east": 0.0, "down": 0.0,
           "vn": 0.0, "ve": 0.0, "vd": 0.0,
           "yaw": 0.0, "mode": "UNKNOWN"}
dist    = {"front": float("nan"), "back": float("nan")}

stop_flag = False
CTRL_ON = (WALL_MODE != "off")
PHASE = ["init"]        # init / climb / hold / ready / side / inspect / stop


class FlightAbort(RuntimeError):
    """主动中止：进入 finally 做收尾/降落。"""


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
        if not math.isfinite(dt) or dt <= 0.0:
            dt = 1e-3
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
#  小工具
# ============================================================
def control_dt(last_t):
    """返回 (now, dt)，dt 用实测值并限幅。"""
    now = time.monotonic()
    dt = now - last_t
    if not math.isfinite(dt) or dt <= 0.0:
        dt = 1e-3
    return now, min(dt, MAX_DT_S)


def pilot_has_control():
    mode = str(current.get("mode", "")).upper()
    return any(m in mode for m in PILOT_MODES)


def should_stop(stop_event):
    # 注意：这里不把 pilot_has_control() 放进来。
    # offboard 刚启动时遥测可能还停留在 STABILIZED/POSCTL，
    # 立刻判断会造成“刚起飞就中止”的误判。
    # 飞手接管只在 finally 里用于决定“是否自动降落”。
    return stop_flag or stop_event.is_set()


def _watch_task(task, name, stop_event):
    """后台任务异常/提前退出时，触发收尾。"""
    def _done(t):
        if t.cancelled():
            return
        exc = t.exception()
        if exc is not None:
            print(f"❌ {name} 异常退出: {type(exc).__name__}: {exc}")
            stop_event.set()
        elif not stop_event.is_set():
            if not stop_flag:
                print(f"⚠️ {name} 提前结束")
            stop_event.set()
    task.add_done_callback(_done)


# ============================================================
#  视觉：取最新结果
# ============================================================
_VIS = {"v": None}
def poll_vision(vq):
    """取最新视觉结果。

    20Hz 轮询 15fps 数据，队列空了必须返回上一条，
    否则 1/3 的 tick 会被判成“没视觉”，触发窗口永远凑不满。
    """
    if vq is None:
        return None
    while True:
        try:
            _VIS["v"] = vq.get_nowait()
        except queue.Empty:
            break
        except (EOFError, OSError):
            break
    v = _VIS["v"]
    if v is None:
        return None
    if (time.time() - v["ts"]) > VISION_STALE_S:
        return None
    return v


def check_cabinet(cnt):
    """返回缺失类列表（IGNORE_CLASSES 不计入）。"""
    return [c for c in REQUIRED_CLASSES if cnt.get(c, 0) <= 0]


# ============================================================
#  后台任务
# ============================================================
def _kb_watch():
    global stop_flag
    try:
        input()
    except (EOFError, OSError):
        # 没有交互终端时（ssh/systemd）直接返回，由信号/异常收尾。
        return
    stop_flag = True
    print("\n⏸️  收到回车，平稳停止")


async def send_velocity(drone, stop_event):
    """20Hz 显式 setpoint。异常会由 _watch_task 捕获。"""
    while not should_stop(stop_event):
        t0 = time.monotonic()
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(
            cmd["forward"], cmd["right"], cmd["down"], cmd["yawspeed"]))
        elapsed = time.monotonic() - t0
        await asyncio.sleep(max(0.0, SEND_PERIOD_S - elapsed))


async def alt_hold_task(stop_event):
    """高度外环 20Hz —— 只写 cmd['down']，爬升阶段不接管。"""
    pid = PID(KP_ALT, KI_ALT, KD_ALT, I_LIMIT_ALT, MAX_DOWN_CMD)
    last_t = time.monotonic()

    while not should_stop(stop_event):
        last_t, dt = control_dt(last_t)

        if (not ALT_HOLD_ENABLE) or PHASE[0] == "climb":
            pid.reset()
        else:
            alt = -current["down"]
            if alt < ALT_MIN_M:
                pid.reset()
                cmd["down"] = -MAX_DOWN_CMD
            else:
                cmd["down"] = pid.update(alt - ALT_TARGET_M, dt)

        await asyncio.sleep(CONTROL_PERIOD_S)


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


async def guard_task(lidar, stop_event):
    """全程距离保护：任何阶段离墙太近都立刻中止。

    ⭐ dist 的唯一写入者。
       之前是每个阶段各写一遍，hold 阶段漏了 → guard 拿到的是过期距离。
       集中在这里以后，加新阶段也不会忘。
    """
    while not should_stop(stop_event):
        if lidar is not None:
            s = lidar.read(0.05)
            dist["front"], dist["back"] = s.front, s.back

        f, b = dist["front"], dist["back"]
        if math.isfinite(f) and f < SIDE_STOP_FRONT:
            print(f"\n🛑 离前墙过近 {f:.2f}m → 立即中止")
            stop_event.set()
        elif math.isfinite(b) and b < BACK_MIN_SAFE:
            print(f"\n🛑 离后墙过近 {b:.2f}m → 立即中止")
            stop_event.set()
        await asyncio.sleep(0.05)


# ============================================================
#  主流程
# ============================================================
async def fly(vq, vstop):
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
        print("❌ 15 秒没连上")
        return

    last_health = None

    async def wait_local_position():
        nonlocal last_health
        async for h in drone.telemetry.health():
            last_health = h
            if h.is_local_position_ok:
                return

    try:
        await asyncio.wait_for(wait_local_position(), timeout=HEALTH_TIMEOUT_S)
        print("✅ 位置 OK")
    except asyncio.TimeoutError:
        print(f"❌ {HEALTH_TIMEOUT_S:.0f} 秒内 local position 未 OK，退出")
        if last_health is not None:
            print(f"  最后状态: local={last_health.is_local_position_ok} "
                  f"global={last_health.is_global_position_ok} "
                  f"home={last_health.is_home_position_ok} "
                  f"armable={last_health.is_armable}")
        return

    lidar = open_lidar() if CTRL_ON else None

    print(f"⚙️  WALL_MODE={WALL_MODE}  走廊 W={W_NOMINAL_M:.2f}m "
          f"| 横向目标 e_lat={E_LAT_TARGET_M:+.2f} 或 front={FRONT_TARGET_M:.2f}m")
    print(f"⚙️  高度环 目标 {ALT_TARGET_M:.2f}m  KP={KP_ALT} KI={KI_ALT}  "
          f"限幅={MAX_DOWN_CMD}  下限={ALT_MIN_M:.2f}")
    print(f"⚙️  侧飞 {V_SIDE:.2f}m/s × {SIDE_TIME_S:.0f}s "
          f"({V_SIDE*SIDE_TIME_S:.2f}m) 方向 {'右' if SIDE_DIR > 0 else '左'}")
    if VIS_ENABLE:
        print(f"⚙️  视觉 触发={TARGET_CLASS} 必检={list(REQUIRED_CLASSES)} "
              f"忽略={list(IGNORE_CLASSES)}")
        print(f"⚙️  悬停 {INSPECT_S:.1f}s  触发窗口 {HYDRANT_HITS}/{HYDRANT_WINDOW_N} 帧  "
              f"冷却 {RESUME_COOLDOWN_S:.0f}s")
    print(f"⚠️  确认侧飞方向 {V_SIDE*SIDE_TIME_S + 1.5:.1f}m 内无障碍")

    threading.Thread(target=_kb_watch, daemon=True).start()

    stop_event = asyncio.Event()
    send_task = asyncio.create_task(send_velocity(drone, stop_event))
    # 立刻挂监控：不要等 arm/offboard 之后才挂，
    # 否则这 1 秒窗口内 send_velocity 异常会没人处理。
    _watch_task(send_task, "send_velocity", stop_event)

    async def stop_before_flight(reason):
        print(f"❌ {reason}")
        stop_event.set()
        try:
            await asyncio.wait_for(send_task, timeout=SEND_STOP_TIMEOUT_S)
        except Exception:
            send_task.cancel()
            await asyncio.gather(send_task, return_exceptions=True)
        if lidar and isinstance(lidar, RealLidar):
            lidar.stop()

    await asyncio.sleep(1.0)
    if stop_event.is_set():
        await stop_before_flight("起飞前 setpoint 任务已退出")
        return

    # ---- 起飞前等视觉就绪（模型加载 + 摄像头出第一帧）----
    if VIS_ENABLE:
        if start_vision_process is None:
            print("❌ 视觉模块没导入成功，不能起飞（或把 VIS_ENABLE 设 False）")
            await stop_before_flight("视觉模块不可用")
            return
        print("⏳ 等视觉就绪 ...")
        t_v, vis0 = time.monotonic(), None
        while time.monotonic() - t_v < VISION_START_TIMEOUT_S:
            if stop_event.is_set() or stop_flag:
                break
            vis0 = poll_vision(vq)
            if vis0 is not None:
                break
            await asyncio.sleep(0.2)
        if vis0 is None:
            print(f"❌ {VISION_START_TIMEOUT_S:.0f}s 内视觉没出数据，取消起飞")
            print("   先单独跑 vision/Yolo_test.py 确认相机和模型")
            await stop_before_flight("视觉未就绪")
            return
        print(f"✅ 视觉就绪: {vis0['counts']} @ {vis0['fps']}fps")

    if lidar and CTRL_ON:
        s0 = lidar.read(0.3)
        if math.isfinite(s0.front) and math.isfinite(s0.back):
            el0 = (s0.front - s0.back) / 2.0 - E_LAT_TARGET_M
            print(f"📍 起飞点: front={s0.front:.3f} back={s0.back:.3f} "
                  f"e_lat={el0*100:+.1f}cm (容差 ±{E_LAT_TOL_TAKEOFF*100:.0f}cm)")
            if abs(el0) > E_LAT_TOL_TAKEOFF:
                print("❌ 起飞点偏太多，请重新摆位后再跑（不要起飞）")
                await stop_before_flight("起飞点偏超限")
                return

    print("🔓 解锁")
    try:
        await drone.action.arm()
    except Exception as e:
        await stop_before_flight(f"解锁失败: {e}")
        return

    if stop_event.is_set():
        await stop_before_flight("解锁后 setpoint 任务已退出")
        try:
            await drone.action.disarm()
        except Exception:
            pass
        return

    print("🚁 进入 Offboard")
    offboard_started = False
    try:
        await drone.offboard.start()
        offboard_started = True
    except OffboardError as e:
        # 不依赖 e._result 的内部结构；不同 MAVSDK 版本字段可能不同。
        await stop_before_flight(f"Offboard 启动失败: {e}")
        try:
            await drone.action.disarm()
        except Exception:
            pass
        return

    if stop_event.is_set():
        await stop_before_flight("Offboard 启动后 setpoint 任务已退出")
        try:
            await asyncio.wait_for(drone.offboard.stop(), timeout=CMD_TIMEOUT_S)
        except Exception:
            pass
        try:
            await drone.action.disarm()
        except Exception:
            pass
        return

    # 先设 climb，再启动 alt_task，避免 alt_task 在 init 阶段写 down。
    PHASE[0] = "climb"
    state_task = asyncio.create_task(track_state(drone))
    alt_task = asyncio.create_task(alt_hold_task(stop_event))
    guard = asyncio.create_task(guard_task(lidar, stop_event))

    # 任务监控：任一任务异常/提前退出，都会 set(stop_event)。
    _watch_task(alt_task, "alt_hold_task", stop_event)
    _watch_task(state_task, "track_state", stop_event)
    _watch_task(guard, "guard_task", stop_event)

    ctrl = WallController()
    tick = 0
    side_abort = ""
    alt_min_seen = 99.0
    inspect_log = []

    try:
        # ========== 阶段 1：爬升（高度环不接管，雷达够高才开）==========
        print("🛫 起飞中...")
        t0 = time.monotonic()
        t_prev = t0
        alt_prev = -current["down"]
        last_t = time.monotonic()

        while not should_stop(stop_event):
            last_t, dt = control_dt(last_t)
            alt = -current["down"]
            s = lidar.read(dt) if lidar else None

            if s is not None and CTRL_ON and alt > CLIMB_LAT_ON_ALT:
                fwd, yr, _, _ = ctrl.update(s, alt, True, dt)
                cmd["forward"], cmd["yawspeed"] = fwd, yr
            else:
                ctrl.pid_lat.reset()
                ctrl.pid_yaw.reset()
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
                      f"实际{v_act:+.3f} fwd={cmd['forward']:+.3f} "
                      f"f={s.front if s else float('nan'):5.3f} "
                      f"b={s.back if s else float('nan'):5.3f} "
                      f"vn={current['vn']:+.3f} ve={current['ve']:+.3f} "
                      f"mode={current['mode']}{flag}")
            if current["down"] <= HOVER_ALT_DOWN + 0.10:
                break

            if time.monotonic() - t0 > TAKEOFF_TIMEOUT_S:
                raise FlightAbort("起飞超时")
            await asyncio.sleep(CONTROL_PERIOD_S)

        if should_stop(stop_event):
            raise FlightAbort("爬升阶段收到停止/接管")
        print(f"   爬升耗时 {time.monotonic()-t0:.1f}s，高度 {alt:+.2f}m")

        # ========== 阶段 2：稳住（高度环接管）==========
        PHASE[0] = "hold"
        print(f"⏸️  悬停稳住 {STABILIZE_S:.0f}s（高度环已接管）...")
        t_s = time.monotonic()
        last_t = time.monotonic()

        while not should_stop(stop_event) and time.monotonic() - t_s < STABILIZE_S:
            last_t, dt = control_dt(last_t)
            if lidar and CTRL_ON:
                alt = -current["down"]
                s = lidar.read(dt)
                fwd, yr, _, _ = ctrl.update(s, alt, True, dt)
                cmd["forward"], cmd["yawspeed"] = fwd, yr
            alt_min_seen = min(alt_min_seen, -current["down"])
            await asyncio.sleep(CONTROL_PERIOD_S)

        if should_stop(stop_event):
            raise FlightAbort("hold 阶段收到停止/接管")

        # ========== 阶段 2b：等横向到位 ==========
        if lidar and CTRL_ON:
            print(f"⏳ 等横向到位 |e| < {READY_ELAT_M*100:.0f}cm ...")
            t_r = time.monotonic()
            ready = False
            last_t = time.monotonic()

            while not should_stop(stop_event) and time.monotonic() - t_r < READY_TIMEOUT_S:
                last_t, dt = control_dt(last_t)
                alt = -current["down"]
                s = lidar.read(dt)
                fwd, yr, _, _ = ctrl.update(s, alt, True, dt)
                cmd["forward"], cmd["yawspeed"] = fwd, yr

                if (math.isfinite(ctrl.e)
                        and abs(ctrl.e) < READY_ELAT_M
                        and alt > LIDAR_MIN_ALT_M):
                    ready = True
                    break
                await asyncio.sleep(CONTROL_PERIOD_S)

            print(f"   {'✅ 到位' if ready else '⚠️ 超时'}: "
                  f"e={ctrl.e*100:+.1f}cm front_use={ctrl.front_used:.3f}m "
                  f"用时 {time.monotonic()-t_r:.1f}s")

            if not ready and ABORT_ON_READY_TIMEOUT:
                raise FlightAbort("横向到位超时")

        if should_stop(stop_event):
            raise FlightAbort("ready 阶段收到停止/接管")

        # ========== 阶段 3：侧飞（两环 + 视觉触发悬停）==========
        PHASE[0] = "side"
        print(f"➡️  开始侧飞 {V_SIDE:.2f}m/s × {SIDE_TIME_S:.0f}s ...")
        t_sd = time.monotonic()
        last_t = time.monotonic()

        hyd_hist       = deque(maxlen=HYDRANT_WINDOW_N)
        last_vis_n     = -1
        cooldown_until = 0.0
        paused_total   = 0.0        # 悬停累计时间，从侧飞行程里扣掉

        async def do_inspect(vis_now):
            """方案 A：原地悬停 INSPECT_S 秒，横向+航向全锁死，只累积视觉。"""
            nonlocal alt_min_seen
            PHASE[0] = "inspect"
            t_i = time.monotonic()
            n0, e0 = current["north"], current["east"]
            f0, b0 = dist["front"], dist["back"]
            seen, last_n, n_print = {}, -1, 0

            print(f"🎯 触发：hyd={vis_now['counts'].get(TARGET_CLASS, 0)} "
                  f"@ {vis_now['fps']}fps → 悬停 {INSPECT_S:.1f}s")

            # 1) 右侧速度斜坡降到 0，减少刹车冲过头
            t_r = time.monotonic()
            while not should_stop(stop_event) and time.monotonic() - t_r < DECEL_S:
                k = 1.0 - (time.monotonic() - t_r) / DECEL_S
                cmd["right"]    = V_SIDE * SIDE_DIR * max(0.0, k)
                cmd["forward"]  = 0.0
                cmd["yawspeed"] = 0.0
                await asyncio.sleep(CONTROL_PERIOD_S)

            # 2) 完全静止悬停，累积视觉（各帧取并集，比单帧稳）
            while not should_stop(stop_event) and time.monotonic() - t_i < INSPECT_S:
                cmd["forward"] = cmd["right"] = cmd["yawspeed"] = 0.0
                # 不碰 cmd["down"]，高度环继续工作

                alt = -current["down"]
                alt_min_seen = min(alt_min_seen, alt)

                vis = poll_vision(vq)
                if vis is not None and vis["n"] != last_n:
                    last_n = vis["n"]
                    for k, c in vis["counts"].items():
                        if c > 0 and k not in IGNORE_CLASSES:
                            seen[k] = max(seen.get(k, 0), c)

                el = time.monotonic() - t_i
                n_print += 1
                if n_print % 5 == 0:      # 20Hz / 5 = 0.25s 一次
                    print(f"    悬停{el:4.1f}s alt={alt:4.2f} "
                          f"f={dist['front']:5.3f} b={dist['back']:5.3f} "
                          f"dN={current['north']-n0:+6.3f} "
                          f"dE={current['east']-e0:+6.3f} seen={seen}")
                await asyncio.sleep(CONTROL_PERIOD_S)

            # 3) 结算
            dn, de = current["north"] - n0, current["east"] - e0
            missing = check_cabinet(seen)
            inspect_log.append({"f": round(dist["front"], 2),
                                "b": round(dist["back"], 2),
                                "counts": dict(seen), "missing": missing,
                                "dN": round(dn, 3), "dE": round(de, 3)})

            print(f"  📋 悬停 {time.monotonic()-t_i:.1f}s 累积检出: {seen}")
            if not seen:
                print("  ⚠️ 一个目标都没看到 → 查相机朝向/距离/曝光")
            elif missing:
                print(f"  ❌ 柜内缺: {', '.join(missing)}")
            else:
                print(f"  ✅ 柜内齐全 "
                      f"({len(REQUIRED_CLASSES)}/{len(REQUIRED_CLASSES)})")
            print(f"  📐 悬停位移 dN={dn*100:+.1f}cm dE={de*100:+.1f}cm "
                  f"| front {f0:.3f}→{dist['front']:.3f} "
                  f"back {b0:.3f}→{dist['back']:.3f}")

            ctrl.pid_lat.reset()       # 重新起步，别带停稳前的旧积分
            ctrl.pid_yaw.reset()
            PHASE[0] = "side"
            return time.monotonic() - t_i

        while (not should_stop(stop_event)
               and (time.monotonic() - t_sd - paused_total) < SIDE_TIME_S):
            last_t, dt = control_dt(last_t)
            alt = -current["down"]
            el = time.monotonic() - t_sd - paused_total   # 只算真正在侧飞的时间
            alt_min_seen = min(alt_min_seen, alt)

            if lidar and CTRL_ON:
                s = lidar.read(dt)
                fwd, yr, f_ok, lost = ctrl.update(s, alt, True, dt)
                cmd["forward"], cmd["yawspeed"] = fwd, yr

                if lost:
                    side_abort = "雷达丢墙"
                elif math.isfinite(s.front) and s.front < SIDE_STOP_FRONT:
                    side_abort = f"离墙过近 {s.front:.2f}m"
                elif math.isfinite(s.back) and s.back < BACK_MIN_SAFE:
                    side_abort = f"离后墙过近 {s.back:.2f}m"

                if side_abort:
                    cmd["right"] = 0.0
                    print(f"🛑 侧飞中止：{side_abort}")
                    break

                cmd["right"] = V_SIDE * SIDE_DIR
            else:
                cmd["forward"] = cmd["yawspeed"] = 0.0
                cmd["right"] = V_SIDE * SIDE_DIR

            # ---- 视觉触发（有新帧才计数，避免同一帧被 20Hz 重复算）----
            if VIS_ENABLE:
                now = time.monotonic()
                vis = poll_vision(vq)
                if vis is not None and vis["n"] != last_vis_n:
                    last_vis_n = vis["n"]
                    hyd_hist.append(vis["counts"].get(TARGET_CLASS, 0) > 0
                                    and now > cooldown_until)
                    if sum(hyd_hist) >= HYDRANT_HITS and now > cooldown_until:
                        print(f"🎯 最近 {len(hyd_hist)} 帧里 "
                              f"{sum(hyd_hist)} 帧看到消防栓")
                        hyd_hist.clear()
                        t_b = time.monotonic()
                        await do_inspect(vis)
                        paused_total += time.monotonic() - t_b
                        cooldown_until = time.monotonic() + RESUME_COOLDOWN_S
                        last_t = time.monotonic()   # 别把 3s 悬停算进 PID 的 dt
                        continue

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
                      f"yaw={current['yaw']:+6.1f}° hyd={sum(hyd_hist)}")
            await asyncio.sleep(CONTROL_PERIOD_S)

        if side_abort:
            if ABORT_ON_SIDE_ABORT:
                raise FlightAbort(f"侧飞中止: {side_abort}")
            print(f"🛑 侧飞提前停止：{side_abort}")

        if should_stop(stop_event):
            raise FlightAbort("side 阶段收到停止/接管")

        # ========== 阶段 4：停 + 悬停（两环继续工作）==========
        PHASE[0] = "stop"
        print(f"⏹️  停止侧飞，悬停 {STOP_HOVER_S:.0f}s ...")
        t_st = time.monotonic()
        last_t = time.monotonic()

        while not should_stop(stop_event) and time.monotonic() - t_st < STOP_HOVER_S:
            last_t, dt = control_dt(last_t)
            if lidar and CTRL_ON:
                alt = -current["down"]
                s = lidar.read(dt)
                fwd, yr, _, _ = ctrl.update(s, alt, True, dt)
                cmd["forward"], cmd["yawspeed"] = fwd, yr
            cmd["right"] = 0.0
            alt_min_seen = min(alt_min_seen, -current["down"])
            await asyncio.sleep(CONTROL_PERIOD_S)

    except FlightAbort as e:
        print(f"\n🛑 主动中止: {e}")
    except Exception as e:
        print(f"\n❌ 飞行阶段异常: {type(e).__name__}: {e}")
        print("   → 立即收尾")
    finally:
        # 关键：在 offboard.stop() 之前记录模式和是否飞手接管。
        pilot_takeover = pilot_has_control()
        mode_before = current.get("mode", "UNKNOWN")
        print(f"🛬 收尾：mode={mode_before} pilot_takeover={pilot_takeover}")

        # 先停控制写入：否则 alt_hold_task 会在收尾时把 cmd["down"] 又写回去。
        stop_event.set()
        for task in (alt_task, state_task, guard):
            task.cancel()
        await asyncio.gather(alt_task, state_task, guard,
                             return_exceptions=True)

        cmd["forward"] = cmd["right"] = cmd["down"] = cmd["yawspeed"] = 0.0

        # 再停 setpoint 流，最后才 offboard.stop，避免发送与 stop 竞争。
        try:
            await asyncio.wait_for(send_task, timeout=SEND_STOP_TIMEOUT_S)
        except asyncio.TimeoutError:
            print(f"⚠️ send_velocity 未在 {SEND_STOP_TIMEOUT_S:.1f}s 内退出，取消")
            send_task.cancel()
            await asyncio.gather(send_task, return_exceptions=True)
        except Exception as e:
            print(f"⚠️ send_velocity 退出异常: {type(e).__name__}: {e}")

        try:
            await asyncio.wait_for(drone.offboard.stop(), timeout=CMD_TIMEOUT_S)
        except asyncio.TimeoutError:
            print(f"⚠️ offboard.stop 超时（>{CMD_TIMEOUT_S:.0f}s）")
        except Exception as e:
            print(f"⚠️ offboard.stop 失败: {type(e).__name__}: {e}")

        # 降落：只要进过 offboard，且不是飞手接管，就尝试 land。
        # 不再用“offboard.stop() 之后是否还是 OFFBOARD”来判断。
        if offboard_started and not pilot_takeover:
            try:
                await asyncio.wait_for(drone.action.land(), timeout=CMD_TIMEOUT_S)
                print("✅ 已发送降落")
            except asyncio.TimeoutError:
                print(f"⚠️ land 超时（>{CMD_TIMEOUT_S:.0f}s）")
            except Exception as e:
                print(f"⚠️ land 失败: {type(e).__name__}: {e}")
        else:
            print(f"⚠️ 未发送自动降落 "
                  f"(offboard_started={offboard_started}, pilot_takeover={pilot_takeover})")

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

        if VIS_ENABLE:
            if inspect_log:
                print(f"   检视记录 {len(inspect_log)} 处：")
                for i, r in enumerate(inspect_log, 1):
                    print(f"     #{i} front={r['f']}m back={r['b']}m "
                          f"counts={r['counts']} 缺={r['missing'] or '无'} "
                          f"位移 dN={r['dN']}m dE={r['dE']}m")
            else:
                print("   （全程没触发检视）")

        vstop.set()          # 通知视觉子进程退出
        print("✅ 完成")


if __name__ == "__main__":
    # ⭐ 必须在 mavsdk 连上之前 fork，否则子进程会继承 gRPC socket
    vq, vstop, vproc = mp.Queue(), mp.Event(), None
    if VIS_ENABLE and start_vision_process is not None:
        vproc = mp.Process(target=start_vision_process,
                           args=(vq, vstop), daemon=True)
        vproc.start()
        print(f"[Main] 视觉子进程 pid={vproc.pid}")
        time.sleep(1.0)

    try:
        asyncio.run(fly(vq, vstop))
    except Exception as e:
        print(f"\n❌ 顶层异常: {type(e).__name__}: {e}")
        print("⚠️ 若飞机还在空中，立刻用遥控器切到 Position/Stabilized 接管！")
    finally:
        if vproc is not None:
            vstop.set()
            vproc.join(timeout=2.0)
            if vproc.is_alive():
                vproc.terminate()
                print("[Main] 视觉子进程已强制结束")
