"""
四阶段起飞 → 转向 → 前后环 → 侧飞

阶段划分（每个阶段只有一个通道写入者）：
    ① climb  起飞爬升 + 【左右居中环】(cmd['right'])
             机头沿走廊 → 前后漂沿走廊走，安全；左右只是保险
    ② turn   转航向 90°，只给 yawspeed，横向环全关
             停止判据 = W = front+back 降到 2.55 以下（≈±12°）
    ③ hold   前后环接管，等 |e_lat| < 8cm
             转完那 1m 的"前后漂"就变成"朝墙漂"，所以必须立刻接管
    ④ side   侧飞（前后环 + H.I. 侧向速度）

⚠️ 前提：Lidar_source.py 已打左右扇区补丁，且 side_ok 能正常为 True
⚠️ 前置：param set MIS_TAKEOFF_ALT 1.3   （本脚本不用 AUTO_TAKEOFF，但设了无害）

停止：按回车（不要 Ctrl+C）
"""

import asyncio
import math
import threading
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

from Lidar_source import RealLidar, make_lidar

# ============================================================
#  配置 —— 按阶段分组，改哪个一目了然
# ============================================================
ADDRESS     = "serial:///dev/pixhawk:57600"
LIDAR_PORT  = "/dev/wheeltec_lidar"
LIDAR_MODEL = "N10"

# ---------- 走廊几何 ----------
W_NOMINAL_M = 2.49      # 走廊宽度标定值（多处实测一致）
W_TOL_M     = 0.15      # |left+right−W| 或 |front+back−W| 超这个 → 读数不可信

# ---------- 高度 ----------
ALT_TARGET_M      = 1.30    # 目标悬停高度 m
CLIMB_RATE        = -0.35   # 爬升速度命令（NED，负=向上）
CLIMB_STOP_M      = 1.35    # 爬到这高度就停爬（进入 hold 后降到 1.30，那 5cm 是预期的）
TAKEOFF_TIMEOUT_S = 30.0    # 起飞超时 s

# ---------- 阶段①：起飞时左右居中（防左右漂）----------
SIDE_ON_ALT_M   = 0.15   # 高度超过这个才开左右环（贴地时读数不可信）
KP_SIDE         = 0.70   # 左右居中 P
KI_SIDE         = 0.10   # 左右不加积分：量小、不需要，加了反而过冲
SIDE_SIGN       = +1.0
MAX_RIGHT       = 0.30   # ⚠️ 关键！很小。左右本来就不漂，只做保险，
                         #    给大了会抢走推力余量 → 反而更爬不动
SIDE_DEADBAND_M = 0.04   # |e_side| < 3cm 一律不动作（抑制噪声）
SIDE_ABORT_M    = 0.90   # 偏离走廊中线超这个 → 中止（2.49/2=1.25，留一半余量）

# ---------- 阶段②：转航向（机头 → 墙）----------
TURN_RATE      = 50.0    # 转速 deg/s（你手动实测约 23°/s）
TURN_SIGN      = -1.0    # -1 = 左转（逆时针，和你手动那次一致）；+1 = 右转
W_STOP         = 2.55    # W 降到这个以下 = 已转到 ±12° 内 → 停转
                         # 换算：acos(2.49/2.55) = 12.4°
                         #       2.58 → ±15°；2.65 → ±20°
TURN_TIMEOUT_S = 6.0     # 转这么久还没到 → 中止
W_SANITY_M     = 0.25    # 转完后 |W−2.49| 超这个 → 中止（没转正/有遮挡）
TURN_DEG       = 88.0    # 目标转角（留 2° 给阶段③ 的航向环去收）
TURN_TOL_DEG   = 3.0     # 到位判据
TURN_GAIN      = 3.0     # yaw 环 P，用于末端减速，避免冲过

# ---------- 阶段③④：前后环（机头对墙后）----------
E_LAT_TARGET_M  = 0.00   # ⚠️ 第一版用 0（走廊居中，两边各 1.25m 余量）
                         #    验证横向环有效后，再改 0.37（front 1.61 / back 0.87）
KP_LAT, KI_LAT, KD_LAT = 0.65, 0.10, 0.0   # 闭环时间常数 τ = 1/KP ≈ 3.3s
MAX_FORWARD     = 0.30   # 前后纠偏速度上限 m/s
WALL_ABORT_M    = 0.35   # |e_lat| 超这个 → 放弃纠偏（fwd 归零）
READY_ELAT_M    = 0.08   # 到位判据：|e_lat| < 8cm
READY_TIMEOUT_S = 20.0   # ⚠️ 给宽容点（τ≈3.3s，原来 12s 太紧会误判超时）

# ---------- 航向环（机头对墙后启用）----------
USE_YAW_LOOP     = True
KP_YAW, KI_YAW, KD_YAW = 3.0, 0.0, 0.0   # 输入单位必须是【度】
MAX_YAWRATE      = 10.0                  # deg/s 上限
YAW_DEADBAND_DEG = 1.5                   # 1.5° 内不动
YAW_ABORT_DEG    = 25.0                  # 超过这个直接放弃（数据不可信）

# ---------- 阶段④：侧飞 ----------
V_SIDE      = 0.30   # 侧飞速度 m/s（先保守；0.35 也走过）
SIDE_TIME_S = 20.0   # 侧飞时长 s
SIDE_DIR    = +1.0   # +1 右 / -1 左

# ---------- 全程保护 ----------
SIDE_STOP_FRONT  = 0.90   # front 低于此值 → 立即中止
BACK_MIN_SAFE    = 0.65   # back  低于此值 → 立即中止
LIDAR_MIN_ALT_M  = 0.50   # 低于此高度不做前后/航向纠偏
MAX_JUMP_M       = 0.25   # 单次跳变阈值 m
MAX_BAD_CNT      = 8      # 连续几次跳变才认账
LOST_TIMEOUT_S   = 1.0    # 丢墙多久中止 s

# ---------- 时序 ----------
CONTROL_PERIOD_S    = 0.05   # 控制环 20Hz
SEND_PERIOD_S       = 0.05   # setpoint 20Hz
MAX_DT_S            = 0.20   # 单次 dt 上限，防阻塞后 PID 爆炸
HEALTH_TIMEOUT_S    = 30.0   # 等 local position OK
SEND_STOP_TIMEOUT_S = 2.0    # 收尾等 send_velocity 退出
CMD_TIMEOUT_S       = 5.0    # offboard.stop / land 最长等待

PILOT_MODES = ("POSCTL", "ALTCTL", "STABILIZED", "MANUAL", "ACRO", "RATTITUDE")

# ============================================================
#  共享状态
# ============================================================
cmd     = {"forward": 0.0, "right": 0.0, "down": 0.0, "yawspeed": 0.0}
current = {"north": 0.0, "east": 0.0, "down": 0.0,
           "vn": 0.0, "ve": 0.0, "vd": 0.0,
           "yaw": 0.0, "mode": "UNKNOWN"}
dist    = {"front": float("nan"), "back": float("nan"),
           "left": float("nan"),  "right": float("nan")}

stop_flag = False
PHASE = ["init"]        # init / climb / turn / hold / side / stop


class FlightAbort(RuntimeError):
    pass


# ============================================================
#  PID（条件积分抗饱和）
# ============================================================
class PID:
    def __init__(self, kp, ki, kd, i_limit, out_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.out_limit = i_limit, out_limit
        self.i, self.prev_e = 0.0, None

    def reset(self):
        self.i, self.prev_e = 0.0, None

    def update(self, e, dt):
        if not math.isfinite(e):
            e = 0.0
        if not math.isfinite(dt) or dt <= 0.0:
            dt = 1e-3
        d = 0.0 if self.prev_e is None else (e - self.prev_e) / dt
        self.prev_e = e
        raw = self.kp * e + self.ki * self.i + self.kd * d
        if not (abs(raw) >= self.out_limit and raw * e > 0):
            self.i = max(-self.i_limit, min(self.i_limit, self.i + e * dt))
        out = self.kp * e + self.ki * self.i + self.kd * d
        return max(-self.out_limit, min(self.out_limit, out))


class FrontGuard:
    """单次跳变 > MAX_JUMP_M 先拒，连续 MAX_BAD_CNT 次才认账"""
    def __init__(self, max_jump=MAX_JUMP_M, max_bad=MAX_BAD_CNT):
        self.prev = self.raw = None
        self.bad = 0
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
    """机头对墙之后用：前后双墙 + 航向"""
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
        self.n_bad = 0
        self.n_est = 0
        self.front_used = float("nan")

    def _sanitize(self, f, b):
        """双墙一致性校验：W 对不上就判 bad（挡住起飞阶段的脏数据）"""
        if not math.isfinite(f):
            return float("nan"), "nofront"
        if not math.isfinite(b) or W_NOMINAL_M <= 0:
            self.good_front = f
            return f, "nogap"
        w = f + b
        if abs(w - W_NOMINAL_M) <= W_TOL_M:
            self.good_front, self.good_back = f, b
            return f, "ok"
        self.n_bad += 1
        return float("nan"), "bad"

    def update(self, s, alt, dt):
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

        f_use, self.w_state = self._sanitize(f_ok, s.back)

        ok = False
        self.e = float("nan")
        if math.isfinite(f_use):
            self.front_used = f_use
            if math.isnan(self.f_min):
                self.f_min = self.f_max = f_use
            self.f_min = min(self.f_min, f_use)
            self.f_max = max(self.f_max, f_use)
            self.f_last = f_use
            self.e = (f_use - s.back) / 2.0 - E_LAT_TARGET_M
            ok = (abs(self.e) < WALL_ABORT_M) and (alt > LIDAR_MIN_ALT_M)

        fwd = self.pid_lat.update(self.e, dt) if ok else 0.0
        if not ok:
            self.pid_lat.reset()

        # ---- 航向环 ----
        if (USE_YAW_LOOP and math.isfinite(s.yaw_err)
                and alt > LIDAR_MIN_ALT_M):
            e_yaw = math.degrees(s.yaw_err)      # ⚠️ 必须传"度"
            if abs(e_yaw) < YAW_DEADBAND_DEG:
                e_yaw = 0.0
            if abs(e_yaw) < YAW_ABORT_DEG:
                yr = self.pid_yaw.update(e_yaw, dt)
            else:
                self.pid_yaw.reset()
                yr = 0.0
        else:
            self.pid_yaw.reset()
            yr = 0.0

        return fwd, yr, f_ok, lost


# ============================================================
#  小工具
# ============================================================
def control_dt(last_t):
    now = time.monotonic()
    dt = now - last_t
    if not math.isfinite(dt) or dt <= 0.0:
        dt = 1e-3
    return now, min(dt, MAX_DT_S)

def wrap180(a):
    """角度归一到 (−180, 180]"""
    return (a + 180.0) % 360.0 - 180.0


def pilot_has_control():
    m = str(current.get("mode", "")).upper()
    return any(x in m for x in PILOT_MODES)


def should_stop(stop_event):
    return stop_flag or stop_event.is_set()


def _watch_task(task, name, stop_event):
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


def _kb_watch():
    global stop_flag
    try:
        input()
    except (EOFError, OSError):
        return
    stop_flag = True
    print("\n⏸️  收到回车，平稳停止")


# ============================================================
#  后台任务
# ============================================================
async def send_velocity(drone, stop_event):
    while not should_stop(stop_event):
        t0 = time.monotonic()
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(
            cmd["forward"], cmd["right"], cmd["down"], cmd["yawspeed"]))
        await asyncio.sleep(max(0.0, SEND_PERIOD_S - (time.monotonic() - t0)))


async def alt_hold_task(stop_event):
    """高度外环 —— 只写 cmd['down']，climb 阶段不接管"""
    pid = PID(KP_ALT_P, 0.10, 0.0, 0.30, 0.20)
    last_t = time.monotonic()
    while not should_stop(stop_event):
        last_t, dt = control_dt(last_t)
        if PHASE[0] == "climb":
            pid.reset()
        else:
            alt = -current["down"]
            if alt < 0.80:
                pid.reset()
                cmd["down"] = -0.20
            else:
                cmd["down"] = pid.update(alt - ALT_TARGET_M, dt)
        await asyncio.sleep(CONTROL_PERIOD_S)


# 高度环 P（单独起个名字，避免和 KP_LAT 混淆）
KP_ALT_P = 0.50


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
        if math.isfinite(s.front) and math.isfinite(s.back) and s.side_ok:
            print(f"✅ 雷达就绪: front={s.front:.3f} back={s.back:.3f} "
                  f"left={s.left:.3f} right={s.right:.3f}")
            print(f"   机头沿走廊时 front+back={s.front + s.back:.3f}"
                  f"（应该远大于 {W_NOMINAL_M}）  "
                  f"left+right={s.left + s.right:.3f}（应该≈{W_NOMINAL_M}）")
            return lidar
        if time.monotonic() - t > 10:
            print("❌ 10 秒没拿到完整数据（含左右）——Lidar_source 补丁打了吗？")
            if isinstance(lidar, RealLidar):
                lidar.stop()
            raise SystemExit(1)
        time.sleep(0.1)


async def guard_task(stop_event):
    """全程墙距保护（用 dist 快照，自己不读雷达）"""
    while not should_stop(stop_event):
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
        print("❌ 15 秒没连上")
        return

    lidar = open_lidar()

    # ========== 阶段 0：前置检查 ==========
    s0 = lidar.read(0.5)
    w0 = s0.front + s0.back
    print(f"\n📍 起飞点: front={s0.front:.3f} back={s0.back:.3f} "
          f"left={s0.left:.3f} right={s0.right:.3f}")
    print(f"   front+back={w0:.3f}  left+right={s0.left + s0.right:.3f}")

    if not math.isfinite(w0) or w0 < 3.5:
        print(f"❌ front+back={w0:.2f}，说明机头已经大致对墙了。")
        print("   本脚本是按【机头沿走廊】设计的，请把机头转向走廊方向再跑。")
        lidar.stop()
        return
    if abs((s0.left + s0.right) - W_NOMINAL_M) > W_TOL_M:
        print(f"❌ left+right={s0.left + s0.right:.2f} 不等于 {W_NOMINAL_M}")
        print("   检查 LEFT_DEG / RIGHT_DEG 是否配反，或飞机不在走廊中间。")
        lidar.stop()
        return
    if abs(s0.left - s0.right) > 0.30:
        print(f"⚠️ 起飞点不居中（偏差 {(s0.right - s0.left)*100:+.0f}cm），"
              f"建议先摆正，否则阶段①一上来就饱和")

    async def wait_lpos():
        async for h in drone.telemetry.health():
            if h.is_local_position_ok:
                return

    try:
        await asyncio.wait_for(wait_lpos(), timeout=HEALTH_TIMEOUT_S)
        print("✅ 位置 OK")
    except asyncio.TimeoutError:
        print(f"❌ {HEALTH_TIMEOUT_S:.0f} 秒内 local position 未 OK，退出")
        lidar.stop()
        return

    threading.Thread(target=_kb_watch, daemon=True).start()

    stop_event = asyncio.Event()
    send_task = asyncio.create_task(send_velocity(drone, stop_event))
    _watch_task(send_task, "send_velocity", stop_event)

    async def stop_before_flight(reason):
        print(f"❌ {reason}")
        stop_event.set()
        try:
            await asyncio.wait_for(send_task, timeout=SEND_STOP_TIMEOUT_S)
        except Exception:
            send_task.cancel()
            await asyncio.gather(send_task, return_exceptions=True)
        if isinstance(lidar, RealLidar):
            lidar.stop()

    await asyncio.sleep(1.0)

    print("🔓 解锁")
    try:
        await drone.action.arm()
    except Exception as e:
        await stop_before_flight(f"解锁失败: {e}")
        return

    print("🚁 进入 Offboard")
    offboard_started = False
    try:
        await drone.offboard.start()
        offboard_started = True
    except OffboardError as e:
        await stop_before_flight(f"Offboard 启动失败: {e}")
        try:
            await drone.action.disarm()
        except Exception:
            pass
        return

    # ---------- 启动后台任务 ----------
    PHASE[0] = "climb"
    state_task = asyncio.create_task(track_state(drone))
    alt_task   = asyncio.create_task(alt_hold_task(stop_event))
    guard      = asyncio.create_task(guard_task(stop_event))
    _watch_task(alt_task, "alt_hold_task", stop_event)
    _watch_task(state_task, "track_state", stop_event)
    _watch_task(guard, "guard_task", stop_event)

    ctrl     = WallController()
    pid_side = PID(KP_SIDE, KI_SIDE, 0.0, 0.50, MAX_RIGHT)
    tick = 0
    side_abort = ""
    alt_min_seen = 99.0

    try:
        # ==================================================
        #  阶段①：起飞爬升 + 左右居中
        # ==================================================
        print(f"\n🛫 ① 起飞（{CLIMB_STOP_M:.2f}m）"
              f"  左右居中环 KP={KP_SIDE} 限幅=±{MAX_RIGHT}")
        print(f"   机头沿走廊 → 前后漂沿走廊走；左右只是保险")
        t0 = time.monotonic()
        last_t = time.monotonic()
        t_prev, alt_prev = t0, -current["down"]

        while not should_stop(stop_event):
            last_t, dt = control_dt(last_t)
            alt = -current["down"]
            s = lidar.read(dt)

            # ---- dist 快照（给 guard_task 用）----
            dist["front"], dist["back"] = s.front, s.back
            dist["left"],  dist["right"] = s.left,  s.right

            # ---- 左右居中环（唯一写 cmd['right'] 的地方）----
            # left+right 必须≈走廊宽，否则说明读数不能当"居中"用
            side_valid = (s.side_ok
                          and abs((s.left + s.right) - W_NOMINAL_M) <= W_TOL_M)

            if side_valid and alt > SIDE_ON_ALT_M:
                e_side = s.right - s.left        # >0 = 偏左 → 往右
                if abs(e_side) < SIDE_DEADBAND_M:
                    e_side = 0.0
                cmd["right"] = SIDE_SIGN * pid_side.update(e_side, dt)
            else:
                pid_side.reset()
                cmd["right"] = 0.0

            # ---- 起飞阶段：前后、航向 一律不动 ----
            cmd["forward"] = 0.0
            cmd["yawspeed"] = 0.0

            # ---- 高度（本阶段由主循环写）----
            cmd["down"] = 0.0 if alt >= CLIMB_STOP_M else CLIMB_RATE

            # ---- 保险：偏中线太多 ----
            if s.side_ok and abs(s.e_side) > SIDE_ABORT_M:
                raise FlightAbort(f"偏走廊中线 {s.e_side*100:+.0f}cm "
                                  f"(left={s.left:.2f} right={s.right:.2f})")

            tick += 1
            if tick % 10 == 0:
                now = time.monotonic()
                d_el = max(1e-3, now - t_prev)
                v_act = (alt - alt_prev) / d_el
                t_prev, alt_prev = now, alt
                yaw_r = math.radians(current["yaw"])
                v_right = (-current["vn"] * math.sin(yaw_r)
                        + current["ve"] * math.cos(yaw_r))
                v_fwd   = ( current["vn"] * math.cos(yaw_r)
                        + current["ve"] * math.sin(yaw_r))
                flag = ""
                if alt > 0.50 and CLIMB_RATE < 0 and v_act < abs(CLIMB_RATE) * 0.6:
                    flag = "  ⚠️爬升速率不足"
                print(f"   alt={alt:4.2f} v↑{v_act:+.3f} "
                      f"f={s.front:5.2f} b={s.back:5.2f} "
                      f"L={s.left:5.3f} R={s.right:5.3f} "
                      f"eS={s.e_side*100:+6.1f}cm r={cmd['right']:+.3f} "
                      f"v前={v_fwd*100:+5.1f} v右={v_right*100:+5.1f}cm/s "
                      f"d={cmd['down']:+.2f} {current['mode']}{flag}")

            if alt >= CLIMB_STOP_M:
                break
            if time.monotonic() - t0 > TAKEOFF_TIMEOUT_S:
                raise FlightAbort(f"起飞超时：{TAKEOFF_TIMEOUT_S:.0f}s 只到 {alt:.2f}m")
            await asyncio.sleep(CONTROL_PERIOD_S)

        print(f"   ✅ 到达 {alt:.2f}m（用时 {time.monotonic()-t0:.1f}s）")

        # ==================================================
        #  阶段②：转航向（yaw 判据，不再依赖 W）
        # ==================================================
        PHASE[0] = "turn"
        pid_side.reset()
        ctrl.pid_lat.reset()          # ★ 误差定义马上要换轴
        ctrl.pid_yaw.reset()
        cmd["forward"] = cmd["right"] = 0.0
        cmd["yawspeed"] = 0.0

        yaw0 = current["yaw"]
        tgt  = TURN_DEG * TURN_SIGN
        print(f"\n🔄 ② 转航向 Δyaw={tgt:+.0f}°  "
              f"（约 {abs(tgt)/TURN_RATE:.1f}s，起始 yaw={yaw0:+.1f}°）")
        print(f"   ⚠️ 这 {abs(tgt)/TURN_RATE:.1f}s 内前后左右都不闭环，"
              f"越短越安全")

        t_turn = time.monotonic()
        dyaw = 0.0
        while not should_stop(stop_event):
            last_t, dt = control_dt(last_t)
            s = lidar.read(dt)
            dist["front"], dist["back"] = s.front, s.back
            dist["left"],  dist["right"] = s.left,  s.right

            dyaw   = wrap180(current["yaw"] - yaw0)
            remain = tgt - dyaw

            if abs(remain) <= TURN_TOL_DEG:
                cmd["yawspeed"] = 0.0
                break

            if time.monotonic() - t_turn > TURN_TIMEOUT_S:
                raise FlightAbort(f"转向超时：Δyaw={dyaw:+.1f}°"
                                  f"（应到 {tgt:+.0f}°）")

            # P 律：远处满速，近处自动减速
            cmd["yawspeed"] = max(-TURN_RATE,
                                  min(TURN_RATE, TURN_GAIN * remain))
            cmd["forward"] = cmd["right"] = 0.0

            tick += 1
            if tick % 10 == 0:
                print(f"   转 {time.monotonic()-t_turn:4.1f}s "
                      f"Δyaw={dyaw:+6.1f}° → {tgt:+.0f}°  "
                      f"f={s.front:5.3f} b={s.back:5.3f} "
                      f"W={s.front + s.back:5.3f}  yr={cmd['yawspeed']:+6.1f}")

            await asyncio.sleep(CONTROL_PERIOD_S)

        # ---- 转完只用 W 做一次合理性检查 ----
        w_now = s.front + s.back
        print(f"   ✅ 转完 Δyaw={dyaw:+.1f}°  W={w_now:.3f}"
              f"（应≈{W_NOMINAL_M}，实测含扇区平均约 2.54）")
        if not math.isfinite(w_now) or abs(w_now - W_NOMINAL_M) > W_SANITY_M:
            raise FlightAbort(f"转完 W={w_now:.2f} 异常 → 没转正或有遮挡，"
                              f"不敢进前后环")

        # ==================================================
        #  阶段③：前后环接管 + 等横向到位
        # ==================================================
        PHASE[0] = "hold"
        print(f"\n🎯 ③ 前后环接管  目标 e_lat={E_LAT_TARGET_M:+.2f}m  "
              f"|e|<{READY_ELAT_M*100:.0f}cm")
        print(f"   ⚠️ 从这一拍起，那个'前后漂'就变成'朝墙漂'了")

        t_r = time.monotonic()
        ready = False
        while not should_stop(stop_event) and \
                time.monotonic() - t_r < READY_TIMEOUT_S:
            last_t, dt = control_dt(last_t)
            alt = -current["down"]
            s = lidar.read(dt)
            dist["front"], dist["back"] = s.front, s.back
            dist["left"],  dist["right"] = s.left,  s.right

            fwd, yr, _, _ = ctrl.update(s, alt, dt)
            cmd["forward"], cmd["yawspeed"] = fwd, yr
            cmd["right"] = 0.0
            alt_min_seen = min(alt_min_seen, alt)

            if math.isfinite(ctrl.e) and abs(ctrl.e) < READY_ELAT_M \
                    and alt > LIDAR_MIN_ALT_M:
                ready = True
                break
            await asyncio.sleep(CONTROL_PERIOD_S)

        print(f"   {'✅ 到位' if ready else '⚠️ 超时'}: "
              f"e={ctrl.e*100:+.1f}cm  f={ctrl.front_used:.3f}  "
              f"用时 {time.monotonic()-t_r:.1f}s")
        if not ready:
            raise FlightAbort(f"横向到位超时（e={ctrl.e*100:+.1f}cm）")

        # ==================================================
        #  阶段④：侧飞
        # ==================================================
        PHASE[0] = "side"
        print(f"\n➡️  ④ 侧飞 {V_SIDE:.2f}m/s × {SIDE_TIME_S:.0f}s "
              f"({V_SIDE*SIDE_TIME_S:.2f}m) 方向{'右' if SIDE_DIR > 0 else '左'}")
        print(f"   ⚠️ 确认侧飞方向 {V_SIDE*SIDE_TIME_S + 1.5:.1f}m 内无障碍")

        t_sd = time.monotonic()
        while not should_stop(stop_event) and \
                time.monotonic() - t_sd < SIDE_TIME_S:
            last_t, dt = control_dt(last_t)
            alt = -current["down"]
            el = time.monotonic() - t_sd
            alt_min_seen = min(alt_min_seen, alt)

            s = lidar.read(dt)
            dist["front"], dist["back"] = s.front, s.back
            dist["left"],  dist["right"] = s.left,  s.right

            fwd, yr, f_ok, lost = ctrl.update(s, alt, dt)
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

            tick += 1
            if tick % 10 == 0:
                w = s.front + s.back
                ey = (math.degrees(s.yaw_err)
                      if math.isfinite(s.yaw_err) else float("nan"))
                yaw_r = math.radians(current["yaw"])
                v_fwd = ( current["vn"] * math.cos(yaw_r)
                        + current["ve"] * math.sin(yaw_r))
                v_right = (-current["vn"] * math.sin(yaw_r)
                           + current["ve"] * math.cos(yaw_r))
                print(f"  侧飞{el:5.1f}s(走{V_SIDE*el:4.2f}m) "
                      f"f={s.front:5.3f} b={s.back:5.3f} W={w:5.3f}"
                      f"[{ctrl.w_state}] e={ctrl.e*100:+6.1f}cm "
                      f"fwd={cmd['forward']:+.3f} "
                      f"v前={v_fwd*100:+5.1f} v右={v_right*100:+5.1f}cm/s "
                      f"alt={alt:4.2f}")

            await asyncio.sleep(CONTROL_PERIOD_S)

        if side_abort:
            raise FlightAbort(f"侧飞中止: {side_abort}")

    except FlightAbort as e:
        print(f"\n🛑 主动中止: {e}")
    except Exception as e:
        print(f"\n❌ 飞行阶段异常: {type(e).__name__}: {e}")
    finally:
        pilot_takeover = pilot_has_control()
        mode_before = current.get("mode", "UNKNOWN")
        print(f"🛬 收尾：mode={mode_before} pilot_takeover={pilot_takeover}")

        stop_event.set()
        for t in (alt_task, state_task, guard):
            t.cancel()
        await asyncio.gather(alt_task, state_task, guard, return_exceptions=True)

        cmd["forward"] = cmd["right"] = cmd["down"] = cmd["yawspeed"] = 0.0

        try:
            await asyncio.wait_for(send_task, timeout=SEND_STOP_TIMEOUT_S)
        except Exception:
            send_task.cancel()
            await asyncio.gather(send_task, return_exceptions=True)

        try:
            await asyncio.wait_for(drone.offboard.stop(), timeout=CMD_TIMEOUT_S)
        except Exception as e:
            print(f"⚠️ offboard.stop: {type(e).__name__}")

        if offboard_started and not pilot_takeover:
            try:
                await asyncio.wait_for(drone.action.land(), timeout=CMD_TIMEOUT_S)
                print("✅ 已发送降落")
            except Exception as e:
                print(f"⚠️ land: {type(e).__name__}")
                print("⚠️ 飞机可能还在空中，立刻遥控接管！")
        else:
            print("⚠️ 已接管 → 不自动降落")

        if isinstance(lidar, RealLidar):
            lidar.stop()

        print("\n📊 本次数据：")
        if math.isfinite(ctrl.f_last):
            print(f"   front 结束 {ctrl.f_last:.3f}  范围 "
                  f"{ctrl.f_min:.3f} ~ {ctrl.f_max:.3f}")
            print(f"   跳变被拒 {ctrl.rej} 次 | 宽度异常 {ctrl.n_bad} 次")
        if alt_min_seen < 90:
            print(f"   全程最低高度 {alt_min_seen:.2f}m")
        print("✅ 完成")


if __name__ == "__main__":
    try:
        asyncio.run(fly())
    except Exception as e:
        print(f"\n❌ 顶层异常: {type(e).__name__}: {e}")
        print("⚠️ 若飞机还在空中，立刻用遥控器切到 POSCTL 接管！")