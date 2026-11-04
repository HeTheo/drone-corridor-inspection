"""
四阶段 + 视觉：起飞 → 转航向 → 中点归位 → 前移到 1.70m → 侧飞+视觉触发

阶段划分（每个阶段只有一个通道写入者）：
    ① climb   起飞爬升 + 【左右居中环】(cmd['right'])
    ② turn    转航向 88°，只给 yawspeed，只用 yaw 遥测判据（不信 W）
    ③a mid    前后环接管，先归位到【走廊中点】(e_lat = 0)  ← 检查点
    ③b tgt    斜坡把定位点改到 front = 1.70m（e_lat = +0.455）
    ④ side    侧飞 + 视觉：最近 6 帧里 3 帧看到 Fire_hydrant → 停下悬停
    ⑤ inspect 原地悬停 3s，累积各帧检出 → 打印 → 降落

⚠️ 前提：Lidar_source.py 已打左右扇区补丁（side_ok 正常）
⚠️ 前提：vision/ 目录下 best.onnx 存在，摄像头 /dev/video0 没被占用
⚠️ 起飞前先单独跑 vision/Yolo_test.py 确认能认出消防栓

停止：按回车（不要 Ctrl+C）
"""

import asyncio
import json                  # ★ 新增
import math
import multiprocessing as mp
import os
import queue
import socket                # ★ 新增
import sys
import threading
import time
from collections import deque

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

from Lidar_source import RealLidar, make_lidar

# ---- 视觉子进程：Pi 上目录是 Vision/，工作区里是 vision/ ----
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

# ============================================================
#  配置
# ============================================================
ADDRESS     = "serial:///dev/pixhawk:57600"
LIDAR_PORT  = "/dev/wheeltec_lidar"
LIDAR_MODEL = "N10"

# ---------- 走廊几何 ----------
W_NOMINAL_M = 2.49      # 走廊宽度标定值
W_TOL_M     = 0.15      # |left+right−W| 或 |front+back−W| 超这个 → 读数不可信

# ---------- 高度 ----------
ALT_TARGET_M      = 1.20    # 目标悬停高度 m
CLIMB_RATE        = -0.35   # 爬升速度命令（NED，负=向上）
CLIMB_STOP_M      = 1.30    # 爬到这高度就停爬（hold 后回落到 1.20）
TAKEOFF_TIMEOUT_S = 30.0

# ---------- 阶段①：起飞时左右居中 ----------
SIDE_ON_ALT_M   = 0.15   # 高于这个才开左右环（贴地读数不可信）
KP_SIDE         = 0.65   # ⚠️ 不要再加：起飞被甩时它已经饱和，加 KP 只会更早饱和
KI_SIDE         = 0.05
KV_SIDE         = 1.80   # ★ 速度阻尼（用 EKF 机体右速度，别用差分）
SIDE_SIGN       = +1.0   # Lidar_source 里 LEFT_DEG/RIGHT_DEG 已对，保持 +1
MAX_RIGHT       = 0.30   # 起跳瞬间有 16~45cm/s 的冲量，唯一的应对是更大的速度权限
SIDE_DEADBAND_M = 0.04   # e_side 死区（= 实际位移 2cm）
SIDE_ABORT_M    = 0.90   # 偏离中线超这个 → 中止（实际偏移 45cm，离墙还有 80cm）

# ---------- 阶段①b：转向前的稳定等待 ----------
SETTLE_VEL_MPS      = 0.15   # |v| 小于这个才算"停住"（4cm/s）
SETTLE_ELAT_M       = 0.20   # |e_side| 小于这个才算"居中"（= 实际 4cm）
SETTLE_ALT_M        = 0.08   # |alt − 1.30| 小于这个才算"高度到位"
SETTLE_HOLD_S       = 0.5    # 上面三条要【连续】成立这么久
STABILIZE_TIMEOUT_S = 10.0   # 等这么久还不稳 → 中止，不要硬转

# ---------- 阶段②：转航向（yaw 判据）----------
TURN_RATE      = 60.0    # deg/s
TURN_SIGN      = -1.0    # -1 = 左转
TURN_DEG       = 88.0    # 目标转角（留 2° 给阶段③ 的航向环收）
TURN_TOL_DEG   = 3.0
TURN_GAIN      = 3.0     # P 律，末端自动减速
TURN_TIMEOUT_S = 6.0
W_SANITY_M     = 0.25    # 转完后 |W−2.49| 超这个 → 中止
TURN_VDAMP     = 1.20   # 转向期间的速度阻尼增益（0 = 关掉）
TURN_VDAMP_MAX = 0.20

# ---------- 阶段③④：前后环 ----------
FRONT_TARGET_M  = 1.65                       # ★ 之前标定过的工作位置
E_LAT_TARGET_M  = FRONT_TARGET_M - W_NOMINAL_M / 2.0   # = +0.455
MID_ELAT_M      = 0.00                       # 阶段③a 的中点目标
MID_TOL_M       = 0.08
MID_TIMEOUT_S   = 20.0
TGT_TOL_M       = 0.06
TGT_TIMEOUT_S   = 25.0
TGT_RAMP_S      = 2.0     # 目标从 0 斜坡到 0.455 的时间，避免阶跃

KP_LAT, KI_LAT, KD_LAT = 0.70, 0.10, 0.0   # 上一轮验证：e_ss = drift/KP
MAX_FORWARD     = 0.40
WALL_ABORT_M    = 0.85   # e 的限幅。换算：e = f − 1.245 − E_TARGET
                         #  阶段③(E=0)：   |e|<0.85 → f>0.395（guard 0.90 先触发）
                         #  阶段④(E=0.455)：|e|<0.85 → f>0.85 （和 guard 几乎重合）
READY_ELAT_M    = 0.08   # 兼容旧名，实际用 MID_TOL_M / TGT_TOL_M
READY_TIMEOUT_S = 20.0

# ---------- 航向环 ----------
USE_YAW_LOOP     = True
KP_YAW, KI_YAW, KD_YAW = 3.0, 0.0, 0.0   # 输入单位必须是【度】
MAX_YAWRATE      = 10.0
YAW_DEADBAND_DEG = 1.5
YAW_ABORT_DEG    = 25.0

# ---------- 阶段④：侧飞 ----------
V_SIDE      = 0.44   # 侧飞速度 m/s
SIDE_TIME_S = 240.0   # 侧飞时长 s（上限）
SIDE_DIR    = +1.0   # +1 右 / -1 左

# ---------- 全程保护 ----------
SIDE_STOP_FRONT  = 0.70   # front 低于此值 → 立即中止
BACK_MIN_SAFE    = 0.50   # ⚠️ 0.65→0.55。front=1.70 时 back≈0.79，留 24cm 余量
LIDAR_MIN_ALT_M  = 0.50
MAX_JUMP_M       = 0.25
MAX_BAD_CNT      = 8
LOST_TIMEOUT_S   = 1.0

# ---------- 视觉 ----------
VIS_ENABLE       = True
TARGET_CLASS     = "Fire_hydrant"
REQUIRED_CLASSES = ("Valve", "Fire_hose", "Fire_nozzle", "Rubber_hose")
IGNORE_CLASSES   = ("Fire_extinguisher",)
HYDRANT_WINDOW_N = 6      # 滑动窗口：最近 6 个视觉帧里
HYDRANT_HITS     = 1      # 有 3 帧看到就触发（不必连续）
VISION_STALE_S   = 1.0    # 视觉数据超过这么久算失效
VISION_START_TIMEOUT_S = 20.0
DECEL_S          = 0.1    # 触发后右侧速度斜坡降到 0
INSPECT_S        = 6.0    # 触发后原地悬停时长
# ★ 识别满 N 个消防栓就停下悬停 + 降落。0 = 不按数量停（飞满 SIDE_TIME_S）
TARGET_HYDRANT_COUNT = 3
RESUME_COOLDOWN_S    = 12.0   # 两次触发最小间隔（防同一个柜子重复计数）
                              # 0.30m/s × 10s = 3.0m > FOV 覆盖的约 1.6m

# ---------- 网页可视化（Viewer/viewer_lite.py）----------
VIEWER_ENABLE   = True
VIEWER_HOST     = "192.168.126.177"   # ⚠️ 改成跑 viewer_lite.py 那台机器的 IP
VIEWER_PORT     = 9999
VIEWER_PERIOD_S = 0.10             # 10Hz
VIEWER_WALL_N   = 1.245            # ⚠️ 必须和 Graound_view_2.html 的 WALL_N 完全一样

# ---------- 时序 ----------
CONTROL_PERIOD_S    = 0.05
SEND_PERIOD_S       = 0.05
MAX_DT_S            = 0.20
HEALTH_TIMEOUT_S    = 30.0
SEND_STOP_TIMEOUT_S = 2.0
CMD_TIMEOUT_S       = 5.0

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
MAPSTAT = {"hyd": 0}     # 已确认的消防栓个数（侧飞循环和 viewer_task 都要读）
PHASE = ["init"]   # init / climb / turn / mid / tgt / side / inspect / stop

# 网页锚定状态：以"侧飞起点"为原点，机身右向为沿走廊方向
VSTATE = {"ready": False, "n0": 0.0, "e0": 0.0,
          "fx": 1.0, "fy": 0.0, "yaw0": 0.0, "north": 0.0}


class ViewerLink:
    """UDP → viewer_lite.py → WebSocket → 网页"""
    def __init__(self, host, port):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.warned = False

    def send(self, payload):
        try:
            self.sock.sendto(
                json.dumps(payload, separators=(",", ":")).encode("utf-8"),
                self.addr)
            self.warned = False
        except Exception as e:
            if not self.warned:
                print(f"⚠️ viewer 发送失败: {type(e).__name__}: {e}")
                self.warned = True


viewer = ViewerLink(VIEWER_HOST, VIEWER_PORT)


class FlightAbort(RuntimeError):
    """主动中止：进 finally 收尾/降落"""


class MissionDone(RuntimeError):
    """任务正常完成（识别到消防栓）：进 finally 收尾/降落"""


# ============================================================
#  PID
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
    """机头对墙之后用：前后双墙 + 航向。

    ⚠️ 目标 e_lat 现在由 update() 传参，不再读全局 —— 因为阶段③a / ③b
       要用两个不同的目标（0 和 +0.455）。
    """

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
        """双墙一致性校验：W 对不上就判 bad"""
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

    def update(self, s, alt, dt, e_target):
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
            self.e = (f_use - s.back) / 2.0 - e_target
            ok = (abs(self.e) < WALL_ABORT_M) and (alt > LIDAR_MIN_ALT_M)

                # 丢墙时指令直接归零：宁可"不推"，也不要"按歪掉的方向推"
        fwd = self.pid_lat.update(self.e, dt) if ok else 0.0
        if not ok:
            self.pid_lat.reset()      # 积分清掉，避免 windup

        # ---- 航向环 ----
        reliable = (self.w_state == "ok")
        if (USE_YAW_LOOP and reliable
                and math.isfinite(s.yaw_err) and alt > LIDAR_MIN_ALT_M):
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


def body_v(vn, ve, yaw_deg):
    """世界 NED 速度 → 机体 (前, 右)"""
    y = math.radians(yaw_deg)
    return ( vn * math.cos(y) + ve * math.sin(y),
            -vn * math.sin(y) + ve * math.cos(y))


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
#  视觉：取最新结果
# ============================================================
_VIS = {"v": None}


def poll_vision(vq):
    """20Hz 轮询 15fps 数据，队列空了必须返回上一条，
    否则 1/3 的 tick 会被判成"没视觉"，触发窗口永远凑不满。"""
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
    """返回缺失类列表（IGNORE_CLASSES 不计入）"""
    return [c for c in REQUIRED_CLASSES if cnt.get(c, 0) <= 0]


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


# 高度环 P（定义在这里，靠运行时查找生效）
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
                  f"（应远大于 {W_NOMINAL_M}）  "
                  f"left+right={s.left + s.right:.3f}（应≈{W_NOMINAL_M}）")
            return lidar
        if time.monotonic() - t > 10:
            print("❌ 10 秒没拿到完整数据（含左右）——Lidar_source 补丁打了吗？")
            if isinstance(lidar, RealLidar):
                lidar.stop()
            raise SystemExit(1)
        time.sleep(0.1)


async def guard_task(stop_event):
    """全程墙距保护（只读 dist 快照，自己不读雷达）
    ⚠️ 雷达只有一个读者（主循环），两个消费者会把事件循环卡死"""
    while not should_stop(stop_event):
        f, b = dist["front"], dist["back"]
        if math.isfinite(f) and f < SIDE_STOP_FRONT:
            print(f"\n🛑 离前墙过近 {f:.2f}m → 立即中止")
            stop_event.set()
        elif math.isfinite(b) and b < BACK_MIN_SAFE:
            print(f"\n🛑 离后墙过近 {b:.2f}m → 立即中止")
            stop_event.set()
        await asyncio.sleep(0.05)


async def viewer_task(stop_event):
    """10Hz 上报位置给网页。锚定之前什么都不发（网页保持 DEMO）。
    ⚠️ 故意不挂 _watch_task：UDP 发不出去不应该中止飞行。"""
    while not should_stop(stop_event):
        if VSTATE["ready"]:
            f = dist["front"]
            if math.isfinite(f):
                VSTATE["north"] = VIEWER_WALL_N - f      # 离墙距离直接用雷达

            dn = current["north"] - VSTATE["n0"]
            de = current["east"]  - VSTATE["e0"]
            along = dn * VSTATE["fx"] + de * VSTATE["fy"]   # 投影到走廊方向

            viewer.send({
                "type":  "pos",
                "north": round(VSTATE["north"], 3),
                "east":  round(along, 3),
                "down":  round(current["down"], 3),
                "yaw":   round(wrap180(current["yaw"] - VSTATE["yaw0"]), 1),
                "hyd":   MAPSTAT["hyd"],
            })
        await asyncio.sleep(VIEWER_PERIOD_S)


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
        print("   检查 LEFT_DEG / RIGHT_DEG，或飞机不在走廊中间。")
        lidar.stop()
        return
    if abs(s0.left - s0.right) > 0.30:
        print(f"⚠️ 起飞点不居中（偏差 {(s0.right - s0.left)*100:+.0f}cm）")

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

    # ---- 起飞前等视觉就绪 ----
    if VIS_ENABLE:
        if start_vision_process is None:
            print("❌ 视觉模块没导入成功（或把 VIS_ENABLE 设 False）")
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

    vis_task = None
    if VIEWER_ENABLE:
        vis_task = asyncio.create_task(viewer_task(stop_event))
        # ⚠️ 故意不挂 _watch_task：UDP 失败不该中止飞行

    ctrl     = WallController()
    pid_side = PID(KP_SIDE, KI_SIDE, 0.0, 0.50, MAX_RIGHT)   # ⚠️ i_limit=1.0
    tick = 0
    side_abort = ""
    alt_min_seen = 99.0
    inspect_log = []
    last_t = time.monotonic()

    try:
        # ==================================================
        #  阶段①：起飞爬升 + 左右居中
        # ==================================================
        print(f"\n🛫 ① 起飞（{CLIMB_STOP_M:.2f}m）"
              f"  左右居中环 KP={KP_SIDE} 限幅=±{MAX_RIGHT}")
        t0 = time.monotonic()
        last_t = time.monotonic()
        t_prev, alt_prev = t0, -current["down"]

        while not should_stop(stop_event):
            last_t, dt = control_dt(last_t)
            alt = -current["down"]
            s = lidar.read(dt)
            vf, vr = body_v(current["vn"], current["ve"], current["yaw"])   # ★ 提前

            dist["front"], dist["back"] = s.front, s.back
            dist["left"],  dist["right"] = s.left,  s.right

            side_valid = (s.side_ok
                          and abs((s.left + s.right) - W_NOMINAL_M) <= W_TOL_M)

            if side_valid and alt > SIDE_ON_ALT_M:
                e_side = s.right - s.left        # >0 = 偏左 → 往右
                if abs(e_side) < SIDE_DEADBAND_M:
                    e_side = 0.0
                u = pid_side.update(e_side, dt) - KV_SIDE * vr
                cmd["right"] = SIDE_SIGN * max(-MAX_RIGHT, min(MAX_RIGHT, u))            
            else:
                pid_side.reset()
                cmd["right"] = 0.0

            cmd["forward"] = 0.0
            cmd["yawspeed"] = 0.0
            cmd["down"] = 0.0 if alt >= CLIMB_STOP_M else CLIMB_RATE

            if s.side_ok and abs(s.e_side) > SIDE_ABORT_M:
                raise FlightAbort(f"偏走廊中线 {s.e_side*100:+.0f}cm "
                                  f"(left={s.left:.2f} right={s.right:.2f})")

            tick += 1
            if tick % 10 == 0:
                now = time.monotonic()
                d_el = max(1e-3, now - t_prev)
                v_act = (alt - alt_prev) / d_el
                t_prev, alt_prev = now, alt
                vf, vr = body_v(current["vn"], current["ve"], current["yaw"])
                flag = ""
                if alt > 0.50 and CLIMB_RATE < 0 and v_act < abs(CLIMB_RATE) * 0.6:
                    flag = "  ⚠️爬升速率不足"
                print(f"   alt={alt:4.2f} v↑{v_act:+.3f} "
                      f"f={s.front:5.2f} b={s.back:5.2f} "
                      f"L={s.left:5.3f} R={s.right:5.3f} "
                      f"eS={s.e_side*100:+6.1f}cm r={cmd['right']:+.3f} "
                      f"v前={vf*100:+5.1f} v右={vr*100:+5.1f}cm/s "
                      f"d={cmd['down']:+.2f} {current['mode']}{flag}")

            if alt >= CLIMB_STOP_M:
                break
            if time.monotonic() - t0 > TAKEOFF_TIMEOUT_S:
                raise FlightAbort(f"起飞超时：{TAKEOFF_TIMEOUT_S:.0f}s 只到 {alt:.2f}m")
            await asyncio.sleep(CONTROL_PERIOD_S)

        print(f"   ✅ 到达 {alt:.2f}m（用时 {time.monotonic()-t0:.1f}s）")

        # ==================================================
        #  阶段①b：起飞后先稳住，再转向
        # ==================================================
        PHASE[0] = "settle"          # ★ 从这一刻起 alt_hold_task 接管 cmd['down']
        print(f"\n⏸️  ①b 起飞后稳定："
              f"|v|<{SETTLE_VEL_MPS*100:.0f}cm/s  "
              f"|eS|<{SETTLE_ELAT_M*100:.0f}cm  "
              f"|alt−{ALT_TARGET_M:.2f}|<{SETTLE_ALT_M*100:.0f}cm  "
              f"连续 {SETTLE_HOLD_S:.1f}s")

        t_st = time.monotonic()
        hold_since = None
        while not should_stop(stop_event):
            last_t, dt = control_dt(last_t)
            alt = -current["down"]
            s = lidar.read(dt)

            dist["front"], dist["back"] = s.front, s.back
            dist["left"],  dist["right"] = s.left,  s.right

            vf, vr = body_v(current["vn"], current["ve"], current["yaw"])
            v_all = math.hypot(vf, vr)

            # ---- 左右居中环继续跑（不要停，停了就会慢慢飘走）----
            side_valid = (s.side_ok
                          and abs((s.left + s.right) - W_NOMINAL_M) <= W_TOL_M)
            if side_valid and alt > SIDE_ON_ALT_M:
                e_side = s.right - s.left
                if abs(e_side) < SIDE_DEADBAND_M:
                    e_side = 0.0
                u = pid_side.update(e_side, dt) - KV_SIDE * vr
                cmd["right"] = SIDE_SIGN * max(-MAX_RIGHT, min(MAX_RIGHT, u))
            else:
                pid_side.reset()
                cmd["right"] = 0.0

            # ---- 前后、航向 一律锁死；down 交给 alt_hold_task ----
            cmd["forward"] = 0.0
            cmd["yawspeed"] = 0.0

            calm = (math.isfinite(s.e_side)
                    and abs(s.e_side) < SETTLE_ELAT_M
                    and v_all < SETTLE_VEL_MPS
                    and abs(alt - ALT_TARGET_M) < SETTLE_ALT_M)

            if calm:
                if hold_since is None:
                    hold_since = time.monotonic()
                elif time.monotonic() - hold_since >= SETTLE_HOLD_S:
                    break
            else:
                hold_since = None

            tick += 1
            if tick % 10 == 0:
                print(f"   稳 {time.monotonic()-t_st:4.1f}s alt={alt:4.2f} "
                      f"eS={s.e_side*100:+6.1f}cm "
                      f"v前={vf*100:+5.1f} v右={vr*100:+5.1f} "
                      f"|v|={v_all*100:4.1f}cm/s r={cmd['right']:+.3f} "
                      f"{'✔' if calm else '…'}")

            if time.monotonic() - t_st > STABILIZE_TIMEOUT_S:
                raise FlightAbort(
                    f"起飞后 {STABILIZE_TIMEOUT_S:.0f}s 仍不稳："
                    f"|v|={v_all*100:.0f}cm/s eS={s.e_side*100:+.0f}cm "
                    f"alt={alt:.2f}m  → 不转向，直接降落")
            await asyncio.sleep(CONTROL_PERIOD_S)

        print(f"   ✅ 已稳定（用时 {time.monotonic()-t_st:.1f}s）"
              f" |v|={v_all*100:.1f}cm/s eS={s.e_side*100:+.1f}cm"
              f" → 允许转向")

        # ==================================================
        #  阶段②：转航向（yaw 判据）
        # ==================================================
        PHASE[0] = "turn"
        pid_side.reset()
        ctrl.pid_lat.reset()
        ctrl.pid_yaw.reset()
        cmd["forward"] = cmd["right"] = 0.0
        cmd["yawspeed"] = 0.0

        yaw0 = current["yaw"]
        tgt  = TURN_DEG * TURN_SIGN
        print(f"\n🔄 ② 转航向 Δyaw={tgt:+.0f}°  "
              f"（约 {abs(tgt)/TURN_RATE:.1f}s，起始 yaw={yaw0:+.1f}°）")
        print(f"   ⚠️ 这 {abs(tgt)/TURN_RATE:.1f}s 内前后左右都不闭环，越短越安全")

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
                raise FlightAbort(f"转向超时：Δyaw={dyaw:+.1f}°（应到 {tgt:+.0f}°）")

            cmd["yawspeed"] = max(-TURN_RATE, min(TURN_RATE, TURN_GAIN * remain))
            if TURN_VDAMP > 0:
                vf, vr = body_v(current["vn"], current["ve"], current["yaw"])
                cmd["forward"] = max(-TURN_VDAMP_MAX,
                                     min(TURN_VDAMP_MAX, -TURN_VDAMP * vf))
                cmd["right"]   = max(-TURN_VDAMP_MAX,
                                     min(TURN_VDAMP_MAX, -TURN_VDAMP * vr))
            else:
                cmd["forward"] = cmd["right"] = 0.0

            tick += 1
            if tick % 10 == 0:
                print(f"   转 {time.monotonic()-t_turn:4.1f}s "
                      f"Δyaw={dyaw:+6.1f}° → {tgt:+.0f}°  "
                      f"f={s.front:5.3f} b={s.back:5.3f} "
                      f"W={s.front + s.back:5.3f}  yr={cmd['yawspeed']:+6.1f}")

            await asyncio.sleep(CONTROL_PERIOD_S)

        w_now = s.front + s.back
        print(f"   ✅ 转完 Δyaw={dyaw:+.1f}°  W={w_now:.3f}（应≈{W_NOMINAL_M}）")
        if not math.isfinite(w_now) or abs(w_now - W_NOMINAL_M) > W_SANITY_M:
            raise FlightAbort(f"转完 W={w_now:.2f} 异常 → 没转正或有遮挡")

        # ==================================================
        #  阶段③：前后环 —— 通用"归位"子流程
        # ==================================================
        async def hold_to(e_target, tol, timeout_s, label):
            """斜坡把 e_lat 拉到 e_target，等 |e−e_target| < tol"""
            nonlocal last_t, alt_min_seen
            print(f"\n🎯 {label}  e_lat→{e_target:+.3f}m "
                  f"(front≈{W_NOMINAL_M/2 + e_target:.2f}m)  "
                  f"|e|<{tol*100:.0f}cm  斜坡 {TGT_RAMP_S:.1f}s")
            t_r = time.monotonic()
            ok = False
            while not should_stop(stop_event) and \
                    time.monotonic() - t_r < timeout_s:
                last_t, dt = control_dt(last_t)
                alt = -current["down"]
                s = lidar.read(dt)
                dist["front"], dist["back"] = s.front, s.back
                dist["left"],  dist["right"] = s.left,  s.right

                k = min(1.0, (time.monotonic() - t_r) / max(1e-3, TGT_RAMP_S))
                fwd, yr, _, _ = ctrl.update(s, alt, dt, e_target * k)
                cmd["forward"], cmd["yawspeed"] = fwd, yr
                cmd["right"] = 0.0
                alt_min_seen = min(alt_min_seen, alt)

                if (k >= 1.0 and math.isfinite(ctrl.e)
                        and abs(ctrl.e) < tol and alt > LIDAR_MIN_ALT_M):
                    ok = True
                    break
                await asyncio.sleep(CONTROL_PERIOD_S)

            print(f"   {'✅ 到位' if ok else '⚠️ 超时'}: "
                  f"e={ctrl.e*100:+.1f}cm  f={ctrl.front_used:.3f}  "
                  f"用时 {time.monotonic()-t_r:.1f}s")
            return ok

        # ---- ③a：先归位到走廊中点（检查点）----
        PHASE[0] = "mid"
        ok_mid = await hold_to(MID_ELAT_M, MID_TOL_M, MID_TIMEOUT_S,
                               "③a 归位到走廊中点")
        if not ok_mid:
            raise FlightAbort(f"中点归位超时（e={ctrl.e*100:+.1f}cm）")
        if should_stop(stop_event):
            raise FlightAbort("mid 阶段收到停止")
        print(f"   ⭐ 检查点通过：转向正确、雷达可信，允许前移到 "
              f"front={FRONT_TARGET_M:.2f}m")

        # ---- ③b：前移到 front = 1.70 ----
        PHASE[0] = "tgt"
        ok_tgt = await hold_to(E_LAT_TARGET_M, TGT_TOL_M, TGT_TIMEOUT_S,
                               f"③b 前移到 front={FRONT_TARGET_M:.2f}m")
        if not ok_tgt:
            raise FlightAbort(f"前移到位超时（e={ctrl.e*100:+.1f}cm）")
        if should_stop(stop_event):
            raise FlightAbort("tgt 阶段收到停止")

        # ==================================================
        #  阶段④：侧飞 + 视觉触发
        # ==================================================
        PHASE[0] = "side"
        print(f"\n➡️  ④ 侧飞 {V_SIDE:.2f}m/s  方向{'右' if SIDE_DIR > 0 else '左'}"
              f"  最长 {SIDE_TIME_S:.0f}s  目标 {TARGET_HYDRANT_COUNT} 个 {TARGET_CLASS}")
        print(f"   ⚠️ 确认侧飞方向 {V_SIDE*SIDE_TIME_S + 1.5:.1f}m 内无障碍")

        # ---- 网页可视化：以侧飞起点为原点建坐标系 ----
        if VIEWER_ENABLE:
            VSTATE["n0"]   = current["north"]
            VSTATE["e0"]   = current["east"]
            VSTATE["yaw0"] = current["yaw"]
            _p = math.radians(current["yaw"])
            VSTATE["fx"], VSTATE["fy"] = -math.sin(_p), math.cos(_p)   # 机身右向
            VSTATE["north"] = VIEWER_WALL_N - FRONT_TARGET_M
            VSTATE["ready"] = True
            print(f"🗺️  网页锚定: P0=({VSTATE['n0']:+.2f}, {VSTATE['e0']:+.2f})  "
                  f"yaw0={VSTATE['yaw0']:+.1f}°  → 网页此刻应变为 LIVE")

        t_sd = time.monotonic()
        last_t = time.monotonic()
        hyd_hist       = deque(maxlen=HYDRANT_WINDOW_N)
        last_vis_n     = -1
        cooldown_until = 0.0
        paused_total   = 0.0

        async def do_inspect(vis_now):
            """识别到消防栓 → 停下悬停 INSPECT_S，累积各帧检出。
               横向/航向锁死（相机视角不能转），前后环保留（防止朝墙漂）。"""
            nonlocal alt_min_seen, last_t
            PHASE[0] = "inspect"
            t_i = time.monotonic()
            n0, e0 = current["north"], current["east"]
            f0, b0 = dist["front"], dist["back"]
            seen, last_n, n_print = {}, -1, 0

            print(f"🎯 触发！hyd={vis_now['counts'].get(TARGET_CLASS, 0)} "
                  f"@ {vis_now['fps']}fps → 悬停 {INSPECT_S:.1f}s")

            # 1) 右侧速度斜坡降到 0，减少刹车冲过头
            t_r = time.monotonic()
            while not should_stop(stop_event) and time.monotonic() - t_r < DECEL_S:
                k = 1.0 - (time.monotonic() - t_r) / DECEL_S
                cmd["right"]    = V_SIDE * SIDE_DIR * max(0.0, k)
                cmd["forward"]  = 0.0
                cmd["yawspeed"] = 0.0
                await asyncio.sleep(CONTROL_PERIOD_S)

            # 2) 原地悬停，累积视觉（各帧取并集）
            while not should_stop(stop_event) and time.monotonic() - t_i < INSPECT_S:
                last_t, dt = control_dt(last_t)
                alt = -current["down"]
                s = lidar.read(dt)
                dist["front"], dist["back"] = s.front, s.back
                dist["left"],  dist["right"] = s.left,  s.right

                fwd, _, _, _ = ctrl.update(s, alt, dt, E_LAT_TARGET_M)
                cmd["forward"] = fwd
                cmd["right"] = cmd["yawspeed"] = 0.0
                alt_min_seen = min(alt_min_seen, alt)

                vis = poll_vision(vq)
                if vis is not None and vis["n"] != last_n:
                    last_n = vis["n"]
                    for k, c in vis["counts"].items():
                        if c > 0 and k not in IGNORE_CLASSES:
                            seen[k] = max(seen.get(k, 0), c)

                n_print += 1
                if n_print % 5 == 0:
                    print(f"    悬停{time.monotonic()-t_i:4.1f}s alt={alt:4.2f} "
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
            if VIEWER_ENABLE and VSTATE["ready"]:
                dn2 = current["north"] - VSTATE["n0"]
                de2 = current["east"]  - VSTATE["e0"]
                viewer.send({
                    "type":    "det",
                    "east":    round(dn2 * VSTATE["fx"] + de2 * VSTATE["fy"], 3),
                    "index":   MAPSTAT["hyd"] - 1,
                    "status":  "ok" if not missing else "missing",
                    "counts":  dict(seen),
                    "missing": missing,
                })

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

            ctrl.pid_yaw.reset()        # 只清航向，前后环的积分留着抗漂
            PHASE[0] = "side"
            return time.monotonic() - t_i

        while (not should_stop(stop_event)
               and (time.monotonic() - t_sd - paused_total) < SIDE_TIME_S):
            last_t, dt = control_dt(last_t)
            alt = -current["down"]
            el = time.monotonic() - t_sd - paused_total   # 只算真正在侧飞的时间
            alt_min_seen = min(alt_min_seen, alt)

            s = lidar.read(dt)
            dist["front"], dist["back"] = s.front, s.back
            dist["left"],  dist["right"] = s.left,  s.right

            fwd, yr, _, lost = ctrl.update(s, alt, dt, E_LAT_TARGET_M)
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

            # ---- 视觉触发（有新帧才计数）----
            if VIS_ENABLE:
                now = time.monotonic()
                vis = poll_vision(vq)
                if vis is not None and vis["n"] != last_vis_n:
                    last_vis_n = vis["n"]
                    hyd_hist.append(
                        vis["counts"].get(TARGET_CLASS, 0) > 0
                        and now > cooldown_until)
                    if sum(hyd_hist) >= HYDRANT_HITS and now > cooldown_until:
                        print(f"🎯 最近 {len(hyd_hist)} 帧里 "
                              f"{sum(hyd_hist)} 帧看到 {TARGET_CLASS}")
                        hyd_hist.clear()

                        # ★ 计数放在 inspect 之前，这样 do_inspect 里能拿到序号
                        MAPSTAT["hyd"] += 1
                        print(f"🔢 第 {MAPSTAT['hyd']} / "
                              f"{TARGET_HYDRANT_COUNT or '∞'} 个")

                        t_b = time.monotonic()
                        await do_inspect(vis)
                        paused_total += time.monotonic() - t_b
                        cooldown_until = time.monotonic() + RESUME_COOLDOWN_S
                        last_t = time.monotonic()   # 别把 3s 悬停算进 PID 的 dt

                        if (TARGET_HYDRANT_COUNT
                                and MAPSTAT["hyd"] >= TARGET_HYDRANT_COUNT):
                            raise MissionDone(
                                f"已确认 {MAPSTAT['hyd']} 个 {TARGET_CLASS}"
                                f" @ front={dist['front']:.2f}m")
                        continue
            tick += 1
            if tick % 10 == 0:
                w = s.front + s.back
                ey = (math.degrees(s.yaw_err)
                      if math.isfinite(s.yaw_err) else float("nan"))
                vf, vr = body_v(current["vn"], current["ve"], current["yaw"])
                print(f"  侧飞{el:5.1f}s(走{V_SIDE*el:4.2f}m) "
                      f"f={s.front:5.3f} b={s.back:5.3f} W={w:5.3f}"
                      f"[{ctrl.w_state}] e={ctrl.e*100:+6.1f}cm "
                      f"fwd={cmd['forward']:+.3f} "
                      f"v前={vf*100:+5.1f} v右={vr*100:+5.1f}cm/s "
                      f"ye={ey:+5.1f}° yr={cmd['yawspeed']:+6.2f} "
                      f"alt={alt:4.2f} hyd={sum(hyd_hist)}")

            await asyncio.sleep(CONTROL_PERIOD_S)

        if side_abort:
            raise FlightAbort(f"侧飞中止: {side_abort}")

        print(f"\n🏁 侧飞里程跑完 {SIDE_TIME_S:.0f}s（{V_SIDE*SIDE_TIME_S:.2f}m），"
              f"未触发视觉")

    except MissionDone as e:
        print(f"\n🏁 任务完成: {e}")
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

        if vis_task is not None:
            vis_task.cancel()
            await asyncio.gather(vis_task, return_exceptions=True)

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
        if VIS_ENABLE:
            if inspect_log:
                print(f"   检视记录 {len(inspect_log)} 处：")
                for i, r in enumerate(inspect_log, 1):
                    print(f"     #{i} front={r['f']}m back={r['b']}m "
                          f"counts={r['counts']} 缺={r['missing'] or '无'} "
                          f"位移 dN={r['dN']}m dE={r['dE']}m")
            else:
                print("   （全程没触发检视）")

        vstop.set()
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
        print("⚠️ 若飞机还在空中，立刻用遥控器切到 POSCTL 接管！")
    finally:
        if vproc is not None:
            vstop.set()
            vproc.join(timeout=2.0)
            if vproc.is_alive():
                vproc.terminate()
                print("[Main] 视觉子进程已强制结束")