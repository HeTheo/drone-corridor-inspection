#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
雷达数据源（统一接口，两种实现）

为什么要有这个文件？
    控制逻辑只关心"墙距/偏角是多少"，不关心它从哪来。
    仿真用 FakeLidar（伪造）、真机用 RealLidar（真雷达）。
    这样调 PID 时，雷达不是你的变量 —— 换数据源只改一行。

统一接口：两种类都有 read(dt) -> LidarSample
"""
import math
import threading
from collections import deque
from dataclasses import dataclass

try:
    import serial
except ImportError:
    serial = None          # 只跑 FakeLidar 时，没装 pyserial 也行


# ============================================================
#  统一返回结构（控制逻辑只认这个）
# ============================================================
@dataclass
class LidarSample:
    front: float     # 机头方向(0°) 到墙的距离（米）
    back: float      # 机尾方向(180°) 到墙的距离（米）
    yaw_err: float   # 机头与"墙法线"的夹角（弧度）；0 = 机头正对墙
    ok: bool         # 数据是否有效

    @property
    def e_lat(self) -> float:
        """横向误差（米）。>0 表示偏向机头那一面墙"""
        return (self.front - self.back) / 2.0


# ============================================================
#  实现 1：FakeLidar —— 仿真用（伪造墙距）
# ============================================================
class FakeLidar:
    """
    假设一条走廊，用飞机当前坐标反算墙距。
    不需要真雷达，也不需要 Gazebo 里搭走廊。

    仿真坐标约定：
        · 走廊沿"东"方向延伸，飞机向东巡航
        · 走廊中心在 north = center_north
        · 机头朝北(yaw≈0) → 正对北墙，此时飞机的"右侧"就是东 ✅
    """
    def __init__(self, current, width=4.0, center_north=0.5,
                 drift_m_s=0.0, wall_yaw_deg=0.0):
        self.current = current            # 共享状态（telemetry 任务刷新）
        self.half = width / 2.0
        self.center = center_north
        self.drift = drift_m_s            # >0 时"走廊"自己慢慢移，模拟持续扰动
        self.wall_yaw_deg = wall_yaw_deg  # 机头正对墙时应有的 yaw

    def read(self, dt=0.05):
        n = self.current.get("north", 0.0)
        yaw = self.current.get("yaw", 0.0)

        self.center += self.drift * dt     # 走廊缓慢漂移（默认 0，不动）

        offset = n - self.center           # 飞机相对走廊中心的横向偏移
        d_front = self.half - offset       # 到北墙
        d_back = self.half + offset        # 到南墙

        yaw_err = math.radians(self.wall_yaw_deg - yaw)
        ok = d_front > 0.1 and d_back > 0.1
        return LidarSample(d_front, d_back, yaw_err, ok)


# ============================================================
#  实现 2：RealLidar —— 真机用
# ============================================================
PROFILES = {
    "N10":  dict(baud=230400, header=b"\xA5\x5A\x3A", frame=55,  n=16, end_off=52),
    "N10P": dict(baud=460800, header=b"\xA5\x5A\x6C", frame=105, n=32, end_off=102),
}


def _parse_frame(d, n, end_off):
    """一帧 -> [(角度°, 距离mm), ...]（和你 Lidar_scan.py 里一致）"""
    start = (d[2]*256 + d[3]) / 100.0
    end = (d[end_off]*256 + d[end_off+1]) / 100.0
    if end < start:
        end += 360.0
    step = (end - start) / (n - 1) if n > 1 else 0.0
    return [((start + i*step) % 360.0, d[4+i*3]*256 + d[4+i*3+1]) for i in range(n)]


def _sector(buf, center, half):
    """筛出某个角度扇区内的点"""
    return [(a, dd) for a, dd in buf
            if abs((a - center + 180) % 360 - 180) <= half and dd > 0]


def _median_m(pts):
    """扇区中位数（mm -> m）"""
    if not pts:
        return float("nan")
    vals = sorted(dd for _, dd in pts)
    return vals[len(vals)//2] / 1000.0


def _median_deque(dq):
    if not dq:
        return float("nan")
    vals = sorted(dq)
    return float(vals[len(vals)//2])


class RealLidar:
    """
    后台线程读串口，read() 永远返回"最新一圈"，不阻塞控制环。
    （这一点很重要：控制环 20Hz，不能被串口读卡住）
    """

    # ⚠️ 下面几个角度必须按你的实际安装实测确认
    FRONT_DEG = 0          # 机头方向
    BACK_DEG = 180         # 机尾方向
    WALL_HALF = 20         # 算墙距的扇区半宽（度）
    YAW_DELTA = 15         # 算 yaw 的两个子扇区中心（±15°）
    YAW_HALF = 5           # 子扇区半宽
    YAW_SIGN = -1.0        # 实测确认；若 yaw 修正方向反了，改成 -1.0
    SMOOTH_N = 5           # 时间平滑窗口（近 5 圈取中位数）

    def __init__(self, port, model="N10"):
        if serial is None:
            raise RuntimeError("真雷达需要先装 pyserial：pip install pyserial")
        self.port, self.model = port, model
        self._latest = LidarSample(float("nan"), float("nan"), float("nan"), False)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._hf = deque(maxlen=self.SMOOTH_N)
        self._hb = deque(maxlen=self.SMOOTH_N)
        self._hy = deque(maxlen=self.SMOOTH_N)
        threading.Thread(target=self._loop, daemon=True).start()

    def read(self, dt=0.05):
        with self._lock:
            return self._latest        # 只读最新值，立刻返回

    def stop(self):
        self._stop.set()

        # ---------------- 后台线程 ----------------
    def _loop(self):
        p = PROFILES[self.model]
        try:
            ser = serial.Serial(self.port, p["baud"], timeout=1)
        except Exception as e:
            print(f"❌ [Lidar] 串口打开失败 {self.port}: {e}")
            return

        buf, prev = [], None
        try:
            while not self._stop.is_set():
                if ser.read(1) != p["header"][0:1]: continue
                if ser.read(1) != p["header"][1:2]: continue
                if ser.read(1) != p["header"][2:3]: continue
                d = ser.read(p["frame"])
                if len(d) != p["frame"]:
                    continue

                pts = _parse_frame(d, p["n"], p["end_off"])
                buf.extend([(a, dd) for a, dd in pts if dd > 0])

                cur = pts[0][0]
                if prev is not None and cur < prev - 100 and len(buf) >= 50:
                    self._publish(buf)      # 转完一圈 -> 计算并发布
                    buf = []
                prev = cur

        except Exception as e:
            print(f"❌ [Lidar] 读串口异常，线程退出: {type(e).__name__}: {e}")
        finally:
            try:
                ser.close()
            except Exception:
                pass

    def _publish(self, buf):
        f = _median_m(_sector(buf, self.FRONT_DEG, self.WALL_HALF))
        b = _median_m(_sector(buf, self.BACK_DEG, self.WALL_HALF))
        dp = _median_m(_sector(buf, +self.YAW_DELTA, self.YAW_HALF))
        dm = _median_m(_sector(buf, -self.YAW_DELTA, self.YAW_HALF))

        if not math.isnan(f): self._hf.append(f)
        if not math.isnan(b): self._hb.append(b)
        ff, bb = _median_deque(self._hf), _median_deque(self._hb)

        # yaw 偏角（小角度近似）：
        #   机头正对墙时 d(+Δ) == d(-Δ)；偏了就会一大一小
        if math.isnan(dp) or math.isnan(dm) or (dp + dm) < 1e-6:
            ye = float("nan")
        else:
            ye = self.YAW_SIGN * (dp - dm) / (
                math.tan(math.radians(self.YAW_DELTA)) * (dp + dm))
            self._hy.append(ye)
            ye = _median_deque(self._hy)

        ok = not (math.isnan(ff) or math.isnan(bb))
        with self._lock:
            self._latest = LidarSample(ff, bb, ye, ok)


# ============================================================
#  工厂：仿真/真机切换只改这一行
# ============================================================
def make_lidar(use_fake, current=None, port=None, model="N10", **kw):
    if use_fake:
        return FakeLidar(current, **kw)
    return RealLidar(port, model)