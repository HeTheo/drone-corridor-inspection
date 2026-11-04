#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
起飞专项：只练"起飞+悬停"，不做走廊巡航

第一次脚本飞行建议：USE_LIDAR=False
  → 先验证"发点流 → 解锁 → Offboard → 起飞 → 悬停 → 降落"整条流程
  → 雷达环关掉，一次只测一个变量
"""
import asyncio, math, time, threading
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from Lidar_source import RealLidar, make_lidar

# ==================== 配置 ====================
ADDRESS = "udpin://0.0.0.0:14540"         # 仿真地址
# ADDRESS = "serial:///dev/ttyUSB0:57600"   # 数传
# ADDRESS = "serial:///dev/pixhawk:57600"     # 机载电脑
# 机载电脑上改成："serial:///dev/pixhawk:57600"

USE_FAKE    = True          # True=FakeLidar（室外推荐）  False=真雷达
USE_LIDAR   = False         # ⭐ 第一次脚本飞行：先关掉雷达环！
LIDAR_PORT  = "/dev/wheeltec_lidar"
LIDAR_MODEL = "N10"

TAKEOFF_ALT = -2.5          # 目标高度 1.5 米
CLIMB_RATE  = -0.4          # 爬升速度（数传延迟大，慢一点）
HOLD_S      = 20.0          # 悬停 10 秒
TAKEOFF_TIMEOUT_S = 20.0    # 起飞超时保护

KP_LAT, KI_LAT, KD_LAT = 0.2, 0.0, 0.0
KP_YAW, KI_YAW, KD_YAW = 0.0, 0.0, 0.0

MAX_FORWARD  = 0.15         # ← 0.4 降到 0.15（第一次别冲太快）
MAX_YAWRATE  = 10.0         # ← 30 降到 10
YAW_DEADBAND_DEG = 1.5      # ⭐ 新增：死区
ELAT_ABORT_M     = 0.80     # ⭐ 新增：偏太多就放弃闭环
LIDAR_MIN_ALT_M  = 0.50     # ⭐ 新增：高度低于 0.5m 不开雷达环（爬升阶段数据不可信）

# ==================== 共享状态 ====================
cmd     = {"forward": 0.0, "right": 0.0, "down": 0.0, "yawspeed": 0.0}
current = {"north": 0.0, "east": 0.0, "down": 0.0, "yaw": 0.0, "mode": "UNKNOWN"}

stop_flag = False
def _kb_watch():
    """后台等回车 -> 平稳停止（不会杀死 mavsdk_server）"""
    global stop_flag
    input()
    stop_flag = True
    print("\n⏸️  收到回车，平稳停止")
threading.Thread(target=_kb_watch, daemon=True).start()


class PID:
    def __init__(self, kp, ki, kd, i_limit, out_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.out_limit = i_limit, out_limit
        self.i, self.prev_e = 0.0, None
    def update(self, e, dt):
        if not math.isfinite(e):        # nan / inf 直接当 0，不污染积分
            e = 0.0
        self.i = max(-self.i_limit, min(self.i_limit, self.i + e*dt))
        d = 0.0 if self.prev_e is None else (e - self.prev_e)/dt
        self.prev_e = e
        out = self.kp*e + self.ki*self.i + self.kd*d
        return max(-self.out_limit, min(self.out_limit, out))


async def send_velocity(drone, cmd, stop_event):
    """后台 10Hz 持续发点流（Offboard 铁律：不能断流）"""
    while not stop_event.is_set():
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(
            cmd["forward"], cmd["right"], cmd["down"], cmd["yawspeed"]))
        await asyncio.sleep(0.1)


async def track_state(drone, current):
    """后台刷新位置/姿态/飞行模式"""
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
            current["mode"] = str(m)          # 记录当前模式
    await asyncio.gather(pos(), att(), mode())


async def main():
    drone = System()
    print(f"⏳ 连接 {ADDRESS} ...")
    await drone.connect(system_address=ADDRESS)
    print("⏳ 等待飞控上线（最多 15 秒）...")

    async def wait_conn():
        async for st in drone.core.connection_state():
            if st.is_connected:
                return True
    try:
        await asyncio.wait_for(wait_conn(), timeout=15)
        print("✅ 已连接飞控")
    except asyncio.TimeoutError:
        print("❌ 15 秒没连上：检查 ADDRESS / 端口名 / 是否关了 QGC / 权限")
        return

    async for h in drone.telemetry.health():
        if h.is_local_position_ok:
            print("✅ 位置 OK")
            break

    # ---- 雷达数据源：只有真正要用真雷达时才打开串口 ----
    if USE_LIDAR and not USE_FAKE:
        lidar = make_lidar(False, port=LIDAR_PORT, model=LIDAR_MODEL)
    else:
        lidar = make_lidar(True, current=current, width=4.0, center_north=0.5)

        # ---- 起飞前确认雷达真的有数据（没有就别飞）----
    if USE_LIDAR:
        print("⏳ 等待雷达数据 ...")
        t_wait = time.monotonic()
        while not lidar.read(0.05).ok:
            if time.monotonic() - t_wait > 10:
                print("❌ 10 秒没拿到雷达数据，退出（不带瞎的飞）")
                if isinstance(lidar, RealLidar):
                    lidar.stop()
                return
            await asyncio.sleep(0.1)

        s0 = lidar.read(0.05)
        print(f"✅ 雷达就绪: front={s0.front:.3f}m  back={s0.back:.3f}m  "
              f"e_lat={s0.e_lat*100:+.1f}cm  yaw_err={math.degrees(s0.yaw_err):+.1f}°")

    print(f"⚙️  配置: USE_FAKE={USE_FAKE} USE_LIDAR={USE_LIDAR} "
          f"ALT={-TAKEOFF_ALT}m HOLD={HOLD_S}s")
    print(f"⚙️  PID:  KP_LAT={KP_LAT} KI_LAT={KI_LAT} KP_YAW={KP_YAW} "
          f"| MAX_FWD={MAX_FORWARD} MAX_YAWRATE={MAX_YAWRATE}")
    print(f"⚙️  安全: ELAT_ABORT={ELAT_ABORT_M}m LIDAR_MIN_ALT={LIDAR_MIN_ALT_M}m "
          f"YAW_DEADBAND={YAW_DEADBAND_DEG}°")

    stop_event = asyncio.Event()
    send_task = asyncio.ensure_future(send_velocity(drone, cmd, stop_event))
    await asyncio.sleep(1.0)          # 先跑 1 秒点流

    # ---- 解锁（若遥控器已解锁，这里会直接成功返回）----
    print("🔓 解锁")
    try:
        await drone.action.arm()
    except Exception as e:
        print(f"❌ 解锁失败: {e}（看看 QGC 的 preflight 提示）")
        stop_event.set(); await send_task
        return

    print("🚁 进入 Offboard")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"❌ Offboard 启动失败: {e._result.result}")
        stop_event.set(); await send_task; await drone.action.disarm()
        return

    state_task = asyncio.ensure_future(track_state(drone, current))
    pid_lat = PID(KP_LAT, KI_LAT, KD_LAT, 1.0, MAX_FORWARD)
    pid_yaw = PID(KP_YAW, KI_YAW, KD_YAW, 10.0, MAX_YAWRATE)
    dt = 0.05

    # ---------- 阶段 1：爬升（横向/朝向可选闭环）----------
    print("🛫 起飞中...")
    tick = 0
    t_takeoff = time.monotonic()
    while not stop_flag:
        s = lidar.read(dt)
        alt = -current["down"]

        # ---- 雷达环：三道闸门 ----
        #   1. 雷达有数据      2. 高度够（避开起飞抖动的假数据）  3. 偏差没超限
        lat_ok = (USE_LIDAR and s.ok and alt > LIDAR_MIN_ALT_M
                  and abs(s.e_lat) < ELAT_ABORT_M)
        yaw_ok = (USE_LIDAR and s.ok and alt > LIDAR_MIN_ALT_M
                  and math.isfinite(s.yaw_err))

        cmd["forward"] = pid_lat.update(s.e_lat, dt) if lat_ok else 0.0

        if yaw_ok:
            e_yaw_deg = math.degrees(s.yaw_err)
            if abs(e_yaw_deg) < YAW_DEADBAND_DEG:
                e_yaw_deg = 0.0
            cmd["yawspeed"] = pid_yaw.update(math.radians(e_yaw_deg), dt)
        else:
            cmd["yawspeed"] = 0.0

        cmd["right"] = 0.0
        cmd["down"]  = 0.0 if current["down"] <= TAKEOFF_ALT + 0.15 else CLIMB_RATE

        tick += 1
        if tick % 10 == 0:
            print(f"  高度={-current['down']:+5.2f}m "
                  f"模式={current.get('mode','?'):>14s} "
                  f"| down={cmd['down']:+.2f} fwd={cmd['forward']:+.2f}")

        if current["down"] <= TAKEOFF_ALT + 0.15:
            break
        if time.monotonic() - t_takeoff > TAKEOFF_TIMEOUT_S:
            print("⚠️ 起飞超时（没到高度），可能没离地")
            break
        await asyncio.sleep(dt)

    reached = current["down"] <= TAKEOFF_ALT + 0.15
    cmd["down"] = 0.0
    if reached:
        print("✅ 到达高度，悬停保持…")
    else:
        print("⚠️ 未到达目标高度就退出了，直接进入收尾")

    # ---------- 阶段 2：悬停保持 ----------
    t0 = time.monotonic()
    while not stop_flag and time.monotonic() - t0 < HOLD_S:
        s = lidar.read(dt)

        # ---- 雷达环（和阶段 1 完全一致）----
        s = lidar.read(dt)
        alt = -current["down"]

        # ---- 雷达环：三道闸门 ----
        #   1. 雷达有数据      2. 高度够（避开起飞抖动的假数据）  3. 偏差没超限
        lat_ok = (USE_LIDAR and s.ok and alt > LIDAR_MIN_ALT_M
                  and abs(s.e_lat) < ELAT_ABORT_M)
        yaw_ok = (USE_LIDAR and s.ok and alt > LIDAR_MIN_ALT_M
                  and math.isfinite(s.yaw_err))

        cmd["forward"] = pid_lat.update(s.e_lat, dt) if lat_ok else 0.0

        if yaw_ok:
            e_yaw_deg = math.degrees(s.yaw_err)
            if abs(e_yaw_deg) < YAW_DEADBAND_DEG:
                e_yaw_deg = 0.0
            cmd["yawspeed"] = pid_yaw.update(math.radians(e_yaw_deg), dt)
        else:
            cmd["yawspeed"] = 0.0

        tick += 1
        if tick % 10 == 0:
            print(f"  悬停 高度={-current['down']:+5.2f}m "
                  f"e_lat={s.e_lat*100:+6.1f}cm "
                  f"fwd={cmd['forward']:+.3f} "
                  f"yaw_err={math.degrees(s.yaw_err):+6.1f}°")
        await asyncio.sleep(dt)

    # ---------- 收尾：降落 ----------
    print("🛬 降落")
    cmd["forward"] = cmd["right"] = cmd["yawspeed"] = 0.0
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

    # ⭐ 如果已经不在 Offboard（比如你拨开关接管了），就不要抢降落了
    mode_now = current.get("mode", "")
    if "OFFBOARD" not in mode_now.upper():
        print(f"⚠️ 检测到模式已是 {mode_now}（你接管了或已退出 Offboard）")
        print("   -> 跳过自动降落，交给你手动")
    else:
        try:
            await drone.action.land()
            print("✅ 已发送降落")
        except Exception as e:
            print(f"⚠️ land 失败: {type(e).__name__}（遥控器接管）")
    if isinstance(lidar, RealLidar):
        lidar.stop()
    print("✅ 完成")


if __name__ == "__main__":
    asyncio.run(main())