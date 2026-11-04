# PROJECT_STATE

> 最后更新：**第 3 次**（每次实质改动递增一次，历史见文末「更新记录」）
> 本文只写**能从代码/参数/日志确认的事实**，无法确认的标 `[未确认]`。
> 当前唯一阻塞点：**起飞阶段横向漂移（3 s 内 0.97 m）**，见 §8 B12/B13。

---

## 1. 项目目标

无人机在**走廊两侧墙之间**自主侧飞巡检，同时保持：

| 保持量 | 目标值 | 实现方式 |
|---|---|---|
| 与墙的横向位置 | 离 front 墙 1.60 m（single）/ 偏离中线 +0.37 m（dual） | 2D 雷达 + 横向 PID |
| 机头与墙垂直 | `yaw_err → 0` | 2D 雷达 + 航向 PID |
| 飞行高度 | 1.30 m | 光流/测距 + 高度外环 |

侧飞途中用摄像头识别消防栓（`Fire_hydrant`），停下精调位置并检测柜内四类物品是否齐全。

来源：`Plan.txt`、`Fly_alt_safe.py` 模块 docstring、`Corridor_mission.py` 状态机。

---

## 2. 当前入口文件（只认一个）

```
/home/orangepi/drone/Control/Fly_alt_safe.py
```

- 本机（开发机）路径：`/home/theo/drone/Control/Fly_alt_safe.py`
- 依赖同目录的 `Lidar_source.py`
- **这是当前唯一在跑真机的入口**（侧飞 + 雷达横向环 + 高度外环）

自述定位（docstring 原文）：*"安全修订版……这是安全修订，不是'保证不炸机'；真机前必须台架拆桨 + SITL/系留验证。"*

其它文件状态：

| 文件 | 状态 |
|---|---|
| `Control/Corridor_mission.py` | 视觉版状态机，手拿 DRY_RUN 已通过；**真飞未做** |
| `Control/Fly_alt_test.py` | 上一版合并，已被 `Fly_alt_safe.py` 取代 |
| `Control/Takeoff_back_wall.py` | **第 3 次更新新增**：单后墙定距 + 垂直爬升测试（起飞问题专用） |
| `Control/Fly_vision_test.py` | **新增，用途 `[未确认]`** |
| `Control/Watch.py`（仅在机载电脑上，开发机无此文件） | 只读监视脚本（不 arm、不发 setpoint），用于对比雷达真值与 EKF 估计 |
| `Control/Side_fly_test_v2.py` / `_v3.py` / `_v4_alt.py` | 历史版本，仅供对照 |
| `Control/Takeoff_test.py` / `Wall_hold_test.py` / `Lidar_check.py` | 单项目测试，已完成 |
| `Lidar/Lidar_scan.py` | 原始点云可视化工具 |
| `vision/Vision_worker.py` | 视觉子进程（ONNX） |
| `Viewer/viewer.py` + `Graound_view_2.html` | 地面站，当前流程未接入 |

---

## 3. 硬件 / 串口 / 波特率 / 雷达型号

### 3.1 串口与端口（代码可确认 ✅）

| 项 | 值 | 来源 |
|---|---|---|
| 飞控连接 | `serial:///dev/pixhawk:57600` | `Fly_alt_safe.py` → `ADDRESS` |
| 雷达串口 | `/dev/wheeltec_lidar` | `Fly_alt_safe.py` → `LIDAR_PORT` |
| 雷达型号 | `"N10"` | `Fly_alt_safe.py` → `LIDAR_MODEL` |
| **雷达波特率** | **230400** | `Lidar_source.py` → `PROFILES["N10"]["baud"]` |
| 雷达帧头 | `A5 5A 3A` | `PROFILES["N10"]["header"]` |
| 雷达帧长 | 55 字节 / 帧 | `PROFILES["N10"]["frame"]` |
| 每帧点数 | 16 | `PROFILES["N10"]["n"]` |
| 结束角偏移 | 52 | `PROFILES["N10"]["end_off"]` |
| 备用型号 | `N10P`：460800 baud，帧头 `A5 5A 6C`，105 字节，32 点，`end_off=102` | 同上 |
| 摄像头 | `/dev/video0` | `Vision_worker.py` → `CAM_INDEX = 0` |
| 视觉模型 | `ONNX`，`MODEL_PATH = ".../Vision/best.onnx"` | ⚠️ 路径大小写有误，见 B10 |

### 3.2 硬件型号

| 项 | 值 | 状态 |
|---|---|---|
| 飞控 | MicoAir743v2（STM32H743VIT6） | `[未确认]` 来自对话，代码中无 |
| 飞控固件 | NuttX-11.0.0 | ✅ 来自 `nsh` 输出；**PX4 具体版本号 `[未确认]`** |
| 机载电脑 | Orange Pi 4 Pro，`aarch64`，Ubuntu 22.04，Python 3.10.12，hostname `orangepi4pro` | ✅ 来自终端 |
| 2D 雷达 | N10（见上表） | ✅ |
| 下视传感器 | 光流 + 测距一体模块 | `[未确认]` 型号 |
| 电池 | 3S 2200 mAh 35C | `[未确认]` 来自对话 |
| 整机起飞重量 | 未测 | `[未确认]` |
| 数传 | **已拆除**（曾引起 `heading estimate not stable`） | ✅ |

### 3.3 软件依赖（`~/drone/venv`）

```
mavsdk          3.17.4     ← 必须 "mavsdk<4"，4.x 接口不兼容
grpcio          1.84.0
pyserial        3.5
numpy           2.2.6
opencv-python   5.0.0.93
matplotlib      3.10.9
```

进入环境：`source ~/drone/venv/bin/activate`（记录在 `进环境.txt`）

`ultralytics` 在 aarch64 上装不上 → 视觉走 `ONNX + cv2.dnn`。

---

## 4. 数据流与三条循环

### 4.1 数据流

```
N10 雷达 (/dev/wheeltec_lidar, 230400)
   │  RealLidar._loop 后台线程：3 字节帧头同步 → 55 字节帧 → 转完一圈才 _publish
   ▼
LidarSample{ front, back, yaw_err(rad), ok }
   │
   ├──► WallController._sanitize() → FrontGuard → PID → cmd["forward"], cmd["yawspeed"]
   │
PX4 ──MAVLink 57600──► MAVSDK telemetry
   │  current{ north, east, down, vn, ve, vd, yaw, mode }
   │
   └──► alt_hold_task → cmd["down"]

cmd{ forward, right, down, yawspeed }
   │
   ▼  send_velocity（20 Hz）
MAVSDK offboard.set_velocity_body(VelocityBodyYawspeed) ──► PX4 内环
```

### 4.2 三条循环

| # | 循环 / 任务 | 频率 | 读什么 | **写什么（唯一写入者）** |
|---|---|---|---|---|
| 1 | **雷达横向环**（主 `while`） | 20 Hz（`CONTROL_PERIOD_S=0.05`） | `lidar.read()`、`current["down"]`、`PHASE` | `cmd["forward"]`、`cmd["yawspeed"]`；`side` 阶段另写 `cmd["right"]` |
| 2 | **高度外环** `alt_hold_task` | 20 Hz | `-current["down"]` | `cmd["down"]`（`climb` 阶段不接管） |
| 3 | **setpoint 发送** `send_velocity` | 20 Hz（`SEND_PERIOD_S=0.05`） | 整个 `cmd` | 不发 cmd，只发 MAVLink |

**关键纪律：一个通道只有一个写入者。**

后台还有三个常驻任务：`track_state`（3 个遥测订阅并发）、`RealLidar._loop`（串口线程）、`_watch_task`（监控上面几个任务是否异常退出）。

### 4.3 雷达环形缓冲的解析逻辑（`Lidar_source.py`）

```python
# _loop: 逐字节对齐帧头 → 读 55 字节 → _parse_frame() → 累积到 buf
# 判断"转完一圈"：相邻帧起始角回绕（cur < prev - 100）且 buf 点数 >= 50
# → _publish(buf)：取 4 个扇区中位数 → 存入 deque(maxlen=SMOOTH_N) → 再取中位数
```

扇区：`_sector(buf, FRONT_DEG=0, WALL_HALF=20)`、`_sector(buf, BACK_DEG=180, 20)`、`_sector(buf, +15, 5)`、`_sector(buf, -15, 5)`。

---

## 5. 当前阶段与状态机

`PHASE[0]` 取值：`init → climb → hold → ready → side → stop`

| 状态 | 行为 | 高度环 | 雷达横向环 |
|---|---|---|---|
| `init` | 等 `local position OK` + 起飞前检查 | 不接管 | 不开 |
| `climb` | 以 `CLIMB_RATE=-0.35` 爬升，`down <= HOVER_ALT_DOWN+0.10 = -1.35` 退出 | **不接管** | 高度 > `CLIMB_LAT_ON_ALT=0.60` 才开 |
| `hold` | 稳住 `STABILIZE_S=3.0` s | 接管 | 开 |
| `ready` | 等 `abs(e_lat) < READY_ELAT_M=0.08`，超时 `READY_TIMEOUT_S=12` s | 接管 | 开 |
| `side` | `right = V_SIDE * SIDE_DIR = +0.30` m/s，跑 `SIDE_TIME_S=20` s | 接管 | 开 |
| `stop` | 停侧飞，悬停 `STOP_HOVER_S=8` s 后收尾 | 接管 | 开 |

**主循环内还有一层 `WALL_MODE` 分支**：

- `"single"`：只用 `front`，误差 `e = front - FRONT_TARGET_M`，带 `FRONT_MIN_SAFE` 保护
- `"dual"`：用 `e = (front - back)/2 - E_LAT_TARGET_M`，带走廊宽度一致性校验
- `"off"`：`fwd = 0`

**中止条件（都会走 `finally` 收尾 + 尝试降落）：**

| 触发 | 参数 |
|---|---|
| 未到位置就超时 → 直接中止 | `ABORT_ON_READY_TIMEOUT = True` |
| 侧飞中止 → 直接中止 | `ABORT_ON_SIDE_ABORT = True` |
| 起飞超时 | `TAKEOFF_TIMEOUT_S = 30` s |
| 等 local position 超时 | `HEALTH_TIMEOUT_S = 30` s |
| 任何后台任务异常退出 | `_watch_task` |
| 雷达丢失 | `LOST_TIMEOUT_S = 1.0` s |
| 横向误差过大 | `WALL_ABORT_M = 0.80` m（**注意：这里是"放弃纠偏"而非中止，见 B11**） |
| 航向偏差过大 | `YAW_ABORT_DEG = 25`° |
| 离前墙过近（`side` 阶段内联判断） | `s.front < SIDE_STOP_FRONT = 0.90` m |
| 离后墙过近（`side` 阶段内联判断） | `s.back < BACK_MIN_SAFE = 0.65` m |
| **离前/后墙过近（全程）** | `guard_task` 每 50 ms 检查 `dist` 快照，`front < 0.90` 或 `back < 0.65` 即中止 |
| **起飞点横向偏超限** | `abs(e_lat) > E_LAT_TOL_TAKEOFF = 0.15` m → **拒绝解锁** |

**另一个状态机（`Corridor_mission.py`，未真飞）：**
`climb → stabilize → cruise → align → inspect → resume → done`

---

## 6. 关键参数、单位、坐标系

### 6.1 坐标系（最容易踩坑的地方）

| 项 | 约定 |
|---|---|
| PX4 / MAVSDK 遥测 | **NED**：`down` 正 = 向下。**高度 `alt = -current["down"]`** |
| `VelocityBodyYawspeed(forward, right, down, yawspeed)` | **机体系**。`forward` 正 = 沿机头；`right` 正 = 机身右侧；`down` 正 = 下降 |
| **`yawspeed` 单位** | **deg/s**，正 = 顺时针（右转）⚠️ 不是 rad/s |
| 雷达角度 | `FRONT_DEG=0` 对准机头，`BACK_DEG=180` 对准机尾 |
| "front 墙" | 机头正对的那面墙 |

### 6.2 雷达派生量

```python
e_lat   = (front - back) / 2        # 单位 m
        # e_lat < 0 → 更靠近 front 墙（需要后退）
        # e_lat = 0 → 在走廊几何中线上
W       = front + back              # 走廊总宽；横向移动时不变 → 自洽性判据
yaw_err = YAW_SIGN * (dp - dm) / (tan(15°) * (dp + dm))   # 单位 rad，小角度近似
```

`WallController` 里实际用的是相对偏移：

```python
e = e_lat - E_LAT_TARGET_M          # dual 模式，E_LAT_TARGET_M = +0.37
fwd = clamp(KP_LAT * e + ..., -MAX_FORWARD, +MAX_FORWARD)
```

### 6.3 `Fly_alt_safe.py` 参数表

**走廊与雷达**（v3）

| 参数 | 值 | 单位 | 说明 |
|---|---|---|---|
| `WALL_MODE` | `"dual"` | — | off / single / dual |
| `W_NOMINAL_M` | 2.49 | m | 走廊宽度标定值 |
| `W_TOL_M` | 0.15 | m | `W` 超差判为局部结构干扰 |
| `BACK_STABLE_M` | 0.06 | m | 认为 back 稳定的阈值 |
| `USE_BACK_ESTIMATE` | `True` | — | 允许用 `W - back` 反推 front |
| `FRONT_TARGET_M` | 1.60 | m | single 模式目标 |
| `E_LAT_TARGET_M` | **+0.37** | m | dual 模式目标（对应 `front≈1.62 / back≈0.87`，离 front 墙更远） |

**侧飞**

| 参数 | 值 | 单位 |
|---|---|---|
| `V_SIDE` | 0.30 | m/s |
| `SIDE_TIME_S` | 20.0 | s |
| `SIDE_DIR` | +1.0 | — |

**高度（v4）**

| 参数 | 值 | 单位 | 说明 |
|---|---|---|---|
| `HOVER_ALT_DOWN` | -1.45 | m | 爬升命令目标；实际在 -1.35 停（见注） |
| `CLIMB_RATE` | -0.35 | m/s | |
| `TAKEOFF_TIMEOUT_S` | 30.0 | s | |
| `STABILIZE_S` | 3.0 | s | |
| `CLIMB_LAT_ON_ALT` | 0.60 | m | 高度超过此值才开横向环 |
| `ALT_HOLD_ENABLE` | `True` | — | |
| `ALT_TARGET_M` | 1.30 | m | 高度环目标 |
| `KP_ALT / KI_ALT / KD_ALT` | 0.50 / 0.10 / 0.00 | — | |
| `I_LIMIT_ALT` | 0.30 | — | 积分限幅 |
| `MAX_DOWN_CMD` | 0.20 | m/s | 高度指令限幅 |
| `ALT_MIN_M` | 0.80 | m | 低于此值强制全力爬升 |

> 注（代码原文）：`climb` 的停止阈值是 `HOVER_ALT_DOWN + 0.10 = -1.35`（高度 1.35 m）；进入 `hold` 后 `ALT_TARGET_M = 1.30` m，会有约 5 cm 主动下降，**是预期行为**。

**到位门**

| 参数 | 值 | 单位 | 说明 |
|---|---|---|---|
| `READY_ELAT_M` | 0.08 | m | 到位判据 |
| `READY_TIMEOUT_S` | 12.0 | s | |
| `STOP_HOVER_S` | 8.0 | s | |

**横向 PID**

| 参数 | 值 | 单位 | 说明 |
|---|---|---|---|
| `KP_LAT / KI_LAT / KD_LAT` | **0.30 / 0.02 / 0.00** | — | 第 3 次更新由 0.35 / 0.04 下调 |
| `MAX_FORWARD` | **0.20** | m/s | 由 0.22 下调 |
| `WALL_ABORT_M` | 0.80 | m | `\|e\|` 超此值 → `ok=False` → `fwd=0`（"放弃纠偏区"，见 B11） |
| `LIDAR_MIN_ALT_M` | 0.50 | m | 低于此高度不做横向/航向纠偏 |
| `FRONT_MIN_SAFE` | **0.70** | m | ⚠️ **仅 `single` 模式生效；`WALL_MODE="dual"` 时完全无效** |
| `SIDE_STOP_FRONT` | **0.90** | m | 第 3 次更新由 0.60 上调 |
| `BACK_MIN_SAFE` | **0.65** | m | 第 3 次更新新增（B1 修复） |
| `E_LAT_TOL_TAKEOFF` | **0.15** | m | 第 3 次更新新增：起飞点偏超此值拒绝解锁 |

> **`WALL_ABORT_M = 0.80` 的实际含义**（`dual` 模式）：
> `e = (f-b)/2 - 0.37`，所以 `|e| ≥ 0.80` 等价于 `front < 0.815 m`（当 `W=2.49`）。
> 也就是说 **`front` 掉到 0.82 m 时横向环主动输出 0**，而在 `guard_task` 里 `front < 0.90` 才中止 —— 两者只差 8 cm。

**航向 PID**

| 参数 | 值 | 单位 | 说明 |
|---|---|---|---|
| `USE_YAW_LOOP` | `True` | — | |
| `KP_YAW / KI_YAW / KD_YAW` | 3.0 / 0.0 / 0.0 | (deg/s)/deg | **输入必须用"度"** |
| `MAX_YAWRATE` | 10.0 | deg/s | |
| `YAW_DEADBAND_DEG` | 1.5 | deg | |
| `YAW_ABORT_DEG` | 25.0 | deg | |

**雷达数据质量**

| 参数 | 值 | 单位 |
|---|---|---|
| `MAX_JUMP_M` | 0.25 | m |
| `MAX_BAD_CNT` | 8 | 次 |
| `LOST_TIMEOUT_S` | 1.0 | s |

**时序与健康检查（本次新增）**

| 参数 | 值 | 单位 | 说明 |
|---|---|---|---|
| `CONTROL_PERIOD_S` | 0.05 | s | 控制环 20 Hz |
| `SEND_PERIOD_S` | 0.05 | s | setpoint 20 Hz；57600 丢包可退回 0.10 |
| `MAX_DT_S` | 0.20 | s | dt 上限，防阻塞后 PID 爆炸 |
| `HEALTH_TIMEOUT_S` | 30.0 | s | 等 local position |
| `SEND_STOP_TIMEOUT_S` | 2.0 | s | 收尾等 `send_velocity` 退出 |
| `CMD_TIMEOUT_S` | 5.0 | s | `offboard.stop` / `land` 最长等待 |
| `ABORT_ON_READY_TIMEOUT` | `True` | — | |
| `ABORT_ON_SIDE_ABORT` | `True` | — | |
| `PILOT_MODES` | `POSCTL/ALTCTL/STABILIZED/MANUAL/ACRO/RATTITUDE` | — | 视为飞手接管 |

### 6.4 `Control/Lidar_source.py` 标定常量（实测确认 ✅）

```python
FRONT_DEG = 0        BACK_DEG = 180
WALL_HALF = 20       YAW_DELTA = 15      YAW_HALF = 5
YAW_SIGN  = -1.0     SMOOTH_N = 2        # 从 5 降到 2，减少延迟
```

### 6.5 飞控参数

**第 2 次更新时从 `nsh` 读出（历史值）：**

```
MPC_THR_HOVER     0.5500     (+)
MPC_USE_HTE       1          ← ⚠️ 会在飞行中在线改写 MPC_THR_HOVER
MPC_THR_MAX       1
MPC_Z_VEL_P_ACC   4.00       ← 旧名 MPC_Z_VEL_P
MPC_Z_VEL_I_ACC   2.00       ← 旧名 MPC_Z_VEL_I
MPC_Z_P           1.00
EKF2_HGT_REF      Range sensor
EKF2_HEAD_NOISE   17.19 deg   ← 默认 5.0，严重偏大
COM_RC_OVERRIDE   0
```

**`MPC_THR_HOVER` 的盲调记录（全部未标定）：**

| 试飞次序 | 值 | 现象 |
|---|---|---|
| ① | 0.5500 | 只有满电才飞得稳 |
| ② | 0.65 | 更不稳，直接往墙冲 |
| ③ | 0.60 | RC 起飞仍飘 |
| ④ | **0.57** | 当前值，`T/W ≈ 1.75` |

> ⚠️ 因为 `MPC_USE_HTE = 1`，上表的值**可能已被在线估计器改写**，实际生效值需重新读取。
> 标定前必须 `param set MPC_USE_HTE 0`，否则量了也不准。

---

## 7. 已确认的结论

全部由实测日志/台架验证：

1. **`e_lat` 符号正确**：飞机靠近 front 墙 → `e_lat` 变负；`fwd = KP*e_lat` 方向与之自洽。
2. **`front` / `back` 波束安装正确**：平移 10.8 cm → `front -10.8 / back +10.8`，1:1 无比例误差。
3. **`W = front + back` 对横向运动不变**（实测 2.473 / 2.478 / 2.485 / 2.490 / 2.502 m），可作自洽性判据。
4. **走廊宽度 ≈ 2.49 m**，多次独立测量一致。
5. **`alt = -down` 是测距真值**（`EKF2_HGT_REF = Range sensor`）→ 掉高是真的，不是估计漂移。
6. **电池不是掉高原因**：电压 12.6 → 12.3 V，只用了约 10%。
7. **悬停时推力够**（纯悬停 25 s 不掉高）；**平移时缺推力** —— `MPC_THR_HOVER` 前馈不足以补偿平移需要的额外推力，PX4 速度环补不上。
8. **脚本侧高度外环有效**：侧飞中最低高度从 **0.53 m → 1.15 m**（目标 1.30，低 15 cm）。
9. **航向 PID 曾把弧度当 deg/s 发给 PX4**（弱 57.3 倍）。证明：`4 × radians(2.8°) = 0.196 ≈ 0.20`（观测值）。改传"度"后正常。
10. **积分饱和（windup），不是增益过大**：`i` 曾精确顶在 `i_limit` 上 → 改成条件积分 `if not (saturated and raw*e > 0)`。
11. **积分分离方案会让机撞墙**：`|e| > i_band` 时冻结积分，导致同号积分永远卸不掉。**不要再用。**
12. **`MPC_Z_VEL_P/I` 在此固件已改名为 `_ACC` 后缀**。
13. **起飞点摆正在目标位置时，起飞阶段不产生横移**（`front=1.618 / back=0.867` → `e_lat ≈ +0.376`）。
14. **`V_SIDE = 0.20~0.30 m/s` 可执行**，实际位移与命令基本一致。
15. **`COM_RC_OVERRIDE = 0`** 后，油门最低不会被判为飞手接管，Offboard 能稳定进入。
16. **Enter 键优雅停止可用**；Ctrl+C 会杀掉 `mavsdk_server`，导致所有收尾 RPC 失败。
17. **`SMOOTH_N` 5 → 2** 后横向振荡明显改善（延迟约 0.5 s）。
18. **视觉 ONNX 单测通过**：5 类可识别（`Fire_extinguisher` 未检出），≈15–16 FPS。
19. **雷达只在静止/慢速时可靠** —— 这是"巡航粗 + 停稳精调"架构的由来。
20. **仿真里 `KP_YAW=30` 不能迁移到真机**（仿真无噪声），真机值 3~5。

### 第 3 次更新新增结论

21. **机头必须垂直于墙 —— 这是硬几何前提，不是可选项**。机头朝走廊方向时实测 `front = 6.5 m`、`back = 3~4.9 m`（相邻读数跳 ±1 m）、`W ≈ 10 m`。`_sanitize()` 要求 `|W - 2.49| ≤ 0.15`，因此恒判 `bad` → `ok=False` → **`cmd["forward"]` 恒为 0，横向环一次都没工作过**。日志印证：该次飞行 `fwd` 全程 `+0.000`。
22. **`yaw_err` 就是"机头与墙面法线的夹角"**，`yaw_err = 0` ⇔ 机头垂直对墙。它由 ±15° 双波束算出，是**小角度近似**：只在 ±15~20° 内有效；`|yaw_err| > YAW_ABORT_DEG (25°)` 时航向环直接输出 0，因此**无法从 90° 找到墙**。精度要求很低：偏差 5° 只带来 0.4% 的距离误差。
23. **`W = front + back ≈ 2.49 m` 是"机头对墙"唯一可靠的判据**，不依赖罗盘、不依赖绝对航向。
24. **起飞阶段必漂**：某次实测 3 s 内水平位移 **0.97 m**（由 `dNorth/dEast` + `yaw=-106°` 换算到机身系：向后 0.68 m、向右 0.69 m），同期垂直只爬 **0.36 m** → 水平 : 垂直 ≈ **2.7 : 1**。
25. **推重比 ≈ 1.75**（由 `MPC_THR_HOVER = 0.57` 反推 `T/W = 1/0.57`）。爬升实际速率只有命令值的 60~75%，日志频繁出现 `⚠️爬升速率不足`。
26. **`MPC_USE_HTE = 1` 会在飞行中在线改写 `MPC_THR_HOVER`**（PX4 悬停推力估计器），导致"每次飞完表现都不一样"。**标定期间必须设为 0**。
27. **遥控纯手动起飞同样会漂、且偶尔抽搐**，悬停后恢复稳定 → 说明起飞段是机体 / PX4 层问题，**与脚本无关**。
28. **重校罗盘后漂移方向翻转**（前 → 后）。机身系漂移方向约 **45° 对角**，符合"光流 X/Y 轴未对齐"的指纹；`SENS_FLOW_ROT` / `EKF2_FLOW_POS_*` 尚未核对 `[未确认]`。
29. **`guard_task` 曾因参数个数不匹配（定义 2 个、调用传 1 个）在 `try` 之前抛 `TypeError`**，此时 `finally` 的 `offboard.stop()` / `land()` 全部不执行。已修（改为只接收 `stop_event`，数据从 `dist` 字典取，且**不再自己读雷达**——两个消费者会阻塞事件循环）。
30. **某次实测 `front` 最低到 0.273 m**（离前墙 27 cm），而 `climb` / `hold` / `ready` 三个阶段当时**没有任何距离保护**。现已由 `guard_task` 覆盖全程。
31. **`e_lat` / `front` / `back` 全部是机体系的**，所以航向整体偏 δ 时命令与测量一起偏 δ、会互相对消 → **罗盘误差不会直接导致"命令后退却往前冲"**。那个症状要从光流速度估计 / 推力余量去找。

---

## 8. 当前 bug / 下一步

### 第 3 次更新 · 状态总览

| 编号 | 问题 | 状态 |
|---|---|---|
| B1 | 后墙保护 | ✅ **已修**（`BACK_MIN_SAFE=0.65` + 全程 `guard_task`） |
| B2 | EKF 航向漂 45° | 🟠 已重校罗盘，最近几次 `yaw` 稳定在 ±5° 内；仍建议在开阔地重校一次 `[未确认]` |
| B3 | 横向 `e` 振荡发散 | 🟢 **已缓解**（`KP_LAT` 0.35→0.30、`MAX_FORWARD` 0.22→0.20），最近一次 `e` 有界 ±17 cm |
| B4 | `side_abort` 只清 `right` | ✅ **已修**（改为 `raise FlightAbort`，走统一收尾、四通道全清零） |
| B5 | `dual` 分支缺近距离保护 | ✅ 由 `guard_task` 覆盖 |
| B6 | `CLIMB_LAT_ON_ALT` 太早 | ✅ 新增起飞点门禁 `E_LAT_TOL_TAKEOFF = 0.15` |
| B7 | 两份 `Lidar_source.py` 不同步 | ⚠️ 仍存在 |
| B8 | `_sanitize` 的 `est` 分支不触发 | ⚠️ 仍存在（`good_back` 从未被赋初值，所以 `est` 永远走不到） |
| B9 | `Corridor_mission.py` `HYDRANT_CONFIRM_N` | ⚠️ 未确认 |
| B10 | `MODEL_PATH` 大小写 | ⚠️ 未修 |
| **B11** | **`WALL_ABORT_M` 形成"放弃纠偏区"** | 🔴 **未修** |
| **B12** | **起飞阶段横向漂移 ~1 m** | 🔴 **当前唯一阻塞点** |
| **B13** | **推重比 ≈ 1.75，推力余量不足** | 🔴 **未解决（硬件层）** |
| **B14** | `MPC_THR_HOVER` 被 `MPC_USE_HTE` 在线改写、且从未标定 | 🔴 **未解决** |
| **B15** | 光流安装/坐标配置未核对（45° 对角漂移） | 🟠 **待查** |
| **B16** | 降落只校验"命令已发出"，不校验"真的落地" | 🟠 **未修** |

### 🔴 B12：**起飞阶段横向漂移 ~1 m（当前唯一阻塞点）**

实测：3 s 内水平位移 **0.97 m**（机身系后 0.68 / 右 0.69），同期只爬 0.36 m。
- 遥控纯手动起飞**同样会漂**（且偶尔抽搐）→ **与脚本无关**
- 悬停后恢复稳定 → 说明是"起飞段"专有问题
- 在 2.49 m 宽的走廊里，目标位置离后墙只有 0.87 m，**这 1 m 的余量走廊给不了**

**已知的三个可能原因（按证据强度）：**
1. **B13 推力余量不足** —— 爬升时倾斜权限被 Z 轴吃掉，横向无控制力
2. **地面效应 + 下视测距死区**（0.2~0.5 m）→ 推力跳变 → "抽搐"
3. **B15 光流对角偏差** → 位置估计有系统偏差，POSCTL 追着错误估计飞

**下一步（按顺序，每步只验一个变量）：**
```
① 地面找姿态：原地转飞机，直到 W = front+back ≈ 2.49，贴胶带做记号
② 标定 MPC_THR_HOVER：先 param set MPC_USE_HTE 0，
   飞一次稳悬停 30 s，拔 SD 卡看 vehicle_thrust_setpoint.xyz 的均值
③ 查光流：SENS_FLOW_ROT / EKF2_FLOW_POS_X/Y/Z，
   日志看 estimator_status_flags.cs_opt_flow 与 vehicle_local_position.dead_reckoning
④ 起飞点改到走廊正中间（front ≈ back ≈ 1.25），两边各留 1.25 m 余量
⑤ 以上都做完再谈侧飞
```

### 🔴 B11：**`WALL_ABORT_M = 0.80` 形成"放弃纠偏区"**

`dual` 模式下：

```python
ok = (abs(self.e) < WALL_ABORT_M and alt > LIDAR_MIN_ALT_M)
fwd = self.pid_lat.update(self.e, dt) if ok else 0.0
if not ok:
    self.pid_lat.reset()
```

`|e| ≥ 0.80` ⇔ `front < 0.815 m`（当 `W=2.49`）。此时**不是继续纠偏，而是 `fwd = 0`、PID 清零**。
某次实测 `front` 最低到 **0.273 m**，那一段完全是自由飞行。

**冲突**：`guard_task` 的 `front < SIDE_STOP_FRONT (0.90)` 是中止线，比 0.815 m 只早 8 cm。**余量太薄。**

**建议**（未实施）：`SIDE_STOP_FRONT` 提到 **0.95~1.00**，让中止永远发生在"放弃纠偏"之前。

### 🔴 B13：**推重比 ≈ 1.75，推力余量不足**

`T/W = 1 / MPC_THR_HOVER = 1/0.57 ≈ 1.75`。参考值：

| `MPC_THR_HOVER` | 推重比 | 评价 |
|---|---|---|
| 0.50 | 2.00 | 舒服 |
| 0.55 | 1.82 | 可以 |
| **0.57** | **1.75** | **偏紧** |
| 0.60 | 1.67 | 很紧 |
| 0.65 | 1.54 | 基本不能飞 |

**唯一解**：称整机重量 → 减重（相机/支架/线材）或换动力（4S / 更大桨 / 更大电机）。
**不要**继续靠调 `MPC_THR_HOVER` 补偿。

### 🔴 B14：**`MPC_THR_HOVER` 被在线改写、且从未标定**

- `MPC_USE_HTE = 1`（`nsh` 输出确认）→ PX4 飞行中在线估计悬停推力并**写回** `MPC_THR_HOVER`
- 已盲试 0.55 → 0.65 → 0.60 → 0.57，**没有一次是量出来的**
- 观察："0.55 时只有满电才稳" ⇒ 真值 **> 0.55**

**标定步骤（不需要数传）：**
1. `param set MPC_USE_HTE 0`（固定住，否则你的值会被覆盖）
2. 遥控 + POSCTL，稳悬停 **1.3 m 以上** 30 s（**必须脱离地面效应**）
3. 落地 → 上锁 → 断电 → 拔 SD 卡 → `review.px4.io` 看 `vehicle_thrust_setpoint.xyz` 的均值
4. `param set MPC_THR_HOVER <那个数>`

### 🟠 B15：**光流安装/坐标配置未核对**

重校罗盘后漂移方向翻转（前 → 后），且机身系下是 **45° 对角** —— 这是"光流 X/Y 轴未对齐"的典型指纹。

待查：

```
param show SENS_FLOW_ROT          # 光流相对机体的旋转
param show EKF2_FLOW_POS_X        # 光流镜头相对重心的偏移
param show EKF2_FLOW_POS_Y
param show EKF2_FLOW_POS_Z
```

日志（ULog + review.px4.io）：

```
estimator_status_flags  →  cs_opt_flow        应为 true
vehicle_local_position  →  dead_reckoning     应为 false
estimator_innovations   →  flow_innov         应无持续偏置
```

### 🟠 B16：**降落只校验"命令已发出"，不校验"真的落地"**

`finally` 里 `CMD_TIMEOUT_S = 5.0` 只是 RPC ack 超时，`print("✅ 已发送降落")` **不等于落地**。
`land()` 失败时只打印一行警告，脚本照常输出 `✅ 完成` —— 飞机可能还在空中。

**建议**：land 后轮询 `alt`/`mode`，落地或上锁才算成功；失败时重试并最终以非零码退出。

### 🟢 B1（已修）：后墙保护

原问题：只检查 `s.front < SIDE_STOP_FRONT`，`dual` 分支没有 `back` 下限，实测 `back` 曾降到 **0.409 m** 而无任何中止。

**已实施**：`BACK_MIN_SAFE = 0.65` + `guard_task`（全程每 50 ms 检查 `dist` 快照）。

---

> **以下 B2 ~ B10 是历史记录（第 2 次更新及以前）。**
> **当前状态一律以上面「第 3 次更新 · 状态总览」为准**，这里只保留当时的分析细节备查。

---

### 🔴 B2：EKF 航向漂移 ~45° / 20 s

实测 `yaw` 从 -42.1° 漂到 -86.9°，而雷达 `yaw_err` 全程只有 ±10°。说明**机身相对墙是正的，但飞控的航向估计是错的**。

后果：`set_velocity_body` 的机体速度会被错误姿态转成 NED —— **指令 `fwd=+0.17`（向前）时飞机实际后退 23 cm**，这是横向误差冲上 +47 cm 的主因之一。

根因：`EKF2_HEAD_NOISE = 17.19°`（默认 5.0）= 罗盘受严重干扰。

**下一步：** 在走廊内、最终装配状态、上电池的情况下重新校准罗盘；不能在校准后拆装硬件。

### 🟠 B3：横向 `e` 振荡且在发散

实测极值：`+30.9 cm (3.0 s) → -17.0 cm (7.6 s) → +47.1 cm (18.9 s)`，周期约 10 s，**振幅在增大**。

判据：`KP_LAT × τ_delay = 0.35 × 1.25 = 0.44`（接近临界）。

**下一步：** `KP_LAT` 0.35 → 0.20，`MAX_FORWARD` 0.22 → 0.14。

### 🟠 B4：`side_abort` 只清了 `cmd["right"]`

代码里 `side_abort` 时只把 `cmd["right"]` 置零，`cmd["forward"]` 和 `cmd["yawspeed"]` 还留着上一拍的 PID 输出。

**修复：** 中止时三个通道一起清零。

### 🟡 B5：`dual` 分支缺少近距离保护

```python
if WALL_MODE == "dual":
    ok = (abs(self.e) < WALL_ABORT_M and alt > LIDAR_MIN_ALT_M)
    # 没有 FRONT_MIN_SAFE，也没有 back 下限
else:
    ok = (... and f_use > FRONT_MIN_SAFE ...)
```

### 🟡 B6：`CLIMB_LAT_ON_ALT = 0.60` 开启太早

起飞点偏 51 cm 时，高度 0.60 m 就开横向环，`MAX_FORWARD` 直接饱和 → 起飞阶段朝墙/后墙移动。

**修复：** 改成 99.0（爬升阶段完全不开），或加起飞前检查 `abs(e_lat) < 0.15`。

### 🟡 B7：**两份 `Lidar_source.py` 不同步**

| 路径 | `SMOOTH_N` | 谁在用 |
|---|---|---|
| `Control/Lidar_source.py` | **2** | ✅ `Fly_alt_safe.py` import 的是这份 |
| `Lidar/Lidar_source.py` | **5** | ❌ 旧副本 |

**改错文件会导致"改了没反应"。** 建议删掉或改名 `Lidar/` 那份。

### 🟡 B8：`_sanitize` 的 `est` 分支几乎不触发

改过阈值后，一次飞行 165 个 bad 帧里 `n_est = 0`。逻辑存在但未被验证有效。

### 🟡 B9：`Corridor_mission.py` 里 `HYDRANT_CONFIRM_N` NameError

配置已改名 `HYDRANT_WINDOW_N` / `HYDRANT_HITS`，但有一处 `print` 仍引用旧名 → 运行时报 `NameError`。**状态：未确认已修。**

### 🟡 B10：`MODEL_PATH` 大小写与目录不一致（Linux 上会直接报错）

```python
# vision/Vision_worker.py 第 16 行
MODEL_PATH = "/home/orangepi/drone/Vision/best.onnx"   # ⚠️ 大写 Vision
```

实际目录是 **`~/drone/vision/`（小写 v）**。Linux 区分大小写 → `cv2.dnn.readNetFromONNX` 会抛异常。

**修复：** 改成 `/home/orangepi/drone/vision/best.onnx`。

（其余视觉参数已核对：`CONF_THRES = 0.25`、`NMS_THRES = 0.45`、`IN_W/IN_H = 320`、`CAM_INDEX = 0`，与你已验证的 `vision/Yolo_test.py` 一致，**无需再改**。）

### 📌 `Plan.txt` 第 2 条未做

> 消防栓门打开时 front 读数变远，需在测完前后距离后加**后方距离限位**。

当前靠 `W` 一致性判定 + `back` 反推处理，但**"限位"逻辑未实现**。

---

## 9. 不要改什么

以下都是实测标定 / 踩过坑换来的约定。改动前必须重新验证。

| 不要动 | 原因 |
|---|---|
| **机头必须垂直于墙摆放** | `FRONT_DEG=0` / `BACK_DEG=180` 的语义就是"机头/机尾对着墙"。机头朝走廊时 `front=6.5`、`back=3~5`、`W≈10`，横向环全判 `bad` → `fwd` 恒 0（见 B11/§7-21） |
| **不要在机头朝走廊时侧飞** | 此时左右才是墙，而 `Lidar_source` **只解析 0° / 180° / ±15° 四个扇区**，左右 90°/270° 完全没有感知 → 侧飞无任何保护 |
| `Lidar_source.py` 的 `FRONT_DEG=0` / `BACK_DEG=180` / `WALL_HALF=20` / `YAW_DELTA=15` / `YAW_HALF=5` / `YAW_SIGN=-1.0` | 全部实测标定，平移 1:1 验证通过 |
| **不要在标定期间开着 `MPC_USE_HTE`** | 它会在线改写 `MPC_THR_HOVER`，导致每次飞行表现都不同（B14） |
| **不要靠调 `MPC_THR_HOVER` 补偿推重比不足** | 推重比 1.75 是硬件问题，参数只能在真值 ±0.02 内有效（B13） |
| `Control/Lidar_source.py` 的 `SMOOTH_N=2` | 降延迟后横向振荡明显改善 |
| `e_lat = (front - back) / 2` 及其符号约定 | 实测符号正确 |
| **`yawspeed` 传"度"而不是弧度** | 曾错 57.3 倍 |
| 高度环作为**独立任务**、只写 `cmd["down"]` | 与雷达环解耦，互不干扰 |
| 通道分工（一个通道一个写入者） | 混写会导致两个环互相覆盖 |
| PID 的条件积分写法 `if not (saturated and raw*e > 0)` | 单纯调小 `KI` 治不了 windup |
| `COM_RC_OVERRIDE = 0` | 否则油门最低被判为接管，踢出 Offboard |
| `EKF2_HGT_REF = Range sensor` | 室内高度必须用测距 |
| `W_NOMINAL_M = 2.49` | 五点实测一致 |
| **不要重新引入"积分分离"** | 上一次尝试几乎撞墙 |
| **不要用 Ctrl+C 中断飞行脚本** | 会杀 `mavsdk_server`，收尾全部失败；用 Enter |
| **不要在跑之前只改本地文件** | 一定要确认 Pi 上是同一份（已坑过两次） |
| 每个外环脚本必须 `try/finally` 保证 `offboard.stop()` + `land()` | |

---

## 10. 测试到哪一步

| # | 项目 | 状态 | 关键结果 |
|---|---|---|---|
| ① | 雷达静态标定（front/back/e_lat/yaw_err 符号） | ✅ 通过 | 平移 10.8 cm → `e_lat` ≈ -10.8 cm，1:1 |
| ② | 单墙定距 DRY_RUN（手推） | ✅ 通过 | `fwd` 方向/力度均正确 |
| ③ | 单墙定距 真飞 | ✅ 通过 | 最终误差 **+0.3 cm**（`e_lat` 从 -31 cm 收敛） |
| ④ | 纯侧飞 + 高度外环（无雷达） | ✅ 通过 | 最低高度 **1.15 m**（目标 1.30，低 15 cm） |
| ⑤ | 侧飞 2.0 m / 10 s @ 0.20 m/s | ✅ 通过 | 横向误差有界 |
| ⑥ | 视觉 ONNX 单测 | ✅ 通过 | 5 类可识别，≈15–16 FPS |
| ⑦ | 视觉状态机 手拿 DRY_RUN | ✅ 通过 | `cruise → align → inspect → resume` 全流程；`📋 累积 2.5s 检出: {4 类}` → `✅ 柜内齐全` |
| ⑧ | 侧飞 + 雷达双墙环 + 高度环（`Fly_alt_test.py`） | ⚠️ 跑通但有问题 | 高度 OK，雷达 0 跳变 / 0 bad 帧；横向 `e` 振荡发散，`back` 最低 0.409 m，航向漂 45° |
| ⑨ | **`Fly_alt_safe.py` 真飞（起飞 + 高度环 + 侧飞 20 s + 降落）** | 🟠 **跑通一次全流程，不可复现** | 首次完整跑通：`e` 有界 **±17 cm**、`yaw` 稳定 **±5°**、`front` 1.334~1.843、全程最低高度 1.22 m。**但同参数后续多次在起飞阶段撞墙/中止** |
| ⑩ | 视觉 + 真飞（`Corridor_mission.py`） | ❌ 未做 | 依赖 ⑨ 稳定 |
| ⑪ | **起飞阶段专项诊断（RC 手动 + 只读监视）** | 🔴 **定位到问题** | 3 s 水平位移 **0.97 m**（机身系后 0.68 / 右 0.69），垂直仅 0.36 m；RC 手动同样漂 + 偶尔抽搐，悬停后稳 → 起飞段是机体/PX4 层问题 |
| ⑫ | **`W = front + back ≈ 2.49` 姿态确认** | ❌ **未做（最先要做）** | 这一步不过，横向环没有任何意义 |

**当前阻塞点：⑫ → ⑨。** 按顺序：

1. **地面确认姿态**：原地转飞机直到 `W = front+back ≈ 2.49 ± 0.15`，贴胶带做记号（解 B11/§7-21）
2. **标定 `MPC_THR_HOVER`**：关 `MPC_USE_HTE` → 稳悬停 30 s → 看 `vehicle_thrust_setpoint.xyz`（解 B14）
3. **查光流**：`SENS_FLOW_ROT` / `EKF2_FLOW_POS_*` / `cs_opt_flow`（解 B15）
4. **称重评估推重比**，必要时减重/换动力（解 B13）
5. 以上四项完成，再回到 ⑨ 重跑全流程

**⑫ 不做，⑨ 永远不可复现。**

---

## 附：文件位置速查

```
~/drone/
├── Control/
│   ├── Fly_alt_safe.py      ← 当前唯一入口（侧飞 + 双墙环 + 高度环）
│   ├── Takeoff_back_wall.py ← 起飞专项测试（单后墙定距 + 垂直爬升）
│   ├── Lidar_source.py      ← 雷达驱动（SMOOTH_N=2，唯一在用的）
│   ├── Corridor_mission.py  ← 视觉版状态机（暂缓）
│   ├── Fly_vision_test.py   ← 用途 [未确认]
│   ├── Lidar_check.py       ← 静态自检（摆位后用这个确认 W≈2.49）
│   ├── Wall_hold_test.py    ← 单墙定距（已验证，起飞段逻辑可参考）
│   └── Side_fly_test_v2/v3/v4_alt.py, Fly_alt_test.py, ...（历史）
├── Lidar/
│   ├── Lidar_source.py      ← ⚠️ 旧副本（SMOOTH_N=5），别改这份
│   └── Lidar_scan.py        ← 点云可视化
├── vision/
│   ├── Vision_worker.py
│   ├── Yolo_test.py         ← CONF=0.25，视觉参数以此为准
│   └── best.onnx
├── Viewer/                  ← 地面站（当前流程未接入）
├── 进环境.txt
├── docs/
│   └── PROJECT_STATE.md     ← 本文件
└── venv/
```

**只在机载电脑上（开发机没有）：**

```
~/drone/Control/Watch.py     ← 只读监视：雷达真值 vs EKF 估计（不 arm / 不发 setpoint）
~/drone/Plan.txt             ← 需求备忘（开发机上没有这份）
```

**仅在本对话中给出、未落盘：**

- `Takeoff_px4_auto.py`（用 `action.takeoff()` 让 PX4 自己起飞，到高度再切 Offboard）
- `find_wall_pose.py`（地面原地转飞机，找 `W≈2.49` 的姿态）

---

## 附：更新记录

> 本档案按「第几次更新」计数。每次实质改动递增一次，不回溯改写历史条目。

| 次序 | 主要内容 |
|---|---|
| **第 1 次** | 建立档案：项目目标、硬件/串口、数据流与三条循环、坐标系约定、参数表、测试进度 ①~⑩ |
| **第 2 次** | 补全 B1~B10；记录 `Fly_alt_safe.py` 安全修订（setpoint 20 Hz、任务监控、超时收尾、实测 dt）；雷达标定常量与飞控参数 |
| **第 3 次** | ① `Fly_alt_safe.py` 真飞跑通一次全流程，但不可复现<br>② 定位起飞横向漂移（3 s / 0.97 m，水平:垂直 = 2.7:1）<br>③ 确认**机头必须垂直对墙**的硬几何前提（机头朝走廊时横向环全判 `bad`、`fwd` 恒 0）<br>④ 反推推重比 ≈ 1.75，确认 `MPC_USE_HTE` 会在线改写 `MPC_THR_HOVER`<br>⑤ 修 B1/B4/B5/B6，新增 B11~B16<br>⑥ 新增 `Takeoff_back_wall.py`；`Watch.py`（机载电脑）<br>⑦ 参数：`KP_LAT` 0.35→0.30、`MAX_FORWARD` 0.22→0.20、`SIDE_STOP_FRONT` 0.60→0.90、新增 `BACK_MIN_SAFE=0.65` / `E_LAT_TOL_TAKEOFF=0.15` |
