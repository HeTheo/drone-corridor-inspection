# 无人机走廊巡检（大创项目）

基于 PX4 + MAVSDK-Python 的四旋翼**走廊自主巡检**原型：用 2D 雷达贴着两面墙侧飞，靠视觉识别消防栓并检查柜内器材是否齐全。

> 当前状态：**全流程跑通过一次（横向 ±17 cm / 航向 ±5°），但不可复现 —— 现在卡在起飞阶段的横向漂移（3 s 内 1 m）。**
> 视觉链路单测通过，未上真机。
> 更新：**第 3 次**（本文与 `docs/PROJECT_STATE.md` 同步递增）。
> 详细技术档案见 [`docs/PROJECT_STATE.md`](docs/PROJECT_STATE.md)。

---

## 目录

- [这个项目做了什么](#这个项目做了什么)
- [现在的进度](#现在的进度)
- [硬件与软件环境](#硬件与软件环境)
- [目录结构](#目录结构)
- [快速开始](#快速开始)
- [飞行前检查清单](#飞行前检查清单)
- [常调参数](#常调参数)
- [已知问题](#已知问题)
- [下一步计划](#下一步计划)

---

## 这个项目做了什么

无人机在一条两侧是墙的走廊里，需要同时满足三件事：

| 保持量 | 目标 | 靠什么实现 |
|---|---|---|
| 横向位置 | 离前墙 1.60 m | 2D 雷达测前后墙距 + PID |
| 机头朝向 | 与墙垂直 | 雷达 ±15° 双波束算偏角 + PID |
| 飞行高度 | 1.30 m | 光流/测距 + 高度外环 |

在此基础上侧飞前进，摄像头识别到消防栓（`Fire_hydrant`）时停下、精调位置，再判断柜内四类器材（`Valve` / `Fire_hose` / `Fire_nozzle` / `Rubber_hose`）是否齐全。

**为什么不用 GPS？** 室内无 GPS，PX4 用**光流 + 下视测距**做 EKF 位置估计。光流没有绝对参考，会缓慢漂移 —— 所以横向偏移必须由**雷达外环**来闭合，不能靠调遥控器微调解决。

---

## 现在的进度

### ✅ 已验证能用

| 模块 | 结果 |
|---|---|
| 雷达标定（前/后波束、符号、1:1 比例） | 平移 10.8 cm → 读数变化 10.8 cm，误差 < 1 cm |
| 单墙定距悬停（`Wall_hold_test.py`） | 最终误差 **+0.3 cm** |
| 高度外环（`alt_hold_task`） | 侧飞中最低高度从 0.53 m 提升到 **1.22 m** |
| **`Fly_alt_safe.py` 全流程（首次）** | 起飞 → 1.35 m → 侧飞 20 s → 降落，`e` 有界 **±17 cm**、`yaw` 稳定 **±5°** |
| 后墙保护 + 起飞点门禁 | 已加（`BACK_MIN_SAFE=0.65`、`E_LAT_TOL_TAKEOFF=0.15`、全程 `guard_task`） |
| 视觉 ONNX 推理 | 树莓派级板子上 **15–16 FPS**，5/6 类可识别 |
| 视觉状态机（手拿 DRY_RUN） | `cruise→align→inspect→resume` 全流程通过 |

### 🔴 当前唯一阻塞点：起飞阶段横向漂移

| 实测 | 数值 |
|---|---|
| 起飞 3 s 内水平位移 | **0.97 m**（机身系：向后 0.68 / 向右 0.69） |
| 同期垂直爬升 | 0.36 m |
| **水平 : 垂直** | **2.7 : 1** |

同参数第一次成功、后续多次撞墙/中止 —— **不可复现**。
**用遥控纯手动起飞也照样漂（还会偶尔抽搐），悬停后就稳了 → 这是机体/PX4 层的问题，不是脚本。**

### ⚠️ 已定位但未解决

| 问题 | 结论 |
|---|---|
| 推重比 ≈ **1.75** | 由 `MPC_THR_HOVER=0.57` 反推，推力余量薄，倾斜权限被爬升吃掉 |
| `MPC_THR_HOVER` 从未标定 | 盲试过 0.55/0.65/0.60/0.57；且 `MPC_USE_HTE=1` 会在飞行中在线改写它 |
| 光流配置未核对 | 漂移在机身系下是 **45° 对角**，符合 `SENS_FLOW_ROT` / `EKF2_FLOW_POS_*` 配错的指纹 |
| `WALL_ABORT_M=0.80` 的"放弃纠偏区" | `front < 0.82 m` 时横向环主动输出 0；实测曾到 **0.273 m** |

### ❌ 还没做

- `W = front+back ≈ 2.49` 的摆位确认（**这一步不过，后面全白做**）
- 完整走廊跑通（可复现）
- 视觉 + 真飞融合（`Corridor_mission.py` 真飞）
- 消防栓开门后的后方距离限位（`Plan.txt` 第 2 条）

---

## 硬件与软件环境

### 硬件

| 部件 | 型号 |
|---|---|
| 飞控 | MicoAir743v2（STM32H743VIT6） |
| 机载电脑 | Orange Pi 4 Pro（aarch64 / Ubuntu 22.04） |
| 2D 雷达 | **N10**（230400 baud，帧头 `A5 5A 3A`） |
| 下视传感器 | 光流 + 测距一体模块 |
| 电池 | 3S 2200 mAh 35C |

### 接口与波特率

| 连接 | 设备节点 | 波特率 |
|---|---|---|
| 飞控 ↔ 机载电脑 | `/dev/pixhawk` | **57600** |
| 2D 雷达 | `/dev/wheeltec_lidar` | **230400** |
| 摄像头 | `/dev/video0` | — |

两个 `udev` 别名是固定下来的，插拔顺序变了也不会变。雷达走 CDC-ACM，帧头 `A5 5A 3A`，帧长 55 字节，每帧 16 点。

### 软件

```bash
# 机载电脑上（~/drone/venv）
source ~/drone/venv/bin/activate
```

| 包 | 版本 | 备注 |
|---|---|---|
| `mavsdk` | 3.17.4 | **必须 < 4**，4.x 接口不兼容 |
| `grpcio` | 1.84.0 | |
| `pyserial` | 3.5 | 雷达串口 |
| `opencv-python` | 5.0.0.93 | |
| `numpy` | 2.2.6 | |

> `ultralytics` 在 aarch64 上装不上（torch / cudnn 依赖问题），所以视觉走 **ONNX + `cv2.dnn`** 这条路线。

---

## 目录结构

```
drone/
├── Control/
│   ├── Fly_alt_safe.py      ⭐ 当前唯一入口（侧飞 + 雷达双墙环 + 高度环）
│   ├── Takeoff_back_wall.py   起飞专项测试（单后墙定距 + 垂直爬升）
│   ├── Lidar_source.py      ⭐ 雷达驱动（FakeLidar / RealLidar 抽象）
│   ├── Lidar_check.py         雷达静态自检（摆位后用它确认 W≈2.49）
│   ├── Wall_hold_test.py      单墙定距测试（已验证，起飞段写法可参考）
│   ├── Corridor_mission.py    视觉版状态机（未真飞）
│   ├── Fly_vision_test.py     用途 [未确认]
│   └── Side_fly_test_v*.py    历史迭代版本
├── vision/
│   ├── Vision_worker.py       视觉子进程（ONNX 推理）
│   ├── Yolo_test.py           单独跑检测（参数以此为准）
│   └── Camera_test.py         摄像头自检
├── Lidar/
│   ├── Lidar_scan.py          原始点云可视化
│   └── Lidar_source.py      ⚠️ 旧副本，别改这份
├── Viewer/                    地面站（当前流程未接入）
├── docs/
│   └── PROJECT_STATE.md     ⭐ 完整技术档案
└── venv/
```

> `Watch.py`（只读监视：雷达真值 vs EKF 估计）和 `Plan.txt` **只在机载电脑上**，开发机没有。

---

## 快速开始

### 1. 先跑雷达自检（不上电，只插雷达）

```bash
cd ~/drone/Control
python Lidar_check.py
```

应该看到 `front` / `back` 读数稳定、手推飞机时一增一减，**并且 `W = front + back ≈ 2.49 m`**。

**这一步不过，别往下走。** 特别是 `W` 不是 2.49 的时候 —— 说明机头没有垂直对墙，后面所有横向逻辑都是空转。

### 2. 飞正式脚本

```bash
cd ~/drone/Control
python Fly_alt_safe.py
```

脚本流程：

```
等 local position OK  →  起飞点检查（|e_lat| ≤ 15 cm 才解锁）
   →  起飞爬升  →  hold 稳定  →  ready（等横向误差 < 8 cm）
   →  side 侧飞 20 s  →  stop 悬停 8 s  →  收尾降落
```

**停止方式：按回车（不是 Ctrl+C）。**

> ⚠️ 当前**起飞阶段横向会漂 ~1 m**，即使起飞点摆正也一样。原因是推重比只有 ~1.75，不是参数问题。
> 详见 [`docs/PROJECT_STATE.md`](docs/PROJECT_STATE.md) §8 B12/B13。

> ⚠️ Ctrl+C 会把 `mavsdk_server` 一起杀掉，导致 `offboard.stop()` 和 `land()` 全部失败 —— 飞机就挂在天上不动了。

### 3. 视觉单独测试（不飞）

```bash
cd ~/drone/vision
python Yolo_test.py
```

---

## 飞行前检查清单

上电前逐条确认：

- [ ] 🔴 **机头垂直于墙**，并确认 `W = front + back ≈ 2.49 ± 0.15 m`
  - ⚠️ **机头朝走廊方向 = 雷达测的不是墙**（`front≈6.5`、`back≈3~5`、`W≈10`），横向环会全判 `bad`、`fwd` 恒为 0，**等于没装控制器**
  - 用 `python Lidar_check.py` 原地转飞机，直到 `W` 落到 2.49，**在地上贴胶带做记号**，以后照记号摆
- [ ] **起飞点摆正**：`front ≈ 1.61 m`、`back ≈ 0.88 m`（dual 模式目标位置）
  - 脚本已加门禁：`|e_lat| > 0.15 m` 会**拒绝解锁**
  - 若只是验证横向环，摆**走廊正中间**（`front≈back≈1.25`）更安全，两边各留 1.25 m
- [ ] 雷达窗口朝向正确，前波束对准机头
- [ ] 桨叶无损伤、电池卡紧
- [ ] **满电起飞**（推重比只有 ~1.75，电压一掉推力余量就没了）
- [ ] 遥控器在手，**已确认切回 Position 模式能接管**
- [ ] 飞控参数 `COM_RC_OVERRIDE = 0`（否则油门最低会被判为飞手接管）
- [ ] `EKF2_HGT_REF = Range sensor`
- [ ] 标定 `MPC_THR_HOVER` 时先 `MPC_USE_HTE = 0`（否则值会被在线改写）
- [ ] 机载电脑上跑的脚本和你本地改的是**同一份**（这个坑踩过两次）
- [ ] 屏幕能看到终端输出（要按回车停）

---

## 常调参数

改 `Control/Fly_alt_safe.py` 顶部配置区。**一次只改一个，改完记下来。**

### 侧飞

```python
V_SIDE      = 0.35      # 侧飞速度 m/s（已验证范围 0.20~0.30，0.35 未充分验证）
SIDE_TIME_S = 20.0      # 侧飞时长 s
SIDE_DIR    = +1.0      # +1 右 / -1 左
```

### 横向 PID（第 3 次更新调整）

```python
KP_LAT, KI_LAT, KD_LAT = 0.30, 0.02, 0.0    # 由 0.35 / 0.04 下调
MAX_FORWARD     = 0.20                       # 由 0.22 下调
WALL_ABORT_M    = 0.80                       # ⚠️ 见下方"放弃纠偏区"
```

判据：`KP_LAT × 雷达延迟` 接近 0.44 就会自激振荡。延迟主要来自雷达平滑窗口。
调整后最近一次实测 `e` 有界 **±17 cm**（此前曾冲到 +47 cm）。

> ⚠️ **"放弃纠偏区"**：`|e| ≥ WALL_ABORT_M (0.80)` 时不是继续纠偏，而是 `fwd = 0` + PID 清零。
> `dual` 模式下这等价于 `front < 0.815 m` —— 也就是**离前墙 82 cm 时横向环主动罢工**。

### 墙距保护（第 3 次更新新增）

```python
SIDE_STOP_FRONT   = 0.90    # side 阶段：front 低于此值中止（原 0.60）
BACK_MIN_SAFE     = 0.65    # side 阶段 + guard_task：back 低于此值中止
E_LAT_TOL_TAKEOFF = 0.15    # 起飞点横向偏差超此值 → 拒绝解锁
```

`guard_task` 每 50 ms 检查一次 `dist` 快照，**climb / hold / ready 三个阶段也受保护**。

### 高度

```python
ALT_TARGET_M   = 1.30     # 目标高度 m
HOVER_ALT_DOWN = -1.35    # 爬升命令目标（NED，负=向上）
CLIMB_RATE     = -0.35    # 爬升速度 m/s
KP_ALT, KI_ALT = 0.50, 0.10
CLIMB_LAT_ON_ALT = 0.60   # 高度超过此值才允许开横向环
```

> 注意：爬升停在 1.35 m，进入 `hold` 后主动降到 1.30 m，这 **5 cm 下降是预期的**。
> `CLIMB_LAT_ON_ALT = 0.60` 配合 `LIDAR_MIN_ALT_M = 0.50` 意味着 **0.6 m 以下完全不纠偏** —— 而实测起飞漂移恰好在这段产生。

### 雷达数据质量

```python
MAX_JUMP_M  = 0.25   # 单次跳变阈值
MAX_BAD_CNT = 8      # 连续几次才接受
W_NOMINAL_M = 2.49   # 走廊宽度标定值（多处实测一致）——也是"机头对墙"的唯一判据
```

### 飞控侧（必须先标定，别盲调）

```bash
param set MPC_USE_HTE 0        # ⚠️ 必须先关，否则 MPC_THR_HOVER 会被在线改写
param show MPC_THR_HOVER       # 当前 0.57 → 推重比 ≈ 1.75，偏紧
```

标定方法：稳悬停 1.3 m 以上 30 s → 拔 SD 卡 → `review.px4.io` 看 `vehicle_thrust_setpoint.xyz` 的均值。

---

## 已知问题

按危险程度排序。完整分析见 `docs/PROJECT_STATE.md` 第 8 节。

| 级别 | 问题 | 影响 |
|---|---|---|
| 🔴 | **起飞阶段横向漂 ~1 m**（3 s 内 0.97 m，水平:垂直 = 2.7:1） | **当前唯一阻塞点**；同参数不可复现，RC 手动也漂 |
| 🔴 | **推重比 ≈ 1.75**（`MPC_THR_HOVER=0.57`） | 推力余量薄，爬升时无横向姿态权限 |
| 🔴 | **`MPC_THR_HOVER` 从未标定 + `MPC_USE_HTE=1` 会在线改写** | 每次飞完表现都不一样 |
| 🟠 | **机头朝走廊 = 雷达完全失效** | `W≈10` → 全判 `bad` → `fwd` 恒 0，横向环等于没装 |
| 🟠 | **`WALL_ABORT_M=0.80` 的"放弃纠偏区"** | `front < 0.82 m` 时横向环主动输出 0；实测到过 0.273 m |
| 🟠 | 光流配置未核对（45° 对角漂移） | `SENS_FLOW_ROT` / `EKF2_FLOW_POS_*` 可能配错 |
| 🟠 | 降落只校验"命令已发出" | `land()` 失败时脚本仍报"✅ 完成"，飞机可能在空中 |
| 🟡 | `MODEL_PATH` 写成 `Vision/`（实际是 `vision/`） | Linux 大小写敏感 → 视觉启动失败 |
| 🟡 | `Corridor_mission.py` 里 `HYDRANT_CONFIRM_N` 未定义 | 运行时报 `NameError` |
| 🟡 | 两份 `Lidar_source.py` 不同步（`SMOOTH_N` 2 vs 5） | 改错文件等于没改 |
| 🟡 | `_sanitize` 的 `est` 分支永远不触发 | `good_back` 从未赋初值，反推逻辑是死代码 |

### ✅ 已修

| 原问题 | 修复 |
|---|---|
| 没有后墙保护 | `BACK_MIN_SAFE=0.65` + 全程 `guard_task` |
| `side_abort` 只清 `right` | 改为 `raise FlightAbort`，走统一收尾、四通道全清零 |
| 起飞点摆偏 | `E_LAT_TOL_TAKEOFF=0.15`，超限拒绝解锁 |
| `guard_task` 参数个数不匹配（会 `TypeError`） | 改为只接收 `stop_event`，数据从 `dist` 取，且**不自己读雷达** |

---

## 下一步计划

### 立刻要做的（**按顺序，每步只验一个变量**）

1. 🔴 **确认摆位**：原地转飞机直到 `W = front + back ≈ 2.49`，**在地上贴胶带做记号**
   —— 这一步不过，横向环没有任何意义（机头朝走廊时 `fwd` 恒为 0）
2. 🔴 **标定 `MPC_THR_HOVER`**：
   - `param set MPC_USE_HTE 0`（必做，否则值会被在线覆盖）
   - 稳悬停 1.3 m 以上 30 s → 拔 SD 卡 → `review.px4.io` 看 `vehicle_thrust_setpoint.xyz` 均值
   - `param set MPC_THR_HOVER <那个数>`
3. 🟠 **查光流**：`SENS_FLOW_ROT` / `EKF2_FLOW_POS_X/Y/Z`；日志看 `cs_opt_flow`、`dead_reckoning`
4. 🟠 **评估推重比**：称整机重量。`T/W < 1.8` 就开始考虑减重或换动力
5. 🟠 **起飞点改到走廊正中间**（`front ≈ back ≈ 1.25`），两边各留 1.25 m 余量
6. 🟠 把 `SIDE_STOP_FRONT` 提到 0.95~1.00，让中止永远早于"放弃纠偏"

### 然后

7. 修 `MODEL_PATH` 大小写，跑通视觉
8. `Plan.txt` 第 2 条：消防栓开门后 front 读数变远，加后方距离限位
9. 完整走廊真飞（可复现）
10. 视觉 + 真飞融合（`Corridor_mission.py`）

> **不要在这四步之前改任何 PID 参数。** 起飞漂移是机体/推重比层面的问题，调增益测不出来。

---

## 踩过的坑（省点你的时间）

| 坑 | 教训 |
|---|---|
| **机头朝走廊方向测横向环** | `front=6.5`、`back=3~5`、`W≈10` → 全判 `bad` → **`fwd` 恒为 0**。横向环一次都没工作过，白调了两周参数。**先看 `W` 是不是 2.49** |
| **盲调 `MPC_THR_HOVER`** | 0.55→0.65→0.60→0.57 全是猜的。而且 `MPC_USE_HTE=1` 会在飞行中**在线改写**它，每次飞完都不一样。**必须先关 HTE 再量** |
| **把起飞漂移当成 PID 问题** | 遥控纯手动起飞也照样漂 → 是推重比（1.75）和地面效应，不是增益 |
| **调参数时不看 `fwd` 是否非零** | 输入被拒时 `fwd` 恒为 0，任何增益都测不出效果。**先确认执行器是通的** |
| `guard_task` 定义 2 个参数、调用只传 1 个 | `TypeError` 抛在 `try` 之前 → `finally` 的 `offboard.stop()`/`land()` 全不执行。**新增任务后立刻 `py_compile`** |
| 让两个任务同时 `lidar.read()` | 阻塞读会卡住事件循环 → `send_velocity` 断流 → PX4 掉出 offboard。**一个雷达只能有一个消费者** |
| 把弧度当 deg/s 发给 PX4 | 航向环弱了 57.3 倍。`yawspeed` 单位是 **deg/s** |
| 积分一直涨到上限 | 不是增益大，是 windup。用条件积分，别调小 `KI` |
| 试"积分分离" | 同号积分永远卸不掉，差点撞墙。**不要用** |
| 用 Ctrl+C 停脚本 | 会杀 `mavsdk_server`，收尾全失败。**用回车** |
| 数传干扰罗盘 | 拆掉数传后 `heading estimate not stable` 消失 |
| 仿真 `KP_YAW=30` 直接用 | 仿真无噪声，真机只能 3~5 |
| 以为掉高是电池问题 | 电压 12.6→12.3 V 只用了 10%。真实原因是平移缺推力 |
| 起飞点摆偏 | 爬升中途就开横向环 → 饱和 → 朝墙冲（现已加门禁） |
| 以为锂电池"低于 12.6 V 就充"会伤电池 | 浅充浅放不伤。真正伤电池的是**充满后长期存放**（>3 天），要放电到 11.4 V 左右储存 |

---

## 参考

- [PX4 光流 + 测距配置](https://docs.px4.io/main/en/sensor/optical_flow.html)
- [MAVSDK-Python 文档](https://mavsdk.mavlink.io/main/en/python/)
- 项目完整技术档案：`docs/PROJECT_STATE.md`
