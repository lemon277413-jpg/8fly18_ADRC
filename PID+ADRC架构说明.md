# 8fly18_ADRC — PID+ADRC 串级飞控架构详解

## 项目概览

本项目基于原 8fly18 双环 PID 飞控改造：**保留外环角度 PID 控制，将内环角速度控制由 PID 替换为一阶线性自抗扰控制器 (First-Order LADRC)**。

```
原项目 (纯PID):                    本项目 (PID+ADRC):
┌──────────┐   ┌──────────┐       ┌──────────┐   ┌──────────┐
│ 角度 PID  │──▶│ 角速度 PID│       │ 角度 PID  │──▶│ 角速度   │
│ (外环)    │   │ (内环)    │       │ (外环)    │   │ LADRC    │
│ 100Hz     │   │ 200Hz     │       │ 100Hz     │   │ (内环)   │
└──────────┘   └──────────┘       └──────────┘   │ 200Hz    │
                                                  └──────────┘
```

---

## 一、文件结构

```
8fly18_ADRC/
├── 8fly18_ADRC.ino          ← 主控程序 (调度器、状态机、控制链路)
├── pid.h / pid.cpp          ← PID 控制器 (角度环、高度环、位置/速度环)
├── adrc_rate.h / adrc_rate.cpp ← 【新增】ADRC 角速度控制器 (替代角速度 PID)
├── AttitudeEstimator.h/.cpp ← Mahony 姿态解算 + MPU6050 驱动
├── uwb.h / uwb.cpp          ← UWB+IMU 扩展卡尔曼滤波融合
├── mtf02.h / mtf02.cpp      ← MTF02 光流/TOF 传感器协议解码
├── CircularDelay.h/.cpp     ← 环形延迟线 (光流-陀螺仪时间对齐)
├── EKF.h / EKF.cpp          ← 一维卡尔曼滤波 (加速度计滤波)
├── flow_decode.h/.c         ← 旧版光流协议解码器 (备用)
├── check.h / check.c        ← CRC 校验算法
└── up_flow.h / up_flow.c    ← 旧版光流融合 (已弃用)
```

### PID 与 ADRC 职责划分

| 控制层 | 控制器 | 类型 | 频率 | 输入 | 输出 |
|--------|--------|------|------|------|------|
| **角度环** (外环) | `PIDController` | 角度 PID ×3 | 100Hz + 50Hz | 角度偏差 (°) | 目标角速度 (°/s) |
| **角速度环** (内环) | `ADRC_Rate` | 一阶 LADRC ×3 | 200Hz | 角速度偏差 (°/s) | 电机修正量 |
| **高度环** | `PIDController` | 高度 PID + 高度速度 PID | 50Hz | 高度偏差 (m) | 油门补偿 |
| **位置环** | `PIDController` | 位置 PID + 速度 PID ×2 | 25Hz/50Hz | 位置偏差 (m) | 目标加速度→目标角度 |

---

## 二、系统调度与数据流

### 2.1 时间基准

硬件定时器 `Timer5` 以 **200Hz (5ms)** 的基准频率触发中断。通过分频产生四级任务速率：

```
ISR(TIMER5_COMPA_vect) ───────── 200Hz 基准
  freqTick % 1 == 0  → flag_200Hz  (每 5ms)
  freqTick % 2 == 0  → flag_100Hz  (每 10ms)
  freqTick % 4 == 0  → flag_50Hz   (每 20ms)
  freqTick % 8 == 0  → flag_25Hz   (每 40ms)
```

### 2.2 主循环完整数据流

```
loop()
│
├─ 200Hz (每 5ms) ─────────────────────────────────────────────┐
│  ┌──────────────────────────────────────────────────────┐     │
│  │ 1. read_IMU_Rate()                                   │     │
│  │    MPU6050 陀螺仪 → 去偏置 → 单位转换 → 低通滤波     │     │
│  │    → rollRate_200hz, pitchRate_200hz, yawRate_200hz  │     │
│  │    (单位: °/s)                                        │     │
│  │                                                       │     │
│  │ 2. ADRC 角速度环 (替代原来的 calculateRatePID)        │     │
│  │    ┌────────────────────────────────────────────┐    │     │
│  │    │ rollOut  = rollADRC.update(targetRollRate, │    │     │
│  │    │                            rollRate_200hz)  │    │     │
│  │    │ pitchOut = pitchADRC.update(targetPitchRate,│    │     │
│  │    │                            pitchRate_200hz) │    │     │
│  │    │ yawOut   = yawADRC.update(targetYawRate,    │    │     │
│  │    │                            yawRate_200hz)   │    │     │
│  │    └────────────────────────────────────────────┘    │     │
│  │    │                                                 │     │
│  │    │ constrain: roll/pitch ±300, yaw ±100            │     │
│  │    ▼                                                 │     │
│  │ 3. escCtrl(throttle, rollOut, pitchOut, yawOut)      │     │
│  │    X型四轴混控 → 4路 PWM → 电调 → 电机               │     │
│  └──────────────────────────────────────────────────────┘     │
│                                                                │
├─ 100Hz (每 10ms) ────────────────────────────────────────────┐│
│  ┌──────────────────────────────────────────────────────┐    ││
│  │ 1. read_IMU_Angle()                                  │    ││
│  │    MPU6050 加速度计+陀螺仪 → Mahony 互补滤波         │    ││
│  │    → roll, pitch, yaw  (单位: °)                     │    ││
│  │                                                       │    ││
│  │ 2. calculateAnglePID()                                │    ││
│  │    ┌────────────────────────────────────────────┐    │    ││
│  │    │ PID 角度环 (横滚)                           │    │    ││
│  │    │   Kp=3.5, Ki=0.2, Kd=0                     │    │    ││
│  │    │   roll ──────────────▶ targetRollRate       │    │    ││
│  │    │   (测量角度)            (目标角速度 °/s)     │    │    ││
│  │    │                                            │    │    ││
│  │    │ PID 角度环 (俯仰)                           │    │    ││
│  │    │   Kp=3.5, Ki=0.2, Kd=0                     │    │    ││
│  │    │   pitch ─────────────▶ targetPitchRate      │    │    ││
│  │    │                                            │    │    ││
│  │    │ PID 角度环 (偏航)                           │    │    ││
│  │    │   Kp=20.5, Ki=0, Kd=0                      │    │    ││
│  │    │   yaw ───────────────▶ targetYawRate        │    │    ││
│  │    └────────────────────────────────────────────┘    │    ││
│  │                                                       │    ││
│  │ 输出: targetRollRate, targetPitchRate, targetYawRate  │    ││
│  │ 这些值供下一个 200Hz 周期的 ADRC 内环使用              │    ││
│  └──────────────────────────────────────────────────────┘    ││
│                                                               ││
├─ 50Hz (每 20ms) ───────────────────────────────────────────┐ ││
│  ┌──────────────────────────────────────────────────────┐  │ ││
│  │ 1. read_remote_control()                             │  │ ││
│  │    RC接收机 PWM → targetRoll/targetPitch (±10°)      │  │ ││
│  │                                                       │  │ ││
│  │ 2. readMTF02() / readUWB()                           │  │ ││
│  │    传感器数据解码                                     │  │ ││
│  │                                                       │  │ ││
│  │ 3. Cal_UWB_Position()                                │  │ ││
│  │    UWB+IMU EKF 融合 → 位置+速度估计                   │  │ ││
│  │                                                       │  │ ││
│  │ 4. Cal_MTF02_Position()                              │  │ ││
│  │    光流+陀螺仪 → 位置+速度+高度                       │  │ ││
│  │                                                       │  │ ││
│  │ 5. 状态机 ──▶ FLY / HOLD_POSITION / ...              │  │ ││
│  │                                                       │  │ ││
│  │ 6. calculateAnglePID()  (第二次计算)                  │  │ ││
│  │    HOLD_POSITION 模式下覆盖了 targetRoll/Pitch        │  │ ││
│  │    → 更新 targetRollRate/PitchRate/YawRate            │  │ ││
│  └──────────────────────────────────────────────────────┘  │ ││
│                                                              │ ││
└─ 25Hz (每 40ms) ─── HOLD_POSITION 时使用 ─────────────────┘ ││
   ┌──────────────────────────────────────────────────────┐   ││
   │ cal_Position_PID()   位置偏差 → 目标速度              │   ││
   └──────────────────────────────────────────────────────┘   ││
```

### 2.3 关键数据变量

```
                              ┌── 系统全局变量 ──┐
                              │                   │
  传感器读数                   │  期望值 (target)  │         控制输出
  ─────────                   │  ──────           │         ────────
  roll               ◀───────▶ targetRoll         │
  pitch              ◀───────▶ targetPitch        │
  yaw                ◀───────▶ targetYaw          │
                              │                   │
  rollRate_200hz     ◀───────▶ targetRollRate     │──────▶ rollOut  (ADRC)
  pitchRate_200hz    ◀───────▶ targetPitchRate    │──────▶ pitchOut (ADRC)
  yawRate_200hz      ◀───────▶ targetYawRate      │──────▶ yawOut   (ADRC)
                              │                   │
  MTF_PosX                    target_X_Position   │
  MTF_PosY                    target_Y_Position   │
  MTF_Height                  target_Height       │
                              └───────────────────┘
```

数据流向：

```
传感器 → 姿态解算 → ┌─ 外环角度PID ──▶ target*Rate ──▶ 内环ADRC ──▶ 电机混控 ──▶ PWM
                    │   (100Hz)          (deg/s)        (200Hz)      (200Hz)
                    │
                    └─ [可选] 位置PID→速度PID→角度 (HOLD_POSITION模式, 50Hz)
                             位置(m)   速度(m/s)  角度(°)
```

---

## 三、PID 与 ADRC 的拼接方式

### 3.1 接口约定

PID 外环和 ADRC 内环之间只通过一个变量传递信息：**目标角速度 (target*Rate)**，单位 °/s。

```
外环输出 (PID)  ──target*Rate──▶  内环输入 (ADRC)
  (角度→角速度)                     (角速度→电机修正)
```

### 3.2 拼接代码

**外环 (100Hz) — 角度 PID 输出目标角速度：**

```cpp
// 文件: 8fly18_ADRC.ino, calculateAnglePID()

void calculateAnglePID() {
    // 角度环 PID 计算 (P=3.5, I=0.2, D=0)
    pidController.calCurrentRollAnglePID(roll, targetRoll);
    pidController.calCurrentPitchAnglePID(pitch, targetPitch);
    pidController.calCurrentYawAnglePID(yaw, targetYaw);

    // 取出角度 PID 输出，作为角速度环的参考输入
    targetRollRate  = pidController.getRollCorrect(PIDController::ANGLE);
    targetPitchRate = pidController.getPitchCorrect(PIDController::ANGLE);
    targetYawRate   = pidController.getYawCorrect(PIDController::ANGLE);
}
```

**内环 (200Hz) — ADRC 接收目标角速度，输出电机修正：**

```cpp
// 文件: 8fly18_ADRC.ino, 200Hz loop

// 读取陀螺仪角速度 (测量值)
read_IMU_Rate();   // → rollRate_200hz, pitchRate_200hz, yawRate_200hz

// ADRC 角速度环 (一阶LADRC)
float rollOut  = rollADRC.update(targetRollRate, rollRate_200hz);
float pitchOut = pitchADRC.update(targetPitchRate, pitchRate_200hz);
float yawOut   = yawADRC.update(targetYawRate, yawRate_200hz);

// 安全限幅
rollOut  = constrain(rollOut, -300.0f, 300.0f);
pitchOut = constrain(pitchOut, -300.0f, 300.0f);
yawOut   = constrain(yawOut, -100.0f, 100.0f);

// 电机混控
escCtrl(throttle, rollOut, pitchOut, yawOut);
```

### 3.3 初始化代码

```cpp
// 文件: 8fly18_ADRC.ino, setup()

// ADRC 初始化 (200Hz, Ts=0.005s)
rollADRC.init(0.005f, 20.0f, 30.0f, 300.0f);    // wc=20, b0=30, ±300
pitchADRC.init(0.005f, 20.0f, 30.0f, 300.0f);
yawADRC.init(0.005f, 25.0f, 40.0f, 100.0f);     // wc=25, b0=40, ±100

// 角度环 PID 仍使用 PIDController 初始化
pidController.cleanRollPIDData();
pidController.cleanPitchPIDData();
pidController.cleanYawPIDData();
```

---

## 四、ADRC 算法详解 (一阶 LADRC)

### 4.1 数学模型

将四轴角速度动态建模为一阶系统：

$$\dot{y} = f(y, w, t) + b_0 \cdot u$$

其中：
- $y$：角速度 (测量值，°/s)
- $u$：控制输入 (电机修正量，PWM 差值)
- $f$：**总扰动** — 包含未建模动态、外部风扰、陀螺效应、参数不确定性
- $b_0$：控制增益估计 (系统对输入的敏感度)

### 4.2 两大核心组件

```
                 ┌─────────────────────────┐
     target ────▶│  (2) 控制律              │──▶ output ──▶ 电机
                 │  u₀ = wc·(ref - z₁)     │
  measured ──┬──▶│  u   = (u₀ - z₂) / b₀   │
             │   └──────────┬──────────────┘
             │              │ z₁, z₂
             │   ┌──────────▼──────────────┐
             └──▶│  (1) ESO 扩张状态观测器   │
                 │  e  = y - z₁             │
                 │  ż₁ = z₂ + b₀u + β₁e    │
                 │  ż₂ = β₂e               │
                 └─────────────────────────┘
```

### 4.3 每步计算 (200Hz)

```cpp
// ===== 第1步：ESO 观测器 =====
float e_eso = measuredRate - z1;           // 观测误差
z1 += Ts * (z2 + b0 * output + beta1 * e_eso);  // 状态估计
z2 += Ts * (beta2 * e_eso);                     // 扰动估计

// ===== 第2步：控制律 =====
float e_ctrl = targetRate - z1;            // 跟踪误差
float u0 = wc * e_ctrl;                    // P 控制 (比例)
float output_raw = (u0 - z2) / b0;         // 扰动补偿 + 增益逆变换

// ===== 第3步：输出限幅 (自动抗饱和) =====
output = clamp(output_raw, -max_output, +max_output);
```

### 4.4 参数表

| 参数 | 公式 | Roll | Pitch | Yaw | 物理意义 |
|------|------|------|-------|-----|---------|
| `wc` | — | 20 rad/s | 20 rad/s | 25 rad/s | **控制器带宽**，越大响应越快 (~3Hz) |
| `b0` | — | 30 | 30 | 40 | **控制增益**，偏大=保守，偏小=激进 |
| `wo` | `3·wc` | 60 | 60 | 75 | **观测器带宽**，必须 ≥ 3倍 wc |
| `β₁` | `2·wo` | 120 | 120 | 150 | ESO 增益1 (状态误差反馈) |
| `β₂` | `wo²` | 3600 | 3600 | 5625 | ESO 增益2 (扰动误差反馈) |
| `Ts` | — | 0.005s | 0.005s | 0.005s | 采样周期 (200Hz) |
| `max_output` | — | ±300 | ±300 | ±100 | 输出限幅 |

### 4.5 ADRC vs PID 对比

| 特性 | PID (原内环) | ADRC (新内环) |
|------|-------------|--------------|
| 可调参数 | 3个 (Kp, Ki, Kd) | **2个** (wc, b0) |
| 积分抗饱和 | 需手动处理 (分离+限幅) | **自动** — 限幅值反馈进ESO |
| 扰动抑制 | 通过积分项缓慢消除 | **主动估计+前馈补偿** (z2) |
| 参数整定 | 经验试凑 | **带宽法** — 调一个 wc 即可 |
| 微分噪声放大 | Kd 直接放大噪声 | **无微分项** — ESO 天然滤波 |
| 超调 | 积分和微分共同作用 | **无超调** (一阶闭环特性) |

---

## 五、完整控制链路 (HOLD_POSITION 模式)

定点悬停时，控制链路最长，覆盖全部六层：

```
第1层  位置环 PID          位置偏差(m)  →  目标速度(m/s)       25Hz
       ┌────────────────────────────────────────────────┐
       │ Kp=1.0, Ki=0, Kd=0                             │
       │ target_speed = PID(pos_error)                   │
       │ posXPID, posYPID (PIDController)                │
       └───────────────────┬────────────────────────────┘
                           │ target_X_Speed, target_Y_Speed
                           ▼
第2层  速度环 PID          速度偏差(m/s) → 目标加速度(m/s²)   50Hz
       ┌────────────────────────────────────────────────┐
       │ Kp=160, Ki=4, Kd=0 (X轴)                       │
       │ Kp=160, Ki=0, Kd=0 (Y轴)                       │
       │ speedXPID, speedYPID (PIDController)            │
       └───────────────────┬────────────────────────────┘
                           │ target_X_Accel, target_Y_Accel
                           ▼
第3层  角度换算            加速度 → 角度                   50Hz
       ┌────────────────────────────────────────────────┐
       │ targetRoll  = -target_Y_Accel / 10.0            │
       │ targetPitch =  target_X_Accel / 10.0            │
       │ constrain(±3°)                                  │
       └───────────────────┬────────────────────────────┘
                           │ targetRoll, targetPitch
                           ▼
第4层  角度环 PID ───────── 外环 ────────────────────── 100Hz
       ┌────────────────────────────────────────────────┐
       │ Kp=3.5, Ki=0.2, Kd=0                           │
       │ rollAnglePID, pitchAnglePID (PIDController)     │
       │ → targetRollRate, targetPitchRate               │
       └───────────────────┬────────────────────────────┘
                           │ target*Rate (°/s)
                           ▼
第5层  角速度环 ADRC ─────── 内环 ───────────────────── 200Hz
       ┌────────────────────────────────────────────────┐
       │ 一阶 LADRC (ADRC_Rate)                          │
       │ ESO → P控制 + 扰动补偿                          │
       │ → rollOut, pitchOut, yawOut                     │
       └───────────────────┬────────────────────────────┘
                           │ rollOut/pitchOut/yawOut
                           ▼
第6层  电机混控            修正量 → PWM                   200Hz
       ┌────────────────────────────────────────────────┐
       │ X型四轴混控 (escCtrl)                           │
       │ ESC_PWM[0..3] = throttle ± 修正量               │
       │ constrain(1000~2000)                            │
       └────────────────────────────────────────────────┘
```

并行链路 (高度控制)：

```
高度环 PID         高度偏差(m) → 目标高度速度(m/s)      50Hz
   Kp=2.0, Ki=0.1, Kd=0 (heightPID)
       │
       ▼
高度速度环 PID     高度速度偏差 → 油门补偿              50Hz
   Kp=100, Ki=0, Kd=0 (heightRatePID)
       │
       ▼
   hold_throttle = hold_base_throttle + heightRateCorrect
   constrain(1100, 1800)
```

---

## 六、模式切换时的状态重置

进入 HOLD_POSITION 定点模式时，需要清除历史累积：

```cpp
// 位置环 & 速度环 PID (高度环特意不清零以保持悬停油门)
pidController.cleanRollPIDData();    // 清空横滚角度PID积分
pidController.cleanPitchPIDData();   // 清空俯仰角度PID积分

// ADRC 内环
rollADRC.reset();                    // z1=0, z2=0, output=0
pitchADRC.reset();                   // 清空历史扰动估计
```

**为什么需要重置 ADRC：** ESO 中的 $z_2$ 累积了手动飞行时的总扰动估计（风、机动等）。如果不重置，进入定点模式时旧的扰动估计会作为错误的前馈量注入，导致切入瞬间的姿态抖动。

---

## 七、调参指南

### 7.1 外环角度 PID (与原项目一致，无需重新调整)

| 轴 | Kp | Ki | Kd |
|----|-----|----|----|
| Roll | 3.5 | 0.2 | 0 |
| Pitch | 3.5 | 0.2 | 0 |
| Yaw | 20.5 | 0 | 0 |

### 7.2 内环 ADRC (新参数，需飞行调优)

| 症状 | 原因 | 调整 |
|------|------|------|
| 响应迟缓，感觉"肉" | `wc` 太小 | 增大 5~10 rad/s |
| 高频抖动/振荡 | `wc` 太大 或 `b0` 太小 | 减小 `wc` 或增大 `b0` |
| 悬停时有稳态偏差 | `b0` 太大 (扰动补偿不足) | 减小 `b0` |
| 剧烈振荡/发散 | `b0` 太小 (过补偿) | 增大 `b0` |
| 抗风能力差 | 观测器太慢 | 增大 `wc` (同步增大 `wo`) |
| 电机声音粗糙 | 观测器太快 (放大噪声) | 减小 `wc` |

### 7.3 推荐调参流程

```
第1步: Props OFF 测试
  上电不装桨，手拿飞机倾斜，观察电机响应方向和幅度是否正确

第2步: 保守起飞
  wc_roll = wc_pitch = 10 rad/s
  确认能稳定悬停

第3步: 逐步增大 wc
  每次 +5 rad/s，直到出现轻微振荡，然后退回 5

第4步: 微调 b0
  如果响应偏软 → 减小 b0 (增强扰动补偿)
  如果出现振荡 → 增大 b0 (减弱扰动补偿)

第5步: 监控 z2 (扰动估计)
  通过串口打印 getZ2()，正常飞行时应在合理范围 ±(max_output * b0 * 0.5)
```

---

## 八、代码隔离说明

原始 `pid.h` / `pid.cpp` **未做任何修改**。ADRC 代码完全独立在 `adrc_rate.h` / `adrc_rate.cpp` 中。

`PIDController` 中 3 个角速度 PID 实例 (`rollRatePID`, `pitchRatePID`, `yawRatePID`) 仍然存在但不被调用，成为死代码（保留以便快速回退到纯 PID 方案）。

要**回退到纯 PID**，只需：
1. 恢复 `8fly18_ADRC.ino` 中 200Hz 循环为 `calculateRatePID()` + `getRollCorrect(RATE)` 模式
2. 恢复 `setup()` 中删除 `rollADRC.init()` 等调用
3. 删除 `#include "adrc_rate.h"`

不需要改 `pid.h` / `pid.cpp` 中的任何代码。
