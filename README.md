# 8fly18 ADRC 飞控

基于 Arduino 的八旋翼无人机飞控系统，外环角度 PID + 内环角速度一阶线性自抗扰控制（LADRC）。

## 硬件

- **MCU**: Arduino Mega 2560
- **IMU**: MPU6050（Mahony 姿态解算）
- **电机**: T-Motor AIR2216 KV880 × 8（同轴共桨，4 路 PWM 信号）
- **遥控器**: PWM 接收机（4 通道）
- **定位**: UWB（Nooploop LinkTrack）/ MTF02 光流模块

## 控制架构

```
遥控器 → 角度PID(外环) → ADRC角速度(内环) → 电机混控 → 8电机
         ↑ 仅P控制         ↑ 一阶LADRC
         KI已关闭           ESO估计+前馈补偿
```

- **外环（角度）**: PID 纯 P 控制，50Hz
- **内环（角速度）**: 一阶线性 ADRC（LADRC），200Hz
- **高度/位置环**: 仍使用 PID

## 状态机

| 状态 | 说明 |
|------|------|
| PROTECT | 上电默认，电机锁死 |
| INIT | 待飞，解锁后进入 |
| CALIBRATE | 地面校准陀螺仪零偏 |
| FLY | 正常飞行 |
| FLY_CALIBRATE | 飞行中微调零偏 |
| HOLD_POSITION | 定高定点（开发中） |

## ADRC 参数

| 轴 | b0 | wc | wo | max_output | 说明 |
|----|----|----|----|------------|------|
| Roll | 18.0 | 5.0 | 15.0 | 150 | 横滚 |
| Pitch | 4.2 | 2.0 | 6.0 | 150 | 俯仰 |
| Yaw | 0.15 | 25.0 | 36.0 | 250 | 偏航 |

参数定义见 `adrc_rate.h`，调参指南见 `ADRC参数调参指南.md`。

## 文件结构

```
8fly18_ADRC.ino         主程序
adrc_rate.h/cpp         一阶LADRC控制器
pid.h/cpp               PID控制器（角度/高度/位置环）
AttitudeEstimator.h/cpp Mahony姿态估计
mtf02.h/cpp             光流传感器驱动
uwb.h/cpp               UWB定位驱动
EKF.h/cpp               扩展卡尔曼滤波
flow_decode.h/c         光流数据解析
check.h/c               校验
up_flow.h/c             光流上传
CircularDelay.h/cpp     环形延时缓冲
```

## 参考

- Gao, Z. (2003). Scaling and bandwidth-parameterization based controller tuning. *ACC 2003*.
- 架构说明: `PID+ADRC架构说明.md`
