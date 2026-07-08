/*
 * adrc_controller.h
 *
 * 二阶自抗扰控制器 (2nd-Order LADRC)
 * 移植自: XDU-Educational-UAV/Drone_Master_ADRC (user/adrc.h)
 *
 * 架构 (per-axis Roll/Pitch):
 *   外环: AttOut = KpOut * (targetAngle - measuredAngle)
 *   内环: 3阶LESO估计 SpeEst/AccEst/w → PD控制律
 *         u = KpIn*(AttOut-SpeEst) - KdIn*AccEst - w/B
 *
 * 与Drone_Master_ADRC的主要差异:
 *   - LESO增益改用带宽参数化 (β1=3wo, β2=3wo², β3=wo³), 替代固定值30/300/1000
 *   - 采样周期T=0.005s (200Hz), 参考为T=0.002s (500Hz)
 *   - 陀螺输入已是deg/s浮点值, 无需GyroToDeg转换
 *   - 控制输出u为PWM修正量 (-200~+200量级)
 */

#ifndef ADRC_CONTROLLER_H
#define ADRC_CONTROLLER_H

/* ================================================================
 * ADRC 参数与状态结构体
 * ================================================================ */
typedef struct {
    // ---- 控制参数 (用户可调) ----
    float KpOut;     // 外环P增益: 角度误差(deg) → 角速度目标(deg/s)
    float KpIn;      // 内环P增益: 角速度跟踪比例增益
    float KdIn;      // 内环D增益: 角加速度阻尼增益
    float B;         // 控制增益: deg/s² per PWM单位 (物理控制力矩)

    // ---- 状态变量 (ESO估计值 + 控制量) ----
    float AttOut;    // 外环输出: 目标角速度 (deg/s)
    float SpeEst;    // ESO z1: 角速度估计值 (deg/s)
    float AccEst;    // ESO z2: 角加速度估计值 (deg/s²)
    float w;         // ESO z3: 总扰动估计值
    float u;         // 控制输出 (PWM修正量)

    // ---- TD状态 (跟踪微分器, 预留) ----
    float x1;        // TD滤波后的参考输入
    float x2;        // TD滤波后的参考微分

    // ---- 配置参数 (ADRC_Init设定) ----
    float wo;        // 观测器带宽 (rad/s)
    float Ts;        // 采样周期 (秒, 200Hz对应0.005)
    float max_u;     // 输出限幅 (绝对值)

    // ---- ESO增益 (由wo自动计算) ----
    float beta1;     // = 3 * wo
    float beta2;     // = 3 * wo²
    float beta3;     // = wo³

} ADRC_Param;

/* ================================================================
 * 函数声明
 * ================================================================ */

/**
 * 初始化ADRC控制器参数
 * @param p       ADRC参数结构体指针
 * @param Ts      采样周期(秒), 200Hz对应0.005
 * @param wo      观测器带宽(rad/s), 推荐10~30
 * @param B       控制增益 (deg/s² per PWM单位)
 * @param KpOut   外环比例增益
 * @param KpIn    内环比例增益
 * @param KdIn    内环微分增益
 * @param max_u   输出限幅(绝对值)
 */
void ADRC_Init(ADRC_Param *p, float Ts, float wo, float B,
               float KpOut, float KpIn, float KdIn, float max_u);

/**
 * 3阶线性扩张状态观测器 (LESO)
 * 估计角速度(SpeEst)、角加速度(AccEst)和总扰动(w)
 *
 * 系统模型: x1'=x2, x2'=B*u+w, y=x1
 * ESO方程:
 *   e = y - SpeEst
 *   SpeEst += (AccEst + beta1*e) * Ts
 *   AccEst += (B*u + w + beta2*e) * Ts
 *   w      += beta3*e * Ts
 *
 * @param p  ADRC参数结构体指针
 * @param y  实测角速度 (deg/s)
 */
void ADRC_LESO(ADRC_Param *p, float y);

/**
 * 跟踪微分器 (TD)
 * 使用fhan最速综合函数对参考输入进行平滑和微分提取
 * 注意: 参考项目中TD代码存在但外层未调用, 此处保留备用
 *
 * @param p  ADRC参数结构体指针
 */
void ADRC_TD(ADRC_Param *p);

/**
 * 清空ADRC所有内部状态
 * 用于模式切换/解锁/油门低于阈值时重置
 *
 * @param p  ADRC参数结构体指针
 */
void ADRC_ParamClear(ADRC_Param *p);

/**
 * fhan最速控制综合函数
 * 用于跟踪微分器(TD)的信号平滑
 *
 * @param x1  位置误差 (当前x1 - 参考输入)
 * @param x2  速度 (当前x2)
 * @return    最速控制量
 */
float ADRC_fhan(float x1, float x2);

/**
 * ADRC外环: 角度误差 → 角速度目标
 * AttOut = KpOut * (targetAngle - measuredAngle)
 * 替代原有的 calculateAnglePID()
 *
 * @param p             ADRC参数结构体指针
 * @param targetAngle   目标角度 (deg)
 * @param measuredAngle 实测角度 (deg)
 */
void ADRC_OuterLoop(ADRC_Param *p, float targetAngle, float measuredAngle);

/**
 * ADRC内环: PD控制律 + 扰动前馈补偿
 * u = KpIn*(AttOut - SpeEst) - KdIn*AccEst - w/B
 * 结果限幅到[-max_u, max_u]并存入p->u
 *
 * 调用前提: 已先调用 ADRC_LESO() 更新状态估计
 *
 * @param p  ADRC参数结构体指针
 * @return   控制输出 (已限幅)
 */
float ADRC_InnerLoop(ADRC_Param *p);

#endif  // ADRC_CONTROLLER_H
