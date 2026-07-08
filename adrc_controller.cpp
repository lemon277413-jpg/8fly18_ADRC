/*
 * adrc_controller.cpp
 *
 * 二阶自抗扰控制器实现
 * 移植自: XDU-Educational-UAV/Drone_Master_ADRC (user/adrc.c, user/mymath.h)
 *
 * 核心算法:
 *   1. 3阶LESO: 从角速度测量值估计 速度/加速度/总扰动 三个状态
 *   2. PD控制律: KpIn*(AttOut-SpeEst) - KdIn*AccEst - w/B
 *   3. 外环P控制: AttOut = KpOut * (targetAngle - measuredAngle)
 *   4. TD (预留): fhan最速跟踪微分器, 代码保留但外层未连接
 *
 * 与参考项目Drone_Master_ADRC的关键差异:
 *   - LESO增益带宽参数化(wo→β1/β2/β3), 替代固定值
 *   - T=0.005s (200Hz), 参考T=0.002s (500Hz)
 *   - 使用标准 math.h (sqrtf/fabsf), 替代自定义 Msqrt/ABS/SIGN
 */

#include "adrc_controller.h"
#include <math.h>

/* ================================================================
 * 内部辅助宏 (替代参考项目mymath.h中的宏)
 * ================================================================ */
#define ADRC_ABS(x)   ((x) >= 0.0f ? (x) : -(x))
#define ADRC_SIGN(x)  ((x) >= 0.0f ? 1.0f : -1.0f)

/* ================================================================
 * TD参数 (fhan跟踪微分器)
 * ================================================================ */
#define TD_R    5000.0f   // 速度因子
#define TD_D    10.0f     // d = r*h
#define TD_D0   0.05f     // d0 = h*d  (h=0.005 → d0=0.05)

/* ================================================================
 * ADRC_Init - 参数初始化
 * ================================================================ */
void ADRC_Init(ADRC_Param *p, float Ts, float wo, float B,
               float KpOut, float KpIn, float KdIn, float max_u)
{
    // 配置参数
    p->Ts    = Ts;
    p->wo    = wo;
    p->B     = B;
    p->KpOut = KpOut;
    p->KpIn  = KpIn;
    p->KdIn  = KdIn;
    p->max_u = max_u;

    // 带宽法参数化 (Gao, 2003)
    // 3阶ESO极点全配置在 -wo
    p->beta1 = 3.0f * wo;
    p->beta2 = 3.0f * wo * wo;
    p->beta3 = wo * wo * wo;

    // 状态清零
    ADRC_ParamClear(p);
}

/* ================================================================
 * ADRC_ParamClear - 状态清零
 * ================================================================ */
void ADRC_ParamClear(ADRC_Param *p)
{
    p->AttOut = 0.0f;
    p->SpeEst = 0.0f;
    p->AccEst = 0.0f;
    p->w      = 0.0f;
    p->u      = 0.0f;
    p->x1     = 0.0f;
    p->x2     = 0.0f;
}

/* ================================================================
 * ADRC_LESO - 3阶线性扩张状态观测器
 *
 * 系统模型 (二阶):
 *   x1_dot = x2              (角速度 = 角加速度的积分)
 *   x2_dot = B*u + w         (角加速度 = 控制力矩 + 扰动)
 *   y = x1                   (测量 = 角速度)
 *
 * 离散ESO方程:
 *   e = y - SpeEst
 *   SpeEst += (AccEst + beta1*e) * Ts
 *   AccEst += (B*u + w + beta2*e) * Ts
 *   w      += beta3*e * Ts
 *
 * @y: 实测角速度 (deg/s)
 * ================================================================ */
void ADRC_LESO(ADRC_Param *p, float y)
{
    float e = y - p->SpeEst;

    // z1: 角速度估计
    p->SpeEst += (p->AccEst + p->beta1 * e) * p->Ts;

    // z2: 角加速度估计 (含控制输入前馈 B*u)
    p->AccEst += (p->B * p->u + p->w + p->beta2 * e) * p->Ts;

    // z3: 总扰动估计
    p->w += p->beta3 * e * p->Ts;
}

/* ================================================================
 * ADRC_OuterLoop - 外环角度P控制
 *
 * AttOut = KpOut * (targetAngle - measuredAngle)
 * 替代原有 calculateAnglePID() 函数
 * ================================================================ */
void ADRC_OuterLoop(ADRC_Param *p, float targetAngle, float measuredAngle)
{
    p->AttOut = p->KpOut * (targetAngle - measuredAngle);
}

/* ================================================================
 * ADRC_InnerLoop - 内环PD控制律 + 扰动补偿
 *
 * 控制律:
 *   u_raw = KpIn*(AttOut - SpeEst) - KdIn*AccEst - w/B
 *
 * 说明:
 *   - KpIn*(AttOut - SpeEst): 角速度误差比例控制
 *   - KdIn*AccEst:           角加速度阻尼 (抑制振荡)
 *   - w/B:                   扰动前馈补偿 (抵消外部扰动)
 *
 * 返回值: 限幅后的控制输出 (PWM修正量)
 * ================================================================ */
float ADRC_InnerLoop(ADRC_Param *p)
{
    // PD控制律 + 扰动补偿
    float u_raw = p->KpIn * (p->AttOut - p->SpeEst)
                - p->KdIn * p->AccEst
                - p->w / p->B;

    // 输出限幅
    if (u_raw > p->max_u) {
        p->u = p->max_u;
    } else if (u_raw < -p->max_u) {
        p->u = -p->max_u;
    } else {
        p->u = u_raw;
    }

    return p->u;
}

/* ================================================================
 * ADRC_fhan - 最速控制综合函数
 *
 * 用于跟踪微分器(TD), 对参考信号进行平滑和微分提取
 * 算法来源: 韩京清《自抗扰控制技术》
 *
 * 参数 (文件级常量):
 *   R  = 5000  速度因子, 越大跟踪越快
 *   H  = Ts    滤波因子 (与采样周期一致)
 *   D  = R*H   线性区间宽度
 *   D0 = H*D   切换区间宽度
 *
 * @x1: 位置误差 (当前x1 - 参考输入AttOut)
 * @x2: 速度 (当前x2)
 * @return: 最速控制加速度
 * ================================================================ */
float ADRC_fhan(float x1, float x2)
{
    float H  = 0.005f;  // 与采样周期Ts一致, 如需可变可改为参数
    float y  = x1 + H * x2;
    float a0 = sqrtf(TD_D * TD_D + 8.0f * TD_R * ADRC_ABS(y));

    float a;
    if (ADRC_ABS(y) > TD_D0) {
        a = x2 + (a0 - TD_D) / 2.0f * ADRC_SIGN(y);
    } else {
        a = x2 + y / H;
    }

    if (ADRC_ABS(a) > TD_D) {
        return -TD_R * ADRC_SIGN(a);
    } else {
        return -TD_R * a / TD_D;
    }
}

/* ================================================================
 * ADRC_TD - 跟踪微分器
 *
 * 离散最速跟踪微分器:
 *   u = fhan(x1 - AttOut, x2)
 *   x1 += Ts * x2
 *   x2 += Ts * u
 *
 * 注意: 参考项目中此函数定义但未在控制回路中调用
 *       此处保留备用, 后续需要时在Motor_Outer_loop中调用
 * ================================================================ */
void ADRC_TD(ADRC_Param *p)
{
    float u = ADRC_fhan(p->x1 - p->AttOut, p->x2);
    p->x1 += p->Ts * p->x2;
    p->x2 += p->Ts * u;
}
