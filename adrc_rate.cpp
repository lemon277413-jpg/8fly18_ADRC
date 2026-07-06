/*
 * adrc_rate.cpp
 *
 * 一阶线性自抗扰控制器实现
 * 参考 PX4 Autopilot src/lib/adrc/Ladrc.cpp
 * 专用于无人机角速度环控制
 *
 * 与PX4仿真模块的关键对齐:
 *   - wo 作为独立参数 (不再硬编码 2*wc)
 *   - z2 钳位自适应: z2_limit = b0 * max_output (PX4: z2_limit = b0)
 *   - init() 参数验证
 *   - 输出限幅后的值反馈回ESO (抗积分饱和)
 */

#include "adrc_rate.h"

/**
 * 构造函数 - 设置安全默认参数
 * 注意: 默认值仅作兜底, 实际参数应通过 init() 传入
 */
ADRC_Rate::ADRC_Rate()
    : z1(0.0f), z2(0.0f), output(0.0f)
    , ref(0.0f), feedback(0.0f), last_error(0.0f)
    , Ts(0.005f), wc(10.0f), b0(20.0f), max_output(300.0f)
    , wo(30.0f), beta1(60.0f), beta2(900.0f)
    , z2_limit(6000.0f)
{
}

/**
 * 根据当前参数重新计算 ESO 增益和 z2 钳位
 */
void ADRC_Rate::recalcGains()
{
    // 带宽法参数化 (Gao, 2003)
    // ESO极点配置: 全部配置在 -wo
    beta1 = 2.0f * wo;
    beta2 = wo * wo;

    // 自适应 z2 钳位 (同PX4: z2_limit = b0)
    // 乘以 max_output 是因为本项目的控制输出是PWM修正量(非归一化)
    z2_limit = b0 * max_output;
}

/**
 * 初始化ADRC控制器
 * @return true=参数有效, false=参数无效
 */
bool ADRC_Rate::init(float Ts_, float b0_, float wc_, float wo_, float max_output_)
{
    // 参数验证 (同PX4 Ladrc::init)
    if (Ts_ <= 0.0f || b0_ <= 0.0f || wc_ <= 0.0f || wo_ <= 0.0f || max_output_ <= 0.0f) {
        return false;
    }

    Ts = Ts_;
    b0 = b0_;
    wc = wc_;
    wo = wo_;           // wo 现在是独立参数, 不再自动计算
    max_output = max_output_;

    recalcGains();

    // 状态初始化
    reset();

    return true;
}

/**
 * 重置所有内部状态
 * 在模式切换(如进入定高定点)时调用，清除历史扰动估计
 */
void ADRC_Rate::reset()
{
    z1 = 0.0f;
    z2 = 0.0f;
    output = 0.0f;
    ref = 0.0f;
    feedback = 0.0f;
    last_error = 0.0f;
}

/**
 * 单步控制更新 - 核心算法
 *
 * 算法步骤 (同PX4 Ladrc::update):
 *   1. ESO观测器: 估计系统状态(z1)和总扰动(z2)
 *   2. 控制律:    P控制 + 扰动前馈补偿
 *   3. 输出限幅:  钳位保护, 限幅后的值反馈回ESO
 *
 * ESO方程:
 *   e  = y - z1                    (观测误差)
 *   z1 += dt * (z2 + b0*u + 2*wo*e)   (状态估计)
 *   z2 += dt * (wo^2 * e)             (扰动估计)
 *
 * 控制律:
 *   u0 = wc * (r - z1)             (P控制, 等效Kp = wc)
 *   u  = (u0 - z2) / b0            (扰动补偿 + 增益逆变换)
 */
float ADRC_Rate::update(float targetRate, float measuredRate)
{
    // 保存输入 (调试用)
    ref = targetRate;
    feedback = measuredRate;

    // ========== 1. ESO观测器更新 ==========
    // 观测误差: 实际测量与状态估计的差值
    float e_eso = measuredRate - z1;

    // 状态估计更新: z1_dot = z2 + b0*u + beta1*e
    z1 += Ts * (z2 + b0 * output + beta1 * e_eso);

    // 扰动估计更新: z2_dot = beta2*e
    z2 += Ts * (beta2 * e_eso);

    // z2自适应钳位 (同PX4: 防止ESO模型失配时扰动估计发散)
    if (z2 > z2_limit)  z2 = z2_limit;
    if (z2 < -z2_limit) z2 = -z2_limit;

    // ========== 2. 控制律计算 ==========
    // 跟踪误差: 目标值与状态估计的差值
    last_error = targetRate - z1;

    // 虚拟控制量: P-only (比例控制), Kp = wc
    float u0 = wc * last_error;

    // 扰动补偿 + 增益逆变换
    float output_raw = (u0 - z2) / b0;

    // ========== 3. 输出限幅 (同PX4: clamp到[-1,1], 本处以max_output为界) ==========
    if (output_raw > max_output) {
        output = max_output;
    } else if (output_raw < -max_output) {
        output = -max_output;
    } else {
        output = output_raw;
    }

    return output;
}

// ---- 在线调参接口 (同PX4 Ladrc) ----

void ADRC_Rate::setB0(float new_b0)
{
    if (new_b0 > 0.0f) {
        b0 = new_b0;
        recalcGains();
    }
}

void ADRC_Rate::setWc(float new_wc)
{
    if (new_wc > 0.0f) {
        wc = new_wc;
    }
}

void ADRC_Rate::setWo(float new_wo)
{
    if (new_wo > 0.0f) {
        wo = new_wo;
        recalcGains();
    }
}

void ADRC_Rate::setZ2Limit(float limit)
{
    if (limit > 0.0f) {
        z2_limit = limit;
    }
}

// ---- 调试接口 ----
float ADRC_Rate::getZ1() const      { return z1; }
float ADRC_Rate::getZ2() const      { return z2; }
float ADRC_Rate::getOutput() const  { return output; }
float ADRC_Rate::getError() const   { return last_error; }
float ADRC_Rate::getRef() const     { return ref; }
float ADRC_Rate::getFeedback() const { return feedback; }
