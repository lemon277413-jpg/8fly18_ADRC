/*
 * adrc_rate.h
 *
 * 一阶线性自抗扰控制器 (First-Order LADRC)
 * 用于替代无人机角速度环PID控制
 *
 * 参考: PX4 Autopilot src/lib/adrc/Ladrc.hpp
 * 算法: 带宽法参数化的一阶LADRC (Gao, 2003)
 *
 * 控制律: u = (wc*(ref - z1) - z2) / b0
 * ESO:    z1_dot = z2 + b0*u + beta1*(y - z1)
 *         z2_dot = beta2*(y - z1)
 *
 * 与PX4仿真模块的关键对齐:
 *   - wo 作为独立参数 (不再自动 2*wc)
 *   - z2 钳位 = b0 * max_output (自适应, 不硬编码)
 *   - init() 返回 bool 做参数验证
 */

#ifndef ADRC_RATE_H
#define ADRC_RATE_H

class ADRC_Rate {
public:
    ADRC_Rate();

    /**
     * 初始化ADRC控制器
     * @param Ts            采样周期(秒)，200Hz对应0.005s
     * @param b0            系统控制增益估计值 (参考PX4: 八旋翼≈150, 需按输出量纲缩放)
     * @param wc            控制器带宽(rad/s)，越大响应越快
     * @param wo            观测器带宽(rad/s)，推荐 wo = 3*wc
     * @param max_output    输出限幅(绝对值)
     * @return              true=参数有效, false=参数无效
     */
    bool init(float Ts, float b0, float wc, float wo, float max_output);

    /**
     * 单步控制更新
     * @param targetRate   目标角速度 (deg/s)
     * @param measuredRate 实际角速度 (deg/s)
     * @return             控制输出 (已限幅)
     */
    float update(float targetRate, float measuredRate);

    /**
     * 重置所有内部状态 (用于模式切换/解锁时)
     */
    void reset();

    // ---- 在线调参 (同PX4 Ladrc接口) ----
    void setB0(float new_b0);
    void setWc(float new_wc);
    void setWo(float new_wo);       // 新增: 独立调整观测器带宽
    void setZ2Limit(float limit);   // 新增: 手动设置z2钳位

    // ---- 调试接口 ----
    float getZ1() const;       // 状态估计值 (估计的角速度)
    float getZ2() const;       // 总扰动估计值
    float getOutput() const;   // 上次控制输出
    float getError() const;    // 跟踪误差 (target - z1)
    float getRef() const;      // 上次参考输入
    float getFeedback() const; // 上次反馈输入

private:
    // ESO状态变量
    float z1;           // 状态估计 (角速度)
    float z2;           // 总扰动估计
    float output;       // 控制输出 (用于ESO反馈)

    // 调试用 - 保存最近一次输入和误差
    float ref;          // 参考输入 (目标角速度)
    float feedback;     // 反馈输入 (实际角速度)
    float last_error;   // 跟踪误差 e_ctrl = ref - z1

    // 配置参数
    float Ts;           // 采样周期
    float wc;           // 控制器带宽
    float b0;           // 系统控制增益
    float max_output;   // 输出限幅

    // 带宽法参数 (Gao 2003)
    float wo;           // 观测器带宽 (独立参数, 推荐 3*wc)
    float beta1;        // 观测器增益1 = 2*wo
    float beta2;        // 观测器增益2 = wo^2

    // z2 自适应钳位 (同PX4: z2_limit = b0 * max_output)
    float z2_limit;

    // 内部: 根据当前参数重新计算 z2 钳位和 ESO 增益
    void recalcGains();
};

#endif
