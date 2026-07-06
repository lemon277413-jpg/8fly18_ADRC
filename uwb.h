#ifndef UWB_H
#define UWB_H
#include <Arduino.h>

// 你的原始滤波结果结构体，无修改
typedef struct {
  float speed;
  float pos;
} EKF_UWB_IMU_DATA;

// 二维卡尔曼滤波类，仅声明与你的逻辑匹配的成员
class EKFFilter {
  private:

    // ====================== 【固定参数：全部在这里修改！】 ======================
    const float uwb_dt = 0.02f;                // 滤波周期
    const float Q_pos = 0.000001f;             // 位置过程噪声
    const float Q_vel = 0.0005f;               // 速度过程噪声
    const float Q_bias = 0.000001f;           // 零偏过程噪声
    const float R_uwb = 0.04f;                 // UWB观测噪声
    const float uwb_jump_threshold = 0.5f;     // UWB跳变阈值
    // ========================================================================

    // 与你原静态变量完全一致的私有状态/参数
    float fusion_pos;
    float fusion_vel;
    float fusion_accel_bias; // 加速度零偏
    float predicted_pos;
    float predicted_vel;
    float predicted_accel_bias; // 预测的加速度零偏 (b_pre)
    float P[3][3];
    float K[3];
    
    float old_uwb_pos; // 用于检测UWB数据跳变的上一个位置值
    int Is_UWB_first_update; // 标志是否第一次更新，用于初始化old_uwb_pos

    int consecutive_uwb_miss_count; // 【新增】连续未使用UWB的帧数计数器

  public:
    EKFFilter(float init_accel_bias = 0.0f);
    // 核心更新函数：参数与你的原函数完全一致
    int update(float imu_accel, float uwb_pos, int Is_UWB_work);
    // 仅保留获取结果的基础接口，匹配你的结构体
    float getPos() const;
    float getSpeed() const;
    float getAccelBias() const; // 获取加速度零偏
};

#endif