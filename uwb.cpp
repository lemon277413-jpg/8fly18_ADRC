#include <Arduino.h>
#include "uwb.h"

// 构造函数：仅接收初始加速度零偏，其他参数用头文件固定值
EKFFilter::EKFFilter(float init_accel_bias) {
  // 状态初始化
  fusion_pos = 0.0f;
  fusion_vel = 0.0f;
  fusion_accel_bias = init_accel_bias;    // 主文件传入的零偏
  
  predicted_pos = 0.0f;
  predicted_vel = 0.0f;
  predicted_accel_bias = init_accel_bias; // 同步初始化预测零偏

  Is_UWB_first_update = 1;
  consecutive_uwb_miss_count = 0;

  // P矩阵初始化（不变）
  P[0][0] = 1.0f; P[0][1] = 0.0f; P[0][2] = 0.0f;
  P[1][0] = 0.0f; P[1][1] = 1.0f; P[1][2] = 0.0f;
  P[2][0] = 0.0f; P[2][1] = 0.0f; P[2][2] = 0.001f;
}

// 核心update函数：完全复现你的原EKF逻辑，逐行对应，现已加入加速度零偏估计
// 修改后：返回值为int，0=未使用UWB，1=使用了UWB
// 核心update函数：在原EKF逻辑基础上，将跳变检测改为「IMU预测一致性检测」，并增加返回值
int EKFFilter::update(float imu_accel, float uwb_pos, int Is_UWB_work) {
  // --------- 第一阶段：预测（加速度更新） ---------
  // 每当加速度计产生数据时执行
  // 1. 状态预测:
  // p_pre = p + v * dt + 0.5 * (a_raw - b) * dt^2
  // v_pre = v + (a_raw - b) * dt
  // b_pre = b (假设零偏短期内是不变的)
  
  // 有效加速度 = 原始加速度 - 融合后的加速度零偏
  float effective_accel = imu_accel - fusion_accel_bias; 
  predicted_pos = fusion_pos + fusion_vel * uwb_dt + 0.5f * effective_accel * uwb_dt * uwb_dt;
  predicted_vel = fusion_vel + effective_accel * uwb_dt;
  predicted_accel_bias = fusion_accel_bias;

  // 2. 协方差预测 (P = A*P*A^T + Q):
  // 按照你提供的展开公式进行计算 (注意数组下标从0开始，对应公式中的1,2,3)
  // P[0][0] -> P11, P[0][1] -> P12, ..., P[2][2] -> P33
  
  // 暂存旧的P矩阵值，因为计算过程中P回被更新覆盖
  float old_P[3][3];
  for(int i=0; i<3; i++) for(int j=0; j<3; j++) old_P[i][j] = P[i][j];
  float dt = uwb_dt;
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt3 * dt;

  // P11 = P11 + 2*dt*P12 + dt^2*P22 - dt^2*P13 - dt^3*P23 + 0.25*dt^4*P33 + Q_p
  P[0][0] = old_P[0][0] + 2.0f*dt*old_P[0][1] + dt2*old_P[1][1] - dt2*old_P[0][2] - dt3*old_P[1][2] + 0.25f*dt4*old_P[2][2] + Q_pos;
  
  // P12 = P12 + dt*P22 - 0.5*dt^2*P13 - dt^2*P23 + 0.5*dt^3*P33
  P[0][1] = old_P[0][1] + dt*old_P[1][1] - 0.5f*dt2*old_P[0][2] - dt2*old_P[1][2] + 0.5f*dt3*old_P[2][2];
  P[1][0] = P[0][1]; // 对称
  
  // P13 = P13 + dt*P23 - 0.5*dt^2*P33
  P[0][2] = old_P[0][2] + dt*old_P[1][2] - 0.5f*dt2*old_P[2][2];
  P[2][0] = P[0][2]; // 对称
  
  // P22 = P22 - 2*dt*P23 + dt^2*P33 + Q_v
  P[1][1] = old_P[1][1] - 2.0f*dt*old_P[1][2] + dt2*old_P[2][2] + Q_vel;
  // P23 = P23 - dt*P33
  P[1][2] = old_P[1][2] - dt*old_P[2][2];
  P[2][1] = P[1][2]; // 对称
  
  // P33 = P33 + Q_b
  P[2][2] = old_P[2][2] + Q_bias;

  // --------- 第二阶段：更新（UWB 修正） ---------
  // 每当 UWB 有新数据时执行
  // 【逻辑修改1】：将原有的「相邻UWB位置差值检测」改为「IMU预测一致性检测」
  // 【逻辑修改2】：新增「连续20帧未使用则强制使用」逻辑，防止滤波器完全发散
  // 新增逻辑：即使UWB工作正常，如果测量值与IMU预测位置偏差超过阈值（uwb_jump_threshold），
  // 则判定为数据跳变，将其视为无效数据处理，直接使用预测值作为融合结果。
  
  // 定义返回值，默认0=未使用UWB
  int uwb_used_flag = 0;

  // 【新增】判断是否需要强制使用UWB：连续丢帧≥20 且 UWB当前工作正常
  bool force_use_uwb = false;
  if (consecutive_uwb_miss_count >= 20 && Is_UWB_work == 1) {
    force_use_uwb = true;
  }

  // 1. 首次UWB数据初始化：避免第一次数据被误判为跳变，直接初始化位置
  if(Is_UWB_first_update && Is_UWB_work == 1) {
    fusion_pos = uwb_pos;
    fusion_vel = 0.0f;
    Is_UWB_first_update = 0;
    uwb_used_flag = 1; // 首次初始化使用了UWB数据
    consecutive_uwb_miss_count = 0; // 【新增】首次使用后重置连续丢帧计数
    return uwb_used_flag;
  }

  // 2. 核心：IMU预测一致性检测 + UWB更新（含强制使用逻辑）
  // 检测条件：UWB工作正常 + (残差小于阈值 或 强制使用标志为真)
  if (Is_UWB_work == 1) {
    // 【IMU预测一致性检测核心逻辑】
    // 残差 = UWB实测位置 - IMU预测的当前位置，超过阈值判定为跳变，直接剔除
    float uwb_residual = fabs(uwb_pos - predicted_pos);
    
    // 【修改】进入条件增加 || force_use_uwb
    if (uwb_residual < uwb_jump_threshold || force_use_uwb) {
      // 无跳变 或 强制使用：执行EKF观测更新，使用UWB数据
      // 1. 计算卡尔曼增益 K (3x1):
      // S = P11 + R
      float S = P[0][0] + R_uwb;
      K[0] = P[0][0] / S; // K1
      K[1] = P[1][0] / S; // K2 (注意P是对称阵，P21 = P12，即 P[1][0] = P[0][1])
      K[2] = P[2][0] / S; // K3 (P31 = P13，即 P[2][0] = P[0][2])

      // 2. 修正状态:
      float residual = uwb_pos - predicted_pos;
      
      fusion_pos = predicted_pos + K[0] * residual;
      fusion_vel = predicted_vel + K[1] * residual;
      // 这里 K3 实现了：根据位置误差自动倒推加速度零偏
      fusion_accel_bias = predicted_accel_bias + K[2] * residual; 

      // 3. 修正协方差 (P = (I - KH)P):
      // 暂存未更新的 P 矩阵值，用于计算
      for(int i=0; i<3; i++) for(int j=0; j<3; j++) old_P[i][j] = P[i][j];
      // P11 = (1 - K1) * P11
      P[0][0] = (1.0f - K[0]) * old_P[0][0];
      // P12 = (1 - K1) * P12
      P[0][1] = (1.0f - K[0]) * old_P[0][1];
      P[1][0] = P[0][1]; // 对称
      // P13 = (1 - K1) * P13
      P[0][2] = (1.0f - K[0]) * old_P[0][2];
      P[2][0] = P[0][2]; // 对称
      
      // P22 = P22 - K2 * P12
      P[1][1] = old_P[1][1] - K[1] * old_P[0][1];
      
      // P23 = P23 - K2 * P13
      P[1][2] = old_P[1][2] - K[1] * old_P[0][2];
      P[2][1] = P[1][2]; // 对称
      
      // P33 = P33 - K3 * P13
      P[2][2] = old_P[2][2] - K[2] * old_P[0][2];

      uwb_used_flag = 1; // 标记使用了UWB数据
    }
  }

  // 3. UWB不工作/检测到跳变：仅用IMU预测值，不使用UWB数据
  if (uwb_used_flag == 0) {
    // 你的逻辑：UWB无效或检测到跳变时更新预测值为融合值
    fusion_pos = predicted_pos;
    fusion_vel = predicted_vel;
    // 加速度零偏也直接使用预测值（假设无变化）
    fusion_accel_bias = predicted_accel_bias;
  }

  // --------- 【新增】第三阶段：更新连续丢帧计数器 ---------
  if (uwb_used_flag == 1) {
    consecutive_uwb_miss_count = 0; // 使用了UWB，重置计数
  } else {
    consecutive_uwb_miss_count++; // 未使用UWB，计数+1
  }

  // 返回最终标记：1=用了UWB，0=没用UWB
  return uwb_used_flag;
}


// 基础接口：仅获取位置/速度，匹配你的结构体
float EKFFilter::getPos() const { return fusion_pos; }
float EKFFilter::getSpeed() const { return fusion_vel; }
// 获取加速度零偏接口
float EKFFilter::getAccelBias() const { return fusion_accel_bias; }