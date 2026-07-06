#ifndef EKF_H
#define EKF_H

//一维卡尔曼滤波
struct _1_ekf_filter
{
    float LastP;  // 上一时刻的估计误差协方差
    float Now_P;  // 当前时刻的预测误差协方差
    float out;    // 滤波器输出（最优估计值）
    float Kg;     // 卡尔曼增益
    float Q;      // 过程噪声协方差
    float R;      // 测量噪声协方差
};

void kalman_1(struct _1_ekf_filter *ekf, float input); // 一维卡尔曼滤波函数

#endif
