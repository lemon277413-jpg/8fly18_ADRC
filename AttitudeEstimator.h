#ifndef ATTIRUDEESTIMATOR_H
#define ATTIRUDEESTIMATOR_H


float AccConvert(int acc);
float GyroConvert(int gyro);
void kalman_1(struct _1_ekf_filter *ekf, float input); // 一维卡尔曼滤波函数
/**
 * 快速平方根倒数算法（Quake III 算法）
 * 功能：计算 1/sqrt(number) 的近似值
 * 原理：利用浮点数的二进制表示和牛顿迭代法快速求解平方根倒数
 * 用途：在姿态解算中用于向量归一化，避免昂贵的除法运算
 * @param number 输入的浮点数
 * @return 1/sqrt(number) 的近似值
 */
float Q_rsqrt(float number);

void CorrectAttitude(float AE_CORroll, float AE_CORpitch);//修正陀螺仪的安装误差，传入参数为测得的roll和pitch角度
void SetupMPU6050();
void GetDataMPU6050Rate(float &rollRate, float &pitchRate, float &yawRate);
void GetDataMPU6050(float &roll, float &pitch, float &yaw, float &rollRate, float &pitchRate, float &yawRate, float &RawrollRate, float &RawpitchRate, float AE_RollOffset, float AE_PitchOffset);
void get_Horizontal_heading_Acc(float& worldAccX, float& worldAccY);//m/s² 水平航向坐标系下
void get_WorldAcc(float& worldAccX, float& worldAccY); //m/s² 世界坐标系下
#endif