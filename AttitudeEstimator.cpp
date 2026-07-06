#include "AttitudeEstimator.h"
#include <Wire.h>
#include <Arduino.h>
#include "EKF.h" 

//#include <Wire.h>
//#include <math.h>

// 0x3B ~ 0x48 14 个空间上存储的是待读取的 7 种数据
// 每种数据占据 2 个字节的空间
// 需要将高位和低位数据组合成两个字节，才能得到完整的数据



//四元数转的旋转矩阵
float AE_QtoR[3][3]; // 四元数到旋转矩阵的转换结果
float AE_Rtemp[3][3]; // 临时存储旋转矩阵的变量

// 加速度陀螺仪 MPU-6050 的 I2C 地址
const int MPU_Addr = 0x68;

// 加速度
// 倍率：2g、4g、8g、16g
int AcX, AcY, AcZ;
float realAcX, realAcY, realAcZ;// 加速度转换后的实际值，单位为 g
float AcX_For_World_Acc, AcY_For_World_Acc, AcZ_For_World_Acc; // 用于世界坐标系加速度计算的加速度值，单位为 m/s^2

float Mahz = 0.0;

// 角速度
// 倍率：250、500、1000、2000
int GyX, GyY, GyZ;
float realGyX, realGyY, realGyZ; // 角速度转换后的实际值，单位为 rad/s
float lastrealGyX=0, lasrealGyY=0, lasrealGyZ=0; // 上一时刻的角速度值，用于计算微分
float AE_last_roll = 0.0, AE_last_pitch = 0.0, AE_last_yaw = 0.0; // 上一时刻的姿态角度，用于计算微分

// temperature
int Tmp;
//float Temperature;

/*角速度计偏移量修正值*/
int i;
int j=0;
int k=0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0; // x,y,z轴的角速度偏移量
float ax_offset = 0, ay_offset = 0; // x,y轴的加速度偏移量

//微分时间获取相关变量
unsigned long lastTime = 0;
unsigned long nowTime = 0;
int AE_dt_us = 0; // 微分时间间隔，单位毫秒
float AE_dt = 0.0;// 微分时间间隔,单位s

//单独读取角速度函数的相关变量
unsigned long RatelastTime = 0;
unsigned long RateNowTime = 0;
int Rate_dt_us = 0; // 角速度微分时间间隔，单位毫秒
float Rate_dt = 0.0;// 角速度微分时间间隔,单位


// 定义三个独立的卡尔曼滤波器，分别处理X、Y、Z轴加速度
_1_ekf_filter accel_ekf[3];

// Mahony滤波器相关变量
typedef struct
{
    float q0;  // 四元数的标量部分（实部）
    float q1;  // 四元数的向量部分 i 分量
    float q2;  // 四元数的向量部分 j 分量
    float q3;  // 四元数的向量部分 k 分量
} Quaternion;
Quaternion NumQ = {1, 0, 0, 0};//四元数

float NormAcc;//存储加速度向量的归一化系数


struct V
{
    float x;
    float y;
    float z;
};
struct V Gravity, Acc, Gyro, AccGravity, GyroIntegError;//分别为理论重力方向， 实测加速度向量（注意需归一化），校正后的角速度， 重力方向误差，PI控制器积分误差
struct V MahonyOutputRad; // Mahony滤波器输出（弧度制）
struct V MahonyOutputDegree; // Mahony滤波器输出（角度制）
float KpDef = 0.5f;      // PI控制器比例增益：决定对当前误差的响应速度
float KiDef = 0.005f;   // PI控制器积分增益：消除长期累积误差和传感器偏置
float q0_t, q1_t, q2_t, q3_t;   // 四元数微分值
float NormQuat;                 // 四元数归一化系数

/*
算法流程图：
    
四元数 NumQ
    ↓ (旋转矩阵转换)
理论重力 Gravity ←————————————┐
    ↓                        │
    ↓ (叉积计算误差)           │
AccGravity ←——— Acc          │
    ↓ (实测重力)              │
    ↓ (PI控制器)              │
校正角速度 Gyro               │
    ↓ (四元数微分方程)         │
更新四元数 ————————————————————┘
*/

float AccConvert(int acc){
    // 将加速度计原始数据转换为实际加速度值
    // 假设量程为 ±2g，转换系数为 16384
    return (acc / 16384.0); // 返回单位为 g 的加速度值
}

float GyroConvert(int gyro){
    // 将陀螺仪原始数据转换为实际角速度值
    // 假设量程为 ±250°/s，转换系数为 7509.9
    return (gyro / 7509.9); // 返回单位为 rad/s 的角速度值
}

/**
 * 快速平方根倒数算法（Quake III 算法）
 * 功能：计算 1/sqrt(number) 的近似值
 * 原理：利用浮点数的二进制表示和牛顿迭代法快速求解平方根倒数
 * 用途：在姿态解算中用于向量归一化，避免昂贵的除法运算
 * @param number 输入的浮点数
 * @return 1/sqrt(number) 的近似值
 */
float Q_rsqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;                    // 计算 number/2，用于牛顿迭代
    y = number;                            // 保存原始值
    i = *(long *)&y;                       // 将浮点数的位模式解释为整数
    i = 0x5f3759df - (i >> 1);            // 魔术数字：初始猜测值的位操作
    y = *(float *)&i;                      // 将整数位模式转回浮点数
    y = y * (threehalfs - (x2 * y * y));   // 1st iteration （第一次牛顿迭代）
                                           // 牛顿迭代公式：y = y * (1.5 - 0.5 * number * y^2)
    return y;
}

//修正陀螺仪的安装误差，传入参数为测得的roll和pitch角度(°)
void CorrectAttitude(float AE_CORroll, float AE_CORpitch) {
    AE_CORroll = AE_CORroll / 180.0 * PI; // 将角度转换为弧度
    AE_CORpitch = AE_CORpitch / 180.0 * PI; // 将角度转换为弧度
    AE_CORroll = -AE_CORroll; // 取负值以适应坐标系(实验室两个MPU6050坐标系镜像)
    AE_CORpitch = -AE_CORpitch; // 取负值以适应坐标系
    // 计算修正后的旋转矩阵
    //https://zhuanlan.zhihu.com/p/195683958
    //公式：Q*RT
    /*R=
[cos(pitch)                    sin(roll)*sin(pitch)                cos(roll)*sin(pitch)              ]
[0                             cos(roll)                           -sin(roll)                        ]
[-sin(pitch)                   sin(roll)*cos(pitch)                cos(roll)*cos(pitch)              ]
    */
    AE_Rtemp[0][0] = AE_QtoR[0][0] * cos(AE_CORpitch) - AE_QtoR[0][2] * sin(AE_CORpitch);
    AE_Rtemp[0][1] = AE_QtoR[0][0] * sin(AE_CORroll) * sin(AE_CORpitch) + AE_QtoR[0][1] * cos(AE_CORroll) + AE_QtoR[0][2] * sin(AE_CORroll) * cos(AE_CORpitch);
    AE_Rtemp[0][2] = AE_QtoR[0][0] * cos(AE_CORroll) * sin(AE_CORpitch) - AE_QtoR[0][1] * sin(AE_CORroll) + AE_QtoR[0][2] * cos(AE_CORroll) * cos(AE_CORpitch);
    AE_Rtemp[1][0] = AE_QtoR[1][0] * cos(AE_CORpitch) - AE_QtoR[1][2] * sin(AE_CORpitch);
    AE_Rtemp[1][1] = AE_QtoR[1][0] * sin(AE_CORroll) * sin(AE_CORpitch) + AE_QtoR[1][1] * cos(AE_CORroll) + AE_QtoR[1][2] * sin(AE_CORroll) * cos(AE_CORpitch);
    AE_Rtemp[1][2] = AE_QtoR[1][0] * cos(AE_CORroll) * sin(AE_CORpitch) - AE_QtoR[1][1] * sin(AE_CORroll) + AE_QtoR[1][2] * cos(AE_CORroll) * cos(AE_CORpitch);
    AE_Rtemp[2][0] = AE_QtoR[2][0] * cos(AE_CORpitch) - AE_QtoR[2][2] * sin(AE_CORpitch);
    AE_Rtemp[2][1] = AE_QtoR[2][0] * sin(AE_CORroll) * sin(AE_CORpitch) + AE_QtoR[2][1] * cos(AE_CORroll) + AE_QtoR[2][2] * sin(AE_CORroll) * cos(AE_CORpitch);
    AE_Rtemp[2][2] = AE_QtoR[2][0] * cos(AE_CORroll) * sin(AE_CORpitch) - AE_QtoR[2][1] * sin(AE_CORroll) + AE_QtoR[2][2] * cos(AE_CORroll) * cos(AE_CORpitch);
    AE_QtoR[0][0] = AE_Rtemp[0][0];
    AE_QtoR[0][1] = AE_Rtemp[0][1];
    AE_QtoR[0][2] = AE_Rtemp[0][2];
    AE_QtoR[1][0] = AE_Rtemp[1][0];
    AE_QtoR[1][1] = AE_Rtemp[1][1];
    AE_QtoR[1][2] = AE_Rtemp[1][2];
    AE_QtoR[2][0] = AE_Rtemp[2][0];
    AE_QtoR[2][1] = AE_Rtemp[2][1];
    AE_QtoR[2][2] = AE_Rtemp[2][2];
}

void SetupMPU6050() {
        Serial.print("111");
        GyroIntegError.x = 0.0f;
        GyroIntegError.y = 0.0f;
        GyroIntegError.z = 0.0f;

        lastTime = micros(); // 初始化上次时间为当前时间

        Wire.begin();
        
        // 设置陀螺仪量程为 ±250°/s
        Wire.beginTransmission(MPU_Addr);
        Wire.write(0x1B);    // 陀螺仪配置寄存器
        Wire.write(0x00);    // 00000000 -> ±250°/s
        Wire.endTransmission();

        // 设置加速度计量程为 ±2g
        Wire.beginTransmission(MPU_Addr);
        Wire.write(0x1C);    // 加速度计配置寄存器
        Wire.write(0x00);    // 00000000 -> ±2g
        Wire.endTransmission();

        // ============== 设置低通滤波器 ==============
        Wire.beginTransmission(0x68);
        Wire.write(0x1A);           // CONFIG寄存器地址
        Wire.write(0x03);           // 低通滤波频率，0x03对应42Hz
        Wire.endTransmission();

        //Serial.begin(57600);

        Serial.print("232");
        if(1){         //暂时不启用偏移量测量
            /*角速度计偏移量计算*/
            for (i=0; i<2000; i++){
                // 读取 MPU-6050 的数据寄存器
            Wire.beginTransmission(MPU_Addr);
            // 从寄存器 0x3B 开始
            Wire.write(0x3B);
            Wire.endTransmission(false);
            // 访问 14 个寄存器
            Wire.requestFrom(MPU_Addr, 14, true);

            // 将高位和低位数据组合成两个字节
            AcX = Wire.read() << 8 | Wire.read();
            AcY = Wire.read() << 8 | Wire.read();
            AcZ = Wire.read() << 8 | Wire.read();
            Tmp = Wire.read() << 8 | Wire.read();
            GyX = Wire.read() << 8 | Wire.read();
            GyY = Wire.read() << 8 | Wire.read();
            GyZ = Wire.read() << 8 | Wire.read();

            realGyX = GyroConvert(GyX); // 转换为实际角速度值，单位rad/s
            realGyY = GyroConvert(GyY);
            realGyZ = GyroConvert(GyZ);
            realAcX = AccConvert(AcX); // 转换为实际加速度值，单位g
            realAcY = AccConvert(AcY);

            gx_offset += realGyX; // 累加x轴角速度的偏移量
            gy_offset += realGyY; // 累加y轴角速度的偏移量
            gz_offset += realGyZ; // 累加z轴角速度的偏移量 
            delay(1);
            }
        }
        Serial.print("323");

        if(1){ //暂时不启用偏移量测量
            gx_offset /= 2000; // 计算x轴角速度的偏移量
            gy_offset /= 2000; // 计算y轴角速度的偏移量
            gz_offset /= 2000; // 计算z轴角速度的偏移量
            // ax_offset /= 2000; // 计算x轴加速度的偏移量
            // ay_offset /= 2000; // 计算y轴加速度的偏移量
        }
        if(0){ //使用手动设置的偏移量
            gx_offset = -0.097552;
            gy_offset = 0.035832;
            gz_offset = -0.002609;
        }
        Serial.println();
        Serial.print("gx_offset:");Serial.print(gx_offset,6);
        Serial.print("  gy_offset:");Serial.print(gy_offset,6);
        Serial.print("  gz_offset:");Serial.print(gz_offset,6);
        Serial.println();

        /*卡尔曼滤波器初始化*/
        for (int j = 0; j < 3; j++) {
            accel_ekf[j].LastP = 0.02;  // 初始估计误差协方差
            accel_ekf[j].Now_P = 0.0;   // 当前预测误差协方差
            accel_ekf[j].out = 0.0;     // 滤波器输出初始值
            accel_ekf[j].Kg = 0.0;      // 卡尔曼增益初始值
            accel_ekf[j].Q = 0.001;     // 过程噪声协方差（系统噪声）
            accel_ekf[j].R = 0.543;//0.543       // 测量噪声协方差（传感器噪声）
        }
        Serial.print("222");
}

void GetDataMPU6050Rate(float &rollRate, float &pitchRate, float &yawRate) {
    RateNowTime = micros();// 获取当前时间
    Rate_dt_us = RateNowTime - RatelastTime; // 计算微分时间间隔，单位为毫秒
    Rate_dt = Rate_dt_us / 1000000.0; // 计算微分时间间隔，单位为秒
    RatelastTime = RateNowTime; // 更新上次时间

    // 读取 MPU-6050 的数据寄存器
    Wire.beginTransmission(MPU_Addr);
    // 从寄存器 0x3B 开始
    Wire.write(0x3B);
    Wire.endTransmission(false);
    // 访问 14 个寄存器
    Wire.requestFrom(MPU_Addr, 14, true);

    // 将高位和低位数据组合成两个字节
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    Tmp = Wire.read() << 8 | Wire.read();
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();

    realGyX = GyroConvert(GyX); // 转换为实际角速度值，单位rad/s
    realGyY = GyroConvert(GyY);
    realGyZ = GyroConvert(GyZ);

    /*step1:陀螺仪零偏校正*/
    realGyX -= gx_offset; // 减去x轴角速度偏移量
    realGyY -= gy_offset; // 减去y轴角速度偏移量
    realGyZ -= gz_offset; // 减去z轴角速度偏移量

    rollRate = realGyX * 180 / PI; // 转换为角速度，单位°/s
    pitchRate = realGyY * 180 / PI; // 转换为角速度，单位°/s
    yawRate = realGyZ * 180 / PI; // 转换为角速度，单位°/s
}

void GetDataMPU6050(float &roll, float &pitch, float &yaw, float &rollRate, float &pitchRate, float &yawRate, float &RawrollRate, float &RawpitchRate, float AE_RollOffset, float AE_PitchOffset) {
    nowTime = micros();// 获取当前时间
    AE_dt_us = nowTime - lastTime; // 计算微分时间间隔，单位为毫秒
    AE_dt = AE_dt_us / 1000000.0; // 计算微分时间间隔，单位为秒
    lastTime = nowTime; // 更新上次时间

    // 读取 MPU-6050 的数据寄存器
    Wire.beginTransmission(MPU_Addr);
    // 从寄存器 0x3B 开始
    Wire.write(0x3B);
    Wire.endTransmission(false);
    // 访问 14 个寄存器
    Wire.requestFrom(MPU_Addr, 14, true);

    // 将高位和低位数据组合成两个字节
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    Tmp = Wire.read() << 8 | Wire.read();
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();

    // 输出原始数据，用于存储到电脑
    // Serial.print("Raw Acc: ");
    // Serial.print(AcX); Serial.print(" ");
    // Serial.print(AcY); Serial.print(" ");
    // Serial.print(AcZ); Serial.print(" ");
    // Serial.print(GyX); Serial.print(" ");
    // Serial.print(GyY); Serial.print(" ");
    // Serial.print(GyZ); Serial.print(" ");
//    Serial.print("AccelX: ");Serial.print(AcX);Serial.print("  ");
//    realAcX=AcX / 16384.0;//当前量程2g
//    Serial.print("realAccelX: ");Serial.print(realAcX);Serial.print("  ");
//    Serial.print("AccelY: ");Serial.print(AcY);Serial.print("  ");
//    Serial.print("AccelZ: ");Serial.print(AcZ);Serial.println("");

//    Serial.print("Temperature: ");Serial.print(Tmp);Serial.print("   ");
//    Temperature=((Tmp/340.0)+36.53);//温度转换
//    Serial.print("realTemperature: ");Serial.print(Temperature);Serial.println("");

//    Serial.print("GyroX: ");Serial.print(GyX);Serial.print("  ");
//    Serial.print("GyroY: ");Serial.print(GyY);Serial.print("  ");
//    Serial.print("GyroZ: ");Serial.print(GyZ);Serial.println();
//    Serial.print("AE_dt_us: ");Serial.print(AE_dt_us);Serial.println();
//    Serial.println();

    /*step0:数据转换*/
    realAcX = AccConvert(AcX); // 转换为实际加速度值，单位g
    realAcY = AccConvert(AcY); 
    realAcZ = AccConvert(AcZ);
    realGyX = GyroConvert(GyX); // 转换为实际角速度值，单位rad/s
    realGyY = GyroConvert(GyY);
    realGyZ = GyroConvert(GyZ);
    // Serial.print("Gyr");Serial.print(realGyX);Serial.print(",");Serial.print(realGyY);Serial.print(",");Serial.print(realGyZ);Serial.print(" ");
    // Serial.println("");

    /*step1:陀螺仪零偏校正*/
    realGyX -= gx_offset; // 减去x轴角速度偏移量
    realGyY -= gy_offset; // 减去y轴角速度偏移量
    realGyZ -= gz_offset; // 减去z轴角速度偏移量

    // 补偿，输出原始角速度用于光流传感器
    RawrollRate = realGyX; // 原始角速度，单位rad/s
    RawrollRate = RawrollRate * 180 / PI; // 转换为角速度，单位°/s
    RawpitchRate = realGyY; // 原始角速度，单位rad/s
    RawpitchRate = RawpitchRate * 180 / PI; // 转换为角速度，单位°/s

    //一阶低通滤波器，作用于角加速度
    realGyX = 0.15 * realGyX + 0.85 * lastrealGyX; // 一阶低通滤波
    realGyY = 0.15 * realGyY + 0.85 * lasrealGyY; // 一阶低通滤波
    realGyZ = 0.15 * realGyZ + 0.85 * lasrealGyZ; // 一阶低通滤波
    lastrealGyX = realGyX; // 更新上一时刻的角速度值
    lasrealGyY = realGyY; // 更新上一时刻的角速度值
    lasrealGyZ = realGyZ; // 更新上一时刻的角速度值
    // Serial.print("Gyr:");Serial.print(realGyX);Serial.print(",");
    // Serial.print(realGyY);Serial.print(",");Serial.print(realGyZ);Serial.print("  ");

    realAcX = -realAcX; // 翻转加速度计数据，适应惯性坐标系(实验室两个MPU6050坐标系镜像)
    realAcY = -realAcY; // 翻转加速度计数据，适应惯性坐标系
    realAcZ = -realAcZ; // 翻转加速度计数据，适应惯性坐标系

    // realAcX -= ax_offset; // 减去x轴加速度偏移量
    // realAcY -= ay_offset; // 减去y轴加速度偏移量
    // Serial.print("ax_offset: "); Serial.print(ax_offset, 4); Serial.print("  ");
    // Serial.print("ay_offset: "); Serial.print(ay_offset, 4); Serial.print("");

    
    AcX_For_World_Acc = realAcX;
    AcY_For_World_Acc = realAcY;
    AcZ_For_World_Acc = realAcZ;

    /*step2:加速度计数据卡尔曼滤波*/
    // 对三轴加速度分别进行卡尔曼滤波
    kalman_1(&accel_ekf[0], realAcX);
    kalman_1(&accel_ekf[1], realAcY); 
    kalman_1(&accel_ekf[2], realAcZ);

    
    // 更新为滤波后的加速度值
    realAcX = accel_ekf[0].out;
    realAcY = accel_ekf[1].out;
    realAcZ = accel_ekf[2].out;
    
    //输出滤波后的加速度和角速度数据
    // Serial.print("Filtered - AcX: ");Serial.print(realAcX, 4);Serial.print("  ");
    // Serial.print("AcY: ");Serial.print(realAcY, 4);Serial.print("  ");
    // Serial.print("AcZ: ");Serial.print(realAcZ, 4);Serial.print("  ");
    // Serial.print("GyX: ");Serial.print(realGyX, 4);Serial.print("  ");
    // Serial.print("GyY: ");Serial.print(realGyY, 4);Serial.print("  ");
    // Serial.print("GyZ: ");Serial.print(realGyZ, 4);Serial.print("  ");
    // Serial.print("AE_dt: ");Serial.print(AE_dt, 4);Serial.print("  ");

    //接下来是Mahony滤波算法的实现
    // ============== 第1步：从当前四元数提取重力在机体坐标系下的表示 ==============
    // 数学原理：四元数到旋转矩阵的转换
    // 
    // 四元数 q = (q₀, q₁, q₂, q₃) 对应的旋转矩阵 R 为：
    // R = [1-2(q₂²+q₃²)   2(q₁q₂-q₀q₃)   2(q₁q₃+q₀q₂)]
    //     [2(q₁q₂+q₀q₃)   1-2(q₁²+q₃²)   2(q₂q₃-q₀q₁)]
    //     [2(q₁q₃-q₀q₂)   2(q₂q₃+q₀q₁)   1-2(q₁²+q₂²)]
    /*

     *****您的Mahony滤波器生成的四元数 NumQ遵循的是 “机体 -> 世界” (q_b2w)​ 的约定。
     因此由上式推导出的旋转矩阵左乘一个向量表示从机体坐标系转换到世界坐标系。而旋转矩阵是一个正交矩阵，逆矩阵等于转置矩阵
     所以这里把把矩阵转置再左乘一个重力向量，得到机体坐标系下的重力向量表示


    */
    //
    // 重力向量在惯性坐标系为 g_inertial = [0, 0, 1]ᵀ
    // 转换到机体坐标系：g_body = R × g_inertial = R的第三列
    //
    // 因此重力在机体坐标系的表示为旋转矩阵的第三列元素：
    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Gravity.y = 2 * (NumQ.q2 * NumQ.q3 + NumQ.q0 * NumQ.q1);
    Gravity.z = 1 - (2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2));
    // Gravity.x = -Gravity.x; // 翻转重力向量，适应惯性坐标系
    // Gravity.y = -Gravity.y; // 翻转重力向量，适应惯性坐标系
    // Gravity.z = -Gravity.z; // 翻转重力向量，适应惯性坐标系



    NormAcc = Q_rsqrt(Gravity.x * Gravity.x + Gravity.y * Gravity.y + Gravity.z * Gravity.z);
    Gravity.x *= NormAcc; // 归一化重力向量
    Gravity.y *= NormAcc;
    Gravity.z *= NormAcc;

    // ============== 第2步：加速度计数据归一化 ==============
    // 将加速度计测量值归一化为单位向量，消除重力大小的影响
    // 这样得到的是加速度计测量的重力方向
    // 数学公式：NormAcc = 1 / √(accX² + accY² + accZ²)
    NormAcc = Q_rsqrt(realAcX * realAcX + realAcY * realAcY + realAcZ * realAcZ);

    Acc.x = realAcX * NormAcc; // 归一化加速度计数据
    Acc.y = realAcY * NormAcc;
    Acc.z = realAcZ * NormAcc;
    // Serial.print("Acc:"); Serial.print(Acc.x); Serial.print(","); Serial.print(Acc.y); Serial.print(","); Serial.print(Acc.z); Serial.println("");
    

    // ============== 第3步：计算重力方向误差 ==============
    // 通过向量叉积计算理论重力方向与实测重力方向的误差
    // 叉积结果表示需要多大的旋转来纠正这个误差
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y); // x轴误差
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z); // y轴误差
    // if(AccGravity.x >0.01 || AccGravity.x < -0.01 || AccGravity.y >0.01|| AccGravity.y < -0.01 ) {
    //     KpDef = 2.0f; // 如果误差较大，增大比例增益
    // }
    // else {
    //     KpDef = 0.8f; // 如果误差较小，恢复比例增益
    // }
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x); // z轴误差
    // Serial.print("AccGravity: "); Serial.print(AccGravity.x, 4); Serial.print(",");
    // Serial.print(AccGravity.y, 4); Serial.print(",");
    // Serial.print(AccGravity.z, 4); Serial.print("  ");

    // ============== 第4步：PI控制器计算校正角速度 ==============
    // 使用PI控制器来校正陀螺仪的漂移和偏置

    // 积分项：累积误差，用于消除陀螺仪的长期漂移
    GyroIntegError.x += AccGravity.x * KiDef * AE_dt; // x轴积分误差
    GyroIntegError.y += AccGravity.y * KiDef * AE_dt; // y轴积分误差
    GyroIntegError.z += AccGravity.z * KiDef * AE_dt; // z轴积分误差

    // 融合校正后的角速度 = 原始角速度 + 比例校正 + 积分校正
    // 这样可以利用加速度计的长期稳定性来校正陀螺仪的短期漂移
    Gyro.x = realGyX + AccGravity.x * KpDef + GyroIntegError.x; // x轴校正角速度
    Gyro.y = realGyY + AccGravity.y * KpDef + GyroIntegError.y; // y轴校正角速度
    Gyro.z = realGyZ + AccGravity.z * KpDef + GyroIntegError.z; // z轴校正角速度



    // ============== 第5步：四元数微分方程求解==============
    // 根据角速度计算四元数的变化率，然后进行数值积分更新四元数
    // 四元数微分方程：dq/AE_dt = 0.5 * q ⊗ ω，其中⊗表示四元数乘法，ω是角速度向量
    q0_t = 0.5*(-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z)  * AE_dt; 
    q1_t = 0.5*(NumQ.q0 * Gyro.x + NumQ.q2 * Gyro.z - NumQ.q3 * Gyro.y)  * AE_dt;
    q2_t = 0.5*(NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z + NumQ.q3 * Gyro.x)  * AE_dt;
    q3_t = 0.5*(NumQ.q0 * Gyro.z + NumQ.q1 * Gyro.y - NumQ.q2 * Gyro.x)  * AE_dt;

    // 更新四元数：NumQ = NumQ + dq
    NumQ.q0 += q0_t;
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t; 

    // Serial.print("q_t: "); Serial.print(q0_t, 4); Serial.print(",");
    // Serial.print(q1_t, 4); Serial.print(",");
    // Serial.print(q2_t, 4); Serial.print(",");
    // Serial.print(q3_t, 4); Serial.print("  ");

    // ============== 第6步：四元数归一化 ==============
    // 为了保持四元数的单位长度，进行归一化处理
    NormQuat = Q_rsqrt(NumQ.q0 * NumQ.q0 + NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3);

    NumQ.q0 *= NormQuat; 
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    //Serial.print("NumQ: ("); Serial.print(NumQ.q0, 4); Serial.print(","); Serial.print(NumQ.q1, 4); Serial.print(","); 
    //Serial.print(NumQ.q2, 4); Serial.print(","); Serial.print(NumQ.q3, 4); Serial.print(")  ");
    // ============== 第7步：四元数转换为欧拉角 ==============
    // 将四元数转换为更直观的欧拉角表示（俯仰角、横滚角、偏航角）
    //计算偏航角

    //7月25日补充(初始姿态角补偿)
    //计算四元数转旋转矩阵
/*机体到世界坐标系的旋转矩阵，即右乘一个向量表示从机体坐标系转换到世界坐标系
    float R[3][3] = {
        {1-2*(q2*q2+q3*q3), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)},
        {2*(q1*q2+q0*q3), 1-2*(q1*q1+q3*q3), 2*(q2*q3-q0*q1)},
        {2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 1-2*(q1*q1+q2*q2)}
    };
*/
    AE_QtoR[0][0] = 1 - 2 * (NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3);
    AE_QtoR[0][1] = 2 * (NumQ.q1 * NumQ.q2 - NumQ.q0 * NumQ.q3);
    AE_QtoR[0][2] = 2 * (NumQ.q1 * NumQ.q3 + NumQ.q0 * NumQ.q2);
    AE_QtoR[1][0] = 2 * (NumQ.q1 * NumQ.q2 + NumQ.q0 * NumQ.q3);
    AE_QtoR[1][1] = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q3 * NumQ.q3);
    AE_QtoR[1][2] = 2 * (NumQ.q2 * NumQ.q3 - NumQ.q0 * NumQ.q1);
    AE_QtoR[2][0] = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    AE_QtoR[2][1] = 2 * (NumQ.q2 * NumQ.q3 + NumQ.q0 * NumQ.q1);
    AE_QtoR[2][2] = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);



    //修正陀螺仪的安装误差，传入参数为测得的roll和pitch角度(°)
    CorrectAttitude(AE_RollOffset, AE_PitchOffset);

    //MahonyOutputRad.y = asin(-(2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2))); // 俯仰角
    MahonyOutputRad.y = asin(-AE_QtoR[2][0]); // 俯仰角
    //MahonyOutputRad.x = atan2(2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3), 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2)); // 横滚角
    MahonyOutputRad.x = atan2(AE_QtoR[2][1], AE_QtoR[2][2]); // 横滚角
    //MahonyOutputRad.z = atan2(2 * (NumQ.q1 * NumQ.q2 + NumQ.q0 * NumQ.q3), 1 - 2 * (NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3)); // 偏航角
    MahonyOutputRad.z = atan2(AE_QtoR[1][0], AE_QtoR[0][0]); // 偏航角
    //MahonyOutputRad.z += realGyZ * AE_dt;

    // 将弧度转换为角度
    MahonyOutputDegree.x = MahonyOutputRad.x * 180 / PI; // 横滚角（Roll）
    MahonyOutputDegree.y = MahonyOutputRad.y * 180 / PI; // 俯仰角（Pitch）
    MahonyOutputDegree.z = MahonyOutputRad.z * 180 / PI; // 偏航角（Yaw）


    // 输出结果
    // Serial.print("AE_dt: "); Serial.print(AE_dt,4); Serial.print("  ");
    // Serial.print("Roll: "); Serial.print(MahonyOutputDegree.x); Serial.print("  ");
    // Serial.print("Pitch: "); Serial.print(MahonyOutputDegree.y); Serial.print("  ");
    // Serial.print("Yaw: "); Serial.print(MahonyOutputDegree.z); Serial.println("");

    //姿态可视化
    // Serial.print("KpDef: ");Serial.print(KpDef);Serial.print("  ");
    // Serial.print(MahonyOutputDegree.x);
    // Serial.print("/");
    // Serial.print(MahonyOutputDegree.y);
    // Serial.print("/");
    // Serial.println(MahonyOutputDegree.z);

    //输出数据
    roll = MahonyOutputDegree.x; // 更新全局变量
    pitch = MahonyOutputDegree.y; // 更新全局变量
    yaw = MahonyOutputDegree.z; // 更新全局变量

    rollRate = (roll - AE_last_roll) / AE_dt; // 计算roll角速度
    pitchRate = (pitch - AE_last_pitch) / AE_dt; // 计算pitch角速度
    yawRate = (yaw - AE_last_yaw) / AE_dt; // 计算yaw角速度

    AE_last_roll = roll; // 更新上次的roll角度
    AE_last_pitch = pitch; // 更新上次的pitch角度
    AE_last_yaw = yaw; // 更新上次的yaw角度



    
}

float last_worldAccX = 0.0f;
float last_worldAccY = 0.0f;

void get_Horizontal_heading_Acc(float& worldAccX, float& worldAccY){//m/s²  Horizontal heading

    // 构建目标姿态的四元数（横滚俯仰为0，偏航保持）
    float target_q0 = cos(MahonyOutputRad.z/2);
    float target_q1 = 0;
    float target_q2 = 0;
    float target_q3 = sin(MahonyOutputRad.z/2);

    float R_b2ly[3][3];//机体坐标系到水平航向坐标系的旋转矩阵

    // 计算相对旋转四元数：q_relative = target_q* ⊗ current_q
    float qr0 = target_q0 * NumQ.q0 + target_q1 * NumQ.q1 + target_q2 * NumQ.q2 + target_q3 * NumQ.q3;
    float qr1 = target_q0 * NumQ.q1 - target_q1 * NumQ.q0 - target_q2 * NumQ.q3 + target_q3 * NumQ.q2;
    float qr2 = target_q0 * NumQ.q2 + target_q1 * NumQ.q3 - target_q2 * NumQ.q0 - target_q3 * NumQ.q1;
    float qr3 = target_q0 * NumQ.q3 - target_q1 * NumQ.q2 + target_q2 * NumQ.q1 - target_q3 * NumQ.q0;

    // 从相对旋转四元数构建从机体坐标系到水平航向坐标系的旋转矩阵
    R_b2ly[0][0] = 1 - 2 * (qr2 * qr2 + qr3 * qr3);
    R_b2ly[0][1] = 2 * (qr1 * qr2 - qr0 * qr3);
    R_b2ly[0][2] = 2 * (qr1 * qr3 + qr0 * qr2);
    R_b2ly[1][0] = 2 * (qr1 * qr2 + qr0 * qr3);
    R_b2ly[1][1] = 1 - 2 * (qr1 * qr1 + qr3 * qr3);
    R_b2ly[1][2] = 2 * (qr2 * qr3 - qr0 * qr1);
    R_b2ly[2][0] = 2 * (qr1 * qr3 - qr0 * qr2);
    R_b2ly[2][1] = 2 * (qr2 * qr3 + qr0 * qr1);
    R_b2ly[2][2] = 1 - 2 * (qr1 * qr1 + qr2 * qr2);
    
    // 将加速度从机体坐标系转换到水平航向坐标系(需要减去重力加速度),单位为g，重力加速度Gravity是单位向量(1g)
    float Acc_GravityX;
    float Acc_GravityY;
    float Acc_GravityZ;
    Acc_GravityX = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Acc_GravityY = 2 * (NumQ.q2 * NumQ.q3 + NumQ.q0 * NumQ.q1);
    Acc_GravityZ = 1 - (2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2));

    worldAccX = R_b2ly[0][0] * (realAcX - Acc_GravityX) + R_b2ly[0][1] * (realAcY - Acc_GravityY) + R_b2ly[0][2] * (realAcZ - Acc_GravityZ);
    worldAccY = R_b2ly[1][0] * (realAcX - Acc_GravityX) + R_b2ly[1][1] * (realAcY - Acc_GravityY) + R_b2ly[1][2] * (realAcZ - Acc_GravityZ);

    worldAccX = -worldAccX * 9.81; // 转换为m/s²
    worldAccY = -worldAccY * 9.81; // 转换为m/s²
    
    worldAccX = 0.25 * worldAccX + 0.75 * last_worldAccX; // 一阶低通滤波
    worldAccY = 0.25 * worldAccY + 0.75 * last_worldAccY; // 一阶低通滤波
    last_worldAccX = worldAccX;
    last_worldAccY = worldAccY;
}

void get_WorldAcc(float& worldAccX, float& worldAccY) {
    //直接利用代码里已经算好的 AE_QtoR (机体到世界的旋转矩阵)
    // AE_QtoR[行][列]

    // 2. 将机体坐标系下的加速度 (realAcX, realAcY, realAcZ) 转换到世界系

    //设置是否使用卡尔曼滤波后的加速度值
        float ax_w = AE_QtoR[0][0] * realAcX + AE_QtoR[0][1] * realAcY + AE_QtoR[0][2] * realAcZ;
        float ay_w = AE_QtoR[1][0] * realAcX + AE_QtoR[1][1] * realAcY + AE_QtoR[1][2] * realAcZ;
        //float az_w = AE_QtoR[2][0] * realAcX + AE_QtoR[2][1] * realAcY + AE_QtoR[2][2] * realAcZ;
    
    
        // float ax_w = AE_QtoR[0][0] * AcX_For_World_Acc + AE_QtoR[0][1] * AcY_For_World_Acc + AE_QtoR[0][2] * AcZ_For_World_Acc;
        // float ay_w = AE_QtoR[1][0] * AcX_For_World_Acc + AE_QtoR[1][1] * AcY_For_World_Acc + AE_QtoR[1][2] * AcZ_For_World_Acc;
        // //float az_w = AE_QtoR[2][0] * AcX_For_World_Acc + AE_QtoR[2][1] * AcY_For_World_Acc + AE_QtoR[2][2] * AcZ_For_World_Acc;
    

    // 3. 在世界系下减去重力 (NED坐标系中重力是 [0, 0, 1]g)
    // 如果你的 Z 轴向上为正，则 az_w = az_w - 1.0;
    // 这里我们只关心水平轴
    worldAccX = ax_w * 9.81f; 
    worldAccY = ay_w * 9.81f;
    //worldAccZ = az_w * 9.81f;
    //worldAccZ = worldAccZ - 9.81f; // 减去重力加速度

    worldAccX = -worldAccX; // 翻转加速度，适应惯性坐标系
    worldAccY = -worldAccY; // 翻转加速度，适应惯性坐标系
}
