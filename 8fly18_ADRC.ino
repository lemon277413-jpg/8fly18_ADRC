// ============================================================
// 8fly飞控 - 定位悬停功能半成品 (增加保护与校准状态)，预留飞行校准和定高定点状态
// 功能：实现四轴无人机的姿态稳定控制、遥控响应、以及保护和校准功能
// 硬件：Arduino + MPU6050 + 4个电机 + 遥控接收器 + MTF02光流模块 + UWB模块
// 兼具切换飞行校准状态和定高定点状态，供后续开发使用
// ============================================================

// 上电默认：进入 PROTECT 保护状态，电机锁死。
// 解锁进入待飞：将油门（Ch2）拉最低，偏航（Ch3）推最右，横滚（Ch0）和俯仰（Ch1）拉最左，保持 0.5 秒。
// 进行校准：在 INIT 待飞状态下，将偏航（Ch3）拉最左，保持 0.5 秒。系统会自动读取当前角度并保存为零偏，随后自动回到 INIT 状态。
// 起飞：推油门即可起飞。
// 飞行校准：在 FLY 飞行状态下，将偏航（Ch3）拉最左，保持 0.04 秒后进入飞行校准状态并保持；将偏航（Ch3）推至大于 1150，保持 0.04 秒后返回飞行状态。
// 定高定点：在 FLY 飞行状态下，将偏航（Ch3）推最右，保持 0.04 秒后进入定高定点状态，前 2 秒除拉低油门回保护状态外不响应其他切换；再次将偏航（Ch3）推最右，保持 0.04 秒后返回飞行状态。

#include <Wire.h>
#include <EnableInterrupt.h>
#include "pid.h"
#include "AttitudeEstimator.h"
#include "mtf02.h"
#include "uwb.h"
#include "CircularDelay.h"
#include "adrc_controller.h"
    float World_AccX;
	float World_AccY;

    float LPF_real_pos_X = 0.0f;
    float LPF_real_pos_Y = 0.0f;

    float accel_bias_X = 0.0f;
    float accel_bias_Y = 0.0f;
// ============================================================
// 1. 硬件引脚与宏定义
// ============================================================
#define PIN_ROLL     10
#define PIN_PITCH    11
#define PIN_THROTTLE 12
#define PIN_YAW      13

#define PIN_MOTOR_LU_1 2
#define PIN_MOTOR_LU_2 3
#define PIN_MOTOR_RU_1 4
#define PIN_MOTOR_RU_2 5
#define PIN_MOTOR_RD_1 6
#define PIN_MOTOR_RD_2 7
#define PIN_MOTOR_LD_1 8
#define PIN_MOTOR_LD_2 9

#define ESC_MIN_PWM 1000
#define ESC_MAX_PWM 2000
#define ESC_ARM_PWM 1000

// ========== 新增：飞行校准相关宏定义 ==========
#define RC_DEADBAND 50        // 遥控器摇杆死区 (±50us，避免回中误触发)
#define TRIM_STEP 0.02f       // 微调步长 (50Hz任务下，每周期调整0.05°，速度适中)
#define MAX_TRIM_ANGLE 3.0f   // 最大微调范围 ±3°


// ============================================================
// 定义状态枚举类 (Enum Class)
// ============================================================
enum class FlightState {
    PROTECT,  // 保护状态 (默认，最高优先级)
    INIT,     // 待起飞状态
    CALIBRATE,// 校准状态
    FLY,       // 飞行状态
    FLY_CALIBRATE, // 飞行校准状态
    HOLD_POSITION  // 定高定点状态
};

// ============================================================
// 【固定配置】定位数据源选择 (仅修改这里即可切换)
// ============================================================
enum class PositionSource {
    UWB_IMU,           // UWB融合IMU (默认)
    OPTICAL_FLOW,      // 纯光流数据
    OPTICAL_FLOW_IMU   // 光流融合IMU数据
};

//固定指定当前数据源：默认UWB，修改等号后面的值即可切换
const PositionSource CURRENT_POS_SOURCE = PositionSource::UWB_IMU;

// ============================================================
// 2. 全局变量
// ============================================================

// ====================== 核心状态机逻辑 (50Hz驱动) ======================
int state_counter = 0; // 状态机计数器，50Hz下每加1代表20ms
FlightState state = FlightState::PROTECT; // 默认保护状态
unsigned long state_timer = 0;             // 用于状态切换计时
bool calibration_done = false;              // 确保校准只执行一次的标志位
bool HOLD_POSITION_return_lock = false; // 定高定点返回飞行的锁定标志，防止重复切换(2秒内)
bool FLY_CALIBRATE_return_lock = false; // 飞行校准返回飞行的锁定标志，防止重复切换(2秒内)

// 校准后的零偏变量 (初始化为默认值)
float calib_roll_offset = -1.08f;
float calib_pitch_offset = -1.89f;
// ========== 新增：飞行校准微调零偏变量 ==========
float fly_calib_roll_offset = 0.0f;    // 飞行中横滚微调零偏 (±3°限制)
float fly_calib_pitch_offset = 0.0f;   // 飞行中俯仰微调零偏 (±3°限制)


// ============================================================
// 200Hz定时器中断 全局变量(核心变量池)
// ============================================================
volatile uint16_t freqTick = 1;  // 核心频率控制变量，volatile必须加（中断修改+主循环读取）
volatile bool flag_200Hz = false;        // 200Hz 任务标志位
volatile bool flag_100Hz = false;        // 100Hz 任务标志位
volatile bool flag_50Hz = false;         // 50Hz 任务标志位
volatile bool flag_25Hz = false;         // 25Hz 任务标志位

// 遥控器数据
volatile int receiver_input[4] = {1500, 1500, 1000, 1500};
unsigned long timer_roll, timer_pitch, timer_throttle, timer_yaw;

//期望位置数据
float target_X_Position = 0.0f; // 期望X轴位置 (单位: 米)
float target_Y_Position = 0.0f; // 期望Y轴位置 (单位: 米)
float target_Height = 0.0f; // 期望高度 (单位: 米)
//期望速度数据
float target_X_Speed = 0.0f; // 期望X轴速度 (单位: m/s)
float target_Y_Speed = 0.0f; // 期望Y轴速度 (单位: m/s)
float target_Height_Speed = 0.0f; // 期望高度速度 (单位: m/s)
//期望加速度数据
float target_X_Accel = 0.0f; // 期望X轴加速度 (单位: m/s²)
float target_Y_Accel = 0.0f; // 期望Y轴加速度 (单位: m/s²)

// 姿态数据
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
float gx = 0.0f, gy = 0.0f, gz = 0.0f;  //这里是100Hz读取的陀螺仪角速度数据，单位是°/s
float Raw_rollRate = 0.0f, Raw_pitchRate = 0.0f;  // 这里是100Hz读取的陀螺仪原始角速度数据，单位是°/s
// 这里是200hz读取的陀螺仪角速度数据，单位是°/s
float rollRate_200hz = 0.0f, pitchRate_200hz = 0.0f, yawRate_200hz = 0.0f;

// PID控制器 (仅高度环、位置环、速度环; 角度环已由ADRC替代)
PIDController pidController;

// ADRC控制器 (二阶LADRC: 3阶LESO + PD控制律, 替代PID+一阶ADRC)
ADRC_Param adrcRoll;
ADRC_Param adrcPitch;
ADRC_Param adrcYaw;

//定高定点状态相关变量
int hold_base_throttle = 1000; // 定高基础油门，进入状态时锁存
int hold_throttle = 1000; // 定高当前油门，根据PID输出调整
bool is_hold_throttle_initialized = false; // 定高油门是否已初始化的标志

// 期望姿态
float targetRoll = 0.0f, targetPitch = 0.0f, targetYaw = 0.0f;

// 电机输出
int ESC_PWM[4] = {ESC_MIN_PWM, ESC_MIN_PWM, ESC_MIN_PWM, ESC_MIN_PWM};

// ============================================================
// 光流数据变量
// ============================================================
uint32_t g_distance;       // 高度 (mm)
uint8_t g_strength;       // 信号强度
int16_t  g_flow_vel_x;     // X轴速度
int16_t  g_flow_vel_y;     // Y轴速度
uint8_t  g_flow_quality;   // 光流置信度

float MTF_measured_X_speed; // MTF测量的X轴速度 (单位: m/s)
float MTF_measured_Y_speed; // MTF测量的Y轴速度 (单位: m/s)
float MTF_measured_Height_speed; // MTF测量的高度速度 (单位: m/s)
float MTF_Height; // MTF测量的高度 (单位: m)
float MTF_PosX; // MTF测量的X轴位置 (单位: m)
float MTF_PosY; // MTF测量的Y轴位置 (单位: m)

float MTF_measured_X_speed_fuse_acc = 0.0f; // 融合加速度的X轴速度 (单位: m/s)
float MTF_measured_Y_speed_fuse_acc = 0.0f; // 融合加速度的Y轴速度 (单位: m/s)
float MTF_PosX_fuse_acc = 0.0f; // 融合加速度的X轴位置 (单位: m)
float MTF_PosY_fuse_acc = 0.0f; // 融合加速度的Y轴位置 (单位: m)

CircularDelayLine pitchDelay; // 俯仰角速度延时环实例
CircularDelayLine rollDelay;  // 横滚角速度延时环实例


// ============================================================
// uwb数据变量
// 数据结构体定义
// ============================================================
typedef struct {
  uint8_t frame_header[3]; // 0x01 0x00 0x02
  int32_t pos[3];          // x, y, z (内部解析用)
} NLink_Frame0;

NLink_Frame0 current_frame;
float real_pos[3];         // 最终位置 (单位: 米)
int Is_UWB_work = 0;       // 数据有效标志
int Is_UWB_Used = 0;       // UWB数据是否被使用，0表示未使用，1表示已使用

//X/Y轴结果结构体
EKF_UWB_IMU_DATA ekf_X;
EKF_UWB_IMU_DATA ekf_Y;

//创建X/Y轴滤波实例
EKFFilter ekfX(-0.33);
EKFFilter ekfY(0.17);

// ============================================================
// 3. 函数前置声明
// ============================================================
void initTimer5_200Hz(); // Timer5 200Hz中断初始化
int32_t readInt24LE(uint8_t* data); //uwb小端解析函数
void processFrameData(); //uwb数据处理函数
void initHardware();
void initESC();
void pwmReceiveRoll();
void pwmReceivePitch();
void pwmReceiveThrottle();
void pwmReceiveYaw();
void read_IMU_Rate(); // 读取角速度数据
void read_IMU_Angle(); // 读取角度数据
void readMTF02(); // 读取MTF02数据
void readUWB(); // 读取UWB数据
void Cal_MTF02_Position(int MTF_dx_raw, int MTF_dy_raw, float MTF_height); // 根据MTF02数据计算位置和速度和高度
void Cal_UWB_Position(); // 根据UWB数据计算位置
void cal_Position_PID();
void cal_Speed_PID();
void cal_Height_PID();
void read_remote_control();
void calculateADRC();
void escCtrl(int throttle, float rollCORR, float pitchCORR, float yawCORR);
void setAllMotorsHigh();
void setAllMotorsLow();
void debugPrint();

// ============================================================
// 4. Setup() 初始化
// ============================================================
void setup() {
    Serial.begin(925600);
    Serial.println("Initializing...");

    initHardware();
    initESC();
    SetupMPU6050();

    // 二阶LADRC控制器 (3阶LESO + PD控制律, 移植自Drone_Master_ADRC)
    // B参数: T-Motor AIR2216 KV880 + T1045 实测 → Δ拉力=1.73g/PWM/电机 @ 50%油门
    // ADRC_Init(Ts, wo, B, KpOut, KpIn, KdIn, max_u)
    //   Roll:  I_xx=0.13, 4电机/侧, 臂=0.30m → B=18.0 °/s²/PWM
    //   Pitch: I_yy=0.56, 同上             → B=4.2  °/s²/PWM
    //   Yaw:   I_zz=0.69, 8电机同轴反扭矩 → B=2.0  °/s²/PWM
    ADRC_Init(&adrcRoll,  0.005f, 20.0f, 18.0f, 3.5f, 1.0f, 0.1f, 150.0f);
    ADRC_Init(&adrcPitch, 0.005f, 10.0f,  4.2f, 3.5f, 1.0f, 0.1f, 150.0f);
    ADRC_Init(&adrcYaw,   0.005f, 15.0f,  2.0f, 5.0f, 1.0f, 0.1f, 200.0f);

    initTimer5_200Hz();

    Serial.println("Ready! System in PROTECT mode.");
  	delay(100);
}

// ============================================================
// 5. Loop() 主循环
// ============================================================
void loop() {

    // ====================== 200Hz 任务 (每5ms执行1次) ======================
    if(flag_200Hz) 
    {
        // long loop_timer = micros();
        flag_200Hz = false; // 【关键】执行完立即清零

        // 读取陀螺仪角速度数据 (200Hz)，并进行低通滤波
        read_IMU_Rate();

        // 原子读取油门，用于判断飞机是否已离地
        int throttle_pre;
        noInterrupts();
        throttle_pre = receiver_input[2];
        interrupts();

        // 内环：二阶ADRC控制 (3阶LESO + PD控制律)
        // 仅在飞行状态下运行，避免ESO在电机未响应时发散
        // 油门>1200才激活: 防止飞机还在地上时ESO因地面对抗而发散(尤其yaw)
        bool adrcActive = (state == FlightState::FLY ||
                           state == FlightState::FLY_CALIBRATE ||
                           state == FlightState::HOLD_POSITION) &&
                           throttle_pre > 1200;

        float rollOut, pitchOut, yawOut;
        if (adrcActive) {
            // 3阶LESO观测 (从角速度测量值估计 速度/加速度/扰动)
            ADRC_LESO(&adrcRoll, rollRate_200hz);
            ADRC_LESO(&adrcPitch, pitchRate_200hz);
            ADRC_LESO(&adrcYaw, yawRate_200hz);
            // PD控制律 + 扰动前馈补偿
            rollOut  = ADRC_InnerLoop(&adrcRoll);
            pitchOut = ADRC_InnerLoop(&adrcPitch);
            yawOut   = ADRC_InnerLoop(&adrcYaw);
        } else {
            // 非飞行状态：冻结ADRC，持续复位ESO状态
            ADRC_ParamClear(&adrcRoll);
            ADRC_ParamClear(&adrcPitch);
            ADRC_ParamClear(&adrcYaw);
            rollOut = 0.0f;
            pitchOut = 0.0f;
            yawOut = 0.0f;
        }
        // 输出限幅已在ADRC_InnerLoop完成(max_u), 此处不再重复


        // TODO: 调参阶段 - pitch+roll+yaw 全部启用

        switch (state) {
            case FlightState::FLY:
            {
                //电机操控
                escCtrl(throttle_pre, rollOut, pitchOut, yawOut);
                break;
            }
            case FlightState::FLY_CALIBRATE:
            {
                escCtrl(throttle_pre, rollOut, pitchOut, yawOut);
                break;
            }
            case FlightState::HOLD_POSITION:
            {
                if(is_hold_throttle_initialized)
                {
                    escCtrl(hold_throttle, rollOut, pitchOut, yawOut);
                }
                else
                {
                    escCtrl(throttle_pre, rollOut, pitchOut, yawOut);
                }
                break;
            }
            default:
            {
                // 待机/保护/校准状态：电机最低转速
                escCtrl(1000, 0, 0, 0);
                break;
            }
        }
        // while (micros() - loop_timer < 5000)
        // {
        //     Serial.println(6);
        // }
    }
        
    // ====================== 100Hz 任务 (每10ms执行1次) ======================
    if(flag_100Hz) 
    {
        flag_100Hz = false; // 【关键】执行完立即清零

        // 姿态解算 (使用校准后的零偏)
        read_IMU_Angle();

        // 外环: ADRC角度环 — AttOut = KpOut * (target - measured)  (100Hz)
        if (state == FlightState::FLY || state == FlightState::FLY_CALIBRATE ||
            state == FlightState::HOLD_POSITION) {
            calculateADRC();
        }
    }

    // ====================== 50Hz 任务 (每20ms执行1次) ======================
    if(flag_50Hz) 
    {
        flag_50Hz = false; // 【关键】执行完立即清零，防止重复执行

        // 原子读取遥控器数据 (AVR上16-bit volatile int非原子, 关中断防竞态)
        int rc_ch0, rc_ch1, rc_ch2, rc_ch3;
        noInterrupts();
        rc_ch0 = receiver_input[0];
        rc_ch1 = receiver_input[1];
        rc_ch2 = receiver_input[2];
        rc_ch3 = receiver_input[3];
        interrupts();

        // ========== 调整顺序：先读取遥控指令，再处理状态机 ==========
        // 确保状态机内设置的targetRoll/targetPitch不会被遥控数据覆盖
        read_remote_control();

        //读取光流数据
        readMTF02();

        //读取uwb数据
        readUWB();

        //uwb和imu融合
        Cal_UWB_Position();

        // 根据MTF02数据计算位置和速度和高度
        Cal_MTF02_Position(g_flow_vel_x, g_flow_vel_y, g_distance);

        // ====================== 核心状态机逻辑 (50Hz驱动) ======================
        // 条件满足计数器+1，不满足归零，>=25则代表0.5秒到了       
        switch (state) {
            // ---------------------------------------------------------
            // 状态 1: 保护状态 (最高优先级)
            // 解锁条件: Ch2<1100, Ch3>1900, Ch0<1100, Ch1<1100 保持 0.5秒 (25次)
            // ---------------------------------------------------------
            case FlightState::PROTECT: {
                bool isUnlockCond = (rc_ch2 < 1100) &&
                                    (rc_ch3 > 1900) &&
                                    (rc_ch0 < 1100) &&
                                    (rc_ch1 < 1100);
                if (isUnlockCond) {
                    state_counter++;
                    if (state_counter >= 25) {
                        state = FlightState::INIT;
                        state_counter = 0;
                        Serial.println("UNLOCKED: Entering INIT (Standby) state.");
                    }
                } else {
                    state_counter = 0; // 条件不满足，重置计数器
                }
                break;
            }

            // ---------------------------------------------------------
            // 状态 2: 待起飞状态
            // 逻辑1: 油门 > 1150 -> 起飞
            // 逻辑2: Ch3 (Yaw) < 1100 保持 0.5秒 (25次) -> 进入校准
            // ---------------------------------------------------------
            case FlightState::INIT: {
                // 检查进入校准的指令 (Yaw向左打满)
                if (rc_ch3 < 1100) {
                    state_counter++;
                    if (state_counter >= 25) { // 0.5秒后进入校准状态 (25 ticks x 20ms)
                        state = FlightState::CALIBRATE;
                        state_counter = 0;
                        calibration_done = false; // 重置校准标志
                        Serial.println("COMMAND: Entering CALIBRATION state.");
                    }
                } else {
                    state_counter = 0; // 摇杆回中，重置校准计时

                    // 正常起飞逻辑
                    if (rc_ch2 > 1150) {
                        state = FlightState::FLY;
                        // 复位ADRC的ESO状态，防止地面扰动估计残留
                        ADRC_ParamClear(&adrcRoll);
                        ADRC_ParamClear(&adrcPitch);
                        ADRC_ParamClear(&adrcYaw);
                        // 清空飞行校准偏置
                        fly_calib_roll_offset = 0.0f;
                        fly_calib_pitch_offset = 0.0f;
                        Serial.println("Taking off!");
                    }
                }
                break;
            }

            // ---------------------------------------------------------
            // 状态 3: 校准状态 (保持不变)
            // ---------------------------------------------------------
            case FlightState::CALIBRATE: {
                state_counter++;
                // 等待1秒(50 ticks)让姿态估计收敛后再保存零偏
                if (!calibration_done && state_counter >= 50) {
                    calib_roll_offset = roll;
                    calib_pitch_offset = pitch;
                    calibration_done = true;
                    ResetGyroIntegError(); // 同步清零陀螺PI积分，防止历史漂移累积

                    Serial.print("Calibration Finished! Offsets -> R: ");
                    Serial.print(calib_roll_offset);
                    Serial.print(" , P: ");
                    Serial.println(calib_pitch_offset);
                }
                if(state_counter >= 100) { // 2秒后自动返回待飞状态
                    state_counter = 0;
                    state = FlightState::INIT;
                    Serial.println("Returning to INIT state.");
                }
                break;
            }

            // ---------------------------------------------------------
            // 状态 4: 飞行状态
            // 逻辑1: receiver_input[3]<1100 保持0.04s -> 飞行校准
            // 逻辑2: receiver_input[3]>1900 保持0.04s -> 定高定点
            // 逻辑3: 油门拉低 -> 回保护
            // ---------------------------------------------------------
            case FlightState::FLY: {
                // 最高优先级：检查是否直接回保护
                if (rc_ch2 < 1100) {
                    state = FlightState::PROTECT;
                    state_counter = 0;
                    Serial.println("Landing... Entering PROTECT.");
                    break;
                }

                if(HOLD_POSITION_return_lock == true || FLY_CALIBRATE_return_lock == true) {
                    state_counter++;
                    if (state_counter >= 100) { // 2秒后解除锁定
                        HOLD_POSITION_return_lock = false;
                        FLY_CALIBRATE_return_lock = false;
                        state_counter = 0;
                        Serial.println("return lock released.");
                    }
                    break; // 在锁定期间不响应其他切换指令
                }
                // 检查进入飞行校准 (Ch3 < 1100)
                else if (rc_ch3 < 1100 && !FLY_CALIBRATE_return_lock) {
                    state_counter++;
                    if (state_counter >= 2) { // 0.04秒
                        state = FlightState::FLY_CALIBRATE;
                        state_counter = 0;
                        Serial.println("Entering FLY_CALIBRATE state.");
                    }
                }
                // 检查进入定高定点 (Ch3 > 1900)
                else if (rc_ch3 > 1900 && !HOLD_POSITION_return_lock) {
                    state_counter++;
                    if (state_counter >= 2) { // 0.04秒
                        state = FlightState::HOLD_POSITION;
                        state_counter = 0;
                        Serial.println("Entering HOLD_POSITION state.");
                    }
                }
                else {
                    state_counter = 0; // 摇杆居中，重置计数
                }
                break;
            }

            // ---------------------------------------------------------
            // 状态 5: 飞行校准状态
            // 逻辑: receiver_input[3]<1100 保持0.04s -> 返回飞行
            // ---------------------------------------------------------
            case FlightState::FLY_CALIBRATE: {
                // 最高优先级：检查是否直接回保护
                if (rc_ch2 < 1100) {
                    state = FlightState::PROTECT;
                    state_counter = 0;
                    Serial.println("Emergency Landing! (Fly trim values preserved in memory.)");
                    break;
                }

                // 检查返回飞行状态 (Ch3 < 1100)
                if(state_counter < 100) { // 前2秒内，允许调整飞行零偏，但不响应切换回飞行状态的指令
                    state_counter++;
                }
                else if (rc_ch3 < 1100) { // 锁定解除，检查返回飞行状态指令
                    state_counter++;
                    if(state_counter >= 102) {  //保持0.04s
                        state = FlightState::FLY;
                        state_counter = 0;
                        FLY_CALIBRATE_return_lock = true; // 设置返回飞行的锁定标志，防止重复切换
                        Serial.println("Returning to FLY state.");
                        Serial.println("=====================================");
                        Serial.println("Exited FLY_CALIBRATE mode!");
                        Serial.print("Final Trim -> R: ");
                        Serial.print(fly_calib_roll_offset, 2);
                        Serial.print("° , P: ");
                        Serial.print(fly_calib_pitch_offset, 2);
                        Serial.println("°");
                        Serial.println("=====================================");
                    }
                }
                else {
                    state_counter = 100; // 锁定解除，等待返回飞行指令，重置计数器为100，避免误触发
                }

                // ====================== 核心：摇杆调整零偏 ======================
                // ========== 新增：只有摇杆明显偏离中点才响应 (避免遥控器中点误差) ==========
                // 激活条件：Roll > 1550 或 < 1450；Pitch > 1550 或 < 1450
                bool roll_active = (rc_ch0 > 1550) || (rc_ch0 < 1450);
                bool pitch_active = (rc_ch1 > 1550) || (rc_ch1 < 1450);

                // 计算摇杆偏离中立点的数值 (1500为中立点)
                int roll_offset = rc_ch0 - 1500;
                int pitch_offset = -(rc_ch1 - 1500);

                // 1. Roll轴零偏调整
                if(roll_active) { // 只有激活时才调整
                    // 摇杆量归一化到 -1 ~ 1，控制调整速度
                    float trim_factor = (float)roll_offset / 500.0f;
                    // 累加微调值
                    fly_calib_roll_offset += trim_factor * TRIM_STEP;
                    // 严格限制在 ±3° 范围内
                    fly_calib_roll_offset = constrain(fly_calib_roll_offset, -MAX_TRIM_ANGLE, MAX_TRIM_ANGLE);
                }

                // 2. Pitch轴零偏调整
                if(pitch_active) { // 只有激活时才调整
                    float trim_factor = (float)pitch_offset / 500.0f;
                    fly_calib_pitch_offset += trim_factor * TRIM_STEP;
                    fly_calib_pitch_offset = constrain(fly_calib_pitch_offset, -MAX_TRIM_ANGLE, MAX_TRIM_ANGLE);
                }

                // ====================== 固定目标角度为0 ======================
                // 进入校准模式后，摇杆不再控制飞机姿态，仅调整零偏，飞机保持水平悬停
                targetRoll = 0.0f;
                targetPitch = 0.0f;
                targetYaw = 0.0f;

                // ====================== 串口调试打印 (500ms一次，避免刷屏) ======================
                static int trim_print_counter = 0;
                trim_print_counter++;
                if(trim_print_counter >= 25) {
                    Serial.print("Fly Trim | R: ");
                    Serial.print(fly_calib_roll_offset, 2);
                    Serial.print("° | P: ");
                    Serial.print(fly_calib_pitch_offset, 2);
                    Serial.print("° | Max: ±");
                    Serial.print(MAX_TRIM_ANGLE);
                    Serial.print("° | Active: R=");
                    Serial.print(roll_active ? "YES" : "NO"); // 打印是否激活
                    Serial.print("/P=");
                    Serial.print(pitch_active ? "YES" : "NO");
                    Serial.println("°");
                    trim_print_counter = 0;
                }
                break;
            }

            // ---------------------------------------------------------
            // 状态 6: 定高定点状态
            // 逻辑: 前2秒锁定，不响应切换。2秒后，再次Ch3>1900保持0.04s -> 返回飞行
            // ---------------------------------------------------------
            case FlightState::HOLD_POSITION: {
                // 最高优先级：检查是否直接回保护 (任何时候都有效)

                if (rc_ch2 < 1100) {
                    state = FlightState::PROTECT;
                    state_counter = 0;
                    Serial.println("Emergency Landing... Entering PROTECT.");
                    break;
                }

                if(state_counter == 0) {
                    // 1. 根据指定数据源，锁存当前瞬时位置为目标位置
                    MTF_PosX = ekf_X.pos; // 直接使用融合后的X轴位置
                    MTF_PosY = ekf_Y.pos; // 直接使用融合后的Y轴位置
                    MTF_PosX_fuse_acc = MTF_PosX; // 融合加速度的X轴位置初始值
                    MTF_PosY_fuse_acc = MTF_PosY; // 融合加速度的Y轴位置初始值
                    switch(CURRENT_POS_SOURCE) {
                        case PositionSource::UWB_IMU:
                            target_X_Position = ekf_X.pos;
                            target_Y_Position = ekf_Y.pos;
                            break;
                        case PositionSource::OPTICAL_FLOW:
                            target_X_Position = MTF_PosX;
                            target_Y_Position = MTF_PosY;
                            break;
                        case PositionSource::OPTICAL_FLOW_IMU:
                            target_X_Position = MTF_PosX;
                            target_Y_Position = MTF_PosY;
                            break;
                        default:
                            target_X_Position = MTF_PosX; // 默认回退到纯光流数据
                            target_Y_Position = MTF_PosY;
                            break;
                    }                 

                    // 2. 锁存当前瞬时高度为目标高度
                    target_Height = MTF_Height;
                    // 3. 锁存当前油门为定高基础油门
                    hold_base_throttle = rc_ch2;
                    // 4. 清零PID积分项，避免手动飞行的历史积分干扰
                    // pidController.cleanPositionPIDData();
                    // pidController.cleanSpeedPIDData();
                    // pidController.cleanHeightPIDData();
                    ADRC_ParamClear(&adrcRoll);   // 清空ADRC状态估计
                    ADRC_ParamClear(&adrcPitch);  // 避免手动飞行历史扰动干扰定点模式
                    // 5. 串口打印锁存的目标值，方便调试
                    Serial.println("=====================================");
                    Serial.println("HOLD_POSITION: Target Locked!");
                    Serial.print("Target Pos X: "); Serial.print(target_X_Position, 3);
                    Serial.print("m | Y: "); Serial.print(target_Y_Position, 3); Serial.println("m");
                    Serial.print("Target Height: "); Serial.print(target_Height, 3); Serial.println("m");
                    Serial.println("=====================================");                    
                }

                // ========== 定高定点核心控制链路 ==========
                // 1. 位置环计算：位置偏差 → 目标速度
                if (flag_25Hz) {
                    cal_Position_PID();
                    flag_25Hz = false;
                }
                // 2. 速度环计算：速度偏差 → 目标滚转/俯仰角度
                cal_Speed_PID();
                // 3. 高度环计算：高度偏差 → 目标油门补偿
                cal_Height_PID();

                // ========== 覆盖目标角度，替换遥控输入 ==========
                // 速度环输出的输出除以10直接作为目标加速度，实现定点控制
                // 原因：小角度时加速度和速度的关系近似线性
                target_X_Accel = -pidController.getSpeedXCorrect();
                target_Y_Accel = -pidController.getSpeedYCorrect();

                targetRoll = -target_Y_Accel / 10.0f;
                targetPitch = target_X_Accel / 10.0f;

                // 限制目标角度范围，避免失控
                targetRoll = constrain(targetRoll, -3.0f, 3.0f);
                targetPitch = constrain(targetPitch, -3.0f, 3.0f);
                
                // ========== 定高油门处理 ========== 
                // 基础油门 + 高度环输出，限幅在安全范围内
                hold_throttle = hold_base_throttle + (int)pidController.getHeightRateCorrect();
                hold_throttle = constrain(hold_throttle, 1100, 1800);
                is_hold_throttle_initialized = true; // 定高油门已初始化，可以开始使用

                // 判断2秒锁定是否解除
                if (state_counter < 100) { // 前2秒锁定
                    state_counter++;
                } 
                else { // 锁定解除，检查返回飞行状态指令
                    if (rc_ch3 > 1900) {
                        state_counter++;
                        if (state_counter >= 102) { // 2秒后再保持0.04s
                            state = FlightState::FLY;
                            state_counter = 0;
                            HOLD_POSITION_return_lock = true; // 设置返回飞行的锁定标志，防止重复切换
                            is_hold_throttle_initialized = false; // 退出定高定点状态，重置定高油门初始化标志
                            targetRoll = 0.0f; // 返回飞行状态后，目标角度回归正常遥控输入控制
                            targetPitch = 0.0f;
                            Serial.println("Exiting HOLD_POSITION, returning to FLY.");
                        }
                    } else {
                        state_counter = 100; // 保持在锁定解除状态，等待下一次指令
                    }
                }
                break;
            }
        }
        // ADRC外环已由100Hz块处理, 此处不再重复调用
        // ADRC状态清零由200Hz块处理(油门<1200时自动复位)

        //打印调试信息
        debugPrint();
    }
}

// ============================================================
// 6. 功能函数实现
// ============================================================

// (Timer中断部分代码保持不变，为节省篇幅此处省略，原样保留即可)
// ... (Timer5 初始化和 ISR 保持原样) ...

// ============================================================
// Timer5 200Hz中断（5ms触发一次）
// 主频16MHz，分频64，CTC模式
// 16MHz / 64 = 250kHz
// 250kHz / 1250 = 200Hz
// ============================================================
void initTimer5_200Hz() {
  cli(); 
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5  = 0;
  OCR5A = 1249;  // 计数从0开始，所以填1249

  TCCR5B |= (1 << WGM52);
  TCCR5B |= (1 << CS51) | (1 << CS50);
  TIMSK5 |= (1 << OCIE5A);
  sei(); 
}

// ============================================================
// 200Hz中断服务函数
// 功能：freqTick 1-200循环，用%2和%4置位标志位
// ============================================================
ISR(TIMER5_COMPA_vect) {
  freqTick++;
  if (freqTick > 200) {
    freqTick = 1;
  }

  // 200Hz：每1次中断置位一次（%1==0）
  if(freqTick % 1 == 0) {
    flag_200Hz = true;
  }


  // 100Hz：每2次中断置位一次（%2==0）
  if(freqTick % 2 == 0) {
    flag_100Hz = true;
  }

  // 50Hz：每4次中断置位一次（%4==0）
  if(freqTick % 4 == 0) {
    flag_50Hz = true;
  }

  // 25Hz：每8次中断置位一次（%8==0）
  if (freqTick % 8 == 0) {
    flag_25Hz = true;
  }
}

// ============================================================
// uwb小端解析函数 (int24 -> int32)
// ============================================================
int32_t readInt24LE(uint8_t* data) {
  int32_t value = data[0] | (data[1] << 8) | (data[2] << 16);
  if (value & 0x800000) value |= 0xFF000000; // 符号扩展
  return value;
}

// ============================================================
// 数据处理函数 (毫米 -> 米)
// ============================================================
void processFrameData() {
    real_pos[0] = current_frame.pos[0] / 1000.0f;
    real_pos[1] = current_frame.pos[1] / 1000.0f;
    real_pos[1] = 3.2f - real_pos[1]; // 根据实际安装调整坐标系(uwb y轴反向)
    real_pos[2] = current_frame.pos[2] / 1000.0f;
    Is_UWB_work = 1;

    LPF_real_pos_X = 0.85f * LPF_real_pos_X + 0.15f * real_pos[0];
    LPF_real_pos_Y = 0.85f * LPF_real_pos_Y + 0.15f * real_pos[1];
}

void initHardware() {

    Serial2.begin(115200); //用于光流模块通信

    Serial3.begin(115200); //用于UWB模块通信
    
    pinMode(PIN_ROLL, INPUT_PULLUP);
    pinMode(PIN_PITCH, INPUT_PULLUP);
    pinMode(PIN_THROTTLE, INPUT_PULLUP);
    pinMode(PIN_YAW, INPUT_PULLUP);

    // 【修复2】为每个引脚单独绑定中断函数，避免使用 arduinoInterruptedPin
    enableInterrupt(PIN_ROLL, pwmReceiveRoll, CHANGE);
    enableInterrupt(PIN_PITCH, pwmReceivePitch, CHANGE);
    enableInterrupt(PIN_THROTTLE, pwmReceiveThrottle, CHANGE);
    enableInterrupt(PIN_YAW, pwmReceiveYaw, CHANGE);

    pinMode(PIN_MOTOR_LU_1, OUTPUT);
    pinMode(PIN_MOTOR_LU_2, OUTPUT);
    pinMode(PIN_MOTOR_RU_1, OUTPUT);
    pinMode(PIN_MOTOR_RU_2, OUTPUT);
    pinMode(PIN_MOTOR_RD_1, OUTPUT);
    pinMode(PIN_MOTOR_RD_2, OUTPUT);
    pinMode(PIN_MOTOR_LD_1, OUTPUT);
    pinMode(PIN_MOTOR_LD_2, OUTPUT);

    Wire.begin();
    TWBR = 12;
}

void initESC() {
    Serial.println("Arming ESCs...");
    Serial.println("ESCs Armed.");
	Serial.println("电调校准完成(接下来111232为正确初始化代码,111为错误初始化代码)");
}

void read_IMU_Rate() {
    float Rate_roll, Rate_pitch, Rate_yaw;
    GetDataMPU6050Rate(Rate_roll, Rate_pitch, Rate_yaw);
    rollRate_200hz = 0.8f * rollRate_200hz + 0.2f * Rate_roll; // 200Hz角速度数据低通滤波
    pitchRate_200hz = 0.8f * pitchRate_200hz + 0.2f * Rate_pitch;
    yawRate_200hz = 0.8f * yawRate_200hz + 0.2f * Rate_yaw;

}

void read_IMU_Angle() {
    // 注意：这里使用了全局变量 calib_roll_offset 和 calib_pitch_offset
    GetDataMPU6050(roll, pitch, yaw, gx, gy, gz, Raw_rollRate, Raw_pitchRate, calib_roll_offset+fly_calib_roll_offset, calib_pitch_offset+fly_calib_pitch_offset);
}

void readMTF02() {
    //处理光流数据
    while (Serial2.available() > 0) {
        uint8_t byteRead = Serial2.read();
        // 调用mtf02.cpp的解析函数，把数据喂进去
        micolink_decode(byteRead);
    }
}

void readUWB() {
    // 接收缓冲区
    static uint8_t buffer[12]; 
    static int buffer_index = 0;

    // ============================================================
    // 不断读取串口数据并解析
    // ============================================================
    while (Serial3.available() > 0) {
        uint8_t byte = Serial3.read();
        Serial.println(byte, HEX);
        // ---------------- 状态机：匹配三字节帧头 ----------------
        if (buffer_index == 0) {
        if (byte == 0x01) buffer[buffer_index++] = byte;
        }
        else if (buffer_index == 1) {
        if (byte == 0x00) buffer[buffer_index++] = byte;
        else buffer_index = 0; // 匹配失败，重置
        }
        else if (buffer_index == 2) {
        if (byte == 0x02) buffer[buffer_index++] = byte;
        else buffer_index = 0; // 匹配失败，重置
        }
        // ---------------- 读取后续9字节数据 ----------------
        else if (buffer_index > 2 && buffer_index < 12) {
        buffer[buffer_index++] = byte;

        // ---------------- 收满12字节，解析一帧 ----------------
        if (buffer_index >= 12) {
            // 保存帧头
            for (int i = 0; i < 3; i++) current_frame.frame_header[i] = buffer[i];
            
            // 解析 X, Y, Z
            for (int i = 0; i < 3; i++) {
            current_frame.pos[i] = readInt24LE(&buffer[3 + i*3]);
            }
            
            // 转换单位并更新标志
            processFrameData();
            
            // 重置缓冲区，准备下一帧
            buffer_index = 0;
        }
        }
    }
}

void Cal_MTF02_Position(int MTF_dx_raw, int MTF_dy_raw, float MTF_height) {
	//以下参考 《飞 控 端 调 试 光 流 方 法 说 明.pdf》文档
    // ============================================================
    // 1. 时间计算：获取函数调用间隔 dt (单位：秒)
    // ============================================================
    static float last_time = 0;           // 静态变量：记录上一次调用的时间戳 (单位：微秒 us)
    static bool is_first_call = true;     // 静态变量：首次调用标志位，用于初始化时间
    static float last_height = 0.0f;          // 静态变量：记录上一次的高度，用于计算高度变化率

    if(is_first_call) {
        last_time = micros();             // 首次调用仅初始化时间戳，不进行计算
        is_first_call = false;
        last_height = MTF_height; // 初始化高度
        return; // 首次调用不计算速度，直接返回
    }
    
    float dt = (micros() - last_time) / 1000000.0f; // 计算时间间隔：(当前时间 - 上次时间) / 1e6，单位从微秒转为秒
    last_time = micros();                 // 更新上次时间戳为当前时间，供下次调用使用

    // ============================================================
    // 2. 光流低通滤波：滤除光流原始数据的高频噪声
    // 滤波系数：0.15 (新数据权重) + 0.85 (历史数据权重)
    // ============================================================
    static float LPF_MTF_dx = 0.0f;      // 静态变量：光流X轴数据的低通滤波结果
    static float LPF_MTF_dy = 0.0f;      // 静态变量：光流Y轴数据的低通滤波结果
    
    LPF_MTF_dx = 0.15f * (float)MTF_dx_raw + 0.85f * LPF_MTF_dx; // 一阶低通滤波：X轴光流数据
    LPF_MTF_dy = 0.15f * (float)MTF_dy_raw + 0.85f * LPF_MTF_dy; // 一阶低通滤波：Y轴光流数据

    float MTF_fixed_dx = LPF_MTF_dx;     // 局部变量：暂存滤波后的X轴光流数据，用于后续计算
    float MTF_fixed_dy = LPF_MTF_dy;     // 局部变量：暂存滤波后的Y轴光流数据，用于后续计算

    // ============================================================
    // 3. 陀螺仪延时对齐：补偿光流传感器的硬件延时
    // 使用循环队列类，将陀螺仪数据回溯到光流数据的时间点
    // ============================================================
    static float LPF_Delay_pitchRate = 0.0f; // 静态变量：延时后俯仰角速度的低通结果
    static float LPF_Delay_rollRate = 0.0f;  // 静态变量：延时后横滚角速度的低通结果

    pitchDelay.update(Raw_pitchRate);    // 将当前俯仰角速度存入循环延时队列
    rollDelay.update(Raw_rollRate);      // 将当前横滚角速度存入循环延时队列
    
    float Delay_Raw_pitchRate = pitchDelay.getDelayed(2.5f); // 从队列中取出 2.5 个周期前的俯仰角速度
    float Delay_Raw_rollRate = rollDelay.getDelayed(2.5f);   // 从队列中取出 2.5 个周期前的横滚角速度

    // 【关键】旧代码参数：延时后增益补偿 (1.7 / 1.5)
    // 作用：校准陀螺仪和光流之间的增益匹配
    Delay_Raw_pitchRate *= 1.7f;         // 俯仰角速度增益补偿
    Delay_Raw_rollRate *= 1.5f;          // 横滚角速度增益补偿

    // 陀螺仪低通滤波 (旧代码参数：0.15/0.85)
    // 对延时并增益补偿后的陀螺仪数据再做一次低通滤波，进一步平滑噪声
    LPF_Delay_pitchRate = 0.15f * Delay_Raw_pitchRate + 0.85f * LPF_Delay_pitchRate; // 延时后俯仰角速度低通滤波
    LPF_Delay_rollRate = 0.15f * Delay_Raw_rollRate + 0.85f * LPF_Delay_rollRate; // 延时后横滚角速度低通滤波

    // ============================================================
    // 4. 光流旋转补偿：抵消飞机自身旋转带来的光流干扰
    // 目的：使得飞机在只有旋转没有平移时，光流最终输出接近零
    // 补偿系数：1.0 (直接相减/相加)
    // ============================================================
    MTF_fixed_dx = MTF_fixed_dx - LPF_Delay_pitchRate; // 从光流X轴中减去俯仰旋转分量
    MTF_fixed_dy = MTF_fixed_dy + LPF_Delay_rollRate; // 从光流Y轴中加上横滚旋转分量

    // ============================================================
    // 5. 单位转换 (旧代码参数：直接除以 100.0f)
    // 作用：将光流原始数据转换为 m/s@1m (即 rad/s)
    // 物理意义：rad/s 表示在 1 米高度下对应的线速度 (m/s)
    // ============================================================
    MTF_fixed_dx /= 100.0f;  // 转换为m/s@1m(即rad/s)
    MTF_fixed_dy /= 100.0f;  // 转换为m/s@1m(即rad/s)

    // ============================================================
    // 6. 结合高度 (旧代码参数：除以 1000.0f，mm->m)
    // 作用：根据当前飞行高度，将 rad/s 转换为实际的线速度 (m/s)
    // ============================================================
    static float last_valid_height = 280.0f; // 静态变量：记录上一次的有效高度，初始默认28cm
    if(MTF_height > 50.0f) {                 // 高度有效性判断：只有高度 > 5cm 才认为数据有效
        last_valid_height = MTF_height;       // 更新有效高度
    }
    float used_height = last_valid_height;    // 局部变量：使用的高度值（无效时用上一次的有效值）

    MTF_fixed_dx *= (used_height / 1000.0f);  // 结合高度计算X轴线速度：rad/s * m = m/s
    MTF_fixed_dy *= (used_height / 1000.0f);  // 结合高度计算Y轴线速度：rad/s * m = m/s

    // ============================================================
    // 7. 积分得到位移 (单位：m)
    // 积分公式：位移 = 速度 × 时间
    // ============================================================
    MTF_PosX += MTF_fixed_dx * dt; // X轴位移累加：当前位移 += 当前速度 × 时间间隔
    MTF_PosY += MTF_fixed_dy * dt; // Y轴位移累加：当前位移 += 当前速度 × 时间间隔

    // ============================================================
    // 8. 最终速度低通滤波 (旧代码参数：0.25/0.75)
    // 作用：对最终计算出的线速度做低通滤波，供位置PID控制器使用
    // 滤波系数：0.25 (新速度权重) + 0.75 (历史速度权重)
    // ============================================================
    MTF_measured_X_speed = 0.25f * MTF_fixed_dx + 0.75f * MTF_measured_X_speed; // X轴最终速度低通滤波
    MTF_measured_Y_speed = 0.25f * MTF_fixed_dy + 0.75f * MTF_measured_Y_speed; // Y轴最终速度低通滤波

    // ============================================================
    // 新增功能：加速度计融合光流滤波
    // ============================================================
    float H_accel_X;
    float H_accel_Y;
    get_Horizontal_heading_Acc(H_accel_X, H_accel_Y); // 获取水平航向坐标系下的加速度

    static float X_speed_fuse_acc_without_filter = 0.0f; // 静态变量：未滤波的速度融合结果
    static float Y_speed_fuse_acc_without_filter = 0.0f;

    X_speed_fuse_acc_without_filter = 0.95f * (X_speed_fuse_acc_without_filter + H_accel_X * dt) + 0.05f * MTF_fixed_dx;
    Y_speed_fuse_acc_without_filter = 0.95f * (Y_speed_fuse_acc_without_filter + H_accel_Y * dt) + 0.05f * MTF_fixed_dy;

    MTF_measured_X_speed_fuse_acc = 0.25f * X_speed_fuse_acc_without_filter + 0.75f * MTF_measured_X_speed_fuse_acc;
    MTF_measured_Y_speed_fuse_acc = 0.25f * Y_speed_fuse_acc_without_filter + 0.75f * MTF_measured_Y_speed_fuse_acc;

    MTF_PosX_fuse_acc = 0.9f * (MTF_PosX_fuse_acc + X_speed_fuse_acc_without_filter * dt) + 0.1f * MTF_PosX; // 位置融合：加速度积分位置与光流位置的融合
    MTF_PosY_fuse_acc = 0.9f * (MTF_PosY_fuse_acc + Y_speed_fuse_acc_without_filter * dt) + 0.1f * MTF_PosY;

    // ============================================================
    // 9. 计算高度维度的位移和速度 
    // ============================================================
    MTF_Height = MTF_height / 1000.0f; // 高度转换为米
    float height_speed = (MTF_Height - last_height) / dt; // 高度速度计算：当前高度 - 上次高度 / 时间间隔
    MTF_measured_Height_speed = 0.3f * height_speed + 0.7f * MTF_measured_Height_speed; // 高度速度低通滤波

    last_height = MTF_Height; // 更新上次高度
}

void Cal_UWB_Position()
{


	get_WorldAcc(World_AccX, World_AccY);//m/s^2
    // EKF融合位置和速度
    int Is_UWB_X_Used;
    int Is_UWB_Y_Used;
	Is_UWB_X_Used = ekfX.update(World_AccX, real_pos[0], Is_UWB_work);
    Is_UWB_Y_Used = ekfY.update(World_AccY, real_pos[1], Is_UWB_work);
    Is_UWB_work = 0; // 使用完毕后重置标志，等待下一帧数据更新
    if(Is_UWB_X_Used && Is_UWB_Y_Used) 
    {
        Is_UWB_Used = 1; // EKF成功使用UWB数据进行更新
    }
    else
    {
        Is_UWB_Used = 0; // EKF未使用UWB数据
    }
  	ekf_X.pos = ekfX.getPos();
  	ekf_X.speed = ekfX.getSpeed();
  	ekf_Y.pos = ekfY.getPos();
  	ekf_Y.speed = ekfY.getSpeed();

    accel_bias_X = ekfX.getAccelBias();
    accel_bias_Y = ekfY.getAccelBias();
}

void cal_Position_PID()
{
    float currentPosX, currentPosY;
    // 读取配置的数据源
    switch(CURRENT_POS_SOURCE) {
        case PositionSource::UWB_IMU:
            currentPosX = ekf_X.pos;
            currentPosY = ekf_Y.pos;
            break;
        case PositionSource::OPTICAL_FLOW:
            currentPosX = MTF_PosX_fuse_acc;
            currentPosY = MTF_PosY_fuse_acc;
            // currentPosX = ekf_X.pos;
            // currentPosY = ekf_Y.pos;
            break;
        case PositionSource::OPTICAL_FLOW_IMU:
            currentPosX = MTF_PosX_fuse_acc;
            currentPosY = MTF_PosY_fuse_acc;
            break;
        default:
            currentPosX = MTF_PosX; // 默认回退到纯光流数据
            currentPosY = MTF_PosY;
            break;
    }
    pidController.calCurrentPosXPID(currentPosX, target_X_Position);
    pidController.calCurrentPosYPID(currentPosY, target_Y_Position);
    // Serial.print(target_Y_Position);Serial.print(",");
    // Serial.print(target_Y_Position+0.1);Serial.print(",");
    // Serial.print(target_Y_Position-0.1);Serial.print(",");
    // Serial.print(MTF_PosY);Serial.print(",");
    // Serial.print(MTF_PosX);Serial.println();
}

void cal_Speed_PID()
{
    float currentSpeedX, currentSpeedY;
    // 固定读取配置的数据源
    switch(CURRENT_POS_SOURCE) {
        case PositionSource::UWB_IMU:
            currentSpeedX = ekf_X.speed;
            currentSpeedY = ekf_Y.speed;
            break;
        case PositionSource::OPTICAL_FLOW:
            currentSpeedX = MTF_measured_X_speed;
            currentSpeedY = MTF_measured_Y_speed;
            break;
        case PositionSource::OPTICAL_FLOW_IMU:
            currentSpeedX = MTF_measured_X_speed_fuse_acc;
            currentSpeedY = MTF_measured_Y_speed_fuse_acc;
            break;
        default:
            currentSpeedX = MTF_measured_X_speed; // 默认回退到纯光流数据
            currentSpeedY = MTF_measured_Y_speed;
            break;
    }
    target_X_Speed = pidController.getPosXCorrect();
    target_Y_Speed = pidController.getPosYCorrect();
    pidController.calCurrentSpeedXPID(currentSpeedX, target_X_Speed);
    pidController.calCurrentSpeedYPID(currentSpeedY, target_Y_Speed);
    // Serial.print(target_Y_Speed);Serial.print(",");
    // Serial.print(target_Y_Speed+0.1);Serial.print(",");
    // Serial.print(target_Y_Speed-0.1);Serial.print(",");
    // Serial.print(MTF_measured_Y_speed);Serial.println();
}

void cal_Height_PID()
{
    pidController.calCurrentHeightPID(MTF_Height, target_Height);
    target_Height_Speed = pidController.getHeightCorrect();
    pidController.calCurrentHeightRatePID(MTF_measured_Height_speed, target_Height_Speed);
}

void read_remote_control() {
    // 原子读取遥控器数据 (AVR上16-bit volatile int非原子, 关中断防竞态)
    int ch0, ch1;
    noInterrupts();
    ch0 = receiver_input[0];
    ch1 = receiver_input[1];
    interrupts();

    targetRoll = (ch0 - 1500)/500.0f * 10.0f; // 【注意】加上 .0f 避免整数除法
    targetPitch = -(ch1 - 1500)/500.0f * 10.0f;
    targetYaw = 0.0f;

    // 使用 Arduino 内置的 constrain()
    targetRoll = constrain(targetRoll, -10.0f, 10.0f);
    targetPitch = constrain(targetPitch, -10.0f, 10.0f);
}

void calculateADRC() {
    // ADRC外环: AttOut = KpOut * (targetAngle - measuredAngle)
    // 三轴均使用角度P控制，将角度误差(deg)转换为角速度目标(deg/s)
    ADRC_OuterLoop(&adrcRoll,  targetRoll,  roll);
    ADRC_OuterLoop(&adrcPitch, targetPitch, pitch);
    ADRC_OuterLoop(&adrcYaw,   targetYaw,   yaw);
}

void escCtrl(int throttle, float rollCORR, float pitchCORR, float yawCORR) {
    int temp;

    temp = (int)(-rollCORR + pitchCORR - yawCORR);
    ESC_PWM[0] = throttle - temp;
	ESC_PWM[0] = constrain(ESC_PWM[0], ESC_MIN_PWM, ESC_MAX_PWM);


    temp = (int)(rollCORR + pitchCORR + yawCORR);
    ESC_PWM[1] = throttle - temp;
	ESC_PWM[1] = constrain(ESC_PWM[1], ESC_MIN_PWM, ESC_MAX_PWM);
    temp = (int)(rollCORR - pitchCORR - yawCORR);
    ESC_PWM[2] = throttle - temp;
	ESC_PWM[2] = constrain(ESC_PWM[2], ESC_MIN_PWM, ESC_MAX_PWM);

    temp = (int)(-rollCORR - pitchCORR + yawCORR);
    ESC_PWM[3] = throttle - temp;
    ESC_PWM[3] = constrain(ESC_PWM[3], ESC_MIN_PWM, ESC_MAX_PWM);
    unsigned long start_time = micros();
    unsigned long t0 = ESC_PWM[0] + start_time;
    unsigned long t1 = ESC_PWM[1] + start_time;
    unsigned long t2 = ESC_PWM[2] + start_time;
    unsigned long t3 = ESC_PWM[3] + start_time;

    setAllMotorsHigh();

    bool done0 = false, done1 = false, done2 = false, done3 = false;
    while (!done0 || !done1 || !done2 || !done3) {
        unsigned long now = micros();
        if (!done0 && now >= t0) {
            digitalWrite(PIN_MOTOR_LD_1, LOW);
            digitalWrite(PIN_MOTOR_LD_2, LOW);
            done0 = true;
        }
        if (!done1 && now >= t1) {
            digitalWrite(PIN_MOTOR_RD_1, LOW);
            digitalWrite(PIN_MOTOR_RD_2, LOW);
            done1 = true;
        }
        if (!done2 && now >= t2) {
            digitalWrite(PIN_MOTOR_RU_1, LOW);
            digitalWrite(PIN_MOTOR_RU_2, LOW);
            done2 = true;
        }
        if (!done3 && now >= t3) {
            digitalWrite(PIN_MOTOR_LU_1, LOW);
            digitalWrite(PIN_MOTOR_LU_2, LOW);
            done3 = true;
        }
    }
}

void setAllMotorsHigh() {
    digitalWrite(PIN_MOTOR_LU_1, HIGH);
    digitalWrite(PIN_MOTOR_LU_2, HIGH);
    digitalWrite(PIN_MOTOR_RU_1, HIGH);
    digitalWrite(PIN_MOTOR_RU_2, HIGH);
    digitalWrite(PIN_MOTOR_RD_1, HIGH);
    digitalWrite(PIN_MOTOR_RD_2, HIGH);
    digitalWrite(PIN_MOTOR_LD_1, HIGH);
    digitalWrite(PIN_MOTOR_LD_2, HIGH);
}

void setAllMotorsLow() {
    digitalWrite(PIN_MOTOR_LU_1, LOW);
    digitalWrite(PIN_MOTOR_LU_2, LOW);
    digitalWrite(PIN_MOTOR_RU_1, LOW);
    digitalWrite(PIN_MOTOR_RU_2, LOW);
    digitalWrite(PIN_MOTOR_RD_1, LOW);
    digitalWrite(PIN_MOTOR_RD_2, LOW);
    digitalWrite(PIN_MOTOR_LD_1, LOW);
    digitalWrite(PIN_MOTOR_LD_2, LOW);
}

// ============================================================
// 为每个通道单独写中断函数，彻底避开 arduinoInterruptedPin 问题
// ============================================================
void pwmReceiveRoll() {
    int pinState = digitalRead(PIN_ROLL);
    unsigned long now = micros();
    if (pinState == HIGH) {
        timer_roll = now;
    } else {
        receiver_input[0] = now - timer_roll;
    }
}

void pwmReceivePitch() {
    int pinState = digitalRead(PIN_PITCH);
    unsigned long now = micros();
    if (pinState == HIGH) {
        timer_pitch = now;
    } else {
        receiver_input[1] = now - timer_pitch;
    }
}

void pwmReceiveThrottle() {
    int pinState = digitalRead(PIN_THROTTLE);
    unsigned long now = micros();
    if (pinState == HIGH) {
        timer_throttle = now;
    } else {
        receiver_input[2] = now - timer_throttle;
    }
}

void pwmReceiveYaw() {
    int pinState = digitalRead(PIN_YAW);
    unsigned long now = micros();
    if (pinState == HIGH) {
        timer_yaw = now;
    } else {
        receiver_input[3] = now - timer_yaw;
    }
}

void debugPrint() {
    // ========== ADRC调参专用输出 (CSV格式, 50Hz) ==========
    // 列说明:
    //   0-2:   roll, pitch, yaw         — 姿态角度 (°)
    //   3-5:   rU, pU, yU               — ADRC控制输出u (电机修正量)
    //   6-8:   rW, pW, yW               — ESO总扰动估计w (ADRC核心指标)
    //   9-11:  rRate, pRate, yRate      — 实测角速度 (°/s)
    //  12-14: rAttOut, pAttOut, yAttOut — 目标角速度 (ADRC外环AttOut)
    //  15-18: 四路PWM值

    // 姿态角度
    Serial.print(roll);    Serial.print(",");
    Serial.print(pitch);   Serial.print(",");
    Serial.print(yaw);     Serial.print(",");

    // ADRC控制输出 (核心: 观察是否饱和/振荡)
    Serial.print(adrcRoll.u);  Serial.print(",");
    Serial.print(adrcPitch.u); Serial.print(",");
    Serial.print(adrcYaw.u);   Serial.print(",");

    // ADRC扰动估计 (核心: w代表总扰动, 平稳飞行时应稳定在某个值)
    Serial.print(adrcRoll.w);  Serial.print(",");
    Serial.print(adrcPitch.w); Serial.print(",");
    Serial.print(adrcYaw.w);   Serial.print(",");

    // 实测角速度 vs 目标角速度 (观察跟踪性能)
    Serial.print(rollRate_200hz);   Serial.print(",");
    Serial.print(pitchRate_200hz);  Serial.print(",");
    Serial.print(yawRate_200hz);    Serial.print(",");

    Serial.print(adrcRoll.AttOut);   Serial.print(",");
    Serial.print(adrcPitch.AttOut);  Serial.print(",");
    Serial.print(adrcYaw.AttOut);    Serial.print(",");

    // 四路PWM输出 (LU, RU, RD, LD)
    Serial.print(ESC_PWM[0]); Serial.print(",");
    Serial.print(ESC_PWM[1]); Serial.print(",");
    Serial.print(ESC_PWM[2]); Serial.print(",");
    Serial.print(ESC_PWM[3]);

    Serial.println();
}