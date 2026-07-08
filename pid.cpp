#include <math.h>
#include <Arduino.h>
#include "PID.h"

// 高度外环PID参数
#define HEIGHT_KP      2.0//1.0    // 高度比例系数
#define HEIGHT_KI      0.1    // 高度积分系数
#define HEIGHT_KD      0.0    // 高度微分系数

// 高度速度环PID参数
#define HEIGHT_RATE_KP   100.0    // 高度速度比例系数
#define HEIGHT_RATE_KI   0.0	// 高度速度积分系数
#define HEIGHT_RATE_KD   0.0    // 高度速度微分系数

// X轴位置环PID参数
#define POS_X_KP       1.0//10.0    // X轴位置比例系数
#define POS_X_KI       0.0    // X轴位置积分系数
#define POS_X_KD       0.0//0.2    // X轴位置微分系数

// X轴速度环PID参数
#define SPEED_X_KP    160.0//4.0    // X轴速度比例系数
#define SPEED_X_KI    4.0    // X轴速度积分系数
#define SPEED_X_KD    0.0//0.3    // X轴速度微分系数

// Y轴位置单环PID参数
#define POS_Y_KP       1.0//7.0    // Y轴位置比例系数
#define POS_Y_KI       0.0    // Y轴位置积分系数
#define POS_Y_KD       0.0    // Y轴位置微分系数

// Y轴速度环PID参数
#define SPEED_Y_KP    160.0//40.0    // Y轴速度比例系数
#define SPEED_Y_KI    0.0    // Y轴速度积分系数
#define SPEED_Y_KD    0.0    // Y轴速度微分系数

// PID输出限制参数
#define ERRORCORR_MAX	500    // PID总输出最大值
#define ERRORCORR_MAX_P	400    // 比例项最大值(未使用)
#define ERRORCORR_MAX_I	35.0   // 积分项最大值
#define ERRORCORR_MAX_D	100.0  // 微分项最大值


/**
 * PID控制器构造函数
 * 初始化高度/位置/速度环PID参数 (角度环已由ADRC替代)
 */
PIDController::PIDController() {
	// 初始化高度PID参数
	heightPID.KP = HEIGHT_KP;
	heightPID.KI = HEIGHT_KI;
	heightPID.KD = HEIGHT_KD;

	heightRatePID.KP = HEIGHT_RATE_KP;
	heightRatePID.KI = HEIGHT_RATE_KI;
	heightRatePID.KD = HEIGHT_RATE_KD;

	//初始化X轴位置PID参数
	posXPID.KP = POS_X_KP;
	posXPID.KI = POS_X_KI;
	posXPID.KD = POS_X_KD;

	//初始化X轴速度PID参数
	speedXPID.KP = SPEED_X_KP;
	speedXPID.KI = SPEED_X_KI;
	speedXPID.KD = SPEED_X_KD;

	//初始化Y轴位置PID参数
	posYPID.KP = POS_Y_KP;
	posYPID.KI = POS_Y_KI;
	posYPID.KD = POS_Y_KD;

	//初始化Y轴速度PID参数
	speedYPID.KP = SPEED_Y_KP;
	speedYPID.KI = SPEED_Y_KI;
	speedYPID.KD = SPEED_Y_KD;

	// 初始化时间戳，用于计算时间间隔
	heightPID.lastTimeStamp = micros();
	heightRatePID.lastTimeStamp = micros();
	posXPID.lastTimeStamp = micros();
	speedXPID.lastTimeStamp = micros();
	posYPID.lastTimeStamp = micros();
	speedYPID.lastTimeStamp = micros();
}

/**
 * PID控制计算核心函数
 * @param PID     指向PID结构体的指针
 * @param target  目标值
 * @param measure 测量值
 * @param dt      时间间隔(微秒)
 */
void PIDController::calCurrentPID(PID *PID, float target, float measure, float dt) {
	dt /= 1000000;  // 将微秒转换为秒
	PID->error = target - measure; // 计算当前误差

	// 积分分离: 只在误差小于阈值时积分以消除稳态误差
	// 误差大于阈值时停止积分防止超调 (标准积分分离策略)
	if (abs(PID->error) < 5.0f) {
		PID->integ += PID->error * dt; // 计算误差积分
	}

	// 计算误差微分（变化率）
	PID->deriv = (dt != 0) ? ((PID->error - PID->lastError) / dt) : 0;

	// 积分项限幅：防止积分项过大导致系统不稳定
	if (abs(PID->KI * PID->integ) > ERRORCORR_MAX_I) {
		PID->integ = (PID->integ > 0) ? ERRORCORR_MAX_I / PID->KI : -ERRORCORR_MAX_I / PID->KI;
	}

	// 微分项限幅：防止微分项过大导致系统震荡
	if (abs(PID->KD * PID->deriv) > ERRORCORR_MAX_D) {
		PID->deriv = (PID->deriv > 0) ? ERRORCORR_MAX_D / PID->KD : -ERRORCORR_MAX_D / PID->KD;
	}

	// 计算PID输出：比例项 + 积分项 + 微分项
	PID->output = (PID->KP * PID->error) + (PID->KI * PID->integ) + (PID->KD * PID->deriv);

	// 总输出限幅
	PID->output = minMax(PID->output, -ERRORCORR_MAX, ERRORCORR_MAX);

	// 保存当前误差作为下次计算的上一次误差
	PID->lastError = PID->error;
}

/**
 * 数值限幅函数
 */
float PIDController::minMax(float value, float min, float max) {
	if (value < min) {return min;}
	if (value > max) {return max;}
	return value;
}

// ========== 位置/速度 PID ==========

void PIDController::calCurrentPosXPID(float measure_Pos_X, float target_Pos_X) {
	float t = 0.0, dt = 0.0;
	t = micros();
	dt = (posXPID.lastTimeStamp > 0) ? (t - posXPID.lastTimeStamp) : 0;
	posXPID.lastTimeStamp = t;
	calCurrentPID(&posXPID, target_Pos_X, measure_Pos_X, dt);
}

void PIDController::calCurrentSpeedXPID(float measure_Speed_X, float target_Speed_X) {
	float t = 0.0, dt = 0.0;
	t = micros();
	dt = (speedXPID.lastTimeStamp > 0) ? (t - speedXPID.lastTimeStamp) : 0;
	speedXPID.lastTimeStamp = t;
	calCurrentPID(&speedXPID, target_Speed_X, measure_Speed_X, dt);
}

void PIDController::calCurrentPosYPID(float measure_Pos_Y, float target_Pos_Y) {
	float t = 0.0, dt = 0.0;
	t = micros();
	dt = (posYPID.lastTimeStamp > 0) ? (t - posYPID.lastTimeStamp) : 0;
	posYPID.lastTimeStamp = t;
	calCurrentPID(&posYPID, target_Pos_Y, measure_Pos_Y, dt);
}

void PIDController::calCurrentSpeedYPID(float measure_Speed_Y, float target_Speed_Y) {
	float t = 0.0, dt = 0.0;
	t = micros();
	dt = (speedYPID.lastTimeStamp > 0) ? (t - speedYPID.lastTimeStamp) : 0;
	speedYPID.lastTimeStamp = t;
	calCurrentPID(&speedYPID, target_Speed_Y, measure_Speed_Y, dt);
}

// ========== 高度 PID ==========

void PIDController::calCurrentHeightPID(float measureHeight, float targetHeight){
	float t = 0.0, dt = 0.0;
	t = micros();
	dt = (heightPID.lastTimeStamp > 0) ? (t - heightPID.lastTimeStamp) : 0;
	heightPID.lastTimeStamp = t;
	calCurrentPID(&heightPID, targetHeight, measureHeight, dt);
}

void PIDController::calCurrentHeightRatePID(float measureHeightRate, float targetHeightRate) {
	float t = 0.0, dt = 0.0;
	t = micros();
	dt = (heightRatePID.lastTimeStamp > 0) ? (t - heightRatePID.lastTimeStamp) : 0;
	heightRatePID.lastTimeStamp = t;
	calCurrentPID(&heightRatePID, targetHeightRate, measureHeightRate, dt);
}

// ========== PID 输出获取 ==========

float PIDController::getPosXCorrect() {
	return posXPID.output;
}

float PIDController::getSpeedXCorrect() {
	return speedXPID.output;
}

float PIDController::getPosYCorrect() {
	return posYPID.output;
}

float PIDController::getSpeedYCorrect() {
	return speedYPID.output;
}

float PIDController::getHeightRateCorrect() {
	return heightRatePID.output;
}

float PIDController::getHeightCorrect() {
	return heightPID.output;
}

// ========== PID 数据清空 ==========

void PIDController::cleanData(PID *PID) {
	PID->error = 0.0;
	PID->lastError = 0.0;
	PID->integ = 0.0;
	PID->deriv = 0.0;
	PID->output = 0.0;
	PID->lastTimeStamp = micros();
}

void PIDController::cleanHeightPIDData() {
	cleanData(&heightPID);
	cleanData(&heightRatePID);
}
