#include <math.h>
#include <Arduino.h>
#include "PID.h"

// 角度环PID参数 - 俯仰轴
#define ANGLE_PITCH_KP 	3.5//8.5    // 俯仰角度比例系数
#define ANGLE_PITCH_KI 	0.0      // ADRC内环已有扰动补偿,去掉积分防耦合振荡
#define ANGLE_PITCH_KD 	0.0     // 俯仰角度微分系数

// 角度环PID参数 - 横滚轴
#define ANGLE_ROLL_KP 	3.5//4.5    // 横滚角度比例系数
#define ANGLE_ROLL_KI 	0.0      // 8电机ADRC内环已有扰动补偿,去掉积分防耦合振荡
#define ANGLE_ROLL_KD 	0.0      // 横滚角度微分系数

// 角度环PID参数 - 偏航轴
// 原值Kp=20.5过高，无磁力计时yaw漂移会导致严重电机差速
// 降低到与roll/pitch同量级，仅提供偏航阻尼
#define ANGLE_YAW_KP 	5.0     // 偏航角速度比例系数 (原20.5)
#define ANGLE_YAW_KI 	0      // 偏航角速度积分系数
#define ANGLE_YAW_KD 	0      // 偏航角速度微分系数

// 角速度环PID参数 - 偏航轴
#define RATE_YAW_KP 	5    // 偏航角速度比例系数
#define RATE_YAW_KI 	0.0   // 偏航角速度积分系数
#define RATE_YAW_KD 	2.0//1.25    // 偏航角速度微分系数

// 角速度环PID参数 - 俯仰轴
#define RATE_PITCH_KP 	6     // 俯仰角速度比例系数
#define RATE_PITCH_KI 	0.00    // 俯仰角速度积分系数
#define RATE_PITCH_KD 	0.35   // 俯仰角速度微分系数//0.045

// 角速度环PID参数 - 横滚轴
#define RATE_ROLL_KP 	  4      // 横滚角速度比例系数
#define RATE_ROLL_KI  	0.2//0.2    // 横滚角速度积分系数
#define RATE_ROLL_KD  	0.25    // 横滚角速度微分系数//0.1

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
 * 初始化所有PID控制器的参数和时间戳
 */
PIDController::PIDController() {
	// 初始化角度环PID参数 - 横滚轴
	rollAnglePID.KP = ANGLE_ROLL_KP;
	rollAnglePID.KI = ANGLE_ROLL_KI;
	rollAnglePID.KD = ANGLE_ROLL_KD;
	
	// 初始化角度环PID参数 - 俯仰轴
	pitchAnglePID.KP = ANGLE_PITCH_KP;
	pitchAnglePID.KI = ANGLE_PITCH_KI;
	pitchAnglePID.KD = ANGLE_PITCH_KD;
	
	// 初始化角度环PID参数 - 偏航轴
	yawAnglePID.KP = ANGLE_YAW_KP;
	yawAnglePID.KI = ANGLE_YAW_KI;
	yawAnglePID.KD = ANGLE_YAW_KD;

	// 初始化角速度环PID参数 - 偏航轴
	yawRatePID.KP = RATE_YAW_KP;
	yawRatePID.KI = RATE_YAW_KI;
	yawRatePID.KD = RATE_YAW_KD;
	
	// 初始化角速度环PID参数 - 横滚轴
	rollRatePID.KP = RATE_ROLL_KP;
	rollRatePID.KI = RATE_ROLL_KI;
	rollRatePID.KD = RATE_ROLL_KD;
	
	// 初始化角速度环PID参数 - 俯仰轴
	pitchRatePID.KP = RATE_PITCH_KP;
	pitchRatePID.KI = RATE_PITCH_KI;
	pitchRatePID.KD = RATE_PITCH_KD;

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
	rollAnglePID.lastTimeStamp = micros();
	pitchAnglePID.lastTimeStamp = micros();
	yawAnglePID.lastTimeStamp = micros();
	yawRatePID.lastTimeStamp = micros();
	rollRatePID.lastTimeStamp = micros();
	pitchRatePID.lastTimeStamp = micros();
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
	// 修复: 原逻辑 abs(error)<10 && abs(error)>1 导致小误差永不积分
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
 * @param value 输入值
 * @param min   最小值
 * @param max   最大值
 * @return      限幅后的值
 */
float PIDController::minMax(float value, float min, float max) {
	if (value < min) {return min;}
	if (value > max) {return max;}
	return value;
}

void PIDController::calCurrentPosXPID(float measure_Pos_X, float target_Pos_X) {
	float t = 0.0, dt = 0.0;
	t = micros();  // 获取当前时间戳
	// 计算时间间隔，如果是第一次调用则dt为0
	dt = (posXPID.lastTimeStamp > 0) ? (t - posXPID.lastTimeStamp) : 0;
	posXPID.lastTimeStamp = t;  // 更新时间戳
	calCurrentPID(&posXPID, target_Pos_X, measure_Pos_X, dt);
}

void PIDController::calCurrentSpeedXPID(float measure_Speed_X, float target_Speed_X) {
	float t = 0.0, dt = 0.0;
	t = micros();  // 获取当前时间戳
	// 计算时间间隔，如果是第一次调用则dt为0
	dt = (speedXPID.lastTimeStamp > 0) ? (t - speedXPID.lastTimeStamp) : 0;
	speedXPID.lastTimeStamp = t;  // 更新时间戳
	calCurrentPID(&speedXPID, target_Speed_X, measure_Speed_X, dt);
}

void PIDController::calCurrentPosYPID(float measure_Pos_Y, float target_Pos_Y) {
	float t = 0.0, dt = 0.0;
	t = micros();  // 获取当前时间戳
	// 计算时间间隔，如果是第一次调用则dt为0
	dt = (posYPID.lastTimeStamp > 0) ? (t - posYPID.lastTimeStamp) : 0;
	posYPID.lastTimeStamp = t;  // 更新时间戳
	calCurrentPID(&posYPID, target_Pos_Y, measure_Pos_Y, dt);
}

void PIDController::calCurrentSpeedYPID(float measure_Speed_Y, float target_Speed_Y) {
	float t = 0.0, dt = 0.0;
	t = micros();  // 获取当前时间戳
	// 计算时间间隔，如果是第一次调用则dt为0
	dt = (speedYPID.lastTimeStamp > 0) ? (t - speedYPID.lastTimeStamp) : 0;
	speedYPID.lastTimeStamp = t;  // 更新时间戳
	calCurrentPID(&speedYPID, target_Speed_Y, measure_Speed_Y, dt);
}

void PIDController::calCurrentHeightPID(float measureHeight, float targetHeight){
	float t = 0.0, dt = 0.0;
	t = micros();  // 获取当前时间戳
	// 计算时间间隔，如果是第一次调用则dt为0
	dt = (heightPID.lastTimeStamp > 0) ? (t - heightPID.lastTimeStamp) : 0;
	heightPID.lastTimeStamp = t;  // 更新时间戳
	calCurrentPID(&heightPID, targetHeight, measureHeight, dt);
}

void PIDController::calCurrentHeightRatePID(float measureHeightRate, float targetHeightRate) {
	float t = 0.0, dt = 0.0;
	t = micros();  // 获取当前时间戳
	// 计算时间间隔，如果是第一次调用则dt为0
	dt = (heightRatePID.lastTimeStamp > 0) ? (t - heightRatePID.lastTimeStamp) : 0;
	heightRatePID.lastTimeStamp = t;  // 更新时间戳
	calCurrentPID(&heightRatePID, targetHeightRate, measureHeightRate, dt);
}
	

/**
 * 计算横滚角度环PID
 * @param measureRoll 测量的横滚角度
 * @param targetRoll  目标横滚角度
 */
void PIDController::calCurrentRollAnglePID(float measureRoll, float targetRoll) {
	float t = 0.0, dt = 0.0;
	t = micros();  // 获取当前时间戳
	// 计算时间间隔，如果是第一次调用则dt为0
	dt = (rollAnglePID.lastTimeStamp > 0) ? (t - rollAnglePID.lastTimeStamp): 0;
	/*
	这是一个三元条件运算符的表达式，等价于：
	if (rollAnglePID.lastTimeStamp > 0) {
	    dt = t - rollAnglePID.lastTimeStamp;
	} else {
    	dt = 0;
	}
	*/
	rollAnglePID.lastTimeStamp = t;  // 更新时间戳
	calCurrentPID(&rollAnglePID, targetRoll, measureRoll, dt);
}

/**
 * 计算俯仰角度环PID
 * @param measurePitch 测量的俯仰角度
 * @param targetPitch  目标俯仰角度
 */
void PIDController::calCurrentPitchAnglePID(float measurePitch, float targetPitch) {
	float t = 0.0, dt = 0.0;
	t = micros();  // 获取当前时间戳
	// 计算时间间隔，如果是第一次调用则dt为0
	dt = (pitchAnglePID.lastTimeStamp > 0) ? (t - pitchAnglePID.lastTimeStamp): 0;
	pitchAnglePID.lastTimeStamp = t;  // 更新时间戳

	calCurrentPID(&pitchAnglePID, targetPitch, measurePitch, dt);
}

// 偏航角度环PID计算函数（已注释掉，未实现）
// void PIDController::calCurrentYawAnglePID(float measureYaw, float targetYaw){
// }

/**
 * 计算偏航角度环PID
 * @param measureYaw 测量的偏航角度
 * @param targetYaw  目标偏航角度
 */
void PIDController::calCurrentYawAnglePID(float measureYaw, float targetYaw) {
	float t = 0.0, dt = 0.0;
	t = micros();  // 获取当前时间戳
	// 计算时间间隔，如果是第一次调用则dt为0
	dt = (yawAnglePID.lastTimeStamp > 0) ? (t - yawAnglePID.lastTimeStamp): 0;
	yawAnglePID.lastTimeStamp = t;  // 更新时间戳

	calCurrentPID(&yawAnglePID, targetYaw, measureYaw, dt);
}

/**
 * 计算偏航角速度环PID
 * @param measureYaw 测量的偏航角速度
 * @param targetYaw  目标偏航角速度
 */
void PIDController::calCurrentYawRatePID(float measureYawRate, float targetYawRate) {
	float t = 0.0, dt = 0.0;
	t = micros();  // 获取当前时间戳
	// 计算时间间隔，如果是第一次调用则dt为0
	dt = (yawRatePID.lastTimeStamp > 0) ? (t - yawRatePID.lastTimeStamp): 0;
	yawRatePID.lastTimeStamp = t;  // 更新时间戳

	calCurrentPID(&yawRatePID, targetYawRate, measureYawRate, dt);
}

/**
 * 计算横滚角速度环PID
 * @param measureRollRate 测量的横滚角速度
 * @param targetRollRate  目标横滚角速度
 */
void PIDController::calCurrentRollRatePID(float measureRollRate, float targetRollRate){
	float t = 0.0, dt = 0.0;
	t = micros();  // 获取当前时间戳
	// 计算时间间隔，如果是第一次调用则dt为0
	dt = (rollRatePID.lastTimeStamp > 0) ? (t - rollRatePID.lastTimeStamp) : 0;
	rollRatePID.lastTimeStamp = t;  // 更新时间戳

	calCurrentPID(&rollRatePID, targetRollRate, measureRollRate, dt);
}

/**
 * 计算俯仰角速度环PID
 * @param measurePitchRate 测量的俯仰角速度
 * @param targetPitchRate  目标俯仰角速度
 */
void PIDController::calCurrentPitchRatePID(float measurePitchRate, float targetPitchRate) {
	float t = 0.0, dt = 0.0;
	t = micros();  // 获取当前时间戳
	// 计算时间间隔，如果是第一次调用则dt为0
	dt = (pitchRatePID.lastTimeStamp > 0) ? (t - pitchRatePID.lastTimeStamp) : 0;
	pitchRatePID.lastTimeStamp = t;  // 更新时间戳

	calCurrentPID(&pitchRatePID, targetPitchRate, measurePitchRate, dt);
}

float PIDController::getPosXCorrect() {
	return posXPID.output;  // 返回X轴位置PID输出
}

float PIDController::getSpeedXCorrect() {
	return speedXPID.output;  // 返回X轴速度PID输出
}

float PIDController::getPosYCorrect() {
	return posYPID.output;  // 返回X轴位置PID输出
}

float PIDController::getSpeedYCorrect() {
	return speedYPID.output;  // 返回Y轴速度PID输出
}

float PIDController::getHeightRateCorrect() {
	return heightRatePID.output;  // 返回高度速度PID输出
}

float PIDController::getHeightCorrect() {
	return heightPID.output;  // 返回高度PID输出
}

/* ===================================================================
 * 
 * PID 修正值获取函数
 * 根据PID类型（角度环或角速度环）返回对应的输出值
 * 
 * ===================================================================
 */

/**
 * 获取横滚轴PID修正值
 * @param pidKind PID类型（ANGLE：角度环，RATE：角速度环）
 * @return        PID输出值
 */
float PIDController::getRollCorrect(PIDKind pidKind) {
	switch (pidKind)
	{ 
	case ANGLE:
		return rollAnglePID.output;  // 返回横滚角度环输出
		break;
	case RATE:
		return rollRatePID.output;   // 返回横滚角速度环输出
		break;
	default:
		return 0;
		break;
	}
}

/**
 * 获取俯仰轴PID修正值
 * @param pidKind PID类型（ANGLE：角度环，RATE：角速度环）
 * @return        PID输出值
 */
float PIDController::getPitchCorrect(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return pitchAnglePID.output; // 返回俯仰角度环输出
		break;
	case RATE:
		return pitchRatePID.output;  // 返回俯仰角速度环输出
		break;
	default:
		return 0;
		break;
	}
}

/**
 * 获取偏航轴PID修正值
 * @param pidKind PID类型（ANGLE：角度环，RATE：角速度环）
 * @return        PID输出值
 * @note          偏航轴只有角速度环，角度环返回0
 */
float PIDController::getYawCorrect(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return yawAnglePID.output;                    // 偏航角度环未实现，返回0
		break;
	case RATE:
		return yawRatePID.output;    // 返回偏航角速度环输出
		break;
	default:
		return 0;
		break;
	}
}

/* =============================================================
 * 
 * 横滚轴相关信息获取函数
 * 用于调试和监控PID控制器状态
 * 
 * =============================================================
 */

/**
 * 获取横滚轴误差值
 * @param pidKind PID类型（ANGLE：角度环，RATE：角速度环）
 * @return        当前误差值
 */
float PIDController::getRollError(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return rollAnglePID.error;   // 返回横滚角度环误差
		break;
	case RATE:
		return rollRatePID.error;    // 返回横滚角速度环误差
		break;
	default:
		return 0;
		break;
	}
}

/**
 * 获取横滚轴积分值
 * @param pidKind PID类型（ANGLE：角度环，RATE：角速度环）
 * @return        当前积分值
 */
float PIDController::getRollInteg(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return rollAnglePID.integ;   // 返回横滚角度环积分值
		break;
	case RATE:
		return rollRatePID.integ;    // 返回横滚角速度环积分值
		break;
	default:
		return 0;
		break;
	}
}

/**
 * 获取横滚轴微分值
 * @param pidKind PID类型（ANGLE：角度环，RATE：角速度环）
 * @return        当前微分值
 */
float PIDController::getRollDeriv(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return rollAnglePID.deriv;   // 返回横滚角度环微分值
		break;
	case RATE:
		return rollRatePID.deriv;    // 返回横滚角速度环微分值
		break;
	default:
		return 0;
		break;
	}
}

/* =============================================================
 * 
 * 俯仰轴相关信息获取函数
 * 用于调试和监控PID控制器状态
 * 
 * =============================================================
 */

/**
 * 获取俯仰轴误差值
 * @param pidKind PID类型（ANGLE：角度环，RATE：角速度环）
 * @return        当前误差值
 */
float PIDController::getPitchError(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return pitchAnglePID.error;  // 返回俯仰角度环误差
		break;
	case RATE:
		return pitchRatePID.error;   // 返回俯仰角速度环误差
		break;
	default:
		return 0;
		break;
	} 
}

/**
 * 获取俯仰轴积分值
 * @param pidKind PID类型（ANGLE：角度环，RATE：角速度环）
 * @return        当前积分值
 */
float PIDController::getPitchInteg(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return pitchAnglePID.integ;  // 返回俯仰角度环积分值
		break;
	case RATE:
		return pitchRatePID.integ;   // 返回俯仰角速度环积分值
		break;
	default:
		return 0;
		break;
	} 
}

/**
 * 获取俯仰轴微分值
 * @param pidKind PID类型（ANGLE：角度环，RATE：角速度环）
 * @return        当前微分值
 */
float PIDController::getPitchDeriv(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return pitchAnglePID.deriv;  // 返回俯仰角度环微分值
		break;
	case RATE:
		return pitchRatePID.deriv;   // 返回俯仰角速度环微分值
		break;
	default:
		return 0;
		break;
	} 
}

/* =============================================================
 * 
 * 偏航轴相关信息获取函数
 * 用于调试和监控PID控制器状态
 * 注意：偏航轴只有角速度环，角度环相关函数返回0
 * 
 * =============================================================
 */

/**
 * 获取偏航轴误差值
 * @param pidKind PID类型（ANGLE：角度环，RATE：角速度环）
 * @return        当前误差值
 * @note          偏航角度环未实现，返回0
 */
float PIDController::getYawError(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return 0;                    // 偏航角度环未实现
		break;
	case RATE:
		return yawRatePID.error;     // 返回偏航角速度环误差
		break;
	default:
		return 0;
		break;
	} 
}

/**
 * 获取偏航轴积分值
 * @param pidKind PID类型（ANGLE：角度环，RATE：角速度环）
 * @return        当前积分值
 * @note          偏航角度环未实现，返回0
 */
float PIDController::getYawInteg(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return 0;                    // 偏航角度环未实现
		break;
	case RATE:
		return yawRatePID.integ;     // 返回偏航角速度环积分值
		break;
	default:
		return 0;
		break;
	} 
}

/**
 * 获取偏航轴微分值
 * @param pidKind PID类型（ANGLE：角度环，RATE：角速度环）
 * @return        当前微分值
 * @note          偏航角度环未实现，返回0
 */
float PIDController::getYawDeriv(PIDKind pidKind) {
	switch (pidKind)
	{
	case ANGLE:
		return 0;                    // 偏航角度环未实现
		break;
	case RATE:
		return yawRatePID.deriv;     // 返回偏航角速度环微分值
		break;
	default:
		return 0;
		break;
	} 
}

/**
 * 清空单个PID控制器的数据
 * @param PID 指向要清空的PID结构体的指针
 * @note      将所有PID相关变量重置为初始状态，并更新时间戳
 */
void PIDController::cleanData(PID *PID) {
	PID->error = 0.0;            // 清空当前误差
	PID->lastError = 0.0;        // 清空上一次误差
	PID->integ = 0.0;            // 清空积分值
	PID->deriv = 0.0;            // 清空微分值
	PID->output = 0.0;           // 清空输出值
	PID->lastTimeStamp = micros(); // 更新时间戳
}

/**
 * 清空横滚轴所有PID数据
 * 包括角度环和角速度环
 */
void PIDController::cleanRollPIDData() {
	cleanData(&rollAnglePID);    // 清空横滚角度环数据
	cleanData(&rollRatePID);     // 清空横滚角速度环数据
}

/**
 * 清空俯仰轴所有PID数据
 * 包括角度环和角速度环
 */
void PIDController::cleanPitchPIDData() {
	cleanData(&pitchAnglePID);   // 清空俯仰角度环数据
	cleanData(&pitchRatePID);    // 清空俯仰角速度环数据
}

/**
 * 清空偏航轴所有PID数据
 * 包括角度环和角速度环
 */
void PIDController::cleanYawPIDData() {
	cleanData(&yawAnglePID);      // 清空偏航角度环数据
	cleanData(&yawRatePID);       // 清空偏航角速度环数据
}

void PIDController::cleanHeightPIDData() {
	cleanData(&heightPID);       // 清空高度环数据
	cleanData(&heightRatePID);   // 清空高度速度环数据
}
