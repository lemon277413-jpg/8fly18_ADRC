
typedef struct {
	float KP;
	float KI;
	float KD;
	float error;
	float lastError;
	float integ;//积分
	float deriv;//微分
	float output;
	long lastTimeStamp;
} PID;

class PIDController {
private:
	// 高度环 + 位置环 + 速度环 PID (角度环已由ADRC替代)
	PID heightPID;
	PID heightRatePID; // 高度速度环PID
	PID posXPID; // X轴位置环PID
	PID speedXPID; // X轴速度环PID
	PID posYPID; // Y轴位置环PID
	PID speedYPID; // Y轴速度环PID

	void calCurrentPID(PID *PID, float target, float measure, float dt);
	void cleanData(PID *PID);
	float minMax(float value, float min, float max);

public:
	PIDController();
	void calCurrentHeightPID(float measureHeight, float targetHeight);
	void calCurrentHeightRatePID(float measureHeightRate, float targetHeightRate);
	void calCurrentPosXPID(float measure_Pos_X, float target_Pos_X);
	void calCurrentSpeedXPID(float measure_Speed_X, float target_Speed_X);
	void calCurrentPosYPID(float measure_Pos_Y, float target_Pos_Y);
	void calCurrentSpeedYPID(float measure_Speed_Y, float target_Speed_Y);

	// PID 输出获取 (高度/位置/速度环)
	float getHeightCorrect();
	float getHeightRateCorrect();
	float getPosXCorrect();
	float getSpeedXCorrect();
	float getPosYCorrect();
	float getSpeedYCorrect();

	void cleanHeightPIDData();
};
