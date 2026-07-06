
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
	PID rollAnglePID;
	PID pitchAnglePID;
	PID yawAnglePID;
	PID yawRatePID;
	PID rollRatePID;
	PID pitchRatePID;
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
	enum PIDKind {
		ANGLE,
		RATE
	};
	PIDController();
	void calCurrentRollAnglePID(float measureRoll, float targetRoll);
	void calCurrentPitchAnglePID(float measurePitch, float targetPitch);
	void calCurrentYawAnglePID(float measureYaw, float targetYaw);
	void calCurrentYawRatePID(float measureYaw, float targetYaw);
	void calCurrentRollRatePID(float measureRollRate, float targetRollRate);
	void calCurrentPitchRatePID(float measurePitchRate, float targetPitchRate);
	void calCurrentHeightPID(float measureHeight, float targetHeight);
	void calCurrentHeightRatePID(float measureHeightRate, float targetHeightRate);
	void calCurrentPosXPID(float measure_Pos_X, float target_Pos_X);
	void calCurrentSpeedXPID(float measure_Speed_X, float target_Speed_X);
	void calCurrentPosYPID(float measure_Pos_Y, float target_Pos_Y);
	void calCurrentSpeedYPID(float measure_Speed_Y, float target_Speed_Y);


	/* ===================================================================
		* 
		*  PID 修正值
		* 
		* ===================================================================
		*/
	float getRollCorrect(PIDKind pidKind);
	float getPitchCorrect(PIDKind pidKind);
	float getYawCorrect(PIDKind pidKind);
	float getHeightCorrect();
	float getHeightRateCorrect();
	float getPosXCorrect();
	float getSpeedXCorrect();
	float getPosYCorrect();
	float getSpeedYCorrect();


	/* =============================================================
	* 
	* roll 相关信息
	* 
	* =============================================================
	*/
	float getRollError(PIDKind kind);
	float getRollInteg(PIDKind kind);
	float getRollDeriv(PIDKind kind);

	/* =============================================================
		* 
		* pitch 相关信息
		* 
		* =============================================================
		*/
	float getPitchError(PIDKind kind);
	float getPitchInteg(PIDKind kind);
	float getPitchDeriv(PIDKind kind);

	/* =============================================================
		* 
		* yaw 相关信息
		* 
		* =============================================================
		*/
	float getYawError(PIDKind kind);
	float getYawInteg(PIDKind kind);
	float getYawDeriv(PIDKind kind);

	void cleanRollPIDData();
	void cleanPitchPIDData();
	void cleanYawPIDData();
	void cleanHeightPIDData();
};
