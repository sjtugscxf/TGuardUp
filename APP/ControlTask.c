#include "main.h"

//github test
void fw_PID_Reset(fw_PID_Regulator_t *pid){
	
}

void fw_PID_Calc(fw_PID_Regulator_t *pid){
	pid->errorCurr = pid->target - pid->feedback;
	if(pid->SumCount <=500){
		pid->errorSum += pid->target - pid->feedback;
		pid->SumCount++;
		}
	else {
	  pid->errorSum = 0;
		pid->SumCount = 0;
	}
	pid->componentKp = pid->kp * pid->errorCurr;
	MINMAX(pid->componentKp, -pid->componentKpMax, pid->componentKpMax);
	pid->componentKi = pid->ki * pid->errorSum;
	MINMAX(pid->componentKi, -pid->componentKiMax, pid->componentKiMax);
	pid->componentKd = pid->kd * (pid->errorCurr - pid->errorLast);
	MINMAX(pid->componentKd, -pid->componentKdMax, pid->componentKdMax);
	
	pid->errorLast = pid->errorCurr;
	
	pid->output = pid->componentKp + pid->componentKi + pid->componentKd;
	MINMAX(pid->output, -pid->outputMax, pid->outputMax);
}

extern fw_PID_Regulator_t pitchPositionPID;
extern fw_PID_Regulator_t yawPositionPID;
extern fw_PID_Regulator_t pitchSpeedPID;
extern fw_PID_Regulator_t yawSpeedPID;

int16_t ProcessYawPID(float target, float velocity_feedback)
{
	return PID_PROCESS_Speed(yawSpeedPID,target,velocity_feedback);
}
int16_t ProcessPitchPID(float target, float position_feedback, float velocity_feedback)
{
	return PID_PROCESS_Double(pitchPositionPID,pitchSpeedPID,target,position_feedback,velocity_feedback);
}
int16_t PID_PROCESS_Double(fw_PID_Regulator_t pid_position,fw_PID_Regulator_t pid_speed,float target, float position_feedback, float velocity_feedback)
{
	//position		
	pid_position.target = target;
	pid_position.feedback = position_feedback;
	pid_position.Calc(&pid_position);
	//speed
	pid_speed.target = pid_position.output;
	pid_speed.feedback = velocity_feedback;
	pid_speed.Calc(&pid_speed);
	return pid_speed.output;
}

int16_t PID_PROCESS_Speed(fw_PID_Regulator_t pid_speed,float target, float velocity_feedback)
{
	pid_speed.target = target;
	pid_speed.feedback = velocity_feedback;
	pid_speed.Calc(&pid_speed);
	return pid_speed.output;
}

float auto_attack_yaw_kp = 0.4;
float auto_attack_pitch_kp = 0.001;
float auto_attack_yaw_kd = 0.01;
float auto_attack_pitch_kd = 0.0;
uint8_t find_enemy = 0;
uint8_t on_enemy = 0;
uint16_t enemy_yaw = YAW_OFFSET;
uint16_t enemy_pitch = PITCH_OFFSET;
uint16_t enemy_detect_cnt = 0;
WorkState_e WorkState = START_STATE;
uint16_t prepare_time = 0;

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t BulletSpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t Bullet2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

PID_Regulator_t BulletPositionPID = BULLET_POSITION_PID_DEFAULT;
PID_Regulator_t Bullet2PositionPID = BULLET_POSITION_PID_DEFAULT;
//PID_Regulator_t BulletSpeedPID = BULLET_SPEED_PID_DEFAULT;//0.0, 0.00003
//PID_Regulator_t Bullet2SpeedPID = BULLET_SPEED_PID_DEFAULT;//0.0, 0.00003

int16_t CMFLIntensity = 0, CMFRIntensity = 0, BulletIntensity = 0,Bullet2Intensity = 0;
int16_t yawIntensity = 0;		
int16_t pitchIntensity = 0;

//??PID???
void CMControlInit(void)
{
	CMRotatePID.Reset(&CMRotatePID);
	CM1SpeedPID.Reset(&CM1SpeedPID);
	CM2SpeedPID.Reset(&CM2SpeedPID);
	BulletSpeedPID.Reset(&BulletSpeedPID);
	Bullet2SpeedPID.Reset(&Bullet2SpeedPID);
	//BulletPositionPID.Reset(&BulletPositionPID);
	//Bullet2PositionPID.Reset(&Bullet2PositionPID);
}

//?????????,??
void ControlCMFL(void)
{		
	CM1SpeedPID.ref =  ChassisSpeedRef.forward_back_ref*0.075+ ChassisSpeedRef.left_right_ref*0.075;
	CM1SpeedPID.ref = 160 * CM1SpeedPID.ref;	
			
	CM1SpeedPID.fdb = CMFLRx.RotateSpeed;

	CM1SpeedPID.Calc(&CM1SpeedPID);
	CMFLIntensity = CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output;
}

void ControlCMFR(void)
{		
	CM2SpeedPID.ref = - ChassisSpeedRef.forward_back_ref*0.075 + ChassisSpeedRef.left_right_ref*0.075;
	CM2SpeedPID.ref = 160 * CM2SpeedPID.ref;	
			
	CM2SpeedPID.fdb = CMFRRx.RotateSpeed;

	CM2SpeedPID.Calc(&CM2SpeedPID);
	CMFRIntensity = CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.output;
}

void ControlBullet(void)
{		
	BulletSpeedPID.kp = 3.0;
	BulletSpeedPID.kd = 0.0;
	BulletSpeedPID.ref = bullet_ref*0.075;
	BulletSpeedPID.ref = 160 * BulletSpeedPID.ref;	
			
	BulletSpeedPID.fdb = BulletRx.RotateSpeed;

	BulletSpeedPID.Calc(&BulletSpeedPID);
	BulletIntensity = CHASSIS_SPEED_ATTENUATION * BulletSpeedPID.output;
}

void ControlBullet2(void)
{		
	Bullet2SpeedPID.kp = 3.0;
	Bullet2SpeedPID.kd = 0.0;
	if(bullet2_ref<300 && bullet2_ref>-300) bullet2_ref = 0;
	Bullet2SpeedPID.ref = bullet2_ref*0.075;
	Bullet2SpeedPID.ref = 160 * Bullet2SpeedPID.ref;	
			
	Bullet2SpeedPID.fdb = Bullet2Rx.RotateSpeed;

	Bullet2SpeedPID.Calc(&Bullet2SpeedPID);
	Bullet2Intensity = CHASSIS_SPEED_ATTENUATION * Bullet2SpeedPID.output;
}

double bullet_angle_target=0;
double bulletRealAngle=0;
double bullet_zero_angle=0;
void setBulletWithAngle(double targetAngle){//360.0 * 12 * 2
		static double AngleLast = 180.0;
		double AngleCurr = BulletRx.angle * 360 / 8192.0;
		static uint8_t isInitiated=0;
		if(isInitiated==0)
		{
			bulletRealAngle=AngleCurr;
			bullet_zero_angle=AngleCurr;
			AngleLast=AngleCurr;
			isInitiated=1;
			return;
		}
		if(AngleCurr - AngleLast > 180){
			bulletRealAngle += AngleCurr - 360 - AngleLast;
		}else if(AngleCurr - AngleLast < -180){
			bulletRealAngle += AngleCurr + 360 - AngleLast;
		}else{
			bulletRealAngle += AngleCurr - AngleLast;
		}
		AngleLast=AngleCurr;
		//RealSpeed
		double realSpeed = BulletRx.RotateSpeed * 6;//???(* 360 / 60.0)

		BulletPositionPID.ref = targetAngle;
		BulletPositionPID.fdb = bulletRealAngle;
		BulletPositionPID.Calc(&BulletPositionPID);
		
		BulletSpeedPID.ref = BulletPositionPID.output;
		BulletSpeedPID.fdb = realSpeed;
		BulletSpeedPID.Calc(&BulletSpeedPID);
		BulletIntensity = BulletSpeedPID.output;
}

double bullet2_angle_target=0;
double bullet2RealAngle=0;
double bullet2_zero_angle=0;
void setBullet2WithAngle(double targetAngle){//360.0 * 12 * 2
		static double AngleLast = 180.0;
		double AngleCurr = Bullet2Rx.angle * 360 / 8192.0;
		static uint8_t isInitiated=0;
		if(isInitiated==0)
		{
			bullet2RealAngle=AngleCurr;
			bullet2_zero_angle=AngleCurr;
			AngleLast=AngleCurr;
			isInitiated=1;
			return;
		}
		if(AngleCurr - AngleLast > 180){
			bullet2RealAngle += AngleCurr - 360 - AngleLast;
		}else if(AngleCurr - AngleLast < -180){
			bullet2RealAngle += AngleCurr + 360 - AngleLast;
		}else{
			bullet2RealAngle += AngleCurr - AngleLast;
		}
		AngleLast=AngleCurr;
		//RealSpeed
		double realSpeed = Bullet2Rx.RotateSpeed * 6;//???(* 360 / 60.0)

		Bullet2PositionPID.ref = targetAngle;
		Bullet2PositionPID.fdb = bullet2RealAngle;
		Bullet2PositionPID.Calc(&Bullet2PositionPID);
		
		Bullet2SpeedPID.ref = Bullet2PositionPID.output;
		Bullet2SpeedPID.fdb = realSpeed;
		Bullet2SpeedPID.Calc(&Bullet2SpeedPID);
		Bullet2Intensity = Bullet2SpeedPID.output;
}

float odometry = 0.0;
float odometry_fact = 0.01;
float odometry_upmax1 = 80000.0;
float odometry_downmax1 = -80000.0;
float odometry_speed1 = 50.0;
float odometry_upmax2 = 10000.0;
float odometry_downmax2 = -10000.0;
float odometry_speed2 = 15.0;
void odometryLoop()
{
	if((CMFLRx.RotateSpeed > 50 || CMFLRx.RotateSpeed < -50) && (CMFRRx.RotateSpeed > 50 || CMFRRx.RotateSpeed < -50))
	{
		odometry += (CMFLRx.RotateSpeed - CMFRRx.RotateSpeed) * odometry_fact;
	}
}

extern FrictionWheelState_e FrictionWheelState;
extern Shoot_State_e ShootState;
//?????
void WorkStateFSM(void)
{
	static int blink_cnt = 0;
	blink_cnt++;
	switch (WorkState)
	{
		case START_STATE:
		{
			if(prepare_time<2000) prepare_time++;
			if(prepare_time == 2000)//??????????
			{
				WorkState = PREPARE_STATE;
			}
			if (inputmode == STOP) WorkState = STOP_STATE;
		}break;
		case PREPARE_STATE:
		{
			if(prepare_time<4000) prepare_time++;
			if(prepare_time == 4000)//??????????
			{
				WorkState = NORMAL_STATE;
				prepare_time = 0;
				GREEN_LED_OFF();
				RED_LED_OFF();
				blink_cnt = 0;
//				printf("START\n");
			}
			
			if (inputmode == STOP) WorkState = STOP_STATE;
		}break;
		case NORMAL_STATE://????????
		{
			if (inputmode == STOP) 
			{
				WorkState = STOP_STATE;
				frictionRamp.ResetCounter(&frictionRamp);
				FrictionWheelState = FRICTION_WHEEL_OFF;
			}
			
			if (blink_cnt == 1000) 
			{
				blink_cnt = 0;
				GREEN_LED_TOGGLE();
			}
		}break;
		case DEFEND_STATE:  //????,??360???
		{
			if (find_enemy == 1) WorkState = ATTACK_STATE;
			
			if (inputmode == STOP) 
			{
				WorkState = STOP_STATE;
//				SetFrictionWheelSpeed(1000); 
				FrictionWheelState = FRICTION_WHEEL_OFF;
				frictionRamp.ResetCounter(&frictionRamp);
			}
			else if (inputmode == REMOTE_INPUT)
			{
//				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
				FrictionWheelState = FRICTION_WHEEL_OFF;
				WorkState = NORMAL_STATE;
			}
			if (blink_cnt == 1000) 
			{
				blink_cnt = 0;
				RED_LED_TOGGLE();
			}
		}break;
		case ATTACK_STATE:  //??????
		{
			static int enemy_lost = 0;
			if (find_enemy == 0) 
			{
				enemy_lost++;
				if (enemy_lost > 100) 
				{
					WorkState = DEFEND_STATE;
//					ChassisSpeedRef.forward_back_ref = odometry_speed1;
					enemy_lost = 0;
				}
			}
			else enemy_lost = 0;
			
			if (inputmode == STOP) 
			{
				WorkState = STOP_STATE;
//				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
				FrictionWheelState = FRICTION_WHEEL_OFF;
			}
			else if (inputmode == REMOTE_INPUT)
			{
//				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
				FrictionWheelState = FRICTION_WHEEL_OFF;
				WorkState = NORMAL_STATE;
			}
			if (blink_cnt == 1000) 
			{
				blink_cnt = 0;
				GREEN_LED_TOGGLE();
				RED_LED_TOGGLE();
			}
		}break;
		case STOP_STATE://????
		{
			if (inputmode == REMOTE_INPUT)
			{
				WorkState = PREPARE_STATE;
				RemoteTaskInit();
			}
		}break;
	}
}

//????CAN????
void setCMMotor()
{
	Set_CM_Speed(CAN2, CMFLIntensity, CMFRIntensity, BulletIntensity, Bullet2Intensity);		 
}

//????CAN????
void setGMMotor()
{
	Set_Gimbal_Current(CAN2, pitchIntensity, yawIntensity); 
}

#define NORMALIZE_ANGLE180(angle) angle = ((angle) > 180) ? ((angle) - 360) : (((angle) < -180) ? (angle) + 360 : angle)
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(5.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(5.0, 0.0, 0.5, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(6.5, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 3500.0);
//fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(30.0, 0.0, 0, 10000.0, 10000.0, 10000.0, 4000.0);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(10.0, 0.0, 0, 10000.0, 10000.0, 10000.0, 2000.0);
#define yaw_zero 7200  //100
#define pitch_zero 3043
float yawRealAngle = 0.0;
float pitchRealAngle = 0.0;
float gap_angle = 0.0;

//????YAW?
void ControlYawSpeed(void)
{	
	yawIntensity = ProcessYawPID(yawSpeedTarget,-MPU6050_Real_Data.Gyro_Z);
}

//????pitch?
void ControlPitch(void)
{
	uint16_t pitchZeroAngle = pitch_zero;
				
	pitchRealAngle = -(GMPITCHRx.angle - pitchZeroAngle) * 360 / 8192.0;
	NORMALIZE_ANGLE180(pitchRealAngle);

	MINMAX(pitchAngleTarget, -80.0f, 90);
				
	pitchIntensity = -ProcessPitchPID(pitchAngleTarget,pitchRealAngle,-MPU6050_Real_Data.Gyro_X);
}

float enemy_yaw_err = 0;
float enemy_yaw_out = 0;
float enemy_pitch_err = 0;
float enemy_pitch_out = 0;

uint16_t disturb_init = 0;
uint16_t disturb_cnt = 0;
float disturb_angle = 8000.0;
//?????
unsigned char testred1 = 0;
unsigned char testred2 = 0;
unsigned char testred3 = 0;
unsigned char testred4 = 0;

uint8_t redBuf = 0;


void sendUpMsg(CAN_TypeDef *CANx)
{
    CanTxMsg tx_message;    
    tx_message.StdId = 0x305;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = redBuf;
    tx_message.Data[1] = bulletFreqBuf;
    tx_message.Data[2] = shooterHeat0Buf[0];
    tx_message.Data[3] = shooterHeat0Buf[1];
    tx_message.Data[4] = bulletSpeedBuf[0];
    tx_message.Data[5] = bulletSpeedBuf[1];
    tx_message.Data[6] = bulletSpeedBuf[2];
    tx_message.Data[7] = bulletSpeedBuf[3];
    CAN_Transmit(CANx,&tx_message);
}

void controlLoop()
{
	testred1 = READ_RED1();
	testred2 = READ_RED2();
	testred3 = READ_RED3();
	testred4 = READ_RED4();
	
	uint8_t gametemp =  (extGameRobotState.gameProgress << 4) & 0xF0;
	redBuf = (gametemp | (testred4<<3) | (testred3<<2) | (testred2<<1) | (testred1)); 
	
	WorkStateFSM();
	
	if(WorkState != STOP_STATE && WorkState != START_STATE) 
	{
		sendUpMsg(CAN2);
		sendUpMsg(CAN1);
	}
}