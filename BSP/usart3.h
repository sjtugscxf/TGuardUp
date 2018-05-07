#ifndef __USART3_H__
#define __USART3_H__
#include "main.h"
void USART3_Configuration(void);

typedef enum
{
	ONLINE,
	OFFLINE
}JudgeState_e;

typedef struct 
{
		uint16_t stageRemainTime;
		uint8_t  gameProgress;
		uint8_t  robotLevel;
		uint16_t remainHP;
		uint16_t maxHP;
}extGameRobotState_t;

typedef struct 
{
		uint8_t bulletType;
		uint8_t bulletFreq;
		float bulletSpeed;
}extShootData_t;

typedef struct 
{
    float chassisVolt;
		float chassisCurrent;
		float chassisPower;
    float chassisPowerBuffer;
		uint16_t shooterHeat0;
    uint16_t shooterHeat1;
}extPowerHeatData_t;

extern uint8_t bulletFreqBuf;
extern uint8_t shooterHeat0Buf[2];
extern uint8_t bulletSpeedBuf[4];

extern extGameRobotState_t extGameRobotState;
extern extShootData_t extShootData;
extern extPowerHeatData_t extPowerHeat;

#endif
