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

extern extShootData_t extShootData;
extern extPowerHeatData_t extPowerHeat;

#endif
