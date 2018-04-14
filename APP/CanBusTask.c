#include "main.h"
#include "stdio.h"

#define CanRxGetU16(canRxMsg, num) (((uint16_t)canRxMsg.Data[num * 2] << 8) | (uint16_t)canRxMsg.Data[num * 2 + 1])
uint8_t isRcanStarted_CMGM = 0;
Motor820RRxMsg_t CMFLRx,CMFRRx,BulletRx,Bullet2Rx;
Motor6623RxMsg_t GMPITCHRx,GMYAWRx;

static uint32_t can_count = 0;

void CanReceiveMsgProcess(CanRxMsg * msg)
{      
        //GMYawEncoder.ecd_bias = yaw_ecd_bias;
        can_count++;
		switch(msg->StdId)
		{
				case CMFL_RXID:
				CMFLRx.angle = ((uint16_t)msg->Data[0]<<8)|(uint16_t)msg->Data[1];
				CMFLRx.RotateSpeed = ((uint16_t)msg->Data[2]<<8)|(uint16_t)msg->Data[3];
				break;
			case CMFR_RXID:
				CMFRRx.angle = ((uint16_t)msg->Data[0]<<8)|(uint16_t)msg->Data[1];
				CMFRRx.RotateSpeed = ((uint16_t)msg->Data[2]<<8)|(uint16_t)msg->Data[3];
				break;
			case BULLET_RXID:
				BulletRx.angle = ((uint16_t)msg->Data[0]<<8)|(uint16_t)msg->Data[1];
				BulletRx.RotateSpeed = ((uint16_t)msg->Data[2]<<8)|(uint16_t)msg->Data[3];
				break;
			case BULLET2_RXID:
				Bullet2Rx.angle = ((uint16_t)msg->Data[0]<<8)|(uint16_t)msg->Data[1];
				Bullet2Rx.RotateSpeed = ((uint16_t)msg->Data[2]<<8)|(uint16_t)msg->Data[3];
				break;
			case GMYAW_RXID:
				GMYAWRx.angle = ((uint16_t)msg->Data[0]<<8)|(uint16_t)msg->Data[1];
				break;
			case GMPITCH_RXID:
				GMPITCHRx.angle = ((uint16_t)msg->Data[0]<<8)|(uint16_t)msg->Data[1];
				break;
			case UPMSG_RXID:
				//testrecv1 = CMGMCanRxMsg.Data[0];
			  //testrecv2 = CMGMCanRxMsg.Data[4];
				break;
			
				default:
				{
				}
		}
}

/********************************************************************************
   �����̵���巢��ָ�ID��Ϊ0x200�������̷���IDΪ0x201-0x204
*********************************************************************************/
void Set_CM_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);
    tx_message.Data[1] = (uint8_t)cm1_iq;
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
    CAN_Transmit(CANx,&tx_message);
}

/********************************************************************************
   ������巢��ָ�ID��Ϊ0x1FF��ֻ����������壬���ݻش�IDΪ0x205��0x206
	 cyq:����Ϊ�������������ָ�
*********************************************************************************/
void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq)
{
    CanTxMsg tx_message;    
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (unsigned char)(gimbal_yaw_iq >> 8);
    tx_message.Data[1] = (unsigned char)gimbal_yaw_iq;
    tx_message.Data[2] = (unsigned char)(gimbal_pitch_iq >> 8);
    tx_message.Data[3] = (unsigned char)gimbal_pitch_iq;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}
