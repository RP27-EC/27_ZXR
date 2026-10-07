#include "can_protocol.h"
#include "communicate.h"

/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x11://Pitch 反馈帧 ID(MST_ID)，DM4310 挂 CAN1
		{
			Pitch_Motor.rx(&Pitch_Motor, rxBuf);
			break;
		}

		case 0x0b:
		{
			L_Wheel.rx(&L_Wheel, rxBuf);
			
			break;
		}
		
		case 0x205:
		{
			R_Fric.rx(&R_Fric, rxBuf);
			break;
		}

		default:
			break;
	}
}
/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	
	switch (canId)
	{
		case 0x12://Yaw 反馈帧 ID，DM4310 挂 CAN2
		{
			Yaw_Motor.rx(&Yaw_Motor, rxBuf);
			break;
		}
		case ID_Board_Rx1://板间通信：遥控通道 ch0~ch3
		{
			Board_Rx_Pkt_01(rxBuf);
			break;
		}
		case ID_Board_Rx2://板间通信：拨杆 / 键鼠
		{
			Board_Rx_Pkt_02(rxBuf);
			break;
		}
		
		default:
			break;
	}
}
