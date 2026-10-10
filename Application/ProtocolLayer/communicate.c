/**
  ******************************************************************************
  * @file    communicate.c
  * @brief   板间通信 —— 下板（发送端）
  *
  *  接收机在下板，上板没有接收机，这里把 rc_sensor_info 打包转发给上板。
  *  发送用经典 CAN 帧
  ******************************************************************************
  */
#include "communicate.h"

/* Private variables ---------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan2;   // 板间通信所在总线

/* Private functions ---------------------------------------------------------*/
/* 小端写入一个 int16 到 buf[off] */
static void Pack16(uint8_t *buf, uint8_t off, int16_t v)
{
	buf[off]     = (uint8_t)((uint16_t)v & 0xFF);
	buf[off + 1] = (uint8_t)(((uint16_t)v >> 8) & 0xFF);
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  把遥控数据发给上板
  */
void Board_Tx(void)
{
	uint8_t buf[8];
	uint8_t i;

	/* 遥控离线：转发全 0 安全值（s1 = 0）。。
	   避免"遥控已关、下板仍在转发旧值"导致上板误判在线、云台继续受控。 */
	if (rc_sensor.work_state != DEV_ONLINE)
	{
		for (i = 0; i < 8; i++)
		{
			buf[i] = 0;
		}
		CAN_SendData(&hfdcan2, ID_Board_Rx1, buf);
		CAN_SendData(&hfdcan2, ID_Board_Rx2, buf);
		return;
	}

	/* 通道包：ch0 ~ ch3 */
	Pack16(buf, 0, rc_sensor_info.ch0);
	Pack16(buf, 2, rc_sensor_info.ch1);
	Pack16(buf, 4, rc_sensor_info.ch2);
	Pack16(buf, 6, rc_sensor_info.ch3);
	CAN_SendData(&hfdcan2, ID_Board_Rx1, buf);

	/* 拨杆 / 键鼠包 */
	buf[0] = rc_sensor_info.s1;
	buf[1] = rc_sensor_info.s2;
	Pack16(buf, 2, rc_sensor_info.mouse_vx);
	Pack16(buf, 4, rc_sensor_info.mouse_vy);
	Pack16(buf, 6, (int16_t)rc_sensor_info.key_v);
	CAN_SendData(&hfdcan2, ID_Board_Rx2, buf);
}
