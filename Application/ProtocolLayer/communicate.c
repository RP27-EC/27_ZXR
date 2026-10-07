/**
  ******************************************************************************
  * @file    communicate.c
  * @brief   板间通信 —— 上板（接收端）
  *
  *  接收机在下板，遥控数据靠下板经 CAN 转发。
  *  把收到的两帧解回 rc_sensor_info，并用心跳驱动 rc_sensor.work_state，
  ******************************************************************************
  */
#include "communicate.h"

/* Private variables ---------------------------------------------------------*/
static uint16_t s_cnt_pkt1 = BOARD_OFFLINE_CNT_MAX;   // 通道包超时计数
static uint16_t s_cnt_pkt2 = BOARD_OFFLINE_CNT_MAX;   // 拨杆包超时计数

/* Private functions ---------------------------------------------------------*/
static int16_t Unpack16(uint8_t *buf, uint8_t off)
{
	return (int16_t)((uint16_t)buf[off] | ((uint16_t)buf[off + 1] << 8));
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  通道包：ch0 ~ ch3
  */
void Board_Rx_Pkt_01(uint8_t *rxbuf)
{
	rc_sensor_info.ch0 = Unpack16(rxbuf, 0);
	rc_sensor_info.ch1 = Unpack16(rxbuf, 2);
	rc_sensor_info.ch2 = Unpack16(rxbuf, 4);
	rc_sensor_info.ch3 = Unpack16(rxbuf, 6);

	s_cnt_pkt1 = 0;
}

/**
  * @brief  拨杆 / 键鼠包：s1, s2, mouse_vx, mouse_vy, key_v
  */
void Board_Rx_Pkt_02(uint8_t *rxbuf)
{
	rc_sensor_info.s1.value = rxbuf[0];
	rc_sensor_info.s2.value = rxbuf[1];
	rc_sensor_info.mouse_vx = Unpack16(rxbuf, 2);
	rc_sensor_info.mouse_vy = Unpack16(rxbuf, 4);
	rc_sensor_info.key_v    = (uint16_t)((uint16_t)rxbuf[6] | ((uint16_t)rxbuf[7] << 8));

	s_cnt_pkt2 = 0;
}

/**
  * @brief  板间心跳，1ms 一次。
  *         用"有没有收到板间包"来驱动 rc_sensor.work_state，
  */
void Board_Heartbeat(void)
{
	if (s_cnt_pkt1 < BOARD_OFFLINE_CNT_MAX) s_cnt_pkt1++;
	if (s_cnt_pkt2 < BOARD_OFFLINE_CNT_MAX) s_cnt_pkt2++;

	rc_sensor.work_state = Board_Is_Online() ? DEV_ONLINE : DEV_OFFLINE;
}

/**
  * @brief  两包都在超时窗口内才算在线
  */
uint8_t Board_Is_Online(void)
{
	return (s_cnt_pkt1 < BOARD_OFFLINE_CNT_MAX) &&
	       (s_cnt_pkt2 < BOARD_OFFLINE_CNT_MAX);
}
