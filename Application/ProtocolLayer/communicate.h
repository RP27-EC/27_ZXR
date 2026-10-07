/**
  ******************************************************************************
  * @file    communicate.h
  * @brief   板间通信 —— 上板（接收端）
  *
  *  接收机接在下板，下板解析 DBUS 后把遥控数据打包成 CAN 帧转发给上板。
  *  本模块负责接收并填回 rc_sensor_info
  *
  *  帧格式（经典 CAN，标准帧，8 字节，小端）：
  *    0xD1: ch0, ch1, ch2, ch3           各 int16，共 8 字节
  *    0xD2: s1, s2, mouse_vx, mouse_vy, key_v
  *          1 + 1 + 2 + 2 + 2 = 8 字节
  *
  *  把板间帧和 Yaw 电机放在同一条 CAN2 上
  ******************************************************************************
  */
#ifndef __COMMUNICATE_H
#define __COMMUNICATE_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "rc_sensor.h"

/* Exported macro ------------------------------------------------------------*/
/* 板间帧 ID，下板发送 ID 和这里一致 */
#define ID_Board_Rx1            0xD1    // 通道包
#define ID_Board_Rx2            0xD2    // 拨杆/键鼠包

/* 板间包超时上限 */
#define BOARD_OFFLINE_CNT_MAX   100

/* Exported functions --------------------------------------------------------*/
void Board_Rx_Pkt_01(uint8_t *rxbuf);   // 在 CAN2 的 0xD1 分支里调用
void Board_Rx_Pkt_02(uint8_t *rxbuf);   // 在 CAN2 的 0xD2 分支里调用
void Board_Heartbeat(void);             // 1ms 调用一次，同时更新 rc_sensor.work_state
uint8_t Board_Is_Online(void);          // 下板是否还在发包

#endif
