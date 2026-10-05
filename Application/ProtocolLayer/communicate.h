/**
  ******************************************************************************
  * @file    communicate.h
  * @brief   板间通信 —— 下板（发送端）
  *
  *  接收机接在下板，上板没有接收机，所以下板解析完 DBUS 后要把遥控数据
  *  打包成 CAN 帧转发给上板。
  *
  *  帧格式（ CAN，标准帧，8 字节，小端），与上板一致：
  *    0xD1: ch0, ch1, ch2, ch3                      各 int16，共 8 字节
  *    0xD2: s1, s2, mouse_vx, mouse_vy, key_v
  *          1 + 1 + 2 + 2 + 2 = 8 字节
  *
  *  发送总线：CAN2（
  ******************************************************************************
  */
#ifndef __COMMUNICATE_H
#define __COMMUNICATE_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "rc_sensor.h"
#include "drv_can.h"

/* Exported macro ------------------------------------------------------------*/
#define ID_Board_Rx1            0xD1    // 通道包
#define ID_Board_Rx2            0xD2    // 拨杆 / 键鼠包

/* Exported functions --------------------------------------------------------*/
void Board_Tx(void);   // 周期调用，把遥控数据发给上板

#endif
