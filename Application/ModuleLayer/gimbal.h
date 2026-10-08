/**
  ******************************************************************************
  * @file    gimbal.h
  * @brief   云台模块（DM-J4310-2EC V1.1，10:1 减速）
  *
  *  控制链路：遥控器 -> 目标角度积分 -> 角度环 -> 速度环 -> 力矩 -> CAN
  *  反馈位置和速度均为【输出轴侧】：motor_angle_sum(rad) / rx_info->speed(rad/s)
  *
  *  所有可调参数集中在 config_gimbal.h
  ******************************************************************************
  */
#ifndef __GIMBAL_H
#define __GIMBAL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "config_gimbal.h"
#include "motor.h"
#include "PID.h"

/* Exported types ------------------------------------------------------------*/
/* 云台轴索引 */
typedef enum
{
	GIMB_YAW = 0,
	GIMB_PITCH,

	GIMB_AXIS_CNT,
} gimbal_axis_e;

/* 云台工作模式 关控/掉线时卸力，恢复需摇杆回中
 *   由左拨杆 s1 档位决定（RC_SW_UP=1 / RC_SW_MID=3 / RC_SW_DOWN=2）：
 *     上档 -> NORMAL 目前单独控制云台
 *     中档 -> MECH   机械模式                                       */
typedef enum
{
	GIMB_MODE_SLEEP = 0,   // 卸力
	GIMB_MODE_NORMAL,      // 单独控制云台（s1 上档）
	GIMB_MODE_MECH,        // 机械模式（s1 中档）：yaw 跟随底盘，pitch 正常受控
} gimbal_mode_t;

/* 单轴状态
   宏只提供上电初值 */
typedef struct
{
	Motor_DM_t *motor;         // 电机本体
	pid_ctrl_t  ag_pid;        // 角度环（外环），输出 rad/s
	pid_ctrl_t  sp_pid;        // 速度环（内环），输出 N·m
	float       target_angle;  // 目标角度(rad)，由遥杆累加得到
	uint8_t     inited;        // 目标角度是否已与反馈同步过
} gimbal_axis_t;

/* Exported variables --------------------------------------------------------*/
/* 云台双轴实例*/
extern gimbal_axis_t g_gimbal[GIMB_AXIS_CNT];

/* 云台当前模式  方便看状态 */
extern gimbal_mode_t g_gimbal_mode;

/* Exported functions --------------------------------------------------------*/
void Gimbal_Init(void);               // 绑定电机并装载默认 PID，任务启动前调用一次
void Gimbal_Ctrl(void);               // 安全检查 + 串级计算 + 下发力矩，周期性调用
void Gimbal_Heartbeat(void);          // 云台电机 1ms 心跳，超时判定掉线
uint8_t Gimbal_Is_Motor_Online(void); // 两轴电机是否都在线
gimbal_mode_t Gimbal_Get_Mode(void);  // 当前模式
void Gimbal_Calibrate_Zero(gimbal_axis_e idx); // 把当前姿态即时设为零位（调试标定）

#endif
