/**
 ******************************************************************************
 * @file    monitor_task.c
 * @brief   监控任务
 *          1. 各模块心跳失联检测
 *          2. 监控遥控器状态，软件复位
 ******************************************************************************
 */
#include "monitor_task.h"
#include "gimbal.h"
#include "communicate.h"

int16_t a;
void StartMonitorTask(void const *argument)
{
	for (;;)
	{
		/* 板间心跳 */
		Board_Heartbeat();

		/* 云台 DM4310 心跳 */
		Gimbal_Heartbeat();

		osDelay(1);
	}
}

