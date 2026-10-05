/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   
  ******************************************************************************
  */
#include "control_task.h"
#include "gimbal_motor.h"
#include "Judge.h"
#include "communicate.h"

void StartCtrlTask(void const * argument)
{

	for(;;)
	{
		Chassis_Step();

		/* 把遥控数据转发给上板 */
		Board_Tx();


//	Back_Group.group_set_torque(&Back_Group);
//	Front_Group.group_set_torque(&Front_Group);
		osDelay(1);
	}
}
