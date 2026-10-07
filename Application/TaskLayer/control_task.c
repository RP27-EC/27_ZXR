/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   控制任务
  *          只负责调度：更新 IMU + 调用云台模块。
  *          云台具体的串级 PID 实现在 ModuleLayer/gimbal.c
  ******************************************************************************
  */
#include "control_task.h"
#include "gimbal.h"

float t;

void StartControlTask(void const * argument)
{
	Gimbal_Init();

	for(;;) 
	{
		if ((imu_sensor.work_state.err_code == IMU_NONE_ERR) || \
				(imu_sensor.work_state.err_code == IMU_DATA_CALI))
		{
			imu_sensor.update(&imu_sensor);
		}

		Gimbal_Ctrl();

		osDelay(1);
	}
}
