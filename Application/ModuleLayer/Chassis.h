#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "main.h"
#include "RM_motor.h"
#include "rc_sensor.h"
#include "PID.h"

#define MAX_WHEEL_SPEED_RAD 25.0f  //限制最大目标速度单位rad/s

/*对外暴露的电机变量*/
extern Motor_RM_t LF_Motor;
extern Motor_RM_t RF_Motor;
extern Motor_RM_t LB_Motor;
extern Motor_RM_t RB_Motor;
extern Motor_RM_Group_t Wheel_Group;

/* 底盘模式 */
typedef enum {
    CHASSIS_MODE_SLEEP = 0,      // 睡眠（卸力）
    CHASSIS_MODE_NORMAL,          // 正常
} chassis_mode_e;

typedef struct {
    chassis_mode_e mode;
    float vx;                     // 前后速度
    float vy;                     // 左右速度
    float wz;                     // 旋转速度
    float wheel_speed[4];         // LF, RF, LB, RB
} chassis_t;

extern chassis_t chassis;

void Chassis_Init(void);
void Chassis_Step(void);

#endif
