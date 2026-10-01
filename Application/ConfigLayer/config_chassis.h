#ifndef __CONFIG_CHASSIS_H
#define __CONFIG_CHASSIS_H

#define OFF_GROUND_TEST 0

#define TIME_STEP			0.001f//任务运行周期，单位：s

#define WHEEL_RADIUS        0.0515f//驱动轮半径，单位：m

//整车长度
#define Car_Length 0.39994f
//整车宽度
#define Car_Width 0.39990f
//麦轮旋转臂
#define ROTATE_R (Car_Length + Car_Width) / 2.0f
//车体运动最大速度
#define MAX_SPEED           1.2f    //单位：m/s
//车体转向运动最大速度
#define MAX_SPIN_SPEED           2.5f    //单位：rad/s
//车体中心离地高度
#define GRAVITY_HIGHT           0.18f    //单位：m
//全车重量
#define CAR_GRAVITY           116.25f    //单位：N

#endif
