#ifndef __CONFIG_GIMBAL_H
#define __CONFIG_GIMBAL_H

/* Includes ------------------------------------------------------------------*/
#include "pid_conf.h"

/**
 ******************************************************************************
 * @file    config_gimbal.h
 * @brief   云台（DM-J4310-2EC V1.1）配置
 *
 *  - 减速比            10:1
 *  - 额定扭矩 / 峰值    3 N·m / 7 N·m
 *  - 额定转速 / 空载最大 120 rpm / 200 rpm  (= 20.94 rad/s)
 *  - 编码器            14 位单圈磁编
 *  - 默认波特率        1 Mbps
 *
 *  位置 / 速度 / 扭矩反馈均为"输出轴侧"（减速之后），
 *  单位分别为 rad / (rad/s) / N·m，位置限定在 [-π, π] rad。
 *
 *  MIT 模式下 Kp = 0、Kd = 0 时，输出扭矩仅由 t_ff 决定，
 *  即"纯扭矩模式"，本配置即采用该模式由外部串级 PID 输出力矩。
 ******************************************************************************
 */

/* DM4310 CAN ID --------------------------------------------------------------*/
#define GIMB_PITCH_ESC_ID       0x001   // Pitch 电机 ID = 1，挂 CAN1
#define GIMB_YAW_ESC_ID         0x002   // Yaw   电机 ID = 2，挂 CAN2
#define GIMB_PITCH_MST_ID       0x011   // Pitch 反馈帧 ID
#define GIMB_YAW_MST_ID         0x012   // Yaw   反馈帧 ID

/* 输出轴/MIT 映射范围：与 DM_Motor.h 中的 P/V/T_MAX设置一致---------------------*/
#define GIMB_MIT_T_MAX          10.0f
#define GIMB_MIT_V_MAX          30.0f

/* 扭矩限幅（N·m）-------------------------------------------------------------*/
#define GIMB_YAW_TORQUE_MAX     3.0f
#define GIMB_PITCH_TORQUE_MAX   3.0f
#define GIMB_TEST_TORQUE_MAX    1.0f

/* 输出轴最大转速（rad/s）-----------------------------------------------------*/
/* 空载最大 200 rpm = 20.94 rad/s，留余量按 60% 取 12 rad/s                    */
#define GIMB_YAW_SPEED_MAX      12.0f
#define GIMB_PITCH_SPEED_MAX    12.0f

/* 角度软限位（rad）：限制目标角范围
 *   Yaw 无限转，GIMB_YAW_ANGLE_MAX 当前不生效
 *   Pitch 有限转，非对称限位                   */
#define GIMB_YAW_ANGLE_MAX      (3.1415926f)
#define GIMB_PITCH_TARGET_MAX   ( 30.0f * 0.0174533f)
#define GIMB_PITCH_TARGET_MIN   (-7.0f * 0.0174533f)

/* 遥控器映射 -----------------------------------------------------------------*/
/* 摇杆量 ±660 -> [-1, 1]，乘以下面的角速度得"目标角度的积分速率"             */
#define RC_CH_VALUE_RANGE       660.0f
#define GIMB_YAW_RC_RATE        3.0f    // rad/s @满杆（角度目标累加速度）
#define GIMB_PITCH_RC_RATE      2.0f

/* 遥控->轴方向符号：摇杆推进方向与电机"正角度"方向可能相反，用本宏翻转*/
#define GIMB_YAW_RC_DIR         -1       // 右摇杆左右 -> Yaw 方向；反了改 -1
#define GIMB_PITCH_RC_DIR       1       // 右摇杆上下 -> Pitch 方向；反了改 -1

/* 遥控死区，抑制中位漂移 ------------------------------------------------------*/
#define GIMB_RC_DEADBAND        20      // |value| 小于此值当作 0

/* 关控保护 -------------------------------------------------------------------*/
/* 恢复控制前是否要求四个摇杆都回中（防止重开控瞬间云台甩到杆位角度）。
   置 0 则遥控一恢复就立即接管 */
#define GIMB_RC_RESUME_NEED_CENTER   1

/* 云台拨杆档位（左拨杆 s1）：RC_SW_UP=1 / RC_SW_MID=3 / RC_SW_DOWN=2
 *   上档(1) = 目前单独控制云台
 *   中档(3) = 机械模式
 *   下档(2) = 卸力                                                               */
#define GIMB_ENABLE_S1_POS           1      // 上档：单独控制云台
#define GIMB_MECH_S1_POS             3      // 中档：机械模式（yaw 跟随底盘）

/* 串级 PID 参数 --------------------------------------------------------------*/
/* 外环（角度环）：输入 rad 误差，输出 rad/s 转速给定                           */
/* 内环（速度环）：输入 rad/s 误差，输出 N·m 力矩给定                           */

// Yaw 轴
#define GIMB_YAW_AG_KP          8.0f
#define GIMB_YAW_AG_KI          0.0f
#define GIMB_YAW_AG_KD          0.0f
#define GIMB_YAW_AG_I_MAX       10.0f
#define GIMB_YAW_AG_OUT_MAX     GIMB_YAW_SPEED_MAX

#define GIMB_YAW_SP_KP          1.3f
#define GIMB_YAW_SP_KI          0.01f
#define GIMB_YAW_SP_KD          0.0f
#define GIMB_YAW_SP_I_MAX       10.0f
#define GIMB_YAW_SP_OUT_MAX     GIMB_YAW_TORQUE_MAX

// Pitch 轴
#define GIMB_PITCH_AG_KP        10.0f
#define GIMB_PITCH_AG_KI        0.0f
#define GIMB_PITCH_AG_KD        0.0f
#define GIMB_PITCH_AG_I_MAX     10.0f
#define GIMB_PITCH_AG_OUT_MAX   GIMB_PITCH_SPEED_MAX

#define GIMB_PITCH_SP_KP        1.5f
#define GIMB_PITCH_SP_KI        0.01f
#define GIMB_PITCH_SP_KD        0.0f
#define GIMB_PITCH_SP_I_MAX     10.0f
#define GIMB_PITCH_SP_OUT_MAX   GIMB_PITCH_TORQUE_MAX

/* 控制周期（s），与 control_task 的 osDelay 保持一致 -----------------------*/
#define GIMB_CONTROL_DT         0.001f

/* Pitch 重力前馈补偿（余弦拟合）---------------------------------------------*/
/* 头部重心不在 pitch 转轴，改为预先标定"各角度维持静止所需力矩"，用
 *     torque_ff = K * cos(θ - θc) + B
 * 拟合（θ 为 pitch 当前单圈角 motor_angle，θc 为重力最大的重心位置角），
 * 运行时作为前馈叠加到速度环（力矩）输出
 */
#define GIMB_PITCH_GRAV_K          4.2072f   // 重力矩幅值 K
#define GIMB_PITCH_GRAV_B         -2.496f    // 零点偏置 B
#define GIMB_PITCH_GRAV_CENTER     0.0856f   // 重心位置角 θc(rad)

/* Pitch 上电回中目标角（相对零位的"水平"角度）-----------------------------*/
#define GIMB_PITCH_LEVEL_ANGLE     0.0f

/* Yaw 上电回中目标角（相对零位的“中位”角度）------------------------------- */
#define GIMB_YAW_CENTER_ANGLE      0.0f

/* 云台零位偏移（rad）：把电机单圈角 motor_angle 偏移到"相对机械水平/中位的控制零位"
 *     真实控制角 = motor_angle - 零位偏移
 * 使pitch"水平"恒为控制角 0、"yaw 中位"恒为控制角 0。
 *
 * 若换电机 / 重刷 zero_position 导致编码器零点变化，再改此处，
 * 或调试期摆好姿态后调一次 Gimbal_Calibrate_Zero(idx) 即时写入 */
#define GIMB_PITCH_ZERO_OFFSET     2.59309077f   
#define GIMB_YAW_ZERO_OFFSET      -0.387884378f  

#endif
