/**
  ******************************************************************************
  * @file    gimbal.c
  * @brief   云台模块实现
  *
  *  达妙 DM-J4310-2EC V1.1（10:1 减速）角度环 -> 速度环 串级 PID，
  *  最终只输出力矩(N·m)，走 MIT 模式 Kp=Kd=0 的纯扭矩通道下发。
  *
  *  遥控器 -> 目标角度积分 -> 角度环(PID,输出 rad/s) -> 速度环(PID,输出 N·m)
  *          -> tx_info->torque -> single_set_torque() -> CAN
  *
  ******************************************************************************
  */
#include "gimbal.h"
#include "rp_math.h"
#include <math.h>

#ifndef M_PI
#define GIMB_PI 3.141592653589793f
#else
#define GIMB_PI ((float)M_PI)
#endif

/* Exported variables --------------------------------------------------------*/
gimbal_axis_t g_gimbal[GIMB_AXIS_CNT];
gimbal_mode_t g_gimbal_mode = GIMB_MODE_SLEEP;   // 上电默认睡眠	


/* Private variables ---------------------------------------------------------*/

/* 每轴的可配置量：遥控角速率、目标角上下限、力矩限幅 */
typedef struct
{
	float rc_rate;
	float ang_min;      /* 目标角下限(rad)，仅Pitch用 */
	float ang_max;      /* 目标角上限(rad)，仅Pitch用*/
	float torque_max;
} gimbal_axis_conf_t;

static const gimbal_axis_conf_t g_axis_conf[GIMB_AXIS_CNT] = {
	[GIMB_YAW]   = { GIMB_YAW_RC_RATE,   -GIMB_YAW_ANGLE_MAX,   GIMB_YAW_ANGLE_MAX,   GIMB_YAW_TORQUE_MAX   },
	[GIMB_PITCH] = { GIMB_PITCH_RC_RATE, GIMB_PITCH_TARGET_MIN, GIMB_PITCH_TARGET_MAX, GIMB_PITCH_TORQUE_MAX },
};

/* 零位偏移
 * 真实控制角 = motor_angle(单圈) - 该偏移，使水平/中位恒为 0 */
static float g_axis_zero_offset[GIMB_AXIS_CNT] = {
	[GIMB_YAW]   = GIMB_YAW_ZERO_OFFSET,
	[GIMB_PITCH] = GIMB_PITCH_ZERO_OFFSET,
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  取轴的真实控制角（已扣除零位偏移）
  *         用单圈 motor_angle
  */
static float Gimbal_Real_Angle(const gimbal_axis_t *axis)
{
	gimbal_axis_e idx = (gimbal_axis_e)(axis - &g_gimbal[0]);
	/* 单圈方案：两轴统一用 motor_angle（[-π,π]） */
	
	return axis->motor->rx_info->motor_angle - g_axis_zero_offset[idx];
}

/**
  * @brief  绑定电机 + 装载 PID 上电默认值
  */
static void Gimbal_Axis_Init(gimbal_axis_t *axis, Motor_DM_t *motor,
                             float ag_kp, float ag_ki, float ag_kd, float ag_i_max, float ag_out_max,
                             float sp_kp, float sp_ki, float sp_kd, float sp_i_max, float sp_out_max)
{
	axis->motor        = motor;
	axis->target_angle = 0.f;
	axis->inited       = 0;

	axis->ag_pid.kp = ag_kp;
	axis->ag_pid.ki = ag_ki;
	axis->ag_pid.kd = ag_kd;
	axis->ag_pid.integral_max = ag_i_max;
	axis->ag_pid.out_max      = ag_out_max;

	axis->sp_pid.kp = sp_kp;
	axis->sp_pid.ki = sp_ki;
	axis->sp_pid.kd = sp_kd;
	axis->sp_pid.integral_max = sp_i_max;
	axis->sp_pid.out_max      = sp_out_max;
}

/**
  * @brief  带死区的摇杆归一化，返回 [-1, 1]
  */
static float RC_Normalize(int16_t ch)
{
	if (abs(ch) < GIMB_RC_DEADBAND)
	{
		return 0.f;
	}
	return (float)ch / RC_CH_VALUE_RANGE;
}

/**
  * @brief  目标角度累加
  */
static void Gimbal_Target_Update(gimbal_axis_t *axis, float rc_norm, float rate, float ang_min, float ang_max)
{
	/* 首次运行或电机离线后恢复时，让目标角度对齐实际角度，避免回差导致暴冲 */
	if (!axis->inited || axis->motor->state->status == DEV_OFFLINE)
	{
		axis->target_angle = Gimbal_Real_Angle(axis);
		axis->inited       = 1;
		return;
	}

	axis->target_angle += rc_norm * rate * GIMB_CONTROL_DT;

	/* Yaw 轴可无限转：目标角折回 [-π, π)，配合 err-wrap 持续单向旋转。
	   Pitch 轴不进此分支，走 constrain 软限位。 */
	if (axis == &g_gimbal[GIMB_YAW])
	{
		if (axis->target_angle >  (float)GIMB_PI) axis->target_angle -= 2.0f * GIMB_PI;
		if (axis->target_angle < -(float)GIMB_PI) axis->target_angle += 2.0f * GIMB_PI;
		return;
	}

	axis->target_angle  = constrain(axis->target_angle, ang_min, ang_max);
}

/**
  * @brief  单轴串级计算，返回力矩(N·m)
  */
static float Gimbal_Axis_Calc(gimbal_axis_t *axis)
{
	pid_ctrl_t *ag = &axis->ag_pid;
	pid_ctrl_t *sp = &axis->sp_pid;

	/* ---- 外环：角度环 给定目标角度，反馈单圈角，输出目标转速 ---- */
	ag->target  = axis->target_angle;
	ag->measure = Gimbal_Real_Angle(axis);
	ag->err     = ag->target - ag->measure;
	/* 单圈角在 ±π 不连续：误差取最短路径，避免跨边界算成≈2π 反向暴冲
	 * Yaw 轴无限转采用单圈方案，feedback 与 target 均在 [-π,π]，故进行处理*/

	{
		if      (ag->err >  (float)GIMB_PI) ag->err -= 2.0f * GIMB_PI;
		else if (ag->err < -(float)GIMB_PI) ag->err += 2.0f * GIMB_PI;
	}
	ag->dout    = ag->kd * (ag->err - ag->last_err);
	single_pid_ctrl(ag);

	/* ---- 内环：速度环 给定目标转速，反馈输出轴转速，输出力矩 ---- */
	sp->target  = ag->out;
	sp->measure = axis->motor->rx_info->speed;
	sp->err     = sp->target - sp->measure;
	sp->dout    = sp->kd * (sp->err - sp->last_err);
	single_pid_ctrl(sp);

	return sp->out;
}

/**
  * @brief  安全卸力：清积分、对齐目标角、输出零力矩
  */
static void Gimbal_Safe(void)
{
	gimbal_axis_t *axis;
	uint8_t i;

	for (i = 0; i < GIMB_AXIS_CNT; i++)
	{
		axis = &g_gimbal[i];

		if (axis->motor == NULL || axis->motor->single_set_torque == NULL)
		{
			continue;
		}

		/* 清积分 防止恢复控制时积分项还在往外顶 */
		pid_clear(&axis->ag_pid);
		pid_clear(&axis->sp_pid);

		/* 目标角钉在反馈值上，清掉inited，
		   保证下次进入 NORMAL 时重新对齐，不会因回差暴冲 */
		axis->target_angle = Gimbal_Real_Angle(axis);
		axis->inited       = 0;

		axis->motor->tx_info->torque = 0.f;
		axis->motor->single_set_torque(axis->motor);
	}
}

/**
  * @brief  Pitch 重力前馈声明
  */
static float Gimbal_Pitch_Gravity_FF(gimbal_axis_t *axis);

/**
  * @brief  单轴总入口：串级 + 限幅 + 下发
  */
static void Gimbal_Axis_Ctrl(gimbal_axis_e idx, float rc_norm)
{
	gimbal_axis_t *axis = &g_gimbal[idx];
	const gimbal_axis_conf_t *conf = &g_axis_conf[idx];
	float torque = 0.f;

	if (axis->motor == NULL || axis->motor->single_set_torque == NULL)
	{
		return;
	}

	Gimbal_Target_Update(axis, rc_norm, conf->rc_rate, conf->ang_min, conf->ang_max);
	torque = Gimbal_Axis_Calc(axis);

	/* Pitch 叠加重力前馈*/
	if (idx == GIMB_PITCH)
	{
		torque += Gimbal_Pitch_Gravity_FF(axis);
	}

	/* MIT 模式 Kp=Kd=0 时扭矩仅由 t_ff 决定；single_set_torque 内部会清零 Kp/Kd */
	axis->motor->tx_info->torque = constrain(torque, -conf->torque_max, conf->torque_max);
	axis->motor->single_set_torque(axis->motor);
}

/**
  * @brief  Pitch 重力前馈
  *
  *  头部重心不在 pitch 转轴，改为预先标定"各角度维持静止所需力矩"，用
  *      torque_ff = K * cos(θ - θc) + B
  *  拟合（θ 为 pitch 当前单圈角 motor_angle，θc 为重力最大的重心位置角），
  *  运行时作为前馈叠加到速度环（力矩）输出，任意角度都能提前顶住重力，
  *  不会"初始下坠"，也不依赖积分项慢慢累积
  */
static float Gimbal_Pitch_Gravity_FF(gimbal_axis_t *axis)
{
    float d = Gimbal_Real_Angle(axis) - GIMB_PITCH_GRAV_CENTER;
    return GIMB_PITCH_GRAV_K * cosf(d) + GIMB_PITCH_GRAV_B;
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  零位标定：把指定轴"当前姿态"即时设为零位（真实角归零）
  *         调试用。例如：把云台摆水平后调一次 Gimbal_Calibrate_Zero(GIMB_PITCH)，
  *         等效于把当前 motor_angle 写入 GIMB_PITCH_ZERO_OFFSET
  */
void Gimbal_Calibrate_Zero(gimbal_axis_e idx)
{
	if (idx >= GIMB_AXIS_CNT)
	{
		return;
	}
	if (g_gimbal[idx].motor != NULL && g_gimbal[idx].motor->rx_info != NULL)
	{
		g_axis_zero_offset[idx] = g_gimbal[idx].motor->rx_info->motor_angle;
	}
}

/**
  * @brief  云台初始化，装载双轴 PID 参数
  */
void Gimbal_Init(void)
{
	Gimbal_Axis_Init(&g_gimbal[GIMB_YAW], &Yaw_Motor,
	                 GIMB_YAW_AG_KP,    GIMB_YAW_AG_KI,    GIMB_YAW_AG_KD,
	                 GIMB_YAW_AG_I_MAX, GIMB_YAW_AG_OUT_MAX,
	                 GIMB_YAW_SP_KP,    GIMB_YAW_SP_KI,    GIMB_YAW_SP_KD,
	                 GIMB_YAW_SP_I_MAX, GIMB_YAW_SP_OUT_MAX);

	Gimbal_Axis_Init(&g_gimbal[GIMB_PITCH], &Pitch_Motor,
	                 GIMB_PITCH_AG_KP,    GIMB_PITCH_AG_KI,    GIMB_PITCH_AG_KD,
	                 GIMB_PITCH_AG_I_MAX, GIMB_PITCH_AG_OUT_MAX,
	                 GIMB_PITCH_SP_KP,    GIMB_PITCH_SP_KI,    GIMB_PITCH_SP_KD,
	                 GIMB_PITCH_SP_I_MAX, GIMB_PITCH_SP_OUT_MAX);
}

/**
  * @brief  云台控制
  *         Pitch 用右摇杆上下 ch1
  *         Yaw  用右摇杆左右 ch0
  *
  *    左拨杆 s1 三档（RC_SW_UP=1 / RC_SW_MID=3 / RC_SW_DOWN=2）：
  *      上档 -> NORMAL 单独控制云台
  *      中档 -> MECH   机械模式：yaw 跟随底盘，pitch 正常受控
  *      下档 -> 卸力
  *
  *    关控保护：
  *    1. 遥控失联/任一电机掉线/拨杆未在受控档 -> SLEEP，清积分 + 卸力
  *    2. 仍处于SLEEP时                       -> 必须先把四个摇杆回中才恢复，防暴冲
  *    SLEEP 期间照样下发零力矩帧，保证反馈链路不断。
  */
void Gimbal_Ctrl(void)
{
	uint8_t rc_ok      = (rc_sensor.work_state == DEV_ONLINE);
	uint8_t motor_ok   = Gimbal_Is_Motor_Online();
	uint8_t sw_rc      = 0;    // 上档：单独控制云台
	uint8_t sw_mech    = 0;    // 中档：机械模式（yaw 跟随底盘）
	uint8_t sw_on      = 0;
	float   yaw_rc     = 0.f;  // 送给 yaw 的遥控量（机械模式保持为 0）
	gimbal_mode_t next_mode    = GIMB_MODE_SLEEP; /* 本帧应处的模式 */
	uint8_t      mode_changed  = 0;             /* 模式是否刚变化 */

	/* 档位识别 */
	sw_rc   = (rc_sensor_info.s1.value == (uint16_t)GIMB_ENABLE_S1_POS);
	sw_mech = (rc_sensor_info.s1.value == (uint16_t)GIMB_MECH_S1_POS);
	sw_on   = (sw_rc || sw_mech);

	/* -------- 安全检查：关控 / 电机掉线 / 拨杆未在受控档（上档或中档）-------- */
	if (!rc_ok || !motor_ok || !sw_on)
	{
		g_gimbal_mode = GIMB_MODE_SLEEP;
		Gimbal_Safe();
		return;
	}

	/* -------- 重开控防暴冲：从 SLEEP 恢复时必须先把四个摇杆回中 -------- */
	if (g_gimbal_mode == GIMB_MODE_SLEEP)
	{
#if GIMB_RC_RESUME_NEED_CENTER
		if (!RC_IsChannelReset())
		{
			Gimbal_Safe();
			return;
		}
#endif
	}

	/* -------- 档位 -> 模式 --------
	赋值前的 g_gimbal_mode 是"上一帧模式"，与之比较即得切档边沿 */
	next_mode     = sw_mech ? GIMB_MODE_MECH : GIMB_MODE_NORMAL;
	mode_changed  = (next_mode != g_gimbal_mode);   /* 含"从 SLEEP 恢复"的情形 */
	g_gimbal_mode = next_mode;

	/* -------- 归中：进入任一受控模式时都归中 -------- */
	if (mode_changed)
	{
		/* 切档/恢复瞬间清积分，避免过冲 */
		pid_clear(&g_gimbal[GIMB_PITCH].ag_pid);
		pid_clear(&g_gimbal[GIMB_PITCH].sp_pid);
		pid_clear(&g_gimbal[GIMB_YAW].ag_pid);
		pid_clear(&g_gimbal[GIMB_YAW].sp_pid);

		g_gimbal[GIMB_PITCH].target_angle = GIMB_PITCH_LEVEL_ANGLE;
		g_gimbal[GIMB_PITCH].inited       = 1;

		g_gimbal[GIMB_YAW].target_angle = GIMB_YAW_CENTER_ANGLE;
		g_gimbal[GIMB_YAW].inited       = 1;
	}

	/* -------- 正常控制 --------
	   机械模式(MECH)：yaw轴跟随底盘
	   单独控制云台(NORMAL)			*/
	if (g_gimbal_mode != GIMB_MODE_MECH)
	{
		yaw_rc = RC_Normalize((int16_t)(GIMB_YAW_RC_DIR * RC_RIGH_CH_LR_VALUE));
	}

	Gimbal_Axis_Ctrl(GIMB_YAW,   yaw_rc);
	Gimbal_Axis_Ctrl(GIMB_PITCH, RC_Normalize((int16_t)(GIMB_PITCH_RC_DIR * RC_RIGH_CH_UD_VALUE)));
}

/**
  * @brief  两轴电机是否都在线
  */
uint8_t Gimbal_Is_Motor_Online(void)
{
	uint8_t i;

	for (i = 0; i < GIMB_AXIS_CNT; i++)
	{
		if (g_gimbal[i].motor == NULL || g_gimbal[i].motor->state == NULL)
		{
			return 0;
		}
		if (g_gimbal[i].motor->state->status == DEV_OFFLINE)
		{
			return 0;
		}
	}
	return 1;
}

/**
  * @brief  当前模式
  */
gimbal_mode_t Gimbal_Get_Mode(void)
{
	return g_gimbal_mode;
}

/**
  * @brief  云台心跳，1ms 调用一次。超时 offline_cnt_max(默认100) 判定掉线
  */
void Gimbal_Heartbeat(void)
{
	uint8_t i;

	for (i = 0; i < GIMB_AXIS_CNT; i++)
	{
		if (g_gimbal[i].motor != NULL && g_gimbal[i].motor->single_heart_beat != NULL)
		{
			g_gimbal[i].motor->single_heart_beat(g_gimbal[i].motor);
		}
	}
}
