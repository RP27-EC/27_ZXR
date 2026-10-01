#include "Chassis.h"
#include "rp_math.h"
#include "config_chassis.h"
#include "fdcan.h"

/* 底盘全局变量 */
chassis_t chassis;

/* ============================================================
 * 四个麦轮电机定义
 * ============================================================ */

/* -------- 左前轮 -------- */
Motor_RM_Born_Info_t LF_Born = {
    .order_correction = 0,
    .rxId = 0,
    .stdId = 0x200,
    .type = _3508_Reduction,
    .hcan = &hfdcan1,
};
Motor_RM_Rx_Info_t LF_Rxinfo;
Motor_RM_Tx_Info_t LF_Txinfo;
Motor_RM_State_t   LF_State;
pid_ctrl_t LF_SpeedPid = {
    .kp = 0.5f, .ki = 0.f, .kd = 0.f,
    .integral_max = 10.0f, .out_max = 3.0f,
};
Motor_RM_Ctrl_Info_t LF_Ctrl = { .speed_ctrl = &LF_SpeedPid };
Motor_RM_t LF_Motor = {
    .born_info = &LF_Born,
    .rx_info   = &LF_Rxinfo,
    .tx_info   = &LF_Txinfo,
    .state     = &LF_State,
    .ctrl      = &LF_Ctrl,
    .single_init = RM_Motor_Init,
};

/* -------- 左后轮 -------- */
Motor_RM_Born_Info_t LB_Born = {
    .order_correction = 0,
    .rxId = 1,
    .stdId = 0x200,
    .type = _3508_Reduction,
    .hcan = &hfdcan1,
};
Motor_RM_Rx_Info_t LB_Rxinfo;
Motor_RM_Tx_Info_t LB_Txinfo;
Motor_RM_State_t   LB_State;
pid_ctrl_t LB_SpeedPid = {
    .kp = 0.5f, .ki = 0.f, .kd = 0.f,
    .integral_max = 10.0f, .out_max = 3.0f,
};
Motor_RM_Ctrl_Info_t LB_Ctrl = { .speed_ctrl = &LB_SpeedPid };
Motor_RM_t LB_Motor = {
    .born_info = &LB_Born,
    .rx_info   = &LB_Rxinfo,
    .tx_info   = &LB_Txinfo,
    .state     = &LB_State,
    .ctrl      = &LB_Ctrl,
    .single_init = RM_Motor_Init,
};

/* -------- 右前轮 -------- */
Motor_RM_Born_Info_t RF_Born = {
    .order_correction = 0,
    .rxId = 2,
    .stdId = 0x200,
    .type = _3508_Reduction,
    .hcan = &hfdcan1,
};
Motor_RM_Rx_Info_t RF_Rxinfo;
Motor_RM_Tx_Info_t RF_Txinfo;
Motor_RM_State_t   RF_State;
pid_ctrl_t RF_SpeedPid = {
    .kp = 0.5f, .ki = 0.f, .kd = 0.f,
    .integral_max = 10.0f, .out_max = 3.0f,
};
Motor_RM_Ctrl_Info_t RF_Ctrl = { .speed_ctrl = &RF_SpeedPid };
Motor_RM_t RF_Motor = {
    .born_info = &RF_Born,
    .rx_info   = &RF_Rxinfo,
    .tx_info   = &RF_Txinfo,
    .state     = &RF_State,
    .ctrl      = &RF_Ctrl,
    .single_init = RM_Motor_Init,
};

/* -------- 右后轮 -------- */
Motor_RM_Born_Info_t RB_Born = {
    .order_correction = 0,
    .rxId = 3,
    .stdId = 0x200,
    .type = _3508_Reduction,
    .hcan = &hfdcan1,
};
Motor_RM_Rx_Info_t RB_Rxinfo;
Motor_RM_Tx_Info_t RB_Txinfo;
Motor_RM_State_t   RB_State;
pid_ctrl_t RB_SpeedPid = {
    .kp = 0.5f, .ki = 0.f, .kd = 0.f,
    .integral_max = 10.0f, .out_max = 3.0f,
};
Motor_RM_Ctrl_Info_t RB_Ctrl = { .speed_ctrl = &RB_SpeedPid };
Motor_RM_t RB_Motor = {
    .born_info = &RB_Born,
    .rx_info   = &RB_Rxinfo,
    .tx_info   = &RB_Txinfo,
    .state     = &RB_State,
    .ctrl      = &RB_Ctrl,
    .single_init = RM_Motor_Init,
};

/* -------- 四个麦轮打包成一个电机组 -------- */
Motor_RM_Group_t Wheel_Group = {
    .motor[0] = &LF_Motor,
    .motor[1] = &RF_Motor,
    .motor[2] = &LB_Motor,
    .motor[3] = &RB_Motor,
    .stdId = 0x200,
    .hcan = &hfdcan1,
    .group_init = RM_Group_Motor_Init,
};

/* ============================================================
 * 底盘初始化
 * ============================================================ */
void Chassis_Init(void)
{
    chassis.mode = CHASSIS_MODE_SLEEP;
    chassis.vx = 0.f;
    chassis.vy = 0.f;
    chassis.wz = 0.f;
    for (int i = 0; i < 4; i++) chassis.wheel_speed[i] = 0.f;

    /* 初始化电机 */
    Wheel_Group.group_init(&Wheel_Group);
}

/* ============================================================
 * 检查四个电机是否全部在线
 * ============================================================ */
static uint8_t Chassis_Is_Wheel_Online(void)
{
    for (int i = 0; i < 4; i++) {
        if (Wheel_Group.motor[i] == NULL) continue;
        if (Wheel_Group.motor[i]->state->status != DEV_ONLINE) return 0;
    }
    return 1;
}

/* ============================================================
 * 清除所有速度环 PID 积分（关控时调用）
 * ============================================================ */
static void Chassis_Clear_All_PID(void)
{
    for (int i = 0; i < 4; i++) {
        if (Wheel_Group.motor[i] != NULL && Wheel_Group.motor[i]->ctrl->speed_ctrl != NULL) {
            pid_clear(Wheel_Group.motor[i]->ctrl->speed_ctrl);
        }
    }
}

/* ============================================================
 * 麦轮逆解算：整车速度 → 四个轮子目标速度(从线速度转化为角速度)
 * ============================================================ */
static void Chassis_Inverse_Kinematics(void)
{
    float vx = chassis.vx;
    float vy = chassis.vy;
    float wz = chassis.wz;

    float max_abs = 0.f;   //四个轮里绝对值最大的
    float k = 1.f;         //缩放系数

    chassis.wheel_speed[0] =   (vx - vy - wz * ROTATE_R)/WHEEL_RADIUS;   // LF
    chassis.wheel_speed[1] =  -(vx + vy + wz * ROTATE_R)/WHEEL_RADIUS;   // RF
    chassis.wheel_speed[2] =   (vx + vy - wz * ROTATE_R)/WHEEL_RADIUS;   // LB
    chassis.wheel_speed[3] =  -(vx - vy + wz * ROTATE_R)/WHEEL_RADIUS;   // RB

    for (int i = 0; i < 4; i++) {                       //等比限幅，超过最大速度时按比例限制
        float a = (chassis.wheel_speed[i] >= 0.f)
                ?  chassis.wheel_speed[i]
                : -chassis.wheel_speed[i];
        if (a > max_abs) max_abs = a;
    }

    if (max_abs > MAX_WHEEL_SPEED_RAD) {
        k = MAX_WHEEL_SPEED_RAD / max_abs;
        for (int i = 0; i < 4; i++) {
            chassis.wheel_speed[i] *= k;
        }
    }


}
/* ============================================================
 * 底盘单步逻辑
 * ============================================================ */
void Chassis_Step(void)
{
    /* --------安全检查-------- */
    uint8_t rc_ok    = (rc_sensor.work_state == DEV_ONLINE);
    uint8_t motor_ok = Chassis_Is_Wheel_Online();

    if (!rc_ok || !motor_ok)
    {
        /* 进入安全模式：清PID + 卸力，防止重开控突然动 */
        if (chassis.mode != CHASSIS_MODE_SLEEP) {
            Chassis_Clear_All_PID();
        }
        chassis.mode = CHASSIS_MODE_SLEEP;
        Wheel_Group.group_sleep(&Wheel_Group);
        Wheel_Group.group_set_torque(&Wheel_Group);

        return;
    }

    /* --------根据遥控器开关决定模式 -------- */
    if (rc_sensor_info.s1 == RC_SW_MID) {       //左拨杆居中时为正常模式
        chassis.mode = CHASSIS_MODE_NORMAL;
    } else {
        chassis.mode = CHASSIS_MODE_SLEEP;
    }

    /* --------计算 vx, vy, wz -------- */
    if (chassis.mode == CHASSIS_MODE_NORMAL)
    {
        /* 遥控器摇杆归一化到 -1~1，再乘最大速度 */
        chassis.vx =   rc_sensor_info.ch3 / 660.f * MAX_SPEED;           //前后：左摇杆上下
        chassis.vy =  -rc_sensor_info.ch2 / 660.f * MAX_SPEED;           //左右：左摇杆左右  注意遥控器左方向为负值需要负号修正
        chassis.wz =  -rc_sensor_info.ch0 / 660.f * MAX_SPIN_SPEED;      //旋转：右摇杆左右  注意遥控器左方向为复制需要负号修正
    }
    else
    {
        chassis.vx = 0.f;
        chassis.vy = 0.f;
        chassis.wz = 0.f;
    }

    /* --------麦轮逆解算 -------- */
    Chassis_Inverse_Kinematics();

    /* --------每个轮子速度环 PID -------- */
    if (chassis.mode == CHASSIS_MODE_NORMAL)
    {
        for (int i = 0; i < 4; i++) {
            Motor_RM_t *m = Wheel_Group.motor[i];
            m->ctrl->speed_ctrl->target  = chassis.wheel_speed[i];
            m->ctrl->speed_ctrl->measure = m->rx_info->speed;
            pid_err_cal(m->ctrl->speed_ctrl);
            single_pid_ctrl(m->ctrl->speed_ctrl);
            m->tx_info->torque = m->ctrl->speed_ctrl->out;
        }
    }
    else
    {
        for (int i = 0; i < 4; i++) {
            Wheel_Group.motor[i]->tx_info->torque = 0.f;
        }
    }

    /* --------发送 CAN -------- */
    Wheel_Group.group_set_torque(&Wheel_Group);
}
