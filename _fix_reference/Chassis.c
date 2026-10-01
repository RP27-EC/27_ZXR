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
    .integral_max = 10.0f, .out_max = 2.0f,
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
    .integral_max = 10.0f, .out_max = 2.0f,
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
    .integral_max = 10.0f, .out_max = 2.0f,
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
    .integral_max = 10.0f, .out_max = 2.0f,
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
 * 四个轮的极性（电机 → 轮胎前进方向）
 * +1：给正转矩时，轮胎上表面朝车头方向转
 * -1：给正转矩时，轮胎上表面朝车尾方向转
 * 这里直接引用 config_chassis.h 里那四个宏，实测不对就改那四个宏
 * ============================================================ */
static const float wheel_dir[4] = {
    L_F_Direction,   // [0] LF
    R_F_Direction,   // [1] RF
    L_B_Direction,   // [2] LB
    R_B_Direction,   // [3] RB
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
 * 麦轮逆解算：整车速度 → 四个轮毂的目标角速度(rad/s)
 *
 * 约定：+ 表示该轮“上表面朝车头方向”转（即让车前进的转向）
 * 适用于 X 型装配（LF/RB 同型号轮，RF/LB 同型号轮）
 *
 *   标准 X 型公式（已查证）：
 *     w_LF = (1/r)·( vx - vy - (a+b)·wz )
 *     w_RF = (1/r)·( vx + vy + (a+b)·wz )
 *     w_LB = (1/r)·( vx + vy - (a+b)·wz )
 *     w_RB = (1/r)·( vx - vy + (a+b)·wz )
 *
 * 两个易错点：
 *   1. 旋转臂是 (a+b)=0.319m，不是轮半径 0.0515m，也不是 Rl=0.226m
 *   2. 这里算出来的是“轮毂该往哪个方向滚”，
 *      电机的极性在下面给 PID 目标值时才乘 wheel_dir[i]，不要在这里混着写
 * ============================================================ */
static void Chassis_Inverse_Kinematics(void)
{
    float vx = chassis.vx;
    float vy = chassis.vy;
    float wz = chassis.wz;

    float raw[4];
    raw[0] =  (vx - vy - wz * ROTATE_R) / WHEEL_RADIUS;   // LF
    raw[1] =  (vx + vy + wz * ROTATE_R) / WHEEL_RADIUS;   // RF
    raw[2] =  (vx + vy - wz * ROTATE_R) / WHEEL_RADIUS;   // LB
    raw[3] =  (vx - vy + wz * ROTATE_R) / WHEEL_RADIUS;   // RB

    /* 整体等比限幅：饱和时四个轮按同一比例缩小，保证合成方向不变。
     * 千万不要逐轮各自截断，那会把方向解算的结果彻底搞歪。 */
    float max_abs = 0.f;
    for (int i = 0; i < 4; i++) {
        float a = (raw[i] >= 0.f) ? raw[i] : -raw[i];
        if (a > max_abs) max_abs = a;
    }
    float k = (max_abs > MAX_WHEEL_SPEED_RAD) ? (MAX_WHEEL_SPEED_RAD / max_abs) : 1.f;

    for (int i = 0; i < 4; i++) {
        chassis.wheel_speed[i] = raw[i] * k;
    }
}
/* ============================================================
 * 底盘单步逻辑
 * ============================================================ */
void Chassis_Step(void)
{
#if OFF_GROUND_TEST
    /* ============ 台架自检模式（config_chassis.h 里置 1 才生效）============
     * 跳过运动学，只给 TEST_INDEX 那一路电机打一个固定小转矩，其余为 0。
     * 用法：抬车离地，把 TEST_INDEX 依次改成 0/1/2/3 各烧一次，记录两件事：
     *   1. 实际转动的是哪个角的轮子  → 确定 CAN ID 与物理角位的映射
     *   2. 轮胎上表面朝车头还是朝车尾 → 确定这一路的极性(+1 / -1)
     * 记下后去 config_chassis.h 填 L_F_Direction 那四个宏，再把本宏改回 0。
     * ==================================================================== */
    #define TEST_INDEX   0       /* 0=LF 1=RF 2=LB 3=RB，逐个测 */
    #define TEST_TORQUE  0.5f    /* N·m，约 2A，很轻，安全 */

    for (int i = 0; i < 4; i++) {
        if (Wheel_Group.motor[i] != NULL) {
            Wheel_Group.motor[i]->tx_info->torque =
                (i == TEST_INDEX) ? TEST_TORQUE : 0.f;
        }
    }
    Wheel_Group.group_set_torque(&Wheel_Group);
    return;
#endif

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

        /* 注意：Group_Motor_Sleep() 只把 torque 置 0，并不会发报文，
         * 必须再调一次 group_set_torque() 把 0 真正发下去，
         * 否则电调会一直保持上一条电流指令。 */
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
        int16_t ch_x  = rc_sensor_info.ch2;   //前后：左摇杆上下
        int16_t ch_y  = rc_sensor_info.ch3;   //左右：左摇杆左右
        int16_t ch_w  = rc_sensor_info.ch0;   //旋转：右摇杆左右

        /* 第一步：钳位到 ±660。DT7 上电瞬间可能吐出一组超阈值的随机数据，
         * 不钳的话 vx/vy/wz 会瞬间超过 MAX_SPEED。 */
        if (ch_x >  660) ch_x =  660;
        if (ch_x < -660) ch_x = -660;
        if (ch_y >  660) ch_y =  660;
        if (ch_y < -660) ch_y = -660;
        if (ch_w >  660) ch_w =  660;
        if (ch_w < -660) ch_w = -660;

        /* 第二步：死区，抑制摇杆零点漂移造成的蠕行 */
        if ((ch_x > -RC_DEADZONE) && (ch_x < RC_DEADZONE)) ch_x = 0;
        if ((ch_y > -RC_DEADZONE) && (ch_y < RC_DEADZONE)) ch_y = 0;
        if ((ch_w > -RC_DEADZONE) && (ch_w < RC_DEADZONE)) ch_w = 0;

        chassis.vx =  ch_x / 660.f * MAX_SPEED;
        chassis.vy =  ch_y / 660.f * MAX_SPEED;
        chassis.wz =  ch_w / 660.f * MAX_SPIN_SPEED;
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

            /* 极性只乘在 target 上！
             * measure 保持电机反馈原值，绝不能一起乘 ——
             * 否则误差会被二次取反，速度环变成正反馈，轮子会直接飞车。
             * 推导：target = D·S，measure = fb，
             *       err = D·S - fb = D·(S - D·fb)，方向始终正确。 */
            m->ctrl->speed_ctrl->target  = chassis.wheel_speed[i] * wheel_dir[i];
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
