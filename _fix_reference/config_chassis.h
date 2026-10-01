#ifndef __CONFIG_CHASSIS_H
#define __CONFIG_CHASSIS_H

#define OFF_GROUND_TEST 0

#define TIME_STEP			0.001f//任务运行周期，单位：s

#define WHEEL_RADIUS  0.0515f//驱动轮半径，单位：m

/* 本车电机输出轴直连轮毂，外部没有减速箱，运动学里不要再用这个宏。
 * 3508 内置的 19:1 减速已经在 RM_motor.c 的 RPM_to_Rads() 里除掉了，
 * 所以 rx_info->speed 的单位就是电机输出轴 rad/s，与轮毂角速度同一把尺子。 */
#define WHEEL_REDUCT_RATIO        (1.f/1.f)//【已弃用，保留仅为兼容旧代码】

//整车长度
#define Car_Length 0.338f
//整车宽度
#define Car_Width 0.3f
//整车旋转半径
#define Rl 0.2259668f
//整车旋转夹角（0~PI/2)
#define theta_Rl 0.7259073169f

/* ===== 麦轮逆解算真正要用的旋转系数 =====
 * 注意：不是轮半径，也不是 Rl（中心到轮子的距离），
 * 而是 半轴距 + 半轮距 = Car_Length/2 + Car_Width/2 = 0.319 m
 * （Rl 是 sqrt(a^2+b^2)=0.226，两者不是一回事，别混用）
 */
#define ROTATE_R            ((Car_Length + Car_Width) / 2.0f)   // 0.319 m
#define CHASSIS_HALF_LENGTH (Car_Length / 2.0f)                 // a = 0.169 m
#define CHASSIS_HALF_WIDTH  (Car_Width  / 2.0f)                 // b = 0.150 m
//车体运动最大速度
#define MAX_SPEED           1.2f    //单位：m/s
//车体转向运动最大速度
#define MAX_SPIN_SPEED           2.5f    //单位：rad/s
//车体中心离地高度
#define GRAVITY_HIGHT           0.18f    //单位：m
//全车重量
#define CAR_GRAVITY           196.f    //单位：N

/*电机方向与归位相关*/
//舵向电机的零点
#define L_F_ZeroPoint    32768
#define L_B_ZeroPoint    32768
#define R_B_ZeroPoint    32768
#define R_F_ZeroPoint    32768

//航向电机正方向
// 【这四个就是四个轮电机的极性，Chassis.c 里直接拿去用】
// 含义：给该路电机一个正转矩时，若轮胎上表面朝车头方向转 → +1，朝车尾 → -1
// 左右镜像安装时通常就是下面这组（左 +1、右 -1）
// 台架单轮测试确认之前，不要假定它一定正确
#define L_F_Direction   1
#define R_F_Direction   -1
#define R_B_Direction   -1
#define L_B_Direction   1

/* 遥控摇杆死区，绝对值小于该值视为 0，防止零点漂移导致蠕行 */
#define RC_DEADZONE      30      // DT7 满量程 660，30 约等于 4.5%

/*卸力阻尼时间与阻尼系数*/
#define DAMPING_DELAY_MAX_CNT     3000   //阻尼持续时间2.5s
#define Wheel_Damping_Coefficient 0.005f //
#define Sd_Damping_Coefficient    0.002f

#define SD_POS_FIX_TOR_K			(30.f)   //关节限位力矩补偿系数 10度1N

#define DISTANCE_ERR_MAX      0.55f


#endif
