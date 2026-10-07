# 27_ZXR_up（上板，hole_infantry_up 分支）长期项目笔记

## 硬件：达妙 DM-J4310-2EC V1.1（云台 Yaw/Pitch 均为该型号）
摘自使用说明书 V1.2，这些参数决定所有限幅取值：
- 减速比 **10:1**；编码器 14 位单圈磁编；默认 CAN 波特率 1 Mbps
- 额定扭矩 **3 N·m** / 峰值 **7 N·m**；额定转速 120 rpm；空载最大 200 rpm = **20.94 rad/s**（输出轴）
- 反馈的位置/速度/扭矩全部是**输出轴侧**，单位 rad / (rad/s) / N·m；位置限定 [-π, π]
- 控制帧 ID = ESC_ID（电机 ID）；**反馈帧 ID = MST_ID，由调试助手设置，默认 0**
  （不是 0x10+ID；真车的 0x11/0x12 只是那边的 MST_ID 配置）
- 反馈为**问询式**：必须周期性下发控制帧，否则无反馈且会触发「通讯丢失防护」退出使能
- MIT 模式 Kp=0 且 Kd=0 → 扭矩只由 t_ff 决定（纯扭矩模式）
- 手册警告：使用内部位置控制时 Kd 不可为 0，否则振荡失控

## CAN 分配（照真车 hole_infantry_up）
- Pitch：ESC_ID 0x001，CAN1；反馈 MST_ID 0x11
- Yaw：ESC_ID 0x002，CAN2；反馈 MST_ID 0x12
- 两条 CAN 均已初始化（CAN1 PD0/PD1、CAN2 PB5/PB6，滤波器组 0 与 14，不过滤）

## 云台控制架构（已确定）
遥控器 → 目标角度积分 → **角度环**(PID, 输出 rad/s) → **速度环**(PID, 输出 N·m)
→ `tx_info->torque` → `single_set_torque()` → CAN。
所有可调参数集中在 `Application/ConfigLayer/config_gimbal.h`。
- 机械模式：角度环反馈用 `Motor_DM_rx_info.motor_angle_sum`（多圈累计）
- 陀螺仪模式（任务18/19）：角度环反馈换成 IMU 的 `base_info.yaw_total_angle`，架构不变
- 速度环反馈统一用 `rx_info->speed`（输出轴 rad/s）

## 工程约定 / 踩过的坑
- `single_pid_ctrl()` **不计算 dout**，调用前必须自行算 `pid->dout = kd*(err-last_err)`
- `DM_Single_Motor_Set_Torque()` 末尾会把 `torque` 清零，每拍都要重新赋值再调用
- `DEV_ONLINE = 0`、`DEV_OFFLINE = 1`，结构体零初始化后初始状态是 ONLINE，掉线状态由心跳 100ms 后纠正
- `Angle_Sum_Cal` 的 `order_correction` 未赋值(=0)时按 1 处理；转向相反要显式设 -1
- `DM_Motor.h` 的 P/V/T/KP/KD 范围必须与达妙调试助手里的设置一致，否则 float↔uint 换算比例错位
- M3508 正确常数：扭矩常数 **0.3**，减速比 **3591/187**（旧代码曾误写 0.246 与 268/17）

## 编译核查（本机无 Keil/ARMCC）
用主机 gcc `-fsyntax-only` + `cmsis_os.h`/`string.h`/`errno.h` 三个 stub 头，
并 `-include math.h`。详见 2026-10-05.md。
