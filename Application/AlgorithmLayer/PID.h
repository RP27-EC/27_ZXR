#ifndef __PID_H
#define __PID_H
#include "main.h"


typedef struct pid_ctrl {
	float	target;
	float	measure;
	float 	err;
	float 	last_err;
	float	kp;
	float 	ki;
	float 	kd;
	float 	pout;
	float 	iout;
	float 	dout;
	float 	out;
	float  	last_dout;
	float	integral;
	float 	integral_max;
	float 	out_max;

} pid_ctrl_t;
void pid_clear(pid_ctrl_t *pid);       // 关控/离线时清积分，防止恢复时积分饱和
void single_pid_ctrl(pid_ctrl_t *pid);

#endif
