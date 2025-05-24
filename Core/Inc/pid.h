#pragma once
#include "main.h"
#define MAX_DUTY 1
enum {
	POSITION_PID = 0, // 位置式
	DELTA_PID,		  // 增量式
};

typedef struct {
	float target;
	float now;
	float error[3];
	float p, i, d;
	float pout, dout, iout;
	float out;
	uint32_t pid_mode;
} pid_t;
void pid_change_k(pid_t *pid, float new_kp, float new_ki, float new_kd);
void pid_cal(pid_t *pid);