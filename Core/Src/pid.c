#include "pid.h"

void pid_change_k(pid_t *pid, float new_kp, float new_ki, float new_kd) {
	pid->p = new_kp;
	pid->i = new_ki;
	pid->d = new_kd;
}
void pid_cal(pid_t* pid) {
	// 计算当前偏差
	pid->error[0] = pid->target - pid->now;

	// 计算输出
	if (pid->pid_mode == DELTA_PID) { // 增量式
		pid->pout = pid->p * (pid->error[0] - pid->error[1]);
		pid->iout = pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
		pid->out += pid->pout + pid->iout + pid->dout;
	} else if (pid->pid_mode == POSITION_PID) { // 位置式
		pid->pout = pid->p * pid->error[0];
		pid->iout += pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - pid->error[1]);
		pid->out = pid->pout + pid->iout + pid->dout;
	}

	// 记录前两次偏差
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
}


