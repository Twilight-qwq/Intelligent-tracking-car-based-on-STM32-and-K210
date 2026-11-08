#ifndef __PID_H
#define __PID_H
#include <stdint.h>


/* ==================== 速度环（PI） ==================== */
typedef struct
{
	float Kp;				//比例项系数
	float Ki;				//积分项系数
	float err;				//本次误差
	float err_integral;		//误差积分累加值
	float output;			//本次输出
	float output_max;		//输出上限（‰），取值应与 Motor.h 的 MOTOR_DUTY_FULLSCALE 一致
	float output_min;		//输出下限（‰，传负值）
}pid_velocity_t;

void PID_VelocityLoop_Init(pid_velocity_t *pid_v , float Kp , float Ki , float output_max , float output_min);
float PID_V_Calc(pid_velocity_t *pid_v , float actual , float target);





/* ==================== 循迹环（PD） ==================== */
typedef struct
{
	float Kp;				//比例项系数
	float Kd;				//微分项系数
	float err;				//本次偏差
	float err_last;			//上次偏差
	float output;			//本次输出
	float output_max;		//输出上限
	float output_min;		//输出下限
	uint8_t isFirstRun;		//首周期标志：屏蔽启动瞬间的D项冲击
}pid_line_t;

void PID_LineLoop_Init(pid_line_t *pid_l , float Kp , float Kd , float output_max , float output_min);
float PID_L_Calc(pid_line_t *pid_l , float err);

#endif
