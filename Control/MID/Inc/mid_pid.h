#ifndef __MID_PID_H
#define __MID_PID_H
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
	float err;				//本次偏差（P项用，已过死区）
	float err_diff;			//本次原始偏差（D项用，没过死区）
	float err_diff_last;	//上次原始偏差（D项用）
	float output;			//本次输出
	float output_max;		//输出上限
	float output_min;		//输出下限
	uint8_t isFirstRun;		//首周期标志：屏蔽启动瞬间的D项冲击
}pid_line_t;

void PID_LineLoop_Init(pid_line_t *pid_l , float Kp , float Kd , float output_max , float output_min);
float PID_L_Calc(pid_line_t *pid_l , float err , float err_diff);


/* ==================== 转向角度环（PD，外环，输出差速） ==================== */
typedef struct
{
	float Kp;				//比例项：还差 1 度给多少差速（单位同 TURN_VALUE）
	float Kd;				//微分项：每 1 度/秒角速度压掉多少差速
	float diff_max;			//差速上限（正数），输出夹在 ±diff_max
	float slew;				//斜率限幅：每拍最多变化多少，防单拍跳变打满反向
	float diff_prev;		//上一拍输出的差速，斜率限幅用
	float err;				//本次误差（度，正数=还差这么多）
	float rate;				//本次角速度（度/秒，和转向同向为正）
	float output;			//本次输出差速
}pid_turn_t;

void PID_TurnLoop_Init(pid_turn_t *pid_t , float Kp , float Kd , float diff_max , float slew);
void PID_TurnLoop_Reset(pid_turn_t *pid_t);
float PID_TurnLoop_Calc(pid_turn_t *pid_t , float target_deg , float actual_deg , float rate_dps);

#endif
