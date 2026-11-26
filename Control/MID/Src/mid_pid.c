#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "mid_pid.h"
/* ==================== 速度环 ==================== */
void PID_VelocityLoop_Init(pid_velocity_t *pid_v , float Kp , float Ki , float output_max , float output_min)
{
	pid_v->Kp = Kp;
	pid_v->Ki = Ki;
	pid_v->err = 0;
	pid_v->err_integral = 0;
	pid_v->output = 0;
	pid_v->output_max = output_max;
	pid_v->output_min = output_min;
}


/* 功能：速度环PI计算
 * 参数1：速度环结构体指针
 * 参数2：本周期实测编码器增量（counts/周期）
 * 参数3：本周期期望编码器增量（counts/周期）
 * 返回：本周期占空比指令（‰），范围[output_min, output_max]
 */
float PID_V_Calc(pid_velocity_t *pid_v , float actual , float target)
{
	//1.误差计算
	pid_v->err = target - actual;

	//2.条件积分：Ki为0时积分无意义；输出已饱和且误差同向时停止累加（抗积分饱和）
	if(pid_v->Ki != 0.0f)
	{
		//is Saturation High/Low是否处于高/低边饱和状态
		uint8_t isSatHigh = (pid_v->output >= pid_v->output_max) && (pid_v->err > 0.0f);//超高限&&正误差
		uint8_t isSatLow = (pid_v->output <= pid_v->output_min) && (pid_v->err < 0.0f);//超低限&&负误差
		if(isSatHigh==0 && isSatLow==0 && fabsf(pid_v->err)<=20)//都不饱和
		{
			pid_v->err_integral += pid_v->err;
		}
	}

	//3.积分限幅：让积分项对输出的“贡献”恰好覆盖整个输出范围，其余情况一律不动
	if(pid_v->Ki > 0.0f)//防止Ki为0时除法得到inf
	{
		float integral_max = pid_v->output_max / pid_v->Ki;
		float integral_min = pid_v->output_min / pid_v->Ki;
		if(pid_v->err_integral > integral_max)
		{
			pid_v->err_integral = integral_max;
		}
		if(pid_v->err_integral < integral_min)
		{
			pid_v->err_integral = integral_min;
		}
	}

	//4.PI控制输出
	pid_v->output = (pid_v->Kp)*(pid_v->err) + (pid_v->Ki)*(pid_v->err_integral);


	//5.输出限幅
	if(pid_v->output > pid_v->output_max)
	{
		pid_v->output = pid_v->output_max;
	}
	if(pid_v->output < pid_v->output_min)
	{
		pid_v->output = pid_v->output_min;
	}
	return pid_v->output;
}

/* ==================== 循迹环 ==================== */
void PID_LineLoop_Init(pid_line_t *pid_l , float Kp , float Kd , float output_max , float output_min)
{
	pid_l->Kp = Kp;
	pid_l->Kd = Kd;
	pid_l->err = 0;
	pid_l->err_diff = 0;
	pid_l->err_diff_last = 0;
	pid_l->output = 0;
	pid_l->output_max = output_max;
	pid_l->output_min = output_min;
	pid_l->isFirstRun = 1;
}

/* 微分限幅（像素/帧）。 */
#define LIMIT_LINE_DIFF_MAX   4.0f

/* 功能：循迹环PD计算
 * 参数1：循迹环结构体指针
 * 参数2：本周期偏差（给P项用，外面已经做过±3死区）
 * 参数3：本周期原始偏差（给D项用，没过死区）
 * 返回：本周期左右轮差速量（counts/周期），叠加到基础速度上
 */
float PID_L_Calc(pid_line_t *pid_l , float err , float err_diff)
{
	//1.滚动保存偏差；首周期把历史对齐到当前值，避免D项出现Kd*(偏差-0)的冲击
	if(pid_l->isFirstRun == 1)
	{
		pid_l->isFirstRun = 0;
		pid_l->err_diff_last = err_diff;
	}
	else
	{
		pid_l->err_diff_last = pid_l->err_diff;
	}
	pid_l->err = err;
	pid_l->err_diff = err_diff;

	//2.微分项：用原始偏差算变化量，并限幅
	float diff = pid_l->err_diff - pid_l->err_diff_last;
	if(diff >  LIMIT_LINE_DIFF_MAX) diff =  LIMIT_LINE_DIFF_MAX;
	if(diff < -LIMIT_LINE_DIFF_MAX) diff = -LIMIT_LINE_DIFF_MAX;

	//3.PD控制输出：P项跟随偏差，D项抑制偏差变化速度
	pid_l->output = (pid_l->Kp)*(pid_l->err) + (pid_l->Kd)*diff;

	//4.输出限幅
	if(pid_l->output > pid_l->output_max)
	{
		pid_l->output = pid_l->output_max;
	}
	if(pid_l->output < pid_l->output_min)
	{
		pid_l->output = pid_l->output_min;
	}
	return pid_l->output;
}


/* ==================== 转向角度环（PD） ==================== */
void PID_TurnLoop_Init(pid_turn_t *pid_t , float Kp , float Kd , float diff_max , float slew)
{
	pid_t->Kp = Kp;
	pid_t->Kd = Kd;
	pid_t->diff_max = diff_max;
	pid_t->slew = slew;
	pid_t->diff_prev = 0.0f;
	pid_t->err = 0.0f;
	pid_t->rate = 0.0f;
	pid_t->output = 0.0f;
}

/* 功能：每次开转前清一次，差速从 0 重新起步（不继承上一个弯的斜率状态）
 * 参数：pid_t 角度环结构体指针
 * 返回：无
 */
void PID_TurnLoop_Reset(pid_turn_t *pid_t)
{
	pid_t->diff_prev = 0.0f;
	pid_t->err = 0.0f;
	pid_t->rate = 0.0f;
	pid_t->output = 0.0f;
}

/* 功能：转向角度环 PD 计算
 * 参数1：pid_t 角度环结构体指针
 * 参数2：目标角（度，正数）
 * 参数3：当前已经转过的角（度，正数）
 * 参数4：当前角速度（度/秒，和转向同向为正）
 * 返回：本拍差速（counts/周期），正数 = 往当前转向方向转，负数 = 反向刹
 * 说明：D 项直接用陀螺仪的角速度，不用角度差分——角度差分噪声大得多
 */
float PID_TurnLoop_Calc(pid_turn_t *pid_t , float target_deg , float actual_deg , float rate_dps)
{
	float diff;

	pid_t->err  = target_deg - actual_deg;
	pid_t->rate = rate_dps;

	//1.PD：P 拉向目标，D 用角速度做阻尼
	diff = (pid_t->Kp) * (pid_t->err) - (pid_t->Kd) * (pid_t->rate);

	//2.输出限幅
	if(diff >  pid_t->diff_max) diff =  pid_t->diff_max;
	if(diff < -pid_t->diff_max) diff = -pid_t->diff_max;

	//3.斜率限幅：陀螺仪单拍跳变会让 err 突然变负，不加这个会一拍打满反向
	if(diff > (pid_t->diff_prev + pid_t->slew)) diff = pid_t->diff_prev + pid_t->slew;
	if(diff < (pid_t->diff_prev - pid_t->slew)) diff = pid_t->diff_prev - pid_t->slew;

	pid_t->diff_prev = diff;
	pid_t->output = diff;
	return diff;
}

