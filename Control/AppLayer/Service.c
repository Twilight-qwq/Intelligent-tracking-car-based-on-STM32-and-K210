#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "Service.h"

#define SPEED_STRAIGHTAWAY			40.0f//直道基速
#define SPEED_SMALLANGLE			25.0f//小角度(直角,钝角)基速
#define SPEED_LARGEANGLE			15.0f//大角度(锐角)基速

static uint32_t task_last_5ms = 0;
static uint32_t task_last_10ms = 0;
static uint32_t task_last_30ms = 0;
static uint32_t task_last_100ms = 0;

static pid_velocity_t pid_v_left;
static pid_velocity_t pid_v_right;
static pid_line_t pid_l;

static uint8_t CarRun_State = 0;
static float target_left = 0.0f;
static float target_right = 0.0f;


void Service_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Encoder_Init();
	Motor_Init();
	Key_Init();//后面删掉
	Voice_Init();
	BlueTooth_Init();
	Debug_Init();
	k210rcv_Init();
	SysTimer_Init();
	PID_VelocityLoop_Init(&pid_v_left,0,0,(float)MOTOR_DUTY_FULLSCALE,-(float)MOTOR_DUTY_FULLSCALE);//后面再调参
	PID_VelocityLoop_Init(&pid_v_right,0,0,(float)MOTOR_DUTY_FULLSCALE,-(float)MOTOR_DUTY_FULLSCALE);//后面再调参
	PID_LineLoop_Init(&pid_l,0,0,SPEED_STRAIGHTAWAY,-(float)SPEED_STRAIGHTAWAY);//后面再调参
	WatchDog_Init();
}

uint32_t Service_Clock(void)
{
	return SysTimer_GetTick();
}


void ServiceTask_Idle(void)
{
	WatchDog_FeedDog();
}


void ServiceTask_5ms(uint32_t curr_ms)//内环速度环控制
{
	if(curr_ms - task_last_5ms < 5) return;
    task_last_5ms = curr_ms; // 够时间了，重置闹钟

	Encoder_Update();
	if(CarRun_State == 0) return;


	float actual_left = (float)Encoder_GetDeltaLeft();
	float actual_right = (float)Encoder_GetDeltaRight();


	float pwm_left = PID_V_Calc(&pid_v_left,actual_left,target_left);
	float pwm_right = PID_V_Calc(&pid_v_right,actual_right,target_right);

	Motor_SetDuty((int16_t)pwm_left,(int16_t)pwm_right);
}



void ServiceTask_30ms(uint32_t curr_ms)//外环方向环控制
{
	if(curr_ms - task_last_30ms < 30) return;
    task_last_30ms = curr_ms;

	if(CarRun_State == 0) return;//关机结束

	static uint8_t lostdata_time = 0; 
	if(g_data_ready==0)//无新数据结束
	{
		lostdata_time++;
		if(lostdata_time > 3)
		{
			CarRun_State = 0;//防止摄像头出问题了小车还在跑
			Motor_SetDuty(0,0);
			lostdata_time = 0;
		}
		return;
	}
	g_data_ready = 0;
	lostdata_time = 0;

	float error = (float)g_target_error;
	if(fabsf(error)<=3)
	{
		error = 0.0f;//忽略细微抖动
	}


	float turn = PID_L_Calc(&pid_l,error);//差速大小
	float base_speed = SPEED_STRAIGHTAWAY;


	switch (g_car_state)
	{
	case 1://直角弯或钝角弯
		base_speed = SPEED_SMALLANGLE;
		break;
	case 2:
		base_speed = SPEED_LARGEANGLE;
		break;
	case 3:
		Motor_SetDuty(0, 0);
		CarRun_State = 0;
		return;
	}

	target_left = base_speed + turn;
	if (target_left  < 0) target_left = 0;

	target_right = base_speed - turn;
	if (target_right < 0) target_right = 0;
}



void ServiceTask_10ms(uint32_t curr_ms)
{
	if(curr_ms - task_last_10ms < 10) return;
    task_last_10ms = curr_ms;
    Key_CheckRequest(curr_ms); 
}



void ServiceTask_100ms(uint32_t curr_ms)//开关控制
{
	if(curr_ms - task_last_100ms < 100) return;
    task_last_100ms = curr_ms;

	if(Key_GetRequest() == 1)
	{
		CarRun_State = !CarRun_State;	
	}

	/*语音模块启动与停车,等确认车能跑了再把按键操控换语音操控
	uint8_t voice_cmd = Voice_GetCommand();
    if (voice_cmd == 0x01) // 假设 0x01 是“启动”
    {
        CarRun_State = 1;
        // printf("Voice Start\r\n");
    }
    else if (voice_cmd == 0x02) // 假设 0x02 是“停车”
    {
        CarRun_State = 0;
        // printf("Voice Stop\r\n");
    }
	*/


	if(CarRun_State == 0)
	{
		Motor_SetDuty(0,0);
		target_left = 0;
		target_right = 0;
		pid_v_left.err_integral = 0;
		pid_v_left.err = 0;
		pid_v_right.err_integral = 0;
		pid_v_right.err = 0;
		pid_l.err_last = 0;
		pid_l.err = 0;
	}
	else if(CarRun_State == 1)//调试使用,使用完删掉!
	{
		Debug_log(g_target_error, g_car_state, target_left, target_right, CarRun_State);
	}
}
