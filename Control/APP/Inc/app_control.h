#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_key.h"//后面删掉
#include "bsp_voice.h"
#include "bsp_bluetooth.h"
#include "bsp_receiverk210.h"
#include "bsp_systimer.h"
#include "mid_pid.h"
#include "mid_protocol.h"
#include "Debug.h"
#include "bsp_watchdog.h"
#include "bsp_indicatorled.h"
#include "bsp_gyro.h"

void Service_Init(void);					//初始化
void ServiceTask_Idle(void);				//喂狗
void ServiceTask_5ms(uint32_t curr_ms);  	//内环速度环控制
void ServiceTask_10ms(uint32_t curr_ms); 	//读取开关状态
void ServiceTask_30ms(uint32_t curr_ms); 	//外环方向环控制
void ServiceTask_100ms(uint32_t curr_ms);	//开关控制
uint32_t Service_Clock(void);				//系统时钟
#endif
