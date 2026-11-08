#ifndef __SERVICE_H
#define __SERVICE_H

#include "Encoder.h"
#include "Motor.h"
#include "Key.h"//后面删掉
#include "Voice.h"
#include "BlueTooth.h"
#include "ReceiverK210.h"
#include "SysTimer.h"
#include "PID.h"
#include "Protocol.h"
#include "Debug.h"
#include "WatchDog.h"

void Service_Init(void);					//初始化
void ServiceTask_Idle(void);				//喂狗
void ServiceTask_5ms(uint32_t curr_ms);  	//内环速度环控制
void ServiceTask_10ms(uint32_t curr_ms); 	//读取开关状态
void ServiceTask_30ms(uint32_t curr_ms); 	//外环方向环控制
void ServiceTask_100ms(uint32_t curr_ms);	//开关控制
uint32_t Service_Clock(void);				//系统时钟
#endif
