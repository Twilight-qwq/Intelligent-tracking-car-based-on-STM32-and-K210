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
void Service_Init(void);//初始化
void ServiceTask_5ms(void);//内环速度环控制
void ServiceTask_10ms(void);//读取开关状态
void ServiceTask_30ms(void);//外环方向环控制
void ServiceTask_100ms(void);//开关控制
#endif
