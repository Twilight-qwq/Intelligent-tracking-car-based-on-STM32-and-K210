#ifndef __APP_SCHEDULER_H
#define __APP_SCHEDULER_H

#include <stdint.h>
#include "app_control.h"

/* 调度层：只管「到点了没有」，到点就把活交给 app_control.c 里对应的那一拍。
 * 主循环只调这 6 个函数，别的都不用管。 */
void Service_Init(void);					//上电初始化
void ServiceTask_Idle(void);				//空闲：喂狗
void ServiceTask_5ms(uint32_t curr_ms);  	//内环速度环
void ServiceTask_10ms(uint32_t curr_ms); 	//读取开关状态
void ServiceTask_30ms(uint32_t curr_ms); 	//外环方向环
void ServiceTask_100ms(uint32_t curr_ms);	//启停/蓝牙/遥测/指示灯
#endif
