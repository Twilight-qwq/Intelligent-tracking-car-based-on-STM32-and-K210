#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include <stdint.h>
void App_Init(void);					//初始化
void AppTask_Idle(void);				//喂狗
void AppTask_5ms(uint32_t curr_ms);  	//内环速度环控制
void AppTask_10ms(uint32_t curr_ms); 	//读取开关状态
void AppTask_30ms(uint32_t curr_ms); 	//外环方向环控制
void AppTask_100ms(uint32_t curr_ms);	//开关控制
uint32_t App_Clock(void);				//系统时钟
#endif
