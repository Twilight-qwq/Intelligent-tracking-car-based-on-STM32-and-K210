#include "stm32f10x.h"
#ifndef __SYSTIMER_H
#define __SYSTIMER_H
void SysTimer_Init(void);//时间戳模块初始化
uint32_t SysTimer_GetTick(void);//获取当前时间戳(当前ms)
#endif
