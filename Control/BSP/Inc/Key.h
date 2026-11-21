#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"

void Key_Init(void);
//把当前系统时间戳作为参数传进来
void Key_CheckRequest(uint32_t current_tick); 
uint8_t Key_GetRequest(void);

#endif
