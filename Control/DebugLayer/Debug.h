#ifndef __DEBUG_H
#define __DEBUG_H
#include "stm32f10x.h"

#define DEBUG_ENABLE 	1//调试模式开关

#if DEBUG_ENABLE
void Debug_Init(void);
void Debug_log(int8_t err, uint8_t state, float t_left, float t_right, uint8_t run_state);
#else
#define Debug_Init()
#define Debug_log(a,b,c,d,e)
#endif



#endif
