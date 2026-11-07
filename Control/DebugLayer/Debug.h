#ifndef __DEBUG_H
#define __DEBUG_H
#include "stm32f10x.h"

#define DEBUG_ENABLE 	1//调试模式开关

//调试数据包结构体
typedef struct {
    float target_left;
    float actual_left;
    float target_right;
    float actual_right;
    float error;
}debugdata_t;

#if DEBUG_ENABLE
void Debug_Init(void);
void Debug_log(debugdata_t *data);
#else
#define Debug_Init()
#define Debug_log(data)
#endif



#endif
