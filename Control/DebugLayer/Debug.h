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
    float error;        //正式固件的视觉误差；台架程序不再发这个通道
}debugdata_t;

#if DEBUG_ENABLE
void Debug_Init(void);
void Debug_log(debugdata_t *data);
/* 台架测试专用：把左右轮实际占空比(‰)塞进 ch2 / ch5。
 * 为什么单独一个函数、而不是往 debugdata_t 里加字段：
 *   1) 正式固件 Service.c 的 Task_SendTelemetry 一行都不用改（不碰你已写好的代码）
 *   2) 没人调用它时它就一直保持 0，不会出现"结构体没赋值 -> 发出垃圾数据"的隐患
 * 切回正式固件入口后没人调它，这两个通道恒为 0，无害。 */
void Debug_SetPwm(float left, float right);
#else
#define Debug_Init()
#define Debug_log(data)
#define Debug_SetPwm(a,b)
#endif



#endif
