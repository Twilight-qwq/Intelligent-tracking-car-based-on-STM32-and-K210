#ifndef __INDICATORLED_H
#define __INDICATORLED_H

#include "stm32f10x.h"

/* PC13 板载 LED 硬件驱动
 * 低电平点亮; 闪烁节奏由 Service 层的 Task_Indicate 决定
 */

void Indicator_Init(void);      //初始化, 上电默认熄灭
void Indicator_TurnOn(void);    //点亮
void Indicator_TurnOff(void);   //熄灭
void Indicator_Toggle(void);    //翻转当前状态

#endif
