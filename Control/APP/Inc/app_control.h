#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include <stdint.h>
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_key.h"//后面删掉
#include "bsp_voice.h"
#include "bsp_bluetooth.h"
#include "bsp_receiverk210.h"
#include "bsp_systimer.h"
#include "mid_pid.h"
#include "mid_protocol.h"
#include "Debug.h"
#include "bsp_watchdog.h"
#include "bsp_indicatorled.h"
#include "bsp_gyro.h"

/* 控制层：所有「车怎么跑」的逻辑都在 app_control.c。
 * 这里每个函数只管「一拍做一次」，什么时候调由 app_scheduler.c 决定。 */
void AppControl_Init(void);					//初始化（BSP 各模块 + PID）
void AppControl_5ms(uint32_t curr_ms);  	//内环速度环控制
void AppControl_10ms(uint32_t curr_ms); 	//读取开关状态
void AppControl_30ms(void); 				//外环方向环控制
void AppControl_100ms(void);				//启停/蓝牙/遥测/指示灯
uint32_t Service_Clock(void);				//系统时钟
#endif
