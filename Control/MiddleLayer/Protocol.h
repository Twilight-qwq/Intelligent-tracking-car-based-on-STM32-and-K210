#ifndef __PROTOCOL_H
#define __PROTOCOL_H
#include "stm32f10x.h"
extern int8_t g_target_error;         // K210发来的偏差 (-128 ~ 127)
extern uint8_t g_car_state;           // K210发来的路况 (0/1/2)
extern volatile uint8_t g_data_ready; // 接收完成标志位 (1=有新数据，0=旧数据)
void Protocol_analysis(uint8_t data);
#endif
