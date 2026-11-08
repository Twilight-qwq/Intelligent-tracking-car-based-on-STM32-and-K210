#ifndef __MOTOR_H
#define __MOTOR_H
#include "stm32f10x.h"

#define MOTOR_DUTY_FULLSCALE	1000	//占空比指令满量程（‰），对应100%占空比

void Motor_Init(void);
void Motor_SetDuty(int16_t left_duty , int16_t right_duty);

#endif
