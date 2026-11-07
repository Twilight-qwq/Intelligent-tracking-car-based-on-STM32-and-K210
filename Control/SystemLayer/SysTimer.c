//时间戳模块
#include "stm32f10x.h"
#include "SysTimer.h"

volatile uint32_t SysTicks;

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
		SysTicks++;
	}
}

void SysTimer_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	//1ms时基：72MHz / (71+1) / (999+1) = 1000Hz
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 999;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 71;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStruct);

	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);

	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStruct);

	TIM_Cmd(TIM3,ENABLE);
}

uint32_t SysTimer_GetTick(void)
{
	return SysTicks;
}
