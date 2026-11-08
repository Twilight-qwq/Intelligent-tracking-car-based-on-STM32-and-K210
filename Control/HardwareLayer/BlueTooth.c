#include "stm32f10x.h"
#include "BlueTooth.h"
#include <stdio.h>
//缩写:bt -- bluetooth

#define BT_QUEUE_SIZE 	32					//蓝牙环形队列大小(蓝牙调参)
//注:必须为2的幂才可使用位与进行环形队列循环

static uint8_t bt_rx_queue[BT_QUEUE_SIZE];	//读到的数据
static volatile uint16_t bt_rx_head = 0;	// 中断往这里写
static volatile uint16_t bt_rx_tail = 0; 	// 主循环从这里读


void BlueTooth_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	USART_InitTypeDef USART_InitStruct;
	USART_InitStruct.USART_BaudRate = 115200;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2,&USART_InitStruct);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3; 
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStruct);

	USART_Cmd(USART2,ENABLE);
}

void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2,USART_IT_RXNE) == SET)
	{
		bt_rx_queue[bt_rx_head] = USART_ReceiveData(USART2);
		bt_rx_head = (bt_rx_head + 1) & (BT_QUEUE_SIZE - 1);
	}
}

uint8_t BlueTooth_ReadByte(uint8_t *data)
{
	if(bt_rx_head == bt_rx_tail) return 0;//无数据需发送

	*data = bt_rx_queue[bt_rx_tail];
	bt_rx_tail = (bt_rx_tail + 1) & (BT_QUEUE_SIZE - 1);
	return 1;
}
