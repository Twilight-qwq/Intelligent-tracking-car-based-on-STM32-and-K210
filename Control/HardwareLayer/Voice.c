#include "stm32f10x.h"
#include "Voice.h"

static volatile uint8_t Voice_Cmd = 0;


void Voice_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = 9600; 
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART3, &USART_InitStruct);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);


	NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStruct);

	USART_Cmd(USART3, ENABLE);
}


uint8_t Voice_GetCommand(void)
{
    uint8_t cmd = Voice_Cmd;
	USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);//防止快速发新指令时无响应
    if (cmd != 0) 
    {
        Voice_Cmd = 0;
    }
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    return cmd;
}


void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
    {
        uint8_t data = USART_ReceiveData(USART3);
        Voice_Cmd = data;
    }
}




