#include "stm32f10x.h"
#include "Debug.h"
#include <stdio.h>


#if DEBUG_ENABLE

#define DEBUG_BUF_SIZE 64
static char debug_tx_buf[DEBUG_BUF_SIZE]={0};

void Debug_Init(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	DMA_DeInit(DMA1_Channel7);
	DMA_InitTypeDef DMA_InitStruct;
	DMA_InitStruct.DMA_BufferSize=0;
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralDST;
	DMA_InitStruct.DMA_M2M=DMA_M2M_Disable;
	DMA_InitStruct.DMA_MemoryBaseAddr=(uint32_t)debug_tx_buf;
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_Mode=DMA_Mode_Normal;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&USART2->DR;
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_Priority=DMA_Priority_Medium;
	DMA_Init(DMA1_Channel7,&DMA_InitStruct);
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
}

void Debug_log(int8_t err, uint8_t state, float t_left, float t_right, uint8_t run_state)
{
	if ((DMA1_Channel7->CCR & DMA_CCR1_EN) != 0)//dma通道7被打开,工作中
	{
		return;//防止新数据覆盖旧数据
	}

	int data_length = snprintf(debug_tx_buf,DEBUG_BUF_SIZE,"Err:%d, St:%d, TL:%.1f, TR:%.1f, Run:%d\n", 
                      err, state, t_left, t_right, run_state);
	//Keil的MicroLIB对浮点%f的支持不完整
	//如果串口助手里TL,TR一直是0.0就是这个原因

	if(data_length <= 0) return;

	if (data_length >= DEBUG_BUF_SIZE)
	{
        data_length = DEBUG_BUF_SIZE - 1;//防止dma发送超出数量限制的字节
    }

	DMA_Cmd(DMA1_Channel7,DISABLE);
	//在修改 DMA 的搬运数量、内存地址等参数时，DMA 通道必须处于关闭状态
	
	DMA_ClearFlag(DMA1_FLAG_TC7);
	//把上一次发送完成标志位清除(TC7),

	DMA_SetCurrDataCounter(DMA1_Channel7,data_length);
	//设置要发送数据的长度
	
	DMA_Cmd(DMA1_Channel7,ENABLE);
	//开启转运数据
}

#endif
