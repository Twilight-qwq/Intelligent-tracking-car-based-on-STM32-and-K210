#include "stm32f10x.h"
#include "Debug.h"
#include <stdio.h>
#include <string.h>




#if DEBUG_ENABLE


#define DEBUG_FLOAT_NUM 5
#define DEBUG_TX_SIZE   (DEBUG_FLOAT_NUM * 4 + 4)  // 结果是 24 字节

typedef enum {
    CH_TARGET_LEFT = 0,  //通道0:左轮目标速度
    CH_ACTUAL_LEFT,      //通道1:左轮实际速度
    CH_TARGET_RIGHT,     //通道2:右轮目标速度
    CH_ACTUAL_RIGHT,     //通道3:右轮实际速度
    CH_ERROR             //通道4:视觉误差
}DebugChannel_e;

static uint8_t debug_tx_buf[DEBUG_TX_SIZE] = {0};
static const uint8_t vofa_frame_tail[] = {0x00, 0x00, 0x80, 0x7F};
//VOFA+ JustFloat固定帧尾

void Debug_Init(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	DMA_DeInit(DMA1_Channel7);
	DMA_InitTypeDef DMA_InitStruct;
	DMA_InitStruct.DMA_BufferSize=0;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)debug_tx_buf;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
	DMA_Init(DMA1_Channel7,&DMA_InitStruct);
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
}

static void Debug_AnalyseData(debugdata_t *data,float *channels)
{
	channels[CH_TARGET_LEFT] 	= 	data->target_left;
    channels[CH_ACTUAL_LEFT] 	= 	data->actual_left;
    channels[CH_TARGET_RIGHT]	= 	data->target_right;
    channels[CH_ACTUAL_RIGHT] 	= 	data->actual_right;
    channels[CH_ERROR] 			= 	data->error;
}

void Debug_log(debugdata_t *data)
{
	if ((DMA1_Channel7->CCR & DMA_CCR1_EN) != 0)//dma通道7被打开,工作中
	{
		return;//防止新数据覆盖旧数据
	}

	float channels[DEBUG_FLOAT_NUM];
	Debug_AnalyseData(data,channels);

	memcpy(debug_tx_buf,channels , sizeof(channels));
	memcpy(debug_tx_buf+sizeof(channels), vofa_frame_tail, sizeof(vofa_frame_tail));

	DMA_Cmd(DMA1_Channel7,DISABLE);
	//在修改 DMA 的搬运数量、内存地址等参数时，DMA 通道必须处于关闭状态
	
	DMA_ClearFlag(DMA1_FLAG_TC7);
	//把上一次发送完成标志位清除(TC7),

	DMA_SetCurrDataCounter(DMA1_Channel7,DEBUG_TX_SIZE);
	//设置要发送数据的长度
	
	DMA_Cmd(DMA1_Channel7,ENABLE);
	//开启转运数据
}
#endif
