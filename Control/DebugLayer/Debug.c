#include "stm32f10x.h"
#include "Debug.h"
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

	/* USART2 发送脚 PA2 由本文件负责配置；
	 * 接收脚 PA3 与串口参数由 BlueTooth.c 的 BlueTooth_Init() 负责。
	 * 调用顺序必须：BlueTooth_Init() 先于 Debug_Init()。
	 * 注意：DEBUG_ENABLE=0 时本函数不编译，PA2 不会配置；
	 *       但那种情况下也没有任何代码往 USART2 发数据，不影响功能。 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);

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
	float channels[DEBUG_FLOAT_NUM];
	uint8_t i;

	Debug_AnalyseData(data,channels);

	memcpy(debug_tx_buf,channels , sizeof(channels));
	memcpy(debug_tx_buf+sizeof(channels), vofa_frame_tail, sizeof(vofa_frame_tail));

	/* 阻塞发送：每发一个字节先等 TXE(发送寄存器空)置位，再塞下一个。
	 * 24 字节在 115200 下大约 2ms，100ms 才发一次，占用可以忽略。
	 * 原来用 DMA1_Channel7 的写法实测一个字节都出不去，
	 * 改成轮询就通了，所以这里不再走 DMA（Debug_Init 里的 DMA 配置保留但不用）。 */
	for(i = 0; i < DEBUG_TX_SIZE; i++)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
		{
			/* 空等硬件把上一个字节搬走 */
		}
		USART_SendData(USART2, debug_tx_buf[i]);
	}
}
#endif
