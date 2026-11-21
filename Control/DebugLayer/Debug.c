#include "stm32f10x.h"
#include "Debug.h"
#include "SysTimer.h"
#include <string.h>




#if DEBUG_ENABLE


#define DEBUG_FLOAT_NUM 5
#define DEBUG_TX_SIZE   (DEBUG_FLOAT_NUM * 4 + 4)  // 结果是 24 字节

/* 用 DMA 还是轮询发波形（2026-11-21 加）
 * 1 = DMA：Debug_log 几微秒就返回，发送交给 DMA，不占主循环
 * 0 = 轮询：老办法，阻塞约 2.1 毫秒，但已验证可用
 * 为什么留开关：DMA 写错的表现是「VOFA+ 一个字节都收不到」，
 *   烧 1 不放心就翻回 0，两分钟内能确认。 */
#define DEBUG_USE_DMA   1

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

static uint8_t debug_dma_busy = 0;   /* 1 = 上一帧还在发（2026-11-21 加） */

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
	/* 2026-11-21 修两处错：
	 *   ① DMA_BufferSize 原来写 0 —— 那是「这次搬几个字节」，写 0 就是搬 0 个，
	 *      使能后立刻结束，一个字节都不会出。这就是当年「DMA 一个字节都出不去」的原因。
	 *   ② 原来没有 DMA_Cmd() 使能通道，通道不工作。
	 * 现在的做法：初始化时按整帧长度配好，通道先关着；
	 *   每次 Debug_log 里先关、重装长度、清标志、再开（见下面的发送部分）。 */
	DMA_InitStruct.DMA_BufferSize = DEBUG_TX_SIZE;
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

	DMA_Cmd(DMA1_Channel7,DISABLE);      /* 先关着，发送时再开 */
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

/* 轮询模式下的发送超时（DEBUG_USE_DMA = 0 时才会用到）
 * 原来那个 while 没有上限：正常 2 毫秒就出来，但万一 USART2 出问题、
 * TXE 永远不置位，主循环就永久卡死 —— 车直接失控。
 * 5 毫秒 = 正常 2.1 毫秒的 2.4 倍余量；超时就放弃这一帧。 */
#define DEBUG_TX_TIMEOUT_MS   5

void Debug_log(debugdata_t *data)
{
	float channels[DEBUG_FLOAT_NUM];
#if !DEBUG_USE_DMA
	uint8_t i;
	uint32_t start_ms;
#endif

	Debug_AnalyseData(data,channels);

	memcpy(debug_tx_buf, channels, sizeof(channels));
	memcpy(debug_tx_buf+sizeof(channels), vofa_frame_tail, sizeof(vofa_frame_tail));

#if DEBUG_USE_DMA
	/* DMA 发送（2026-11-21 改）
	 * 顺序不能错：先关通道 → 重装字节数 → 清完成标志 → 再开通道。
	 * 关通道再重装，是为了防止「上一帧还没发完就改计数器」把数据发乱。 */
	if (debug_dma_busy)
	{
		if (DMA_GetFlagStatus(DMA1_FLAG_TC7) == SET)
		{
			debug_dma_busy = 0;      /* 上一帧已经发完了 */
		}
		else
		{
			/* 上一帧还在发（正常不会发生：24 字节只要 2.1 毫秒，100 毫秒才发一次）。
			 * 真撞上就丢掉这一帧，别去改还在用的缓冲区。 */
			return;
		}
	}

	DMA_Cmd(DMA1_Channel7, DISABLE);
	DMA_SetCurrDataCounter(DMA1_Channel7, DEBUG_TX_SIZE);
	DMA_ClearFlag(DMA1_FLAG_TC7);
	DMA_Cmd(DMA1_Channel7, ENABLE);
	debug_dma_busy = 1;
#else
	start_ms = SysTimer_GetTick();

	/* 阻塞发送：每发一个字节先等 TXE(发送寄存器空)置位，再塞下一个。
	 * 24 字节在 115200 下大约 2ms，100ms 才发一次，占用可以忽略。 */
	for(i = 0; i < DEBUG_TX_SIZE; i++)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
		{
			/* 等太久就放弃这一帧，别把主循环挂在这里 */
			if((SysTimer_GetTick() - start_ms) > DEBUG_TX_TIMEOUT_MS)
			{
				return;
			}
		}
		USART_SendData(USART2, debug_tx_buf[i]);
	}
#endif
}
#endif
