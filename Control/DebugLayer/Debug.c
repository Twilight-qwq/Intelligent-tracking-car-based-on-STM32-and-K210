#include "stm32f10x.h"
#include "Debug.h"
#include <string.h>




#if DEBUG_ENABLE


#define DEBUG_FLOAT_NUM 6
#define DEBUG_TX_SIZE   (DEBUG_FLOAT_NUM * 4 + 4)  // 结果是 28 字节

/* 通道顺序就是 VOFA+ 里从上到下的顺序，改这里必须同步改 VOFA+ 的通道数 */
typedef enum {
    CH_TARGET_LEFT = 0,  //通道0:左轮目标速度
    CH_ACTUAL_LEFT,      //通道1:左轮实际速度
    CH_PWM_LEFT,         //通道2:左轮占空比(‰)
    CH_TARGET_RIGHT,     //通道3:右轮目标速度
    CH_ACTUAL_RIGHT,     //通道4:右轮实际速度
    CH_PWM_RIGHT         //通道5:右轮占空比(‰)
}DebugChannel_e;

/* 台架测试用的占空比，由 main.c 调 Debug_SetPwm() 填进来。
 * 放在这里而不是 debugdata_t 里，是为了不破坏正式固件的数据包结构。 */
static float debug_pwm[2] = {0.0f, 0.0f};

void Debug_SetPwm(float left, float right)
{
	debug_pwm[0] = left;
	debug_pwm[1] = right;
}

static uint8_t debug_tx_buf[DEBUG_TX_SIZE] = {0};
static const uint8_t vofa_frame_tail[] = {0x00, 0x00, 0x80, 0x7F};
//VOFA+ JustFloat固定帧尾

/* ---------- DMA 寄存器位 ---------- */
#define DMA_CCR_EN      0x0001u     /* 通道使能 */
#define DMA_CCR_DIR     0x0010u     /* 1 = 内存 -> 外设 */
#define DMA_CCR_MINC    0x0080u     /* 内存地址递增 */
/* CCR 基础配置：内存->外设、内存递增、8 位、普通优先级、非循环、非M2M */
#define DMA7_CCR_BASE   (DMA_CCR_DIR | DMA_CCR_MINC)

/* 通道7的四个标志位在 IFCR/ISR 的 bit24~27 */
#define DMA_IFCR_CH7    0x0F000000u

/* USART_CR3 的 DMAT 位（第7位）：允许发送寄存器空事件触发 DMA */
#define USART_CR3_DMAT_BIT  0x0080u

void Debug_Init(void)
{
	/* DMA1 时钟（AHB 总线） */
	RCC->AHBENR |= RCC_AHBPeriph_DMA1;

	/* USART2 发送脚 PA2 配置成复用推挽；
	 * 接收脚 PA3 与串口参数由 BlueTooth.c 的 BlueTooth_Init() 负责。
	 * 调用顺序必须：BlueTooth_Init() 先于 Debug_Init()。 */
	RCC->APB2ENR |= RCC_APB2Periph_GPIOA;
	GPIOA->CRL &= ~(0xFu << 8);          /* PA2 四位清零 */
	GPIOA->CRL |=  (0xBu << 8);          /* 1011 = 复用推挽输出 50MHz */

	/* ---- 直接把 DMA1 通道7 的寄存器写成我们要的状态 ---- */
	DMA1_Channel7->CCR  = 0;                       /* 先关通道 */
	DMA1->IFCR          = DMA_IFCR_CH7;            /* 清通道7所有标志 */
	DMA1_Channel7->CPAR = (uint32_t)&USART2->DR;   /* 外设地址 */
	DMA1_Channel7->CMAR = (uint32_t)debug_tx_buf;  /* 内存地址 */
	DMA1_Channel7->CNDTR = DEBUG_TX_SIZE;          /* 传输个数 */
	DMA1_Channel7->CCR  = DMA7_CCR_BASE;           /* 配好但 EN=0 */

	/* 打开 USART2 的发送 DMA 请求 */
	USART2->CR3 |= USART_CR3_DMAT_BIT;
}

static void Debug_AnalyseData(debugdata_t *data,float *channels)
{
	channels[CH_TARGET_LEFT] 	= 	data->target_left;
    channels[CH_ACTUAL_LEFT] 	= 	data->actual_left;
    channels[CH_PWM_LEFT] 		= 	debug_pwm[0];
    channels[CH_TARGET_RIGHT]	= 	data->target_right;
    channels[CH_ACTUAL_RIGHT] 	= 	data->actual_right;
    channels[CH_PWM_RIGHT] 		= 	debug_pwm[1];
}

/* 把一帧数据交给 DMA 发出去，立即返回。
 * 28 字节在 115200 下约 2.4ms，这 2.4ms 由硬件搬，CPU 全程不参与。
 *
 * 注意：这里【不做"通道忙就跳过"的判断】，而是每帧都强制把通道复位再装填。
 * 原因：一旦通道卡在"未完成"状态（EN 一直为 1），如果靠 EN 判断就会永远跳过去，
 * 表现就是"只发出第一帧，之后再也不动，要按复位才好"。
 * 每 10ms 发 28 字节只需 2.4ms，强制复位不会截断上一帧。 */
void Debug_log(debugdata_t *data)
{
	float channels[DEBUG_FLOAT_NUM];

	Debug_AnalyseData(data,channels);

	memcpy(debug_tx_buf, channels, sizeof(channels));
	memcpy(debug_tx_buf+sizeof(channels), vofa_frame_tail, sizeof(vofa_frame_tail));

	/* 强制复位通道 -> 清标志 -> 设地址和长度 -> 确认DMA请求 -> 重新使能 */
	DMA1_Channel7->CCR  &= ~DMA_CCR_EN;
	DMA1->IFCR           = DMA_IFCR_CH7;
	DMA1_Channel7->CMAR  = (uint32_t)debug_tx_buf;
	DMA1_Channel7->CNDTR = DEBUG_TX_SIZE;
	USART2->CR3         |= USART_CR3_DMAT_BIT;
	DMA1_Channel7->CCR  |= DMA_CCR_EN;
}
#endif
