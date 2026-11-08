#include "stm32f10x.h"
#include "Encoder.h"

/* ==================== 引脚宏定义 ==================== */
/* 左编码器：TIM2 部分重映射1 -> CH1=PA15, CH2=PB3
 *   注意1：PA15/PB3 复位后默认是 JTAG 的 JTDI/JTDO，
 *          必须在 main 里先关 JTAG，再调用 Encoder_Init()
 *   注意2：PB3 硬件上必须串 1kΩ 电阻，否则插着下载器复位时会和 JTDO 对顶
 * 右编码器：TIM4 -> CH1=PB6, CH2=PB7（无重映射） */
#define ENCODER_GPIO_CLK		(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO)
//左轮电机编码器：pa15，pb3
#define ENCODER_LEFT_A_PORT		GPIOA
#define ENCODER_LEFT_A_PIN		GPIO_Pin_15
#define ENCODER_LEFT_B_PORT		GPIOB
#define ENCODER_LEFT_B_PIN		GPIO_Pin_3
#define ENCODER_LEFT_TIM		TIM2
#define ENCODER_LEFT_TIMCLK		RCC_APB1Periph_TIM2
//右轮电机编码器：pb6，pb7
#define ENCODER_RIGHT_A_PORT	GPIOB
#define ENCODER_RIGHT_A_PIN		GPIO_Pin_6
#define ENCODER_RIGHT_B_PORT	GPIOB
#define ENCODER_RIGHT_B_PIN		GPIO_Pin_7
#define ENCODER_RIGHT_TIM		TIM4
#define ENCODER_RIGHT_TIMCLK	RCC_APB1Periph_TIM4

/* 增量缓存：由 Encoder_Update() 刷新，避免上层重复调用把增量清零 */
static int16_t EncLeft_LastCounter = 0;
static int16_t EncRight_LastCounter = 0;
static int16_t EncLeft_DeltaCounter = 0;
static int16_t EncRight_DeltaCounter = 0;

/* ==================== 初始化 ==================== */
/* 功能：初始化左右编码器（TIM2重映射 / TIM4）
 * 参数：无
 * 返回：无
 * 注意：本函数内部要做 TIM2 引脚重映射，必须在关闭 JTAG 之后调用
 */
void Encoder_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	//1.时钟：GPIOA(PA15) + GPIOB(PB3/PB6/PB7) + AFIO(重映射)
	RCC_APB2PeriphClockCmd(ENCODER_GPIO_CLK,ENABLE);

	//2.左编码器引脚重映射：TIM2_CH1->PA15, TIM2_CH2->PB3
	//  必须用库函数，禁止 AFIO->MAPR |= ... 这种裸寄存器写法：
	//  SWJ_CFG 是只写位，读回值未定义，读-改-写会随机制造“重新上电就坏”
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2,ENABLE);

	//3.编码器A/B相：上拉输入
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Pin = ENCODER_LEFT_A_PIN;

	//左轮编码器初始化
	GPIO_Init(ENCODER_LEFT_A_PORT,&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = ENCODER_LEFT_B_PIN;
	GPIO_Init(ENCODER_LEFT_B_PORT,&GPIO_InitStruct);

	//右轮编码器初始化
	GPIO_InitStruct.GPIO_Pin = ENCODER_RIGHT_A_PIN | ENCODER_RIGHT_B_PIN;
	GPIO_Init(ENCODER_RIGHT_A_PORT,&GPIO_InitStruct);

	//4.定时器时钟
	RCC_APB1PeriphClockCmd(ENCODER_LEFT_TIMCLK,ENABLE);
	RCC_APB1PeriphClockCmd(ENCODER_RIGHT_TIMCLK,ENABLE);

	//5.时基：16位计数器，不分频，自动重装65535
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 65535;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(ENCODER_LEFT_TIM,&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInit(ENCODER_RIGHT_TIM,&TIM_TimeBaseInitStruct);

	//6.输入捕获：只改滤波，其余用默认值
	TIM_ICInitTypeDef TIM_ICInitStruct;
	TIM_ICStructInit(&TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_ICFilter = 0xF;


	/* ---- 左编码器 TIM2 ---- */
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
	TIM_ICInit(ENCODER_LEFT_TIM,&TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(ENCODER_LEFT_TIM,&TIM_ICInitStruct);
	TIM_EncoderInterfaceConfig(ENCODER_LEFT_TIM,TIM_EncoderMode_TI12,
								TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//不反相
	TIM_SetCounter(ENCODER_LEFT_TIM,0);
	TIM_Cmd(ENCODER_LEFT_TIM,ENABLE);


	/* ---- 右编码器 TIM4 ---- */
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
	TIM_ICInit(ENCODER_RIGHT_TIM,&TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(ENCODER_RIGHT_TIM,&TIM_ICInitStruct);
	TIM_EncoderInterfaceConfig(ENCODER_RIGHT_TIM,TIM_EncoderMode_TI12,
								TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	TIM_SetCounter(ENCODER_RIGHT_TIM,0);
	TIM_Cmd(ENCODER_RIGHT_TIM,ENABLE);


	//7.清增量缓存
	EncLeft_LastCounter = 0;
	EncRight_LastCounter = 0;
	EncLeft_DeltaCounter = 0;
	EncRight_DeltaCounter = 0;
}

/* ==================== 采样 ==================== */
/* 功能：采样一次编码器，刷新本周期增量
 * 参数：无
 * 返回：无
 * 说明：按控制周期调用，一个周期内只调一次！！
 */
void Encoder_Update(void)
{
	int16_t CurrCounter = 0;
	//1.左轮：先转uint16再转int16，16位计数器回绕时增量仍然正确
	CurrCounter = (int16_t)TIM_GetCounter(ENCODER_LEFT_TIM);
	EncLeft_DeltaCounter = (int16_t)((uint16_t)CurrCounter - (uint16_t)EncLeft_LastCounter);
	EncLeft_LastCounter = CurrCounter;

	//2.右轮：同上
	CurrCounter = (int16_t)TIM_GetCounter(ENCODER_RIGHT_TIM);
	EncRight_DeltaCounter = (int16_t)((uint16_t)CurrCounter - (uint16_t)EncRight_LastCounter);
	EncRight_LastCounter = CurrCounter;
}

/* 功能：读取本周期增量（可重复读取，不会清零）
 * 参数：无
 * 返回：本周期编码器增量（counts），正=正转
 * 用途：传给pid当速度环控制参数
 */
int16_t Encoder_GetDeltaLeft(void)
{
	return EncLeft_DeltaCounter;
}

int16_t Encoder_GetDeltaRight(void)
{
	return EncRight_DeltaCounter;
}







/* ==================== 原始读数（调试用） ==================== */
int16_t Encoder_GetRawLeft(void)
{
	return (int16_t)TIM_GetCounter(ENCODER_LEFT_TIM);
}

int16_t Encoder_GetRawRight(void)
{
	return (int16_t)TIM_GetCounter(ENCODER_RIGHT_TIM);
}
