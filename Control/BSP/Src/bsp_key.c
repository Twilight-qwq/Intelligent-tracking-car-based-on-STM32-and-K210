#include "bsp_key.h"

#define KEY_CLK             RCC_APB2Periph_GPIOA
#define KEY_PORT            GPIOA
#define KEYA_PIN            GPIO_Pin_0   // 原为 PA8，与左轮 PWM(TIM1_CH1) 撞脚，已挪到 PA0

static uint8_t  Key_Request    = 0;    //待处理的按键事件(0=无 1=有一次按下)
static uint8_t  KeyStableLevel = 1;    //已确认的稳定电平(1=松开 0=按下)
static uint8_t  KeyLastSample  = 1;    //上一次采样值,用于判断电平是否变化
static uint32_t KeyA_Ticks     = 0;    //采样值最后一次变化的时刻

void Key_Init(void)
{
    RCC_APB2PeriphClockCmd(KEY_CLK, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Pin  = KEYA_PIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(KEY_PORT, &GPIO_InitStruct);
}

// 改造：形参接收当前时间
void Key_CheckRequest(uint32_t current_tick)
{
    uint8_t current_level = GPIO_ReadInputDataBit(KEY_PORT, KEYA_PIN);

    //1.采样值与上次不同 -> 重新开始计时
    if (current_level != KeyLastSample)
    {
        KeyA_Ticks    = current_tick;
        KeyLastSample = current_level;
    }

    //2.连续 20ms 采样值没变 -> 认为电平真的稳定了
    if ((current_tick - KeyA_Ticks) >= 20)
    {
        //3.稳定电平与"已确认电平"不同 -> 提交这次变化
        if (current_level != KeyStableLevel)
        {
            KeyStableLevel = current_level;
            if (current_level == 0)      //低电平 = 按下
            {
                Key_Request = 1;
            }
        }
    }
}

uint8_t Key_GetRequest(void)
{
    if (Key_Request == 1)
    {
        Key_Request = 0; 
        return 1;
    }
    return 0;
}
