#include "key.h"

#define KEY_CLK             RCC_APB2Periph_GPIOA
#define KEY_PORT            GPIOA
#define KEYA_PIN            GPIO_Pin_0   // 原为 PA8，与左轮 PWM(TIM1_CH1) 撞脚，已挪到 PA0

static uint8_t  Key_Request = 0;       
static uint8_t  KeyAPastLevel = 1;     
static uint32_t KeyA_Ticks = 0;        

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
    
    if (current_level != KeyAPastLevel) 
    {
        // 记录当前时间
        KeyA_Ticks = current_tick; 
    }
    
    if ((current_tick - KeyA_Ticks) >= 20) 
    {
        if (current_level != KeyAPastLevel) 
        {
            KeyAPastLevel = current_level; 
            if (current_level == 0) 
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
