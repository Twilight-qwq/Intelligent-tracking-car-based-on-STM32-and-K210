#include "IndicatorLED.h"

#define LED_PORT    GPIOC
#define LED_PIN     GPIO_Pin_13

void Indicator_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;  //推挽输出
    GPIO_InitStruct.GPIO_Pin   = LED_PIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;   //PC13 属低功耗引脚, 规格上限 2MHz
    GPIO_Init(LED_PORT, &GPIO_InitStruct);

    Indicator_TurnOff();
}

void Indicator_TurnOn(void)
{
    GPIO_ResetBits(LED_PORT, LED_PIN);//低电平点亮
}

void Indicator_TurnOff(void)
{
    GPIO_SetBits(LED_PORT, LED_PIN);//高电平熄灭
}

void Indicator_Toggle(void)
{
    if (GPIO_ReadOutputDataBit(LED_PORT, LED_PIN) == SET)
    {
        Indicator_TurnOn();
    }
    else
    {
        Indicator_TurnOff();
    }
}
