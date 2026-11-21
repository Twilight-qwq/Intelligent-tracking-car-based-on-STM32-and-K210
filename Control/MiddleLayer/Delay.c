#include "stm32f10x.h"

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，单位微秒
  * @retval 无
  * @note   2026-11-21 补两处保护：
  *         ① xus = 0 时直接返回。原因：LOAD 写 0 之后计数器不计数，
  *            COUNTFLAG 永远不置位，下面那个 while 会死等，程序就卡在这里。
  *         ② xus > 233015 时先分段。原因：LOAD 只有 24 位（最大 16777215），
  *            72 × 233015 = 16777080 刚好装得下；再大会静默回绕，
  *            实际延时远小于请求值（不会报错，只是悄悄变短）。
  */
void Delay_us(uint32_t xus)
{
	if (xus == 0)
	{
		return;
	}

	while (xus > 233015u)
	{
		Delay_us(233015u);
		xus -= 233015u;
	}

	SysTick->LOAD = 72 * xus;				//设置定时器重装值
	SysTick->VAL = 0x00;					//清空当前计数值
	SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
	SysTick->CTRL = 0x00000004;				//关闭定时器
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 
