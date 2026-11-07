#ifndef __ENCODER_H
#define __ENCODER_H
#include "stm32f10x.h"

void Encoder_Init(void);
/* 功能：初始化左右编码器（TIM2重映射 / TIM4）
 * 参数：无
 * 返回：无
 * 注意：本函数内部要做 TIM2 引脚重映射，必须在关闭 JTAG 之后调用
 */

void Encoder_Update(void);
/* 功能：采样一次编码器，刷新本周期增量
 * 参数：无
 * 返回：无
 * 说明：按控制周期调用，一个周期内只调一次！！
 */

int16_t Encoder_GetDeltaLeft(void);
int16_t Encoder_GetDeltaRight(void);
/* 功能：读取本周期增量（可重复读取，不会清零）
 * 参数：无
 * 返回：本周期编码器增量（counts），正=正转
 */

int16_t Encoder_GetRawLeft(void);
int16_t Encoder_GetRawRight(void);
/* 功能：调试-读取当前cnt原始值（可重复读取，不会清零）
 * 参数：无
 * 返回：本周期编码器cnt值（counts），正=正转
 */
#endif
