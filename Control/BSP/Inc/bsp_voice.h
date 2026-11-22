#ifndef __BSP_VOICE_H
#define __BSP_VOICE_H
#include "stm32f10x.h"

/* 语音模块（USART3，PB10 = TX / PB11 = RX，默认 9600 波特率）
 *
 * 模块发过来的帧（3 个字节）：
 *     启动  AA 01 55
 *     停车  AA 02 55
 *
 * Voice_GetCommand() 返回 0x01 / 0x02，没有新指令时返回 0。
 * 帧的解析在 MID 层 Protocol.c 的 Protocol_VoiceParse() 里。 */

void Voice_Init(void);
uint8_t Voice_GetCommand(void);

#endif
