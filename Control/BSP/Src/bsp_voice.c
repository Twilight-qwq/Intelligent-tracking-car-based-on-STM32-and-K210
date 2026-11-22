#include "stm32f10x.h"
#include "bsp_voice.h"
#include "mid_protocol.h"


/* =========================================================================
 * 语音模块走 USART3：PB10 = TX（单片机发），PB11 = RX（单片机收）。
 *
 * 模块 → 单片机的帧（2026-11-21 定，两帧都是 3 个字节）：
 *     启动    AA 01 55
 *     停车    AA 02 55
 *
 * 帧头帧尾和视觉、蓝牙用同一套（0xAA / 0x55），但这是**另一个串口**，
 * 不会和 USART1 上的视觉帧、USART2 上的蓝牙帧混在一起。
 *
 * 【为什么以前那个写法不行】                                        2026-11-21
 * 以前的中断是这样：每收到一个字节就 Voice_Cmd = data。
 * 一帧有 3 个字节，最后一个永远是帧尾 0x55，
 * 所以主循环取到的永远是 0x55 —— 启动和停车两个指令都变成了 0x55，
 * 上层判断 0x01 / 0x02 永远不成立。
 * 正确做法：先按帧收，等帧尾 0x55 到了，才承认"这一帧合法"，
 * 那一帧里的指令字节才算数。组帧的活交给 MID 层
 * Protocol_VoiceParse()，本文件只负责把字节搬过去。
 * ========================================================================= */

/* 串口波特率。必须和语音模块设置的一致，不一致收到的就是乱码（表现为毫无反应）。
 * 9600 是沿用原来的值；模块说明书写 115200 的话，只改这一行。 */
#define VOICE_UART_BAUD   9600


void Voice_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;      // PB10 = USART3_TX，复用推挽
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;        // PB11 = USART3_RX，上拉输入
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = VOICE_UART_BAUD; // 见上面的 #define
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_Parity = USART_Parity_No;   // 无校验
    USART_InitStruct.USART_StopBits = USART_StopBits_1;// 1 位停止位
    USART_InitStruct.USART_WordLength = USART_WordLength_8b; // 8 位数据
    USART_Init(USART3, &USART_InitStruct);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);     // 收到一个字节就进中断


	NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStruct);

	USART_Cmd(USART3, ENABLE);
}


/* 取走一条语音指令。
 * 返回 0x01 = 启动，0x02 = 停车，0 = 这段时间没有新指令。
 * 取走就清零，同一条指令不会被执行两次。
 *
 * 真正的解析在 MID 层的 Protocol_GetVoice()，这里只做转发，
 * 好处是上层（Service.c）不用关心协议细节，接口名也不用改。
 *
 * 注意：这个包装只把「指令码」交出去。以后要是用上长帧（带参数的那种），
 * 参数会被丢掉 —— 那种情况别用这个函数，直接调 Protocol_GetVoice(&cmd) 取全。 */
uint8_t Voice_GetCommand(void)
{
    VoiceCommand_t voice_cmd;

    if (Protocol_GetVoice(&voice_cmd) == 0)
    {
        return 0;
    }
    return voice_cmd.cmd;
}


void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
    {
        /* 读 DR 会自动把 RXNE 清掉，不需要再手动清标志。
         * 每来一个字节就交给 Protocol 里的状态机，凑够 AA xx 55 才算一帧。 */
        uint8_t data = USART_ReceiveData(USART3);
        Protocol_VoiceParse(data);
    }
}
