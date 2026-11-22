#ifndef __MID_PROTOCOL_H
#define __MID_PROTOCOL_H
#include "stm32f10x.h"

#define FRAME_HEADER        0xAA  //统一帧头
#define FRAME_TAIL          0x55 // 统一帧尾



typedef struct {
    int8_t  error;           // 视觉误差
    uint8_t car_state;       // 路况状态
} VisionData_t;

void Protocol_VisionParse(uint8_t data);
uint8_t Protocol_GetVision(VisionData_t *out_data);







// 速度环 (内环 PI)
#define BT_CMD_SET_V_L_KP       0x01  // 左轮速度环 Kp
#define BT_CMD_SET_V_L_KI       0x02  // 左轮速度环 Ki
#define BT_CMD_SET_V_R_KP       0x03  // 右轮速度环 Kp
#define BT_CMD_SET_V_R_KI       0x04  // 右轮速度环 Ki
// 方向环 (外环 PD)
#define BT_CMD_SET_LINE_KP      0x05  // 循迹方向环 Kp
#define BT_CMD_SET_LINE_KD      0x06  // 循迹方向环 Kd
// 基础速度 (路况调速)
#define BT_CMD_SET_SPD_STRA     0x07  // 直道基础速度
#define BT_CMD_SET_SPD_SMALL    0x08  // 小角度弯 (直角/钝角)
#define BT_CMD_SET_SPD_LARGE    0x09  // 大角度弯 (锐角)
// 运行控制 (启停)
#define BT_CMD_SET_RUN_STATE    0x0A  // 1.0=启动 0.0=停车 (与按键共用 CarRun_State)


typedef struct {
    uint8_t cmd_type;   // 指令类型 (比如 0x01 代表改左Kp, 0x02 代表改左Kd)
    float   param_val;  // 参数数值 (比如 1.5)
} BtCommand_t;

uint8_t Protocol_BtParse(uint8_t rx_byte, BtCommand_t *out_cmd);


/* =========================================================================
 * 语音模块协议（USART3，PB10/PB11）                        2026-11-21 新增
 * -------------------------------------------------------------------------
 * 和视觉、蓝牙共用帧头 0xAA / 帧尾 0x55，但跑在另一个串口上，三方互不干扰。
 *
 * 两种帧，靠「指令字节」的大小区分，不需要额外的类型字段：
 *
 *   短帧（不带参数，共 3 字节）：
 *        AA | 指令(0x01~0x7F) | 55
 *   长帧（带参数）：
 *        AA | 指令(0x80~0xFF) | 长度 n | n 个参数字节 | 55
 *
 * 指令表（短帧）：
 *   0x01  启动     ->  AA 01 55
 *   0x02  停车     ->  AA 02 55
 *   0x03 ~ 0x7F    还没用到。以后加新指令（比如"暂停""报数"）
 *                  只需在下面加一行 #define，解析代码一个字都不用改。
 *
 * 【0x00 是保留值，不能当指令用】                            2026-11-21
 *   因为 Voice_GetCommand() 用 0 表示"这段时间没有新指令"。
 *   模块要是发 AA 00 55，返回的也是 0，上层会当成"没收到"，指令就丢了。
 *   所以新指令请从 0x03 开始往上排，别用 0x00。
 *
 * 为什么还要留长帧：
 *   以后万一要"把参数从语音端发下来"（比如改速度、指定第几号弯），
 *   短帧的 1 个字节装不下。长帧一次最多带 VOICE_PAYLOAD_MAX 个字节，
 *   超过就整帧作废，不会写到缓冲区外面去。
 * ========================================================================= */
#define VOICE_CMD_START      0x01   // 启动
#define VOICE_CMD_STOP       0x02   // 停车
#define VOICE_CMD_EXT_MIN    0x80   // >= 这个值的指令，后面跟着「长度 + 参数」
#define VOICE_PAYLOAD_MAX    8      // 长帧最多几个参数（不够就把这个数改大）

typedef struct {
    uint8_t cmd;                        // 指令码（0x01 启动 / 0x02 停车 / >=0x80 长帧）
    uint8_t len;                        // 长帧的参数个数；短帧是 0
    uint8_t data[VOICE_PAYLOAD_MAX];    // 长帧的参数
} VoiceCommand_t;

void    Protocol_VoiceParse(uint8_t data);              // 中断里每收到一个字节就喂进来
uint8_t Protocol_GetVoice(VoiceCommand_t *out_cmd);     // 返回 1 = 拿到一条新指令，0 = 没有

/* 【以后用长帧时注意】                                       2026-11-21
 * BSP/Voice.c 里的 Voice_GetCommand() 只返回「指令码」，
 * 长帧带的参数会被它丢掉。所以：
 *   - 短帧（启动/停车这种）→ 用 Voice_GetCommand() 就行，返回值直接比 0x01 / 0x02
 *   - 长帧（>=0x80，带参数）→ 必须自己调 Protocol_GetVoice(&cmd)，
 *                             从 cmd.data[] 里取参数，别走那个包装函数 */

#endif
