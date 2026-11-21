#include <string.h>
#include "Protocol.h"


static int8_t  target_error = 0;
static uint8_t car_state = 0;

/* 影子变量（2026-11-21 新增，修「帧撕裂」）
 * 为什么需要：偏差字节和状态字节是分两次写进 target_error / car_state 的，
 *   中间隔 87 微秒（115200 波特率，一个字节 10 位）。如果主循环正好在这 87 微秒里读，
 *   就会读到「新的偏差 + 旧的状态」，两个数来自不同帧。
 * 怎么修：整帧收完（等到帧尾 0x55）的那一瞬间，把两个字节一次复制进影子；
 *   主循环只读影子。复制只发生在完整帧的边界上，所以永远读不到半新半旧的组合。
 * 代价：每帧多复制 2 个字节。 */
static int8_t  target_error_shadow = 0;
static uint8_t car_state_shadow = 0;

static volatile uint8_t data_ready = 0;

//视觉状态机
typedef enum {
    V_WAIT_HEADER = 0, // 等待帧头 0xAA
    V_RECV_ERROR,      // 接收偏差值
    V_RECV_STATE,      // 接收路况状态
    V_WAIT_TAIL        // 等待帧尾 0x55
} VisionState_t;

//蓝牙状态机
typedef enum {
    B_WAIT_HEADER = 0, // 等待帧头 0xAA
    B_RECV_CMD,        // 接收(recv)指令类型 (0x01改Kp, 0x02改Kd...)
    B_RECV_DATA,       // 接收 4 个字节的数据
    B_WAIT_TAIL        // 等待帧尾 0x55
} BtState_t;

//K210数据解析
void Protocol_VisionParse(uint8_t data)
{
    static VisionState_t current_state = V_WAIT_HEADER;
    
    switch(current_state)
    {
        case V_WAIT_HEADER:
            if(data == FRAME_HEADER) current_state = V_RECV_ERROR; // 【修改点】
            break;
            
        case V_RECV_ERROR:
            target_error = (int8_t)data; 
            current_state = V_RECV_STATE;
            break;
            
        case V_RECV_STATE:
            car_state = data;
            current_state = V_WAIT_TAIL;
            break;
            
        case V_WAIT_TAIL:
            if(data == FRAME_TAIL)
            {
                /* 2026-11-21 修：整帧收完了，才把两个字节一次搬进影子 */
                target_error_shadow = target_error;
                car_state_shadow    = car_state;
                data_ready = 1;
            }
            current_state = V_WAIT_HEADER; 
            break;
            
        default:
            current_state = V_WAIT_HEADER;
            break;
    }
}


uint8_t Protocol_GetVision(VisionData_t *out_data)
{
    if (data_ready == 1)
    {
        uint32_t primask = __get_PRIMASK(); 
        __disable_irq(); 
        // 关总中断，防止读取时被两个数据K210新数据覆盖单个,导致数据错乱

        out_data->error = target_error_shadow;
        out_data->car_state = car_state_shadow;
        data_ready = 0;
        
        __set_PRIMASK(primask);
        return 1; //成功拿到新数据
    }
    return 0; // 没新数据
}




uint8_t Protocol_BtParse(uint8_t rx_byte, BtCommand_t *out_cmd)
{
    static BtState_t current_state = B_WAIT_HEADER;
    static uint8_t temp_cmd = 0;     //暂存指令类型(指令码)
    static uint8_t data_buf[4];      //暂存4个字节的浮点数
    static uint8_t data_index = 0;   //记录当前存到第几个字节

    switch(current_state)
    {
        case B_WAIT_HEADER:
            if(rx_byte == FRAME_HEADER)
            {
                current_state = B_RECV_CMD; 
            }
            //如果收到别的乱码不理会,继续等0xAA
            break;

        case B_RECV_CMD:
            temp_cmd = rx_byte;             //记下需要改变的参数
            data_index = 0;                 //准备开始收4个字节的数据
            current_state = B_RECV_DATA;
            break;

        case B_RECV_DATA://一个浮点数4字节
            data_buf[data_index++] = rx_byte;
            
            if(data_index >= 4)//4字节收齐
            {
                current_state = B_WAIT_TAIL;//等帧尾
            }
            break;

        case B_WAIT_TAIL:
            if(rx_byte == FRAME_TAIL)//帧尾正确
            {
                out_cmd->cmd_type = temp_cmd;
                //把 4 个零散的字节拼成一个完整的浮点数
                memcpy(&(out_cmd->param_val), data_buf, 4);
                
                current_state = B_WAIT_HEADER;//重置状态准备下一包
                return 1;//解析成功
            }
            else//帧尾不对,说明中间丢包了或者被干扰了
            {
                current_state = B_WAIT_HEADER;//重置，重新等帧头
            }
            break;

        default:
            current_state = B_WAIT_HEADER;
            if(rx_byte == FRAME_HEADER) current_state = B_RECV_CMD;
            break;
    }
    
    return 0;//还没收全就返回0
}


/* =========================================================================
 * 语音帧解析（USART3）                                     2026-11-21 新增
 * -------------------------------------------------------------------------
 * 一帧 3 个字节：AA | 指令 | 55。最后那个字节永远是 0x55，
 * 所以不能在中断里"来一个字节就存一个"，那样只会剩下 0x55，
 * 启动和停车就分不出来了 —— 必须凑够整帧才算数。
 *
 * 两种帧（详见 Protocol.h 的说明）：
 *   短帧：AA | 指令(0x01~0x7F) | 55
 *   长帧：AA | 指令(0x80~0xFF) | 长度 n | n 个参数 | 55
 *
 * 抗干扰：任何时刻又收到 0xAA，就当作新一帧的开头重新收；
 *         帧尾对不上，整帧作废，回到等帧头。
 * ========================================================================= */
typedef enum {
    VO_WAIT_HEADER = 0, // 等帧头 0xAA
    VO_RECV_CMD,        // 等指令字节
    VO_RECV_LEN,        // 长帧：等"参数个数"
    VO_RECV_DATA,       // 长帧：收参数
    VO_WAIT_TAIL        // 等帧尾 0x55
} VoiceState_t;

static uint8_t voice_cmd_raw    = 0;   // 本帧的指令码（还没验证帧尾）
static uint8_t voice_data_len   = 0;   // 本帧参数个数
static uint8_t voice_data_index = 0;   // 本帧已经收了几个参数
static uint8_t voice_data_buf[VOICE_PAYLOAD_MAX];

/* 整帧验证通过之后，搬到这里等主循环取走。
 * 为什么要分两份（buf 和 out_buf）：中断可能在你正读 out_buf 的时候开始收下一帧，
 * 分两份 + 取的时候关中断，就永远读不到"半新半旧"的组合。 */
static volatile uint8_t voice_ready = 0;
static uint8_t voice_out_cmd = 0;
static uint8_t voice_out_len = 0;
static uint8_t voice_out_buf[VOICE_PAYLOAD_MAX];

void Protocol_VoiceParse(uint8_t data)
{
    static VoiceState_t current_state = VO_WAIT_HEADER;
    uint8_t i;

    switch(current_state)
    {
        case VO_WAIT_HEADER:
            if(data == FRAME_HEADER) current_state = VO_RECV_CMD;
            break;  //其他乱码一律不管，继续等 0xAA

        case VO_RECV_CMD:
            /* 连着两个 0xAA：后一个是真正的帧头，这一帧重新开始 */
            if(data == FRAME_HEADER) break;
            voice_cmd_raw    = data;
            voice_data_len   = 0;
            voice_data_index = 0;
            /* 指令字节自己就说明了这一帧带不带参数，不用额外字段 */
            current_state = (data >= VOICE_CMD_EXT_MIN) ? VO_RECV_LEN : VO_WAIT_TAIL;
            break;

        case VO_RECV_LEN:
            if(data > VOICE_PAYLOAD_MAX)     //长度超了：这一帧作废，绝不越界
            {
                current_state = VO_WAIT_HEADER;
                break;
            }
            voice_data_len = data;
            current_state = (data == 0) ? VO_WAIT_TAIL : VO_RECV_DATA;
            break;

        case VO_RECV_DATA:
            voice_data_buf[voice_data_index++] = data;
            if(voice_data_index >= voice_data_len) current_state = VO_WAIT_TAIL;
            break;

        case VO_WAIT_TAIL:
            if(data == FRAME_TAIL)           //帧尾对上了，这一帧合法
            {
                voice_out_cmd = voice_cmd_raw;
                voice_out_len = voice_data_len;
                for(i = 0; i < voice_data_len; i++)
                {
                    voice_out_buf[i] = voice_data_buf[i];
                }
                voice_ready = 1;
                current_state = VO_WAIT_HEADER;
            }
            else if(data == FRAME_HEADER)    //上一帧残缺，把这个 0xAA 当新帧头
            {
                current_state = VO_RECV_CMD;
            }
            else                             //帧尾不对，整帧作废
            {
                current_state = VO_WAIT_HEADER;
            }
            break;

        default:
            current_state = VO_WAIT_HEADER;
            break;
    }
}


uint8_t Protocol_GetVoice(VoiceCommand_t *out_cmd)
{
    uint8_t i;
    uint32_t primask;

    if(voice_ready == 0) return 0;   //没有新指令

    /* 取的时候关总中断，防止刚读一半中断又改了 voice_out_* */
    primask = __get_PRIMASK();
    __disable_irq();

    out_cmd->cmd = voice_out_cmd;
    out_cmd->len = voice_out_len;
    for(i = 0; i < voice_out_len; i++)
    {
        out_cmd->data[i] = voice_out_buf[i];
    }
    voice_ready = 0;

    __set_PRIMASK(primask);
    return 1;   //拿到一条新指令
}
