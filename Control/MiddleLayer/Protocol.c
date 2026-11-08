#include <string.h>
#include "Protocol.h"


static int8_t  target_error = 0;
static uint8_t car_state = 0;
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
            if(data == FRAME_TAIL) data_ready = 1; // 【修改点】
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

        out_data->error = target_error;
        out_data->car_state = car_state;
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
