#include "stm32f10x.h"
#include "Protocol.h"
typedef enum {
    WAIT_HEADER = 0, // 等待帧头 0xAA
    RECV_ERROR,      // 接收偏差值
    RECV_STATE,      // 接收路况状态
    WAIT_TAIL        // 等待帧尾 0x55
} ParseState;

int8_t  g_target_error = 0;
uint8_t g_car_state = 0;
volatile uint8_t g_data_ready = 0;

void Protocol_analysis(uint8_t data)
{
	// static 变量只在第一次调用时初始化，之后会保留上一次的值
    static ParseState current_state = WAIT_HEADER;
    
    switch(current_state)
    {
        case WAIT_HEADER:
            if(data == 0xAA) 
			{
                current_state = RECV_ERROR; // 看到帧头，准备收偏差
            }
            break;
            
        case RECV_ERROR:
            // 核心坑点：必须强制转成 int8_t，否则负数会变成 200 多！
            g_target_error = (int8_t)data; 
            current_state = RECV_STATE;
            break;
            
        case RECV_STATE:
            g_car_state = data;
            current_state = WAIT_TAIL;
            break;
            
        case WAIT_TAIL:
            if(data == 0x55)
			{
                // 帧尾校验成功，数据完整！
                g_data_ready = 1; // 拉高标志位，通知主循环来拿数据
            }
            // 不管收到的是不是 0x55（可能是干扰或丢包），这一包都结束了
            // 必须重新回到找帧头的状态，防止卡死
            current_state = WAIT_HEADER; 
            break;
            
        default:
            current_state = WAIT_HEADER; // 防错保护
            break;
    }
}

