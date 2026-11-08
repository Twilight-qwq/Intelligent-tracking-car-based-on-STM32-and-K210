#ifndef __PROTOCOL_H
#define __PROTOCOL_H
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


typedef struct {
    uint8_t cmd_type;   // 指令类型 (比如 0x01 代表改左Kp, 0x02 代表改左Kd)
    float   param_val;  // 参数数值 (比如 1.5)
} BtCommand_t;

uint8_t Protocol_BtParse(uint8_t rx_byte, BtCommand_t *out_cmd);
#endif
