#ifndef __DEBUG_H
#define __DEBUG_H
#include "stm32f10x.h"

#define DEBUG_ENABLE 	1//调试模式开关

//调试数据包结构体
typedef struct {
    float target_left;
    float actual_left;
    float target_right;
    float actual_right;
    float error;
    /* 下面两个是 2026-11-22 加的「转弯点标定」通道。
     * 为什么要在事件发生时锁存，而不是每 100 毫秒采一次：
     *   遥测每 100 毫秒发一帧，车速 70 厘米/秒时一帧就走 7 厘米；
     *   而我们关心的转弯窗口本身只有 20 厘米宽 —— 直接采会差出三分之一。
     *   所以在「视觉报弯」和「开始转」那两个瞬间把里程存下来，之后一直不变，
     *   VOFA+ 上看到的就是精确值，采样误差为 0。 */
    float mileage_arm;      // 视觉报弯那一刻的段内里程（厘米）
    float mileage_turn;     // 真正开始转那一刻的段内里程（厘米）
}debugdata_t;

#if DEBUG_ENABLE
void Debug_Init(void);
void Debug_log(debugdata_t *data);
#else
#define Debug_Init()
#define Debug_log(data)
#endif



#endif
