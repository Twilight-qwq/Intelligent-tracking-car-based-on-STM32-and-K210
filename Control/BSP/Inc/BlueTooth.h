#ifndef __BLUETOOTH_H
#define	__BLUETOOTH_H
#include "stm32f10x.h"
void BlueTooth_Init(void);
uint8_t BlueTooth_ReadByte(uint8_t *data);
#endif
