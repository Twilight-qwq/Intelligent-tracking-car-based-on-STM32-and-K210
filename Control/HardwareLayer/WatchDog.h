#ifndef __WATCHDOG_H
#define __WATCHDOG_H
#include "stm32f10x.h"

#define WATCHDOG_ENABLE		0


void WatchDog_Init(void);
void WatchDog_FeedDog(void);

#endif
