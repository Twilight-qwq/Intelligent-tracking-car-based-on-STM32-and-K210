#include "stm32f10x.h"
#include "WatchDog.h"

void WatchDog_Init(void)
{
	#if WATCHDOG_ENABLE
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_64);
	IWDG_SetReload(625);
	IWDG_ReloadCounter();
	IWDG_Enable();
	#endif
}

void WatchDog_FeedDog(void)
{
	#if WATCHDOG_ENABLE
	IWDG_ReloadCounter();
	#endif
}

