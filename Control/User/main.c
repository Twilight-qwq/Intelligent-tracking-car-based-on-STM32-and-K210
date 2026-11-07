#include "Service.h"

typedef struct slice
{
	uint32_t last_10ms;
	uint32_t last_5ms;
	uint32_t last_30ms;
	uint32_t last_100ms;
	uint32_t curr_ms;
}timeslice_t;
static timeslice_t timeslice = {0};

int main(void)
{
	Service_Init();
	while(1)
	{
		timeslice.curr_ms = SysTimer_GetTick();

		if(timeslice.curr_ms-timeslice.last_5ms >= 5)
		{
			timeslice.last_5ms = timeslice.curr_ms;
			ServiceTask_5ms();
		}

		if(timeslice.curr_ms-timeslice.last_10ms >= 10)
		{
			timeslice.last_10ms = timeslice.curr_ms;
			ServiceTask_10ms();
		}

		if(timeslice.curr_ms-timeslice.last_100ms >= 100)
		{
			timeslice.last_100ms = timeslice.curr_ms;
			ServiceTask_100ms();
		}

		if(timeslice.curr_ms-timeslice.last_30ms >= 30)
		{
			timeslice.last_30ms = timeslice.curr_ms;
			ServiceTask_30ms();
		}
	}
}

