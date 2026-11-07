#include "Service.h"
int main(void)
{
	Service_Init();
	while(1)
	{
		ServiceTask_Idle();
		uint32_t timestamp = Service_Clock();
		ServiceTask_5ms(timestamp);
		ServiceTask_10ms(timestamp);
		ServiceTask_30ms(timestamp);
		ServiceTask_100ms(timestamp);
	}
}
//µ˜ ‘:VOFA+ JustFloat–≠“È
