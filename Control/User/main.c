#include "app_control.h"
int main(void)
{
	Service_Init();
	while(1)
	{
		AppTask_Idle();
		uint32_t timestamp = Service_Clock();
		AppTask_5ms(timestamp);
		AppTask_10ms(timestamp);
		AppTask_30ms(timestamp);
		AppTask_100ms(timestamp);
	}
}
//调试:VOFA+ JustFloat协议
