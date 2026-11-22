#include <stdint.h>
#include "app_scheduler.h"		//先引自己，保证头文件自身是自洽的
#include "app_control.h"		//调度层要调控制层的那四拍
#include "bsp_watchdog.h"		//喂狗

/* ===== 四个任务的闹钟 =====
 * 每个任务记住「上一次真正执行是什么时候」，主循环转得再快，也只按自己的节拍走。
 * 为什么放在这一层：节拍是「调度」的事；app_control.c 只管一拍该干什么。
 * 为什么用 curr_ms 做减法、而不是自己数次数：数据手册的 SysTick 毫秒计数是 32 位的，
 *   减法在回绕时也安全（跑满 49.7 天绕一圈也不会算错）。 */
static uint32_t task_last_5ms   = 0;
static uint32_t task_last_10ms  = 0;
static uint32_t task_last_30ms  = 0;
static uint32_t task_last_100ms = 0;

void Service_Init(void)
{
	AppControl_Init();
}

/* 空闲任务：主循环每一圈都调一次，用来喂独立看门狗。
 * 只有主循环真的在转，狗才不会被饿死 —— 卡死在某个任务里就会复位。 */
void ServiceTask_Idle(void)
{
	WatchDog_FeedDog();
}

void ServiceTask_5ms(uint32_t curr_ms)
{
	if(curr_ms - task_last_5ms < 5) return;
	task_last_5ms = curr_ms;			// 重置闹钟
	AppControl_5ms(curr_ms);
}

void ServiceTask_10ms(uint32_t curr_ms)
{
	if(curr_ms - task_last_10ms < 10) return;
	task_last_10ms = curr_ms;
	AppControl_10ms(curr_ms);
}

void ServiceTask_30ms(uint32_t curr_ms)
{
	if(curr_ms - task_last_30ms < 30) return;
	task_last_30ms = curr_ms;
	AppControl_30ms();
}

void ServiceTask_100ms(uint32_t curr_ms)
{
	if(curr_ms - task_last_100ms < 100) return;
	task_last_100ms = curr_ms;
	AppControl_100ms();
}
