#include "stm32f4xx_hal.h"
#include <string.h>
#include "timer.h"

#define MAX_TIMERS	10

static __IO uint32_t uwTimingDelay=0;

typedef struct
{
	uint32_t expire;
}timer_struct;

char init=0;

timer_struct timers[MAX_TIMERS];

int TimerStart(unsigned int delay_ms)
{
	if(init == 0){
		memset(&timers, 0, MAX_TIMERS*sizeof(timer_struct));
		timers[0].expire = delay_ms;
		init = 1;
		return 1;
	}

	for(int i=0; i<MAX_TIMERS; i++){
		if(timers[i].expire == 0){
			timers[i].expire = uwTimingDelay + delay_ms;
			return i++;
		}
	}

	return 0;
}

void TimerStop(int handle)
{
	if((handle <= 0) || (handle >= MAX_TIMERS))
		return;

	timers[handle--].expire = 0;
}

int TimerIsExpired(int handle)
{
	if((handle <= 0) || (handle >= MAX_TIMERS))
		return 1;

	if(timers[handle--].expire <= uwTimingDelay){
		timers[handle].expire = 0;
		return 1;
	}

	return 0;
}

unsigned int TimerUpTime(void)
{
	return (unsigned int)uwTimingDelay;
}

void TimerDelay(unsigned int delay_ms)
{
	uint32_t expire = uwTimingDelay + delay_ms;

	while(uwTimingDelay < expire);
}
// ----------------------------------------------------------------------------


void SysTick_Handler(void)
{
	uwTimingDelay++;
  HAL_IncTick();
}

