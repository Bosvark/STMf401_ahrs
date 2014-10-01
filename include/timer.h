#ifndef _TIMER_H_
#define _TIMER_H_

int TimerStart(unsigned int delay_ms);
void TimerStop(int handle);
int TimerIsExpired(int handle);
unsigned int TimerUpTime(void);
void TimerDelay(unsigned int delay_ms);

#endif // _TIMER_H_
