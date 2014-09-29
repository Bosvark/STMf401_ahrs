#ifndef _ESC_H_
#define _ESC_H_

void ESC_Init(void);
void ESC_Start(int esc_channel);
void ESC_Speed(uint32_t speed, int esc_channel);

#endif // _ESC_H_
