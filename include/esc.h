#ifndef _ESC_H_
#define _ESC_H_

typedef struct
{
	uint32_t motor1;
	uint32_t motor2;
	uint32_t motor3;
	uint32_t motor4;
}MotorSpeed;

#define ESC_OFF		1000

volatile MotorSpeed motor_speed;

void ESC_Init(void);
void ESC_Start(int esc_channel);
void ESC_Speed(uint32_t speed, int esc_channel);
void ESC_Update(void);

#endif // _ESC_H_
