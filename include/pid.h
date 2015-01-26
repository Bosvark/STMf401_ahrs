#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

#define VV_MAX				500
#define VV_MIN				0
#define VV_DEADLINE			0

#define SPEED_KP      15
#define SPEED_KI      0
#define SPEED_KD      0

#define SPEED_KP_1    15
#define SPEED_KI_1    5
#define SPEED_KD_1    0

#define SPEED_KP_2    15
#define SPEED_KI_2    5
#define SPEED_KD_2    0

typedef struct PID
{
	int16_t vi_Ref;
	int16_t vi_FeedBack;
	int16_t vi_PreError;
	int16_t vi_PreDerror;
	int16_t v_Kp;
	int16_t v_Ki;
	int16_t v_Kd;
	int16_t vl_PreU;
}PID;

PID PIDYaw;
PID PIDRoll;
PID PIDPitch;

void PIDInit(void);
int16_t PIDCalc(PID *pp);

#endif /* __SCHEDULE_H__ */
