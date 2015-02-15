#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

#define MANUAL 0
#define AUTOMATIC 1

typedef struct
{
	unsigned long lastTime;
	double Input;
	double Output;
	double Setpoint;
	double ITerm;
	double lastInput;
	double kp;
	double ki;
	double kd;
	unsigned int SampleTime;
	double outMin;
	double outMax;
	int inAuto;
}Pid;

Pid PIDYaw;
Pid PIDRoll;
Pid PIDPitch;

int PidCompute(Pid *pid);
void PidSetTunings(Pid *pid, double Kp, double Ki, double Kd);
void PidSetSampleTime(Pid *pid, int NewSampleTime);
void PidSetOutputLimits(Pid *pid, double Min, double Max);
void PidSetMode(Pid *pid, int Mode);
void PidInitialize(Pid *pid);

#endif /* __SCHEDULE_H__ */
