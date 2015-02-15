#include "stm32f4xx_hal.h"
#include "pid.h"

/***************************************************************************************************************
 * Code based on by http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/          *
 ***************************************************************************************************************/

int PidCompute(Pid *pid)
{
   if(!pid->inAuto)
	   return 0;

   unsigned long now = HAL_GetTick();

   int timeChange = (now - pid->lastTime);

   if(timeChange>=pid->SampleTime)
   {
      //Compute all the working error variables
      double error = pid->Setpoint - pid->Input;
      pid->ITerm += (pid->ki * error);

      if(pid->ITerm > pid->outMax)
    	  pid->ITerm = pid->outMax;
      else if(pid->ITerm < pid->outMin)
    	  pid->ITerm = pid->outMin;

      double dInput = (pid->Input - pid->lastInput);

      //Compute PID Output
      pid->Output = pid->kp * error + pid->ITerm - pid->kd * dInput;

      if(pid->Output > pid->outMax)
    	  pid->Output = pid->outMax;
      else if(pid->Output < pid->outMin)
    	  pid->Output = pid->outMin;

      //Remember some variables for next time
      pid->lastInput = pid->Input;
      pid->lastTime = now;
   }

   return 1;
}

void PidSetTunings(Pid *pid, double Kp, double Ki, double Kd)
{
  double SampleTimeInmSec = ((double)pid->SampleTime);
  pid->kp = Kp;
  pid->ki = Ki * SampleTimeInmSec;
  pid->kd = Kd / SampleTimeInmSec;
}

void PidSetSampleTime(Pid *pid, int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime / (double)pid->SampleTime;
      pid->ki *= ratio;
      pid->kd /= ratio;
      pid->SampleTime = (unsigned long)NewSampleTime;
   }
}

void PidSetOutputLimits(Pid *pid, double Min, double Max)
{
   if(Min > Max)
	   return;

   pid->outMin = Min;
   pid->outMax = Max;

   if(pid->Output > pid->outMax)
	   pid->Output = pid->outMax;
   else if(pid->Output < pid->outMin)
	   pid->Output = pid->outMin;

   if(pid->ITerm > pid->outMax)
	   pid->ITerm = pid->outMax;
   else if(pid->ITerm < pid->outMin)
	   pid->ITerm = pid->outMin;
}

void PidSetMode(Pid *pid, int Mode)
{
    char newAuto = (Mode == AUTOMATIC);

    if(newAuto && !pid->inAuto)
    {  //we just went from manual to auto
    	PidInitialize(pid);
    }

    pid->inAuto = newAuto;
}

void PidInitialize(Pid *pid)
{
	pid->lastInput = pid->Input;
	pid->ITerm = pid->Output;

	if(pid->ITerm > pid->outMax)
		pid->ITerm= pid->outMax;
	else if(pid->ITerm < pid->outMin)
		pid->ITerm = pid->outMin;
}
