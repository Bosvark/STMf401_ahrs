#include "pid.h"

void PIDInit(void)
{
	PIDYaw.vi_Ref = 0 ;
	PIDYaw.vi_FeedBack = 0 ;

	PIDYaw.vi_PreError = 0 ;
	PIDYaw.vi_PreDerror = 0 ;

	PIDYaw.v_Kp = SPEED_KP;
	PIDYaw.v_Ki = SPEED_KI;
	PIDYaw.v_Kd = SPEED_KD;

	PIDYaw.vl_PreU = 0;
  /*********************************************/
	PIDRoll.vi_Ref = 0 ;
	PIDRoll.vi_FeedBack = 0 ;

	PIDRoll.vi_PreError = 0 ;
	PIDRoll.vi_PreDerror = 0 ;

	PIDRoll.v_Kp = SPEED_KP_1;
	PIDRoll.v_Ki = SPEED_KI_1;
	PIDRoll.v_Kd = SPEED_KD_1;

	PIDRoll.vl_PreU = 0;
   /*********************************************/
	PIDPitch.vi_Ref = 0 ;
	PIDPitch.vi_FeedBack = 0 ;

	PIDPitch.vi_PreError = 0 ;
	PIDPitch.vi_PreDerror = 0 ;

	PIDPitch.v_Kp = SPEED_KP_2;
	PIDPitch.v_Ki = SPEED_KI_2;
	PIDPitch.v_Kd = SPEED_KD_2;

	PIDPitch.vl_PreU = 0;
}

int16_t PIDCalc(PID *pp)
{
	int16_t  error,d_error,dd_error;

    error = pp->vi_Ref - pp->vi_FeedBack;

    d_error = error - pp->vi_PreError;

    dd_error = d_error - pp->vi_PreDerror;

    pp->vi_PreError = error;
    pp->vi_PreDerror = d_error;

    if( ( error < VV_DEADLINE ) && ( error > -VV_DEADLINE ) )
    {
        ;
    }
    else
    {
        pp->vl_PreU += (pp -> v_Kp * d_error + pp -> v_Ki * error + pp->v_Kd*dd_error);
    }

    if( pp->vl_PreU >= VV_MAX )
        pp->vl_PreU = VV_MAX;
    else if( pp->vl_PreU <= VV_MIN )
        pp->vl_PreU = VV_MIN;

     return (pp->vl_PreU);
}

/******************************************************************************/
