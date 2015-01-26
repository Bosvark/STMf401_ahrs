#include <stm32f4xx_hal.h>
#include "usbd_cdc_if_template.h"
#include "FreeIMU_serial.h"

int FreeIMU_serial(FreeIMU_Func *funcs)
{
	char command;

	if(VCP_read(&command, 1) <= 0) return 0;

	switch(command)
	{
		case 'v':
			if(funcs->GetVersion != NULL)
				funcs->GetVersion();
			break;

		case 'r':
			if(funcs->GetIMURaw != NULL)
				funcs->GetIMURaw();
			break;
		case 'a':
		{
			char count=0;

			while(VCP_read(&count, 1) <= 0) HAL_Delay(1);

			if(funcs->GetAttitude != NULL)
				funcs->GetAttitude(count);

			break;
		}
		case 'b':
		{
			char count=0;

			while(VCP_read(&count, 1) <= 0) HAL_Delay(1);

			if(funcs->GetBaseValues != NULL)
				funcs->GetBaseValues(count);

			break;
		}
		case 'q':
		{
			char count=0;

			while(VCP_read((char*)&count, 1) <= 0) HAL_Delay(1);

			if(funcs->GetQuat != NULL)
				funcs->GetQuat(count);

			break;
		}
		case 'C':
			if(funcs->LoadCalibrationValues != NULL)
				funcs->LoadCalibrationValues();
			break;
		case 'c':
			if(funcs->Calibrate != NULL)
				funcs->Calibrate();
		case 'd':
			if(funcs->Debug != NULL)
				funcs->Debug();
			break;
		case 'x':
			if(funcs->ClearCalibration != NULL)
				funcs->ClearCalibration();
			break;
	}

	return 1;
}
