#include <stm32f4xx_hal.h>
#include <math.h>
#include "usbd_cdc_if_template.h"
#include "FreeIMU_serial.h"
#include "imu.h"
#include "eeprom.h"
#include "exp_board.h"
#include "utils.h"

#define VERSION_MAJOR	2
#define VERSION_MINOR	1

static CalibVals calibration;
static float previous_time=0;

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
		case '.':
		{
			if(funcs->GetAttitude != NULL)
				funcs->GetAttitude(1);

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

void FreeIMUVersion(void)
{
	char buffer[20];
	sprintf(buffer, "AHRS ver:%d.%d\r\n", VERSION_MAJOR, VERSION_MINOR);
	VCP_write(buffer, strlen(buffer));
}

void FreeIMURaw(void)
{
	char outbuff[60];
	Point3df accel_xyz, gyro_xyz, mag_xyz;

	accelerometer_read(&accel_xyz);
	gyro_read(&gyro_xyz);
	MagPointRaw(&mag_xyz);

	sprintf(outbuff, "%d,%d,%d,%d,%d,%d,%d,%d,%d,0,0,\n",(int)accel_xyz.x, (int)accel_xyz.y, (int)accel_xyz.z, (int)gyro_xyz.x, (int)gyro_xyz.y, (int)gyro_xyz.z, (int)mag_xyz.x, (int)mag_xyz.y, (int)mag_xyz.z);
	VCP_write(outbuff, strlen(outbuff));
}

void FreeIMUBaseInt(char count)
{
	char outbuff[100];
	char i;
	Point3df accel_xyz, gyro_xyz, mag_xyz;
	int pos=0;
	int16_t ival;

	ExpLedOn(ORANGE_LED);

	for(i=0; i<count; i++){
		ExpLedToggle(ORANGE_LED);

		accelerometer_read(&accel_xyz);
		gyro_read(&gyro_xyz);
		MagPointRaw(&mag_xyz);

		pos = 0;
		ival = (int16_t)accel_xyz.x;
		memcpy(&outbuff[pos], &ival, sizeof(int16_t));
		pos += sizeof(int16_t);

		ival = (int16_t)accel_xyz.y;
		memcpy(&outbuff[pos], &ival, sizeof(int16_t));
		pos += sizeof(int16_t);

		ival = (int16_t)accel_xyz.z;
		memcpy(&outbuff[pos], &ival, sizeof(int16_t));
		pos += sizeof(int16_t);

		ival = (int16_t)gyro_xyz.x;
		memcpy(&outbuff[pos], &ival, sizeof(int16_t));
		pos += sizeof(int16_t);

		ival = (int16_t)gyro_xyz.y;
		memcpy(&outbuff[pos], &ival, sizeof(int16_t));
		pos += sizeof(int16_t);

		ival = (int16_t)gyro_xyz.z;
		memcpy(&outbuff[pos], &ival, sizeof(int16_t));
		pos += sizeof(int16_t);

		ival = (int16_t)mag_xyz.x;
		memcpy(&outbuff[pos], &ival, sizeof(int16_t));
		pos += sizeof(int16_t);

		ival = (int16_t)mag_xyz.y;
		memcpy(&outbuff[pos], &ival, sizeof(int16_t));
		pos += sizeof(int16_t);

		ival = (int16_t)mag_xyz.z;
		memcpy(&outbuff[pos], &ival, sizeof(int16_t));
		pos += sizeof(int16_t);

		outbuff[pos++] = '\r';
		outbuff[pos++] = '\n';

		VCP_write(outbuff, pos);
	}

	ExpLedOff(ORANGE_LED);
}

static Point3df gyro_xyz_filtered;
static char filter_init=0;

void FreeIMUSendYawPitchRoll(int count)
{
	Point3df gyro_xyz;
	Point3df accel_xyz;
	Point3df mag_xyz;
	char outbuff[60];
	int pos=0, i;

	if(filter_init == 0){
		memset(&gyro_xyz_filtered, 0, sizeof(Point3df));
		filter_init = 1;
	}

	ExpLedOn(ORANGE_LED);

	for(i=0; i<count; i++){
		ExpLedToggle(ORANGE_LED);

		gyro_read(&gyro_xyz);
		accelerometer_read(&accel_xyz);
		MagPointRaw(&mag_xyz);

		// Apply calibration values
		accel_xyz.x = (accel_xyz.x - (float)calibration.offsets[0]) / calibration.scales[0];
		accel_xyz.y = (accel_xyz.y - (float)calibration.offsets[1]) / calibration.scales[1];
		accel_xyz.z = (accel_xyz.z - (float)calibration.offsets[2]) / calibration.scales[2];

		mag_xyz.x = (mag_xyz.x - (float)calibration.offsets[3]) / calibration.scales[3];
		mag_xyz.y = (mag_xyz.y - (float)calibration.offsets[4]) / calibration.scales[4];
		mag_xyz.z = (mag_xyz.z - (float)calibration.offsets[5]) / calibration.scales[5];

		// Normalize acceleration measurements so they range from 0 to 1
		float accxnorm = accel_xyz.x/sqrtf(accel_xyz.x*accel_xyz.x+accel_xyz.y*accel_xyz.y+accel_xyz.z*accel_xyz.z);
		float accynorm = accel_xyz.y/sqrtf(accel_xyz.x*accel_xyz.x+accel_xyz.y*accel_xyz.y+accel_xyz.z*accel_xyz.z);

		// calculate pitch and roll
		float Pitch = asinf(-accxnorm);
		float Roll = asinf(accynorm/cosf(Pitch));

		// tilt compensated magnetic sensor measurements
		float magxcomp = mag_xyz.x*cosf(Pitch)+mag_xyz.z*sinf(Pitch);
		float magycomp = mag_xyz.x*sinf(Roll)*sinf(Pitch)+mag_xyz.y*cosf(Roll)-mag_xyz.z*sinf(Roll)*cosf(Pitch);

		// arctangent of y/x converted to degrees
		float heading = (float)(180*atan2f(magycomp,magxcomp)/PI);
		if(heading < 0)
		heading +=360;
		previous_time = (float)HAL_GetTick();

/*
		float tau=0.075;
		float a=0.0;
		float Complementary(float newAngle, float newRate,int looptime) {
		float dtC = float(looptime)/1000.0;
		a=tau/(tau+dtC);
		x_angleC= a* (x_angleC + newRate * dtC) + (1-a) * (newAngle);
		return x_angleC;
		}
*/
/*
		// Adjust the gyro readings with a complimentary filter
		// angle = 0.98 *(angle+gyro*dt) + 0.02*acc
		if(previous_time > 0){
		float dt = ((float)HAL_GetTick() - previous_time)/1000000;
		gyro_xyz.x = 0.98 *(gyro_xyz.x+gyro_xyz.x*dt) + 0.02*accel_xyz.x;
		gyro_xyz.y = 0.98 *(gyro_xyz.y+gyro_xyz.y*dt) + 0.02*accel_xyz.y;
		gyro_xyz.z = 0.98 *(gyro_xyz.z+gyro_xyz.z*dt) + 0.02*accel_xyz.z;
		}
*/
		pos = 0;
		hex_to_ascii((unsigned char*)&heading, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		outbuff[pos++] = ',';
		hex_to_ascii((unsigned char*)&Pitch, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		outbuff[pos++] = ',';
		hex_to_ascii((unsigned char*)&Roll, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		outbuff[pos++] = ',';
		outbuff[pos++] = '\r';
		outbuff[pos++] = '\n';

		VCP_write(outbuff, pos);
	}

	ExpLedOff(ORANGE_LED);
}

void FreeIMUWriteCalibration(void)
{
	unsigned char inbuff[36];
	int len=0;

	memset(inbuff, 0, sizeof(inbuff));
	memset(&calibration, 0, sizeof(CalibVals));

	while(len < sizeof(inbuff)){
		int iret = VCP_read((unsigned char*)&inbuff[len], sizeof(inbuff)-len);

		if(iret > 0)
			len += iret;
	}

	memcpy((unsigned char*)calibration.offsets, inbuff, 12);
	memcpy((unsigned char*)calibration.scales, &inbuff[12], 24);

	EEPROMSet(VAR_CALIBRATION, (uint8_t*)&calibration);
}

void FreeIMUReadCalibration(void)
{
	char outbuff[100];
	int pos=0;

	EEPROMGet(VAR_CALIBRATION, (uint8_t*)&calibration);

	// Accelerometer offsets
	memcpy(outbuff, "acc offset: ", 12);
	pos+=12;
	hex_to_ascii((unsigned char*)&calibration.offsets[0], &outbuff[pos], sizeof(uint16_t));
	pos += sizeof(uint16_t)*2;
	outbuff[pos++] = ',';
	hex_to_ascii((unsigned char*)&calibration.offsets[1], &outbuff[pos], sizeof(uint16_t));
	pos += sizeof(uint16_t)*2;
	outbuff[pos++] = ',';
	hex_to_ascii((unsigned char*)&calibration.offsets[2], &outbuff[pos], sizeof(uint16_t));
	pos += sizeof(uint16_t)*2;
	outbuff[pos++] = '\n';

	VCP_write(outbuff, pos);

	// Magnetomer offsets
	pos=0;
	memcpy(outbuff, "mag offset: ", 12);
	pos+=12;
	hex_to_ascii((unsigned char*)&calibration.offsets[3], &outbuff[pos], sizeof(uint16_t));
	pos += sizeof(uint16_t)*2;
	outbuff[pos++] = ',';
	hex_to_ascii((unsigned char*)&calibration.offsets[4], &outbuff[pos], sizeof(uint16_t));
	pos += sizeof(uint16_t)*2;
	outbuff[pos++] = ',';
	hex_to_ascii((unsigned char*)&calibration.offsets[5], &outbuff[pos], sizeof(uint16_t));
	pos += sizeof(uint16_t)*2;
	outbuff[pos++] = '\n';

	VCP_write(outbuff, pos);

	// Accelerometer scale
	pos=0;
	memcpy(outbuff, "acc scale: ", 11);
	pos+=11;
	hex_to_ascii((unsigned char*)&calibration.scales[0], &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;
	outbuff[pos++] = ',';
	hex_to_ascii((unsigned char*)&calibration.scales[1], &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;
	outbuff[pos++] = ',';
	hex_to_ascii((unsigned char*)&calibration.scales[2], &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;
	outbuff[pos++] = '\n';

	VCP_write(outbuff, pos);

	//Magnetometer scale
	pos=0;
	memcpy(outbuff, "mag scale: ", 11);
	pos+=11;
	hex_to_ascii((unsigned char*)&calibration.scales[3], &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;
	outbuff[pos++] = ',';
	hex_to_ascii((unsigned char*)&calibration.scales[4], &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;
	outbuff[pos++] = ',';
	hex_to_ascii((unsigned char*)&calibration.scales[5], &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;
	outbuff[pos++] = '\n';

	VCP_write(outbuff, pos);
}

void FreeIMUClearCalibration(void)
{
	memset(&calibration, 0, sizeof(CalibVals));
	EEPROMSet(VAR_CALIBRATION, (uint8_t*)&calibration);
}

