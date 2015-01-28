#include <string.h>
#include <math.h>
#include <stm32f401_discovery.h>
#include <stm32f4xx_hal.h>
#include <stm32f401_discovery_gyroscope.h>
#include <stm32f401_discovery_accelerometer.h>
#include "imu.h"

static Point3df gyro_xyz_filtered;
static char filter_init=0;

static float comp_pitch=0;
static float comp_roll=0;
static float comp_yaw=0;
static float previous_time=0;

static float gyro_offs_xyz[3];
static float gyro_noise_xyz[3];

void ImuInit(void)
{
	BSP_GYRO_Init();
	if(BSP_ACCELERO_Init() != ACCELERO_OK)
		BSP_LED_On(LED3);
	MagInit();

	Point3df prev_gyro_xyz;
	memset(&prev_gyro_xyz, 0, sizeof(Point3df));
	memset(&gyro_offs_xyz, 0, sizeof(gyro_offs_xyz));

	yaw=0.0;
	pitch=0.0;
	roll=0.0;

	zero_gyro();
	zero_mag();
}

void ImuYawPitchRoll(float *yaw, float *pitch, float *roll)
{
	Point3df gyro_xyz;
	Point3df accel_xyz;
	Point3df mag_xyz;
	const float gyroScale = 0.001, accScale = 16348.0, alpha = 0.98;
	float acc_pitch=0, gyro_pitch=0, error_angle=0, acc_roll=0, gyro_roll=0, dt=0;

	if(filter_init == 0){
		memset(&gyro_xyz_filtered, 0, sizeof(Point3df));
		filter_init = 1;
	}

	gyro_read(&gyro_xyz);
	accelerometer_read(&accel_xyz);
	MagPointRaw(&mag_xyz);

	gyro_xyz.x = ((gyro_xyz.x - gyro_offs_xyz[0]) * gyroScale);
	gyro_xyz.y = ((gyro_xyz.y - gyro_offs_xyz[1]) * gyroScale);
	gyro_xyz.z = ((gyro_xyz.z - gyro_offs_xyz[2]) * gyroScale);

	accel_xyz.x = accel_xyz.x / accScale;
	accel_xyz.y = accel_xyz.y / accScale;
	accel_xyz.z = accel_xyz.z / accScale;

	float current_time = (float)HAL_GetTick()/1000000;
	dt = current_time - previous_time;

	acc_pitch = atanf((accel_xyz.x / sqrtf(powf(accel_xyz.y, 2) + powf(accel_xyz.z, 2)))) * 180.0/PI;
	gyro_pitch += gyro_xyz.x * dt;

	acc_roll = atanf((accel_xyz.y / sqrtf(powf(accel_xyz.x, 2) + powf(accel_xyz.z, 2)))) * 180.0/PI;
	gyro_roll -= gyro_xyz.y * dt;

	// Complementary Filter Implementation #3:
	comp_pitch = comp_pitch + gyro_pitch*dt;
	error_angle = acc_pitch - comp_pitch;
	comp_pitch = comp_pitch + (1-alpha)*error_angle;

	comp_roll = comp_roll + gyro_roll*dt;
	error_angle = acc_roll - comp_roll;
	comp_roll = comp_roll + (1-alpha)*error_angle;

	comp_yaw = MagHeading(&mag_xyz, &accel_xyz);

	previous_time = current_time;

	if(yaw != NULL)
		*yaw = comp_yaw;

	if(pitch != NULL)
		*pitch = comp_pitch;

	if(roll != NULL)
		*roll = comp_roll;
}

void gyro_read(Point3df *xyz)
{
	float gyro_xyz[3];

	BSP_GYRO_GetXYZ(gyro_xyz);

	xyz->x = gyro_xyz[0] - gyro_offs_xyz[0];
	xyz->y = gyro_xyz[1] - gyro_offs_xyz[1];
	xyz->z = gyro_xyz[2] - gyro_offs_xyz[2];

}

float calc_noise(float current_level, float value)
{
    float noise = 0.0f;

    if(value > current_level){
        noise = value;
        return noise;
    }

    if(value < -current_level){
        noise = -value;
        return noise;
    }

    return current_level;
}


void zero_gyro(void)
{
	float gyro_xyz[3];
	int i;

	for(i=0; i<GYRO_ZERO_SAMPLES; i++){
		BSP_GYRO_GetXYZ(gyro_xyz);

		gyro_offs_xyz[0] += gyro_xyz[0];
		gyro_offs_xyz[1] += gyro_xyz[1];
		gyro_offs_xyz[2] += gyro_xyz[2];
	}

	gyro_offs_xyz[0] = gyro_offs_xyz[0] / GYRO_ZERO_SAMPLES;
	gyro_offs_xyz[1] = gyro_offs_xyz[1] / GYRO_ZERO_SAMPLES;
	gyro_offs_xyz[2] = gyro_offs_xyz[2] / GYRO_ZERO_SAMPLES;

	for(i=0; i<GYRO_ZERO_SAMPLES; i++){
		BSP_GYRO_GetXYZ(gyro_xyz);

		gyro_noise_xyz[0] = calc_noise(gyro_noise_xyz[0], gyro_xyz[0]-gyro_offs_xyz[0]);
		gyro_noise_xyz[1] = calc_noise(gyro_noise_xyz[1], gyro_xyz[1]-gyro_offs_xyz[1]);
		gyro_noise_xyz[2] = calc_noise(gyro_noise_xyz[2], gyro_xyz[2]-gyro_offs_xyz[2]);
	}
}

void accelerometer_read(Point3df *xyz)
{
	int16_t accel_xyz[3];

	BSP_ACCELERO_GetXYZ(accel_xyz);

	xyz->x = (float)accel_xyz[0];
	xyz->y = (float)accel_xyz[1];
	xyz->z = (float)accel_xyz[2];

#ifdef SWAP_AXIS
	xyz->x = xyz->x * -1.0f;
	xyz->y = xyz->y * -1.0f;
	xyz->z = xyz->z * -1.0f;
#endif // SWAP_AXIS

}

void MagInit(void)
{
	LSM303DLHCMag_InitTypeDef LSM303DLHC_InitStruct;
	LSM303DLHC_InitStruct.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_ENABLE;
	LSM303DLHC_InitStruct.MagOutput_DataRate = LSM303DLHC_ODR_220_HZ;
	LSM303DLHC_InitStruct.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
	LSM303DLHC_InitStruct.MagFull_Scale = 32; // LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
	LSM303DLHC_MagInit(&LSM303DLHC_InitStruct);
}

void zero_mag(void)
{
#if 0
	Point3df magpoint;
	int i;

	memset(&mag_offs_min, 0, sizeof(Point3df));
	memset(&mag_offs_max, 0, sizeof(Point3df));

	for(i=0; i<MAG_ZERO_SAMPLES; i++){
		MagPointRaw(&magpoint);

		if(magpoint.x > mag_offs_max.x) mag_offs_max.x = magpoint.x;
		if(magpoint.y > mag_offs_max.y) mag_offs_max.y = magpoint.y;
		if(magpoint.z > mag_offs_max.z) mag_offs_max.z = magpoint.z;

		if(magpoint.x <= mag_offs_min.x) mag_offs_min.x = magpoint.x;
		if(magpoint.y <= mag_offs_min.y) mag_offs_min.y = magpoint.y;
		if(magpoint.z <= mag_offs_min.z) mag_offs_min.z = magpoint.z;

		mag_offs.x += magpoint.x;
		mag_offs.y += magpoint.y;
		mag_offs.z += magpoint.z;
	}

	mag_offs.x = mag_offs.x / MAG_ZERO_SAMPLES;
	mag_offs.y = mag_offs.y / MAG_ZERO_SAMPLES;
	mag_offs.z = mag_offs.z / MAG_ZERO_SAMPLES;
#else
	mag_offs_min.x = -254;
	mag_offs_min.y = -357;
	mag_offs_min.z = -222;
	mag_offs_max.x = 267;
	mag_offs_max.y = 204;
	mag_offs_max.z = 235;

#endif
}

void MagPointRaw(Point3df *magpoint)
{
	int16_t mag_xyz[3];
	uint8_t tmpbuffer[6] ={0};

	while((LSM303DLHC_MagGetDataStatus() & 1) != 1);

	for(int i=0; i<6; i++)
		tmpbuffer[i] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M+i);

	// NOTE: z comes before y
	mag_xyz[0] = (int16_t)((tmpbuffer[0] << 8) + tmpbuffer[1]);	// Mx
	mag_xyz[1] = (int16_t)((tmpbuffer[4] << 8) + tmpbuffer[5]);	// My
	mag_xyz[2] = (int16_t)((tmpbuffer[2] << 8) + tmpbuffer[3]);	// Mz

	magpoint->x = (float)mag_xyz[0];
	magpoint->y = (float)mag_xyz[1];
	magpoint->z = (float)mag_xyz[2];

#ifdef SWAP_AXIS
	magpoint->x = magpoint->x * -1.0f;
	magpoint->y = magpoint->y * -1.0f;
	magpoint->z = magpoint->z * -1.0f;
#endif // SWAP_AXIS
}

// Calibration values
// Min X: -254.00 Min Y: -357.00 Min Z: -222.00
// Max X:  267.00 Max Y:  204.00 Max Z:  235.00

float MagHeading(Point3df *magpoint, Point3df *accpoint)
{
	// use calibration values to shift and scale magnetometer measurements
	float Magx = ((magpoint->x-mag_offs_min.x)/(mag_offs_max.x-mag_offs_min.x)*2-1) - mag_offs.x;
	float Magy = ((magpoint->y-mag_offs_min.y)/(mag_offs_max.y-mag_offs_min.y)*2-1) - mag_offs.y;
	float Magz = ((magpoint->z-mag_offs_min.z)/(mag_offs_max.z-mag_offs_min.z)*2-1) - mag_offs.z;

	// Normalize acceleration measurements so they range from 0 to 1
	float accxnorm = accpoint->x/sqrtf(accpoint->x*accpoint->x+accpoint->y*accpoint->y+accpoint->z*accpoint->z);
	float accynorm = accpoint->y/sqrtf(accpoint->x*accpoint->x+accpoint->y*accpoint->y+accpoint->z*accpoint->z);

	// calculate pitch and roll
	float Pitch = asinf(-accxnorm);
	float Roll = asinf(accynorm/cosf(Pitch));

	// tilt compensated magnetic sensor measurements
	float magxcomp = Magx*cosf(Pitch)+Magz*sinf(Pitch);
	float magycomp = Magx*sinf(Roll)*sinf(Pitch)+Magy*cosf(Roll)-Magz*sinf(Roll)*cosf(Pitch);

	// arctangent of y/x converted to degrees
	float heading = (float)(180*atan2f(magycomp,magxcomp)/PI);

	if(heading < 0)
		heading +=360;

	return heading;
}
