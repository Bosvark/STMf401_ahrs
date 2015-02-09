#ifndef _IMU_H_
#define _IMU_H_

#define PI			3.142857143F

#define DEG2RAD(x)((x*PI)/180)

#define CALIBRATION_MAGIC_NO	89674523.0

#define GYRO_ZERO_SAMPLES	1000.0
#define MAG_ZERO_SAMPLES	1000.0

typedef struct
{
	float x;
	float y;
	float z;
}Point3df;

typedef struct
{
	float dt_x;
	float dt_y;
	float dt_z;
}Dt3df;

float yaw;
float pitch;
float roll;

volatile Point3df mag_offs_min;
volatile Point3df mag_offs_max;
volatile Point3df mag_offs;

void ImuInit(void);
void ImuYawPitchRoll(float *yaw, float *pitch, float *roll);

void gyro_read(Point3df *xyz);
void zero_mag(void);
void zero_gyro(void);
void accelerometer_read(Point3df *xyz);
void MagInit(void);
void MagPointRaw(Point3df *magpoint);
float MagHeading(Point3df *magpoint, Point3df *accpoint);

#endif // _IMU_H_
