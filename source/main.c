#include <stdint.h>
#include <usbd_core.h>
#include <usbd_cdc.h>
#include <usbd_cdc_if_template.h>
#include <usbd_desc.h>
#include <stm32f4xx_hal.h>
#include <stm32f401_discovery.h>
#include <stm32f401_discovery_gyroscope.h>
#include <stm32f401_discovery_accelerometer.h>
#include <math.h>
#include "usbd_conf.h"
#include "usbd_desc.h"
#include "MadgwickAHRS.h"
#include "FreeIMU_serial.h"
#include "calibration.h"

#define SWAP_AXIS

#define VERSION_MAJOR	1
#define VERSION_MINOR	1

#define PI			3.142857143F

#define DEG2RAD(x)((x*PI)/180)

#define CALIBRATION_MAGIC_NO	89674523.0

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

USBD_HandleTypeDef USBD_Device;

static FreeIMU_Func fimu_funcs;
static CalibVals calvals;
static Dt3df compl_filter;
float previous_time=0;

void gyro_read(Point3df *xyz);
void accelerometer_read(Point3df *xyz);
void MagInit(void);
static void MagPointRaw(Point3df *magpoint);
//static float MagHeading(Point3df *magpoint, Point3df *accpoint);
void hex_to_ascii(const unsigned char *source, char *dest, unsigned int source_length);
void send_quaternion(char count);
void send_data_raw(Point3df *gyro, Point3df *accelorometer, Point3df *magnetometer, float heading);
static void SystemClock_Config(void);

void version(void);
void imu_raw(void);
void imu_base_int(char count);
void read_calibration(void);
void write_calibration(void);
void clear_calibration(void);

int main(void)
{
	HAL_Init();

	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	BSP_LED_Init(LED6);

	SystemClock_Config();

	// Read calibration values or set defaults if not calibrated.
	readCalibration((unsigned char*)&calvals, sizeof(CalibVals));

	if(calvals.magic != CALIBRATION_MAGIC_NO){
		memset(calvals.offsets, 0, sizeof(calvals.offsets));
		memset(calvals.scales, 1.0, sizeof(calvals.scales));
		BSP_LED_On(LED3);
	}else{
		BSP_LED_On(LED4);
	}

	BSP_GYRO_Init();
	if(BSP_ACCELERO_Init() != ACCELERO_OK)
		BSP_LED_On(LED3);
	MagInit();

	Point3df prev_gyro_xyz;
	memset(&prev_gyro_xyz, 0, sizeof(Point3df));

	memset(&compl_filter, 0, sizeof(Dt3df));

	fimu_funcs.GetVersion = &version;
	fimu_funcs.GetIMURaw = &imu_raw;
	fimu_funcs.GetBaseValues = &imu_base_int;
	fimu_funcs.GetQuat = &send_quaternion;
	fimu_funcs.LoadCalibrationValues = &read_calibration;
	fimu_funcs.Calibrate = &write_calibration;
	fimu_funcs.ClearCalibration = &clear_calibration;

	USBD_Init(&USBD_Device, &VCP_Desc, 0);

	USBD_RegisterClass(&USBD_Device, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_Template_fops);
	USBD_Start(&USBD_Device);

	for(;;){
		FreeIMU_serial(&fimu_funcs);
/*
		Point3df gyro_xyz;
		gyro_read(&gyro_xyz);

		Point3df accel_xyz;
		accelerometer_read(&accel_xyz);

		Point3df mag_xyz;
		MagPointRaw(&mag_xyz);

		MadgwickAHRSupdate(gyro_xyz.x, gyro_xyz.y, gyro_xyz.z, accel_xyz.x, accel_xyz.y, accel_xyz.z, mag_xyz.x, mag_xyz.y, mag_xyz.z);

		float heading = MagHeading(&mag_xyz, &accel_xyz);

		FreeIMU_serial(&fimu_funcs);

		BSP_LED_Off(LED3);
		BSP_LED_Off(LED4);
		BSP_LED_Off(LED5);
		BSP_LED_Off(LED6);

		if(gyro_xyz.x > prev_gyro_xyz.x + tolerance) 	BSP_LED_On(LED4);
		else if(gyro_xyz.x < prev_gyro_xyz.x - tolerance)	BSP_LED_On(LED5);

		if(gyro_xyz.y > prev_gyro_xyz.y + tolerance) 	BSP_LED_On(LED3);
		else if(gyro_xyz.y < prev_gyro_xyz.y - tolerance)	BSP_LED_On(LED6);

		memcpy(&prev_gyro_xyz, &gyro_xyz, sizeof(Point3df));
*/
	}
}

void version(void)
{
	char buffer[20];

	sprintf(buffer, "AHRS ver:%d.%d\r\n", VERSION_MAJOR, VERSION_MINOR);
	VCP_write(buffer, strlen(buffer));
}

void imu_raw(void)
{
	char outbuff[60];
	Point3df accel_xyz, gyro_xyz, mag_xyz;

	accelerometer_read(&accel_xyz);
	gyro_read(&gyro_xyz);
	MagPointRaw(&mag_xyz);

	sprintf(outbuff, "%d,%d,%d,%d,%d,%d,%d,%d,%d,0,0,\n",(int)accel_xyz.x, (int)accel_xyz.y, (int)accel_xyz.z,
												(int)gyro_xyz.x, (int)gyro_xyz.y, (int)gyro_xyz.z,
												(int)mag_xyz.x, (int)mag_xyz.y, (int)mag_xyz.z);

	VCP_write(outbuff, strlen(outbuff));
}

void imu_base_int(char count)
{
	char outbuff[100];
	char i;
	Point3df accel_xyz, gyro_xyz, mag_xyz;
	int pos=0;
	int16_t ival;

	BSP_LED_Off(LED6);

	for(i=0; i<count; i++){
		BSP_LED_Toggle(LED3);

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

	BSP_LED_Off(LED3);
	BSP_LED_On(LED6);

}

void send_quaternion(char count)
{
	char outbuff[60];
	int pos=0, i;

	BSP_LED_Off(LED6);

	for(i=0; i<count; i++){
		BSP_LED_Toggle(LED3);

		Point3df gyro_xyz;
		gyro_read(&gyro_xyz);

		Point3df accel_xyz;
		accelerometer_read(&accel_xyz);

		Point3df mag_xyz;
		MagPointRaw(&mag_xyz);

		// Apply calibration values
		accel_xyz.x = (accel_xyz.x - (float)calvals.offsets[0]) / calvals.scales[0];
		accel_xyz.y = (accel_xyz.y - (float)calvals.offsets[1]) / calvals.scales[1];
		accel_xyz.z = (accel_xyz.z - (float)calvals.offsets[2]) / calvals.scales[2];

		mag_xyz.x = (mag_xyz.x - (float)calvals.offsets[3]) / calvals.scales[3];
		mag_xyz.y = (mag_xyz.y - (float)calvals.offsets[4]) / calvals.scales[4];
		mag_xyz.z = (mag_xyz.z - (float)calvals.offsets[5]) / calvals.scales[5];

		// Adjust the gyro readings with a complimentary filter
		// angle = 0.98 *(angle+gyro*dt) + 0.02*acc
		if(previous_time > 0){
			float dt = ((float)HAL_GetTick() - previous_time)/1000000;
			gyro_xyz.x = 0.98 *(gyro_xyz.x+gyro_xyz.x*dt) + 0.02*accel_xyz.x;
			gyro_xyz.y = 0.98 *(gyro_xyz.y+gyro_xyz.y*dt) + 0.02*accel_xyz.y;
			gyro_xyz.z = 0.98 *(gyro_xyz.z+gyro_xyz.z*dt) + 0.02*accel_xyz.z;
		}

		previous_time = (float)HAL_GetTick();

		MadgwickAHRSupdate(DEG2RAD(gyro_xyz.x), DEG2RAD(gyro_xyz.y), DEG2RAD(gyro_xyz.z), accel_xyz.x, accel_xyz.y, accel_xyz.z, mag_xyz.x, mag_xyz.y, mag_xyz.z);

		pos = 0;

		hex_to_ascii((unsigned char*)&q0, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;
		outbuff[pos++] = ',';

		hex_to_ascii((unsigned char*)&q1, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;
		outbuff[pos++] = ',';

		hex_to_ascii((unsigned char*)&q2, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;
		outbuff[pos++] = ',';

		hex_to_ascii((unsigned char*)&q3, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;
		outbuff[pos++] = ',';
		outbuff[pos++] = '\r';
		outbuff[pos++] = '\n';

		VCP_write(outbuff, pos);
	}

	BSP_LED_Off(LED3);
	BSP_LED_On(LED6);
}

void send_data_raw(Point3df *gyro, Point3df *accelorometer, Point3df *magnetometer, float heading)
{
	char outbuff[80];
	int pos=0;

	outbuff[pos++] = 0x02;		// STX

	// Gyro points
	hex_to_ascii((unsigned char*)&gyro->x, &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;

	hex_to_ascii((unsigned char*)&gyro->y, &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;

	hex_to_ascii((unsigned char*)&gyro->z, &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;

	// Accelerometer points
	hex_to_ascii((unsigned char*)&accelorometer->x, &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;

	hex_to_ascii((unsigned char*)&accelorometer->y, &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;

	hex_to_ascii((unsigned char*)&accelorometer->z, &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;

	// Magnetometer points
	hex_to_ascii((unsigned char*)&magnetometer->x, &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;

	hex_to_ascii((unsigned char*)&magnetometer->y, &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;

	hex_to_ascii((unsigned char*)&magnetometer->z, &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;

	// Heading
	hex_to_ascii((unsigned char*)&heading, &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;

	outbuff[pos++] = 0x03;		// ETX

	VCP_write(outbuff, pos);
}

void read_calibration(void)
{
	char outbuff[100];
	int pos=0;

	readCalibration((unsigned char*)&calvals, sizeof(CalibVals));
/*
	if(calvals.magic != CALIBRATION_MAGIC_NO){
		memset(&calvals, 0, sizeof(CalibVals));
	}
*/
	// Accelerometer offsets
	memcpy(outbuff, "acc offset: ", 12);
	pos+=12;
	hex_to_ascii((unsigned char*)&calvals.offsets[0], &outbuff[pos], sizeof(uint16_t));
	pos += sizeof(uint16_t)*2;
	outbuff[pos++] = ',';
	hex_to_ascii((unsigned char*)&calvals.offsets[1], &outbuff[pos], sizeof(uint16_t));
	pos += sizeof(uint16_t)*2;
	outbuff[pos++] = ',';
	hex_to_ascii((unsigned char*)&calvals.offsets[2], &outbuff[pos], sizeof(uint16_t));
	pos += sizeof(uint16_t)*2;
	outbuff[pos++] = '\n';
	VCP_write(outbuff, pos);

	// Magnetomer offsets
	pos=0;
	memcpy(outbuff, "mag offset: ", 12);
	pos+=12;
	hex_to_ascii((unsigned char*)&calvals.offsets[3], &outbuff[pos], sizeof(uint16_t));
	pos += sizeof(uint16_t)*2;
	outbuff[pos++] = ',';
	hex_to_ascii((unsigned char*)&calvals.offsets[4], &outbuff[pos], sizeof(uint16_t));
	pos += sizeof(uint16_t)*2;
	outbuff[pos++] = ',';
	hex_to_ascii((unsigned char*)&calvals.offsets[5], &outbuff[pos], sizeof(uint16_t));
	pos += sizeof(uint16_t)*2;
	outbuff[pos++] = '\n';
	VCP_write(outbuff, pos);

	// Accelerometer scale
	pos=0;
	memcpy(outbuff, "acc scale: ", 11);
	pos+=11;
	hex_to_ascii((unsigned char*)&calvals.scales[0], &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;
	outbuff[pos++] = ',';
	hex_to_ascii((unsigned char*)&calvals.scales[1], &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;
	outbuff[pos++] = ',';
	hex_to_ascii((unsigned char*)&calvals.scales[2], &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;
	outbuff[pos++] = '\n';
	VCP_write(outbuff, pos);

	//Magnetometer scale
	pos=0;
	memcpy(outbuff, "mag scale: ", 11);
	pos+=11;
	hex_to_ascii((unsigned char*)&calvals.scales[3], &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;
	outbuff[pos++] = ',';
	hex_to_ascii((unsigned char*)&calvals.scales[4], &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;
	outbuff[pos++] = ',';
	hex_to_ascii((unsigned char*)&calvals.scales[5], &outbuff[pos], sizeof(float));
	pos += sizeof(float)*2;
	outbuff[pos++] = '\n';
	VCP_write(outbuff, pos);
}
//f4fe60014e01ddffc2ffb000
//126a7546b05480464dbe8346bc976d433d1776432fb34a43

void write_calibration(void)
{

	unsigned char inbuff[36];
	int len=0;

	memset(inbuff, 0, sizeof(inbuff));
	memset(&calvals, 0, sizeof(CalibVals));

	while(len < sizeof(inbuff)){
		int iret = VCP_read((unsigned char*)&inbuff[len], sizeof(inbuff)-len);

		if(iret > 0)
			len += iret;
	}

	calvals.magic = CALIBRATION_MAGIC_NO;
	memcpy((unsigned char*)calvals.offsets, inbuff, 12);
	memcpy((unsigned char*)calvals.scales, &inbuff[12], 24);

	writeCalibration((unsigned char*)&calvals, sizeof(CalibVals));

/*
	memcpy((unsigned char*)calvals.offsets, "\xf4\xfe\x60\x01\x4e\x01\xdd\xff\xc2\xff\xb0\x00", 12);
	memcpy((unsigned char*)calvals.scales, "\x12\x6a\x75\x46\xb0\x54\x80\x46\x4d\xbe\x83\x46\xbc\x97\x6d\x43\x3d\x17\x76\x43\x2f\xb3\x4a\x43", 24);
	calvals.magic = CALIBRATION_MAGIC_NO;
	writeCalibration((unsigned char*)&calvals, sizeof(CalibVals));
*/
}

void clear_calibration(void)
{
return;
	CalibVals calvals;
	memset(&calvals, 0, sizeof(CalibVals));
	writeCalibration((unsigned char*)&calvals, sizeof(CalibVals));
}

void gyro_read(Point3df *xyz)
{
	float gyro_xyz[3];

	BSP_GYRO_GetXYZ(gyro_xyz);

	xyz->x = gyro_xyz[0];
	xyz->y = gyro_xyz[1];
	xyz->z = gyro_xyz[2];
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

static void MagPointRaw(Point3df *magpoint)
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
/*
static float MagHeading(Point3df *magpoint, Point3df *accpoint)
{
	float Mag_minx = -254;
	float Mag_miny = -357;
	float Mag_minz = -222;
	float Mag_maxx = 267;
	float Mag_maxy = 204;
	float Mag_maxz = 235;

	// use calibration values to shift and scale magnetometer measurements
	float Magx = (magpoint->x-Mag_minx)/(Mag_maxx-Mag_minx)*2-1;
	float Magy = (magpoint->y-Mag_miny)/(Mag_maxy-Mag_miny)*2-1;
	float Magz = (magpoint->z-Mag_minz)/(Mag_maxz-Mag_minz)*2-1;

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
*/
const char    hexlookup[] = {"0123456789ABCDEF"};

void hex_to_ascii(const unsigned char *source, char *dest, unsigned int source_length)
{
	unsigned int i;
	unsigned char temp;

	for (i = 0; i < source_length; i++) {
		temp = source[i];
		temp >>= 4;
		dest[i*2] = hexlookup[temp];

		temp = source[i];
		temp &= 0x0f;
		dest[(i*2)+1] = hexlookup[temp];
	}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 84000000
  *            HCLK(Hz)                       = 84000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

#include <errno.h>
caddr_t _sbrk(int incr)
{
  extern char _estack; /* Defined by the linker. */
  extern char _Min_Heap_Size; /* Defined by the linker. */
  static char* current_heap_end;
  char* current_block_address;

  if (current_heap_end == 0)
    current_heap_end = &_estack;;

  current_block_address = current_heap_end;

  // Need to align heap to word boundary, else will get
  // hard faults on Cortex-M0. So we assume that heap starts on
  // word boundary, hence make sure we always add a multiple of
  // 4 to it.
  incr = (incr + 3) & (~3); // align value to 4
  if (current_heap_end + incr > &_Min_Heap_Size)
    {
      // Some of the libstdc++-v3 tests rely upon detecting
      // out of memory errors, so do not abort here.
#if 0
      extern void abort (void);

      _write (1, "_sbrk: Heap and stack collision\n", 32);

      abort ();
#else
      // Heap has overflowed
      errno = ENOMEM;
      return (caddr_t) - 1;
#endif
    }

  current_heap_end += incr;

  return (caddr_t) current_block_address;
}

extern PCD_HandleTypeDef hpcd;

void OTG_FS_IRQHandler(void)
{
	HAL_PCD_IRQHandler(&hpcd);
}
