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
#include "exp_board.h"
#include "esc.h"
#include "timer.h"
#include "altitude.h"

//#define SWAP_AXIS

#define VERSION_MAJOR	1
#define VERSION_MINOR	1

#define PI			3.142857143F

#define DEG2RAD(x)((x*PI)/180)

#define CALIBRATION_MAGIC_NO	89674523.0

static float gyro_offs_xyz[3];
static float gyro_noise_xyz[3];

#define GYRO_ZERO_SAMPLES	10000.0
#define MAG_ZERO_SAMPLES	10000.0

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

static Point3df mag_offs_min;
static Point3df mag_offs_max;
static Point3df mag_offs;

static FreeIMU_Func fimu_funcs;
float previous_time=0;

void gyro_read(Point3df *xyz);
void zero_mag(void);
void zero_gyro(void);
void accelerometer_read(Point3df *xyz);
void MagInit(void);
void MagPointRaw(Point3df *magpoint);
float MagHeading(Point3df *magpoint, Point3df *accpoint);
void hex_to_ascii(const unsigned char *source, char *dest, unsigned int source_length);
void send_quaternion(char count);
void get_yaw_pitch_roll(float *yaw, float *pitch, float *roll);
void send_yaw_pitch_roll(char count);
static void SystemClock_Config(void);

void version(void);
void imu_raw(void);
void imu_base_int(char count);

int main(void)
{
	HAL_Init();

	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
//	BSP_LED_Init(LED6);

	SystemClock_Config();

	ExpLedInit();
	ExpBuzzerInit();
//	EXTILine0_Config();

	BSP_LED_On(LED3);

	BSP_GYRO_Init();
	if(BSP_ACCELERO_Init() != ACCELERO_OK)
		BSP_LED_On(LED3);
	MagInit();

	Point3df prev_gyro_xyz;
	memset(&prev_gyro_xyz, 0, sizeof(Point3df));
	memset(&gyro_offs_xyz, 0, sizeof(gyro_offs_xyz));

	fimu_funcs.GetVersion = &version;
	fimu_funcs.GetIMURaw = &imu_raw;
	fimu_funcs.GetBaseValues = &imu_base_int;
	fimu_funcs.GetQuat = &send_quaternion;
	fimu_funcs.GetAttitude = &send_yaw_pitch_roll;

	USBD_Init(&USBD_Device, &VCP_Desc, 0);

	USBD_RegisterClass(&USBD_Device, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_Template_fops);
	USBD_Start(&USBD_Device);

//	BSP_LED_On(LED6);
	zero_gyro();
	zero_mag();
//	BSP_LED_Off(LED6);

	ExpBuzzerOff();

	TIM_Config();
	ESC_Init();
	AltInit();

	PwmInfo pwm;
	char outbuff[60];
	int count=0;

	ESC_Start(1);
	ESC_Start(2);
	ESC_Start(3);
	ESC_Start(4);

	int timer=0;

/*
	int up=1;
	int servo=1000;
	for(;;){
		if(up==1){
			servo += 1;

			if(servo >= 2000)
				up = 0;
		}else{
			servo -= 1;

			if(servo <= 1000)
				up = 1;
		}

		HAL_Delay(1);

		ESC_Speed(servo, 1);
	}
*/
	uint32_t start = HAL_GetTick();
	uint32_t current = start;

	for(;;){
//		FreeIMU_serial(&fimu_funcs);
/*
		if(GetPwmInfo(&pwm)){

//			sprintf(outbuff, "%d  ->  %04d  %04d  %04d  %04d\r\n", ++count, (int)pwm.pwmval1, (int)pwm.pwmval2, (int)pwm.pwmval3, (int)pwm.pwmval4);
//			VCP_write(outbuff, strlen(outbuff));

			BSP_LED_Toggle(LED5);

			ESC_Speed(pwm.pwmval1, 1);
			ESC_Speed(pwm.pwmval2, 2);
			ESC_Speed(pwm.pwmval3, 3);
			ESC_Speed(pwm.pwmval4, 4);
		}
*/
		float pitch=0.0;
		get_yaw_pitch_roll(NULL, &pitch, NULL);

		int servo_pitch=1000 + (pitch*(1000.0/90.0));
		ESC_Speed(servo_pitch, 1);

		current = HAL_GetTick();

		if(current - start > 500){
			float temperature = AltReadTemperature();
			sprintf(outbuff, "Temp: %d\r\n", (int)temperature);
			VCP_write(outbuff, strlen(outbuff));

			int32_t pressure = AltReadPressure();
			sprintf(outbuff, "Pressure: %d\r\n", pressure);
			VCP_write(outbuff, strlen(outbuff));

			float altitude = AltReadAltitude();
			sprintf(outbuff, "Altitude: %d\r\n", (int32_t)altitude);
			VCP_write(outbuff, strlen(outbuff));

			start = HAL_GetTick();
		}
/*
		if(TimerIsExpired(timer)){
			timer = TimerStart(500);
//			ExpBuzzerToggle();
			ExpLedToggle(RED_LED);
		}
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

	sprintf(outbuff, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,0,\n",(int)accel_xyz.x, (int)accel_xyz.y, (int)accel_xyz.z,
												(int)gyro_xyz.x, (int)gyro_xyz.y, (int)gyro_xyz.z,
												(int)mag_xyz.x, (int)mag_xyz.y, (int)mag_xyz.z, (int)HAL_GetTick());

	VCP_write(outbuff, strlen(outbuff));
}

void imu_base_int(char count)
{
	char outbuff[100];
	char i;
	Point3df accel_xyz, gyro_xyz, mag_xyz;
	int pos=0;
	int16_t ival;

//	BSP_LED_Off(LED6);

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
//	BSP_LED_On(LED6);

}

void send_quaternion(char count)
{
	Point3df gyro_xyz;
	Point3df accel_xyz;
	Point3df mag_xyz;
	char outbuff[60];
	int pos=0, i;
	const float gyroSensitivity = (500.0f * 2.0f)/65535; //0.0175f;

//	BSP_LED_Off(LED6);

	for(i=0; i<count; i++){
		BSP_LED_Toggle(LED3);

		gyro_read(&gyro_xyz);
		accelerometer_read(&accel_xyz);
		MagPointRaw(&mag_xyz);

		gyro_xyz.x = ((gyro_xyz.x - gyro_offs_xyz[0]) * gyroSensitivity);
		gyro_xyz.y = ((gyro_xyz.y - gyro_offs_xyz[1]) * gyroSensitivity);
		gyro_xyz.z = ((gyro_xyz.z - gyro_offs_xyz[2]) * gyroSensitivity);

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
//	BSP_LED_On(LED6);
}

static Point3df gyro_xyz_filtered;
static char filter_init=0;

static float comp_pitch=0;
static float comp_roll=0;
static float comp_yaw=0;

void get_yaw_pitch_roll(float *yaw, float *pitch, float *roll)
{
	Point3df gyro_xyz;
	Point3df accel_xyz;
	Point3df mag_xyz;
	const float gyroScale = 0.001, accScale = 16348.0, alpha = 0.98;
	float acc_pitch, gyro_pitch, error_angle, acc_roll, gyro_roll, dt;

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

void send_yaw_pitch_roll(char count)
{
	char outbuff[60];
	int pos=0, i=0;

	for(i=0; i<count; i++){
		float yaw=0.0, pitch=0.0, roll=0.0;

		get_yaw_pitch_roll(&yaw, &pitch, &roll);

		pos = 0;
		hex_to_ascii((unsigned char*)&comp_yaw, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;
		outbuff[pos++] = ',';

		hex_to_ascii((unsigned char*)&comp_pitch, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;
		outbuff[pos++] = ',';

		hex_to_ascii((unsigned char*)&comp_roll, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;
		outbuff[pos++] = ',';
		outbuff[pos++] = '\r';
		outbuff[pos++] = '\n';

		VCP_write(outbuff, pos);
	}

	BSP_LED_Off(LED3);
//	BSP_LED_On(LED6);
}

void gyro_read(Point3df *xyz)
{
	float gyro_xyz[3];

	BSP_GYRO_GetXYZ(gyro_xyz);

	xyz->x = gyro_xyz[0];// - gyro_offs_xyz[0];
	xyz->y = gyro_xyz[1];// - gyro_offs_xyz[1];
	xyz->z = gyro_xyz[2];// - gyro_offs_xyz[2];

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
#endif

mag_offs_min.x = -254;
mag_offs_min.y = -357;
mag_offs_min.z = -222;
mag_offs_max.x = 267;
mag_offs_max.y = 204;
mag_offs_max.z = 235;
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
	float Magx = ((magpoint->x-mag_offs_min.x)/(mag_offs_max.x-mag_offs_min.x)*2-1);// - mag_offs.x;
	float Magy = ((magpoint->y-mag_offs_min.y)/(mag_offs_max.y-mag_offs_min.y)*2-1);// - mag_offs.y;
	float Magz = ((magpoint->z-mag_offs_min.z)/(mag_offs_max.z-mag_offs_min.z)*2-1);// - mag_offs.z;

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
