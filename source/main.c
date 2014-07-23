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

#define PI			3.142857143F

typedef struct
{
	float x;
	float y;
	float z;
}Point3df;

USBD_HandleTypeDef USBD_Device;

void gyro_read(Point3df *xyz);
void accelerometer_read(Point3df *xyz);
void MagInit(void);
static void MagPointRaw(Point3df *magpoint);
static float MagHeading(Point3df *magpoint, Point3df *accpoint);
void hex_to_ascii(const unsigned char *source, char *dest, unsigned int source_length);

static void SystemClock_Config(void);

int main(void)
{
	HAL_Init();
	BSP_LED_Init(LED4);

	SystemClock_Config();

	USBD_Init(&USBD_Device, &VCP_Desc, 0);

	USBD_RegisterClass(&USBD_Device, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_Template_fops);
	USBD_Start(&USBD_Device);

	BSP_GYRO_Init();
	BSP_ACCELERO_Init();
	MagInit();

	BSP_LED_Off(LED4);

	Point3df min_mag_xyz, max_mag_xyz;
	memset(&min_mag_xyz, 0, sizeof(Point3df));
	memset(&max_mag_xyz, 0, sizeof(Point3df));

	for(;;){
		BSP_LED_Toggle(LED4);

		char outbuff[80];
		int pos=0;

		outbuff[pos++] = 0x02;		// STX
/*
		// Gyro points
		Point3df gyro_xyz;
		gyro_read(&gyro_xyz);

		hex_to_ascii((unsigned char*)&gyro_xyz.x, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		hex_to_ascii((unsigned char*)&gyro_xyz.y, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		hex_to_ascii((unsigned char*)&gyro_xyz.z, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		// Accelerometer points
		Point3df accel_xyz;
		accelerometer_read(&accel_xyz);

		hex_to_ascii((unsigned char*)&accel_xyz.x, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		hex_to_ascii((unsigned char*)&accel_xyz.y, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		hex_to_ascii((unsigned char*)&accel_xyz.z, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;
*/
		// Magnetometer points
		Point3df mag_xyz;
		MagPointRaw(&mag_xyz);

		if(mag_xyz.x < min_mag_xyz.x) min_mag_xyz.x = mag_xyz.x;
		if(mag_xyz.y < min_mag_xyz.y) min_mag_xyz.y = mag_xyz.y;
		if(mag_xyz.z < min_mag_xyz.z) min_mag_xyz.z = mag_xyz.z;

		if(mag_xyz.x > max_mag_xyz.x) max_mag_xyz.x = mag_xyz.x;
		if(mag_xyz.y > max_mag_xyz.y) max_mag_xyz.y = mag_xyz.y;
		if(mag_xyz.z > max_mag_xyz.z) max_mag_xyz.z = mag_xyz.z;

		//Min mag points
		hex_to_ascii((unsigned char*)&min_mag_xyz.x, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		hex_to_ascii((unsigned char*)&min_mag_xyz.y, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		hex_to_ascii((unsigned char*)&min_mag_xyz.z, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		// Raw mag points
		hex_to_ascii((unsigned char*)&mag_xyz.x, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		hex_to_ascii((unsigned char*)&mag_xyz.y, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		hex_to_ascii((unsigned char*)&mag_xyz.z, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		//Max mag points
		hex_to_ascii((unsigned char*)&max_mag_xyz.x, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		hex_to_ascii((unsigned char*)&max_mag_xyz.y, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		hex_to_ascii((unsigned char*)&max_mag_xyz.z, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		// Dummy
		float heading = 0;
		hex_to_ascii((unsigned char*)&heading, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

/*
		// Heading
		float heading = MagHeading(&mag_xyz, &accel_xyz);

		hex_to_ascii((unsigned char*)&heading, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;
*/
		outbuff[pos++] = 0x03;		// ETX

		VCP_write(outbuff, pos);

		HAL_Delay(100);
	}
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

	for(int i=0; i<6; i++)
		tmpbuffer[i] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M+i);

	mag_xyz[0] = (int16_t)((tmpbuffer[0] << 8) + tmpbuffer[1]);	// Mx
	mag_xyz[1] = (int16_t)((tmpbuffer[4] << 8) + tmpbuffer[5]);	// My
	mag_xyz[2] = (int16_t)((tmpbuffer[2] << 8) + tmpbuffer[3]);	// Mz

	magpoint->x = (float)mag_xyz[0];
	magpoint->y = (float)mag_xyz[1];
	magpoint->z = (float)mag_xyz[2];
}

static float MagHeading(Point3df *magpoint, Point3df *accpoint)
{
	// Normalize acceleration measurements so they range from 0 to 1
	float accxnorm = accpoint->x/sqrtf(accpoint->x*accpoint->x+accpoint->y*accpoint->y+accpoint->z*accpoint->z);
	float accynorm = accpoint->y/sqrtf(accpoint->x*accpoint->x+accpoint->y*accpoint->y+accpoint->z*accpoint->z);

	// calculate pitch and roll
	float Pitch = asinf(-accxnorm);
	float Roll = asinf(accynorm/cosf(Pitch));

	// tilt compensated magnetic sensor measurements
	float magxcomp = magpoint->x*cosf(Pitch)+magpoint->z*sinf(Pitch);
	float magycomp = magpoint->x*sinf(Roll)*sinf(Pitch)+magpoint->y*cosf(Roll)-magpoint->z*sinf(Roll)*cosf(Pitch);

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
