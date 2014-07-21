#include <usbd_core.h>
#include <usbd_cdc.h>
#include <usbd_cdc_if_template.h>
#include <usbd_desc.h>
#include <stm32f4xx_hal.h>
#include <stm32f401_discovery.h>
#include <stm32f401_discovery_gyroscope.h>
#include <stm32f401_discovery_accelerometer.h>
#include "usbd_conf.h"
#include "usbd_desc.h"

typedef struct
{
	float x;
	float y;
	float z;
}Point3df;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}Point3di;

USBD_HandleTypeDef USBD_Device;

void gyro_read(Point3df *xyz);
void accelerometer_read(Point3di *xyz);
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

	BSP_LED_Off(LED4);

	for(;;){
		Point3df gyro_xyz;
		gyro_read(&gyro_xyz);

		Point3di accel_xyz;
		accelerometer_read(&accel_xyz);

		char outbuff[80];
		int pos=0;

		outbuff[pos++] = 0x02;		// STX

		// Gyro points
		hex_to_ascii((unsigned char*)&gyro_xyz.x, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		hex_to_ascii((unsigned char*)&gyro_xyz.y, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		hex_to_ascii((unsigned char*)&gyro_xyz.z, &outbuff[pos], sizeof(float));
		pos += sizeof(float)*2;

		// Accelerometer points
		hex_to_ascii((unsigned char*)&accel_xyz.x, &outbuff[pos], sizeof(int16_t));
		pos += sizeof(int16_t)*2;

		hex_to_ascii((unsigned char*)&accel_xyz.y, &outbuff[pos], sizeof(int16_t));
		pos += sizeof(int16_t)*2;

		hex_to_ascii((unsigned char*)&accel_xyz.z, &outbuff[pos], sizeof(int16_t));
		pos += sizeof(int16_t)*2;

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

void accelerometer_read(Point3di *xyz)
{
	int16_t accel_xyz[3];

	BSP_ACCELERO_GetXYZ(accel_xyz);

	xyz->x = accel_xyz[0];
	xyz->y = accel_xyz[1];
	xyz->z = accel_xyz[2];
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
