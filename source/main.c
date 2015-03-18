#include <stdint.h>
#include <usbd_core.h>
#include <usbd_cdc.h>
#include <usbd_cdc_if_template.h>
#include <usbd_desc.h>
#include <stm32f4xx_hal.h>
#include <stm32f401_discovery.h>
#include "usbd_conf.h"
#include "usbd_desc.h"
#include "exp_board.h"
#include "esc.h"
#include "timer.h"
#include "altitude.h"
#include "eeprom.h"
#include "flashmem.h"
#include "pid.h"
#include "imu.h"
#include "FreeIMU_serial.h"
#include "utils.h"

#define DEBUG_ON

//#define SWAP_AXIS

#define VERSION_MAJOR	1
#define VERSION_MINOR	1

USBD_HandleTypeDef USBD_Device;

static char armed_flag = 0;
static char calibrating_flag = 0;

extern PwmInfo rx_channel;

static void SystemClock_Config(void);

void unarmed_process(PwmInfo *pwm);
void armed_process(PwmInfo *pwm);
void MyPidInit(void);
void version(void);
void imu_raw(void);
void imu_base_int(char count);
/*
void FlashTest(void)
{
	char outbuff[60];
	uint8_t id=0, mem_type=0, capacity=0;
	FlashStatusRegister status;

	FlashWREN();
	FlashPageProgram(0, "Hello World!\0", 13);
	FlashWRDI();

	while(1){
		FlashMemChipID(&id, &mem_type, &capacity);
		sprintf(outbuff, "FlashMemChipID id:0x%02x mem_type:0x%02x capacity:0x%02x\r\n", id, mem_type, capacity);
		VCP_write(outbuff, strlen(outbuff));

		FlashWREN();
		FlashStatus(&status, &id, &mem_type);

		sprintf(outbuff, "Status after WREN %i rdsr1:0x%02x rdsr1:0x%02x\r\n", status.wel, id, mem_type);
		VCP_write(outbuff, strlen(outbuff));

		FlashWRDI();
		FlashStatus(&status, &id, &mem_type);

		sprintf(outbuff, "Status after WRDI %irdsr1:0x%02x rdsr1:0x%02x\r\n", status.wel, id, mem_type);
		VCP_write(outbuff, strlen(outbuff));

		FlashFastRead(0, (unsigned char*)outbuff, 13);
		VCP_write(outbuff, strlen(outbuff));

		HAL_Delay(1000);
	}
}
*/
/*
void EEPROM_Test(void){
	char outbuff[60];
	uint8_t inbuffer[16], var_test2[13];
	uint32_t var_test1=0, counter=0, max_count=0, i, j;

	while(1)
	{
		EEPROMGet(VAR_TEST1, (uint8_t*)&var_test1);
		EEPROMGet(VAR_TEST2, (uint8_t*)&var_test2);

		sprintf(outbuff, "VAR_TEST1 read from flash: %d\r\n", (unsigned int)var_test1);
		VCP_write(outbuff, strlen(outbuff));

		counter = 0;

		if(max_count == 0)
			max_count = var_test1 + 100;

		sprintf(outbuff, "\r\nSector 0:\r\n");
		VCP_write(outbuff, strlen(outbuff));

		for(j=0; j<16; j++){
			for(i=0; i<16; i++){
				FlashFastRead(&counter, (unsigned char*)inbuffer, sizeof(inbuffer));

				memset(outbuff, 0, sizeof(outbuff));
				sprintf(outbuff, "0x%08x ", (unsigned int)counter);
				hex_to_ascii(inbuffer, &outbuff[strlen(outbuff)], sizeof(inbuffer));
				sprintf(&outbuff[strlen(outbuff)], "\r\n");
				VCP_write(outbuff, strlen(outbuff));

				counter += sizeof(inbuffer);
			}
		}

		counter = 0x1000;
		sprintf(outbuff, "\r\nSector 1:\r\n");
		VCP_write(outbuff, strlen(outbuff));

		for(j=0; j<16; j++){
			for(i=0; i<16; i++){
				FlashFastRead(&counter, (unsigned char*)inbuffer, sizeof(inbuffer));

				memset(outbuff, 0, sizeof(outbuff));
				sprintf(outbuff, "0x%08x ", (unsigned int)counter);
				hex_to_ascii(inbuffer, &outbuff[strlen(outbuff)], sizeof(inbuffer));
				sprintf(&outbuff[strlen(outbuff)], "\r\n");
				VCP_write(outbuff, strlen(outbuff));

				counter += sizeof(inbuffer);
			}
		}

		var_test1++;
		EEPROMSet(VAR_TEST1, (uint8_t*)&var_test1);

		sprintf(var_test2, "Hallo%08x", var_test1);
		EEPROMSet(VAR_TEST2, (uint8_t*)var_test2);

		if(var_test1 >= max_count){
			sprintf(outbuff, "\r\nTEST COMPLETE\r\n");
			VCP_write(outbuff, strlen(outbuff));
			break;
		}

//		HAL_Delay(1000);
	}

	while(1);
}
*/
/*
typedef enum
{
	START,
	CMD,
	ADDR,
	VAL,
	GO
}STATE;
void command(void)
{
	char in, cmd, value[30];
	uint32_t addr;
	STATE state = START;
	int counter=0;

	while(1){
		VCP_read(&in, 1);

		switch(state)
		{
			case START:
				if(in == 0x08)	// backspace
					break;

				cmd = in;
				addr = 0;
				counter = 4;
				memset(value, 0, sizeof(value));

				state = CMD;
			case CMD:
				if(in == 0x08){	// backspace
					if(counter == 0)
						state = START;
					else
						counter--;

					break;
				}

				if(cmd == 's'){		// Sector erase
					addr |= (uint32_t)(in << counter++);
				}
			case ADDR:
			case VAL:
		}
	}
}
*/

void send_attitude_bytes(void)
{
	char outbuff[20];
//	float yaw=0,pitch=0,roll=0;

	ExpLedOn(ORANGE_LED);

//	ImuYawPitchRoll(&yaw, &pitch, &roll);

	sprintf(outbuff, "%d, %d, %d\n", (int)roll, (int)pitch, (int)yaw);
	VCP_write(outbuff, strlen(outbuff));

	ExpLedOff(ORANGE_LED);
}

void freeimu(void)
{
	static FreeIMU_Func fimu_funcs;

	fimu_funcs.GetVersion = &FreeIMUVersion;
	fimu_funcs.GetIMURaw = &FreeIMURaw;
	fimu_funcs.GetBaseValues = &FreeIMUBaseInt;
//	fimu_funcs.GetQuat = &send_quaternion;
	fimu_funcs.GetAttitude = &FreeIMUSendYawPitchRoll;
	fimu_funcs.GetAttitudeBytes = &send_attitude_bytes;
	fimu_funcs.LoadCalibrationValues = &FreeIMUReadCalibration;
	fimu_funcs.Calibrate = &FreeIMUWriteCalibration;
	fimu_funcs.ClearCalibration = NULL;//&FreeIMUClearCalibration;
	fimu_funcs.FormatFlash = &FreeIMUFormatFlash;

	for(;;){
		FreeIMU_serial(&fimu_funcs);
	}
}

int main(void)
{
	char outbuff[100];
	FreeIMU_Func fimu_funcs;

	SystemClock_Config();

	HAL_Init();

	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
//	BSP_LED_Init(LED6);

	USBD_Init(&USBD_Device, &VCP_Desc, 0);

	USBD_RegisterClass(&USBD_Device, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_Template_fops);
	USBD_Start(&USBD_Device);

	ExpLedInit();
	ExpBuzzerInit();
//	EXTILine0_Config();

	ExpBuzzerOff();

	BSP_LED_On(LED3);

#if 0
	// Pause until told to go
	{
		char in[5];
		in[0] = 0;

		while(1){
			VCP_read(in, 5);

			if(memcmp(in , "go", 2) == 0){
				VCP_write("Ok\r\n", 4);
				break;
			}
		}
	}
#endif

	if(EEPROMInit() < 0){
		ExpLedOn(RED_LED);
		ExpBuzzerTune(BUZZER_2_SHORTS_LONG);

		while(1){
			FreeIMU_serial(&fimu_funcs);
			ExpBuzzerHandler();
		}
	}

	ExpLedOn(ORANGE_LED);

	TIM_Config();
	ESC_Init();
//	AltInit();
	if(ImuInit() < 0){
		ExpLedOn(RED_LED);
		ExpBuzzerTune(BUZZER_2_SHORTS_LONG);

		while(1){
			FreeIMU_serial(&fimu_funcs);
			ExpBuzzerHandler();
		}
	}

	MyPidInit();

	ESC_Start(1);
	ESC_Start(2);
	ESC_Start(3);
	ESC_Start(4);

	fimu_funcs.GetVersion = &FreeIMUVersion;
	fimu_funcs.GetIMURaw = &FreeIMURaw;
	fimu_funcs.GetBaseValues = &FreeIMUBaseInt;
//	fimu_funcs.GetQuat = &send_quaternion;
	fimu_funcs.GetAttitude = &FreeIMUSendYawPitchRoll;
	fimu_funcs.LoadCalibrationValues = &FreeIMUReadCalibration;
	fimu_funcs.Calibrate = &FreeIMUWriteCalibration;
	fimu_funcs.ClearCalibration = &FreeIMUClearCalibration;
	fimu_funcs.GetAttitudeBytes = &send_attitude_bytes;
	fimu_funcs.FormatFlash = &FreeIMUFormatFlash;

	ExpLedOff(ORANGE_LED);
//freeimu();
#define IMU_TIME		1
#define RX_TIME			3
#define MOTOR_TIME		3
#define COMMS_TIME		10

	uint32_t current = HAL_GetTick();
	uint32_t rx_time = current + 500;
	uint32_t motor_time = current + MOTOR_TIME;
	uint32_t comms_time = current + COMMS_TIME;		// Update comms every 100ms
	uint32_t imu_time = current + IMU_TIME;
	uint16_t speed_step=1;

	double throttle_window=0.0;

	memset(&rx_channel, 0, sizeof(PwmInfo));

	PidSetSampleTime(&PIDRoll, MOTOR_TIME);

/*
	CalibVals calibration2;
	memcpy(&calibration2, "\x7d\xff\x24\x02\xda\xfe\xc6\xff\xac\xff\xb8\xff\xd7\x5a\x7e\x46\xdf\xf8\x81\x46\x7b\xcc\x84\x46\x82\xa9\x46\x43\x23\xc6\x84\x43\xc4\x1e\x55\x43",sizeof(CalibVals));
	EEPROMSet(VAR_CALIBRATION, (uint8_t*)&calibration2);
*/

	for(;;){

		current = HAL_GetTick();

		if(current > rx_time)
			break;

		// Check for high throttle on startup, indicates ESC calibration
		GetPwmInfo(&rx_channel);

		if((rx_channel.pwmval3 >= RX_CHAN_DEFAULT_UPPER_LIMIT_LOW) && (rx_channel.pwmval3 <= RX_CHAN_DEFAULT_UPPER_LIMIT_HIGH)){
			// Throttle is max
			calibrating_flag = 1;
			ExpLedOn(ORANGE_LED);

			ExpBuzzerTune(BUZZER_CALIBRATION_START);

			break;
		}
	}

	rx_time = current + RX_TIME;			// Check receiver every 30ms

	for(;;){
		ExpBuzzerHandler();

		current = HAL_GetTick();

//		if(current > imu_time)
		{
			ImuYawPitchRoll(&yaw, &pitch, &roll);
			// Update loop time
			imu_time = HAL_GetTick() + IMU_TIME;
		}

		FreeIMU_serial(&fimu_funcs);

		if(current > rx_time)
		{

			// Process receiver input
			GetPwmInfo(&rx_channel);

			if((rx_channel.pwmval3 >= RX_CHAN_DEFAULT_LOWER_LIMIT_LOW) && (rx_channel.pwmval3 <= RX_CHAN_DEFAULT_LOWER_LIMIT_HIGH)
					&& (rx_channel.pwmval4 >= RX_CHAN_DEFAULT_LOWER_LIMIT_LOW) && (rx_channel.pwmval4 <= RX_CHAN_DEFAULT_LOWER_LIMIT_HIGH)
					&& ((armed_flag == 1) || (calibrating_flag == 1))){
				// Throttle is lower left corner
				armed_flag = 0;
				calibrating_flag = 0;

				ExpLedOff(ORANGE_LED);

				ExpBuzzerTune(BUZZER_2_LONGS);
			}else if((rx_channel.pwmval3 >= RX_CHAN_DEFAULT_LOWER_LIMIT_LOW) && (rx_channel.pwmval3 <= RX_CHAN_DEFAULT_LOWER_LIMIT_HIGH)
					&& (rx_channel.pwmval4 >= RX_CHAN_DEFAULT_UPPER_LIMIT_LOW) && (rx_channel.pwmval4 <= RX_CHAN_DEFAULT_UPPER_LIMIT_HIGH)
					&& (armed_flag == 0) && (calibrating_flag == 0)){
				// Throttle is lower right corner
				armed_flag = 1;

				ExpBuzzerTune(BUZZER_2_SHORTS);
			}

			// Update loop time
			rx_time = HAL_GetTick() + RX_TIME;
		}

		if(!armed_flag){
			ExpLedOff(GREEN_LED);

			ESC_Speed(ESC_OFF, 1);
			ESC_Speed(ESC_OFF, 2);
			ESC_Speed(ESC_OFF, 3);
			ESC_Speed(ESC_OFF, 4);
		}

		if(armed_flag){
			ExpLedOn(GREEN_LED);

			if(PIDRoll.kp != (double)((rx_channel.pwmval1 - 1100)/25))
				PidSetTunings(&PIDRoll, (rx_channel.pwmval1 - 1100)/25, 0, 0);

			PIDRoll.Input = (double)roll;

			if(PidCompute(&PIDRoll)){

				throttle_window = (((double)rx_channel.pwmval3 * 5) / 100) * (PIDRoll.Output/15);	// throttle_window = 5% of throttle value /

				motor_speed.motor1 = rx_channel.pwmval3 - (uint32_t)throttle_window;
				motor_speed.motor2 = rx_channel.pwmval3 + (uint32_t)throttle_window;
			}

			ESC_Update();
		}

		if(calibrating_flag){
			motor_speed.motor1 = rx_channel.pwmval3;
			motor_speed.motor2 = rx_channel.pwmval3;
			motor_speed.motor3 = rx_channel.pwmval3;
			motor_speed.motor4 = rx_channel.pwmval3;

			ESC_Update();

		}
#if 0
		if(current > comms_time)
		{
			int s1 = rx_channel.pwmval3 - (uint32_t)throttle_window;
			int s2 = rx_channel.pwmval3 + (uint32_t)throttle_window;
			int kp = (rx_channel.pwmval1 - 1100)/25;

			sprintf(outbuff, "R: %04d T: %04d kP: %04d s2: %04d s1: %04d step: %04d error: %04d\r\n", (int)roll, (int)rx_channel.pwmval3, kp, s2, s1, (int)throttle_window, (int)PIDRoll.Output);
			VCP_write(outbuff, strlen(outbuff));

			// Update loop time
			comms_time = HAL_GetTick() + COMMS_TIME;
		}
#endif
	}
}

void unarmed_process(PwmInfo *pwm)
{
	GetPwmInfo(pwm);

	if((pwm->pwmval3 == 0) || (pwm->pwmval4 == 0))
		return;

#ifdef DEBUG_ON
	char outbuff[60];
	int count=0;
	sprintf(outbuff, "%d  ->  %04d  %04d  %04d  %04d\r\n", ++count, (int)pwm->pwmval1, (int)pwm->pwmval2, (int)pwm->pwmval3, (int)pwm->pwmval4);
//	VCP_write(outbuff, strlen(outbuff));
#endif // DEBUG_ON

	if((pwm->pwmval3 >= 1000) &&(pwm->pwmval3 <= 1150) && (pwm->pwmval4 >= 1900) && (pwm->pwmval4 <= 2000)){
		ExpBuzzerOn();
		HAL_Delay(500);
		ExpBuzzerOff();

//		ESC_Start(1);
//		ESC_Start(2);
//		ESC_Start(3);
		ESC_Start(4);

		armed_flag = 1;
	}
}

void armed_process(PwmInfo *pwm)
{
	GetPwmInfo(pwm);

	if((pwm->pwmval3 == 0) || (pwm->pwmval4 == 0))
		return;

#ifdef DEBUG_ON
	char outbuff[60];
	int count=0;
	sprintf(outbuff, "%d  ->  %04d  %04d  %04d  %04d\r\n", ++count, (int)pwm->pwmval1, (int)pwm->pwmval2, (int)pwm->pwmval3, (int)pwm->pwmval4);
//	VCP_write(outbuff, strlen(outbuff));
#endif // DEBUG_ON
	// Default ranges for upper and lower rx limits


	if((pwm->pwmval3 >= RX_CHAN_DEFAULT_LOWER_LIMIT_LOW) && (pwm->pwmval3 <= RX_CHAN_DEFAULT_LOWER_LIMIT_HIGH)
		&& (pwm->pwmval4 >= RX_CHAN_DEFAULT_LOWER_LIMIT_LOW) && (pwm->pwmval4 <= RX_CHAN_DEFAULT_LOWER_LIMIT_HIGH)){
		ESC_Speed(0, 1);
		ESC_Speed(0, 2);
		ESC_Speed(0, 3);
		ESC_Speed(0, 4);

		ExpBuzzerOn();
		HAL_Delay(500);
		ExpBuzzerOff();
		HAL_Delay(250);
		ExpBuzzerOn();
		HAL_Delay(500);
		ExpBuzzerOff();

		armed_flag = 0;
VCP_write(outbuff, strlen(outbuff));
	}
}

void MyPidInit(void)
{
	memset(&PIDYaw, 0, sizeof(Pid));
	memset(&PIDRoll, 0, sizeof(Pid));
	memset(&PIDPitch, 0, sizeof(Pid));

	PidSetOutputLimits(&PIDYaw, 1000, 2000);
	PidSetOutputLimits(&PIDRoll, -15, 15);
	PidSetOutputLimits(&PIDPitch, 1000, 2000);

	PidSetMode(&PIDYaw, AUTOMATIC);
	PidSetMode(&PIDRoll, AUTOMATIC);
	PidSetMode(&PIDPitch, AUTOMATIC);
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
