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

#define DEBUG_ON

//#define SWAP_AXIS

#define VERSION_MAJOR	1
#define VERSION_MINOR	1

USBD_HandleTypeDef USBD_Device;

static char armed_flag = 0;
static char calibrating_flag = 0;

extern PwmInfo rx_channel;

void hex_to_ascii(const unsigned char *source, char *dest, unsigned int source_length);
static void SystemClock_Config(void);

void unarmed_process(PwmInfo *pwm);
void armed_process(PwmInfo *pwm);
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
int main(void)
{
	char outbuff[100];

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

		while(1){
			ExpBuzzerToggle();
			HAL_Delay(300);
		}
	}

	ImuInit();

	TIM_Config();
	ESC_Init();
	AltInit();

	PIDInit();

	ESC_Start(1);
	ESC_Start(2);
	ESC_Start(3);
	ESC_Start(4);

#define RX_TIME			3
#define COMMS_TIME		10

	uint32_t current = HAL_GetTick();
	uint32_t rx_time = current + 500;
	uint32_t comms_time = current + COMMS_TIME;		// Update comms every 100ms

	int motorspeed = 1000;
	int speed_step = 1;

	float min=100.0, max=-100.0;

	memset(&rx_channel, 0, sizeof(PwmInfo));


	for(;;){

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

		ImuYawPitchRoll(NULL, NULL, &roll);

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

			PIDYaw.vl_PreU = 1;
			PIDPitch.vl_PreU = 1;
			PIDRoll.vl_PreU = 1;
		}

		if(armed_flag){
			ExpLedOn(GREEN_LED);

			// Apply throttle

			PIDRoll.v_Kp = (rx_channel.pwmval1 - 1100)/25;
			PIDRoll.vi_Ref = 0;
			PIDRoll.vi_FeedBack = roll;
			PIDRoll.vl_PreU = PIDCalc(&PIDRoll);

			motor_speed.motor1 = rx_channel.pwmval3 - PIDRoll.vl_PreU;
			motor_speed.motor2 = rx_channel.pwmval3 + PIDRoll.vl_PreU;

			ESC_Update();

		}

		if(calibrating_flag){
			motor_speed.motor1 = rx_channel.pwmval3;
			motor_speed.motor2 = rx_channel.pwmval3;
			motor_speed.motor3 = rx_channel.pwmval3;
			motor_speed.motor4 = rx_channel.pwmval3;

			ESC_Update();

		}

		if(current > comms_time)
		{

//			sprintf(outbuff, "Armed %d Roll: %04d min: %04d max: %04d Throttle: %04d kP: %04d\r\n", (int)armed_flag, (int)roll, (int)min, (int)max, (int)rx_channel.pwmval3, (int)((rx_channel.pwmval1 - 1100)/25));
//			sprintf(outbuff, "Armed %d Throttle: %04d Rudder: %04d kP: %04d\r\n", (int)armed_flag, (int)rx_channel.pwmval3, (int)rx_channel.pwmval4, (int)((rx_channel.pwmval1 - 1100)/25));
			sprintf(outbuff, "Roll: %04d Throttle: %04d kP: %04d speed: %04d step: %04d\r\n", (int)roll, (int)rx_channel.pwmval3, (int)(rx_channel.pwmval1 - 1100)/25, (int)motor_speed.motor1, (int)PIDRoll.vl_PreU);
			VCP_write(outbuff, strlen(outbuff));

			// Update loop time
			comms_time = HAL_GetTick() + COMMS_TIME;
		}
/*
		if(armed_flag == 0){
			ExpLedOn(RED_LED);
			ExpLedOff(GREEN_LED);

			motorspeed = 1000;
			speed_step = 1;

			if(current % 2 == 0){	// 500HZ
				ImuYawPitchRoll(NULL, NULL, &roll);
			}

			if(current % 30 == 10){	// 30Hz
				ExpLedToggle(ORANGE_LED);

				PwmInfo pwm;
				GetPwmInfo(&pwm);

				unarmed_process(&pwm);

				if(armed_flag == 1)
					continue;

				ImuYawPitchRoll(NULL, NULL, &roll);
				roll = roll * -1;

				if(roll < min)
					min = roll;

				if(roll > max)
					max = roll;

				sprintf(outbuff, "Roll: %04d min: %04d max: %04d Throttle: %04d kP: %04d\r\n", (int)roll, (int)min, (int)max, (int)pwm.pwmval3, (int)((pwm.pwmval1 - 1100)/25));
				VCP_write(outbuff, strlen(outbuff));

				ESC_Speed(motorspeed, 4);
			}

#if 0
			PwmInfo pwm;
			GetPwmInfo(&pwm);
			sprintf(outbuff, "Channel 1: %04d\r\n", pwm.pwmval1);
			VCP_write(outbuff, strlen(outbuff));
#endif
		}else{
			ExpLedOff(RED_LED);
			ExpLedOn(GREEN_LED);

			if(current % 2 == 0){	// 500HZ
				ImuYawPitchRoll(NULL, NULL, &roll);
			}

			if(current % 30 == 10){	// 30Hz
				ExpLedToggle(ORANGE_LED);

				roll = roll * -1;

				PwmInfo pwm;
				GetPwmInfo(&pwm);

				armed_process(&pwm);

				if(armed_flag == 0)
					continue;

				//
				// SAFETY FIRST!!!
				//
				if((rx_channel.pwmval3 >= 1000) && (rx_channel.pwmval3 <= 1800))
					motorspeed = rx_channel.pwmval3;

				if((rx_channel.pwmval1 >= 1000) && (rx_channel.pwmval1 <= 1800))
					PIDRoll.v_Kp = (rx_channel.pwmval1 - 1100)/25;

				PIDRoll.vi_Ref = 0;
				PIDRoll.vi_FeedBack = roll;
				PIDRoll.vl_PreU = speed_step;
				speed_step = PIDCalc(&PIDRoll);

				sprintf(outbuff, "Roll: %04d  Speed: %04d speed_step: %04d Throttle: %04d kP: %04d\r\n", (int)roll, (int)motorspeed+speed_step, speed_step, rx_channel.pwmval3, (int)PIDRoll.v_Kp);
				VCP_write(outbuff, strlen(outbuff));

//				ESC_Speed(motorspeed+speed_step, 4);
			}
		}
*/
	}

/*
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
	}
*/
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
		ImuYawPitchRoll(NULL, &pitch, NULL);

//		int servo_pitch=1000 + (pitch*(1000.0/90.0));
//		ESC_Speed(servo_pitch, 1);

//		sprintf(outbuff, "Pitch -> %d\r\n", (int)pitch);
//		VCP_write(outbuff, strlen(outbuff));

/*
		uint8_t id=0, type=0,cap=0;
		FlashMemChipID(&id, &type, &cap);
		sprintf(outbuff, "Chip 0x%02X 0x%02X 0x%02X\r\n", id, type, cap);
		VCP_write(outbuff, strlen(outbuff));
*/
/*
//		Moves a servo in step with angle
		float pitch=0.0;
		ImuYawPitchRoll(NULL, &pitch, NULL);

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
*/
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
