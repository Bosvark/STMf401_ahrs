#include <math.h>
#include "stm32f4xx_hal.h"
#include "altitude.h"

#define BMP085_CONTROL           0xF4
#define BMP085_TEMPDATA          0xF6
#define BMP085_PRESSUREDATA      0xF6
#define BMP085_READTEMPCMD       0x2E
#define BMP085_READPRESSURECMD   0x34

#define BMP085_ULTRALOWPOWER 0
#define BMP085_STANDARD      1
#define BMP085_HIGHRES       2
#define BMP085_ULTRAHIGHRES  3

const float sealevelPressure = 101325.0;

static I2C_HandleTypeDef I2cAltHandle;

static int16_t ac1=0;
static int16_t ac2=0;
static int16_t ac3=0;
static uint16_t ac4=0;
static uint16_t ac5=0;
static uint16_t ac6=0;
static int16_t b1=0;
static int16_t b2=0;
static int16_t mb=0;
static int16_t mc=0;
static int16_t md=0;

static uint8_t oversampling=BMP085_ULTRALOWPOWER;//BMP085_ULTRAHIGHRES;

//static int32_t UT=0, B5=0;

static uint32_t I2cxTimeout = ALTITUDE_I2Cx_TIMEOUT_MAX;

void read_calibration(void);
uint16_t readRawTemperature(void);
uint32_t readRawPressure(void);
int32_t computeB5(int32_t UT);
static void Write8(uint8_t reg, uint8_t val);
static uint16_t Read8(uint8_t reg);
static uint16_t Read16(uint8_t reg);
static void WriteData(uint8_t Reg);
static void I2Cx_Error (void);

int AltInit(void)
{
	I2cAltHandle.Instance = ALTITUDE_I2Cx;
	I2cAltHandle.Init.OwnAddress1 =  0x43;
	I2cAltHandle.Init.ClockSpeed = ALTITUDE_I2Cx_MAX_COMMUNICATION_FREQ;
	I2cAltHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
	I2cAltHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	I2cAltHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
	I2cAltHandle.Init.OwnAddress2 = 0x00;
	I2cAltHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
	I2cAltHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;

	GPIO_InitTypeDef GPIO_InitStructure;

	ALTITUDE_I2Cx_CLOCK_ENABLE();
	ALTITUDE_I2Cx_GPIO_CLK_ENABLE();

	GPIO_InitStructure.Pin = ALTITUDE_I2Cx_SDA_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Alternate = ALTITUDE_I2Cx_AF;
	HAL_GPIO_Init(ALTITUDE_I2Cx_GPIO_PORT_SDA, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = ALTITUDE_I2Cx_SCL_PIN;
	HAL_GPIO_Init(ALTITUDE_I2Cx_GPIO_PORT_SCL, &GPIO_InitStructure);

	ALTITUDE_I2Cx_FORCE_RESET();
	ALTITUDE_I2Cx_RELEASE_RESET();

	/* Enable and set I2Cx Interrupt to the highest priority */
	HAL_NVIC_SetPriority(ALTITUDE_I2Cx_EV_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ALTITUDE_I2Cx_EV_IRQn);

	/* Enable and set I2Cx Interrupt to the highest priority */
	HAL_NVIC_SetPriority(ALTITUDE_I2Cx_ER_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ALTITUDE_I2Cx_ER_IRQn);

	HAL_I2C_Init(&I2cAltHandle);

	read_calibration();

	return 0;
}

float AltReadTemperature(void)
{
  int32_t UT, B5;     // following ds convention
  float temp=0.0f;

  UT = (int32_t)readRawTemperature();

  B5 = computeB5(UT);
  temp = (B5+8) >> 4;
  temp /= 10;

  return temp;
}

int32_t AltReadPressure(void)
{
  int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
  uint32_t B4, B7;

  UT = readRawTemperature();
  UP = readRawPressure();
/*
#if BMP085_DEBUG == 1
  // use datasheet numbers!
  UT = 27898;
  UP = 23843;
  ac6 = 23153;
  ac5 = 32757;
  mc = -8711;
  md = 2868;
  b1 = 6190;
  b2 = 4;
  ac3 = -14383;
  ac2 = -72;
  ac1 = 408;
  ac4 = 32741;
  oversampling = 0;
#endif
*/
  B5 = computeB5(UT);

  // do pressure calcs
  B6 = B5 - 4000;
  X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;

  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );

  if (B7 < 0x80000000) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

  p = p + ((X1 + X2 + (int32_t)3791)>>4);

  return p;
}

float AltReadAltitude(void)
{
  float altitude;

  float pressure = (float)AltReadPressure();

  altitude = 44330.0 * (1.0 - powf(pressure /sealevelPressure,0.1903));

  return altitude;
}

// 9F1B
// FBE4
// C7D8
// 82F0
// 63B4
// 32AD
// 1973
// 0024
// 8000
// D1F6
// 09C5
// -31204,-1052,-14376,33520,25524,12973,6515,36,-32768,-11786,2501,32768,14071
// 1C86E4FBD8C7F082B463AD32731924000080F6D1C5095C4200800000F7360000,
// Temp: 87
// -31204,-1052,-14376,33520,25524,12973,6515,36,-32768,-11786,2501,32768,14071


// Arduino
//ac1 = 8363
//ac2 = -1052
//ac3 = -14376
//ac4 = 33520
//ac5 = 25524
//ac6 = 12973
//b1 = 6515
//b2 = 36
//mb = -32768
//mc = -11786
//md = 2501
//Temperature = Raw temp: 22054
//28.50 *C
//Pressure = Raw temp: 22053
//Raw pressure: 293723
//X1 = 0
//X2 = 0
//B5 = 4551
//B6 = 551
//X1 = 1
//X2 = -284
//B3 = 66338
//X1 = -967
//X2 = 7
//B4 = 33274
//B7 = 1421156250
//p = 85421
//X1 = 5140
//X2 = -9590
//p = 85379
//85379 Pa


void read_calibration(void)
{

	if(Read8(0xD0) != 0x55)
		return;

	ac1 = (int16_t)Read16(0xAA);
	ac2 = (int16_t)Read16(0xAC);
	ac3 = (int16_t)Read16(0xAE);
	ac4 = Read16(0xB0);
	ac5 = Read16(0xB2);
	ac6 = Read16(0xB4);
	b1 = (int16_t)Read16(0xB6);
	b2 = (int16_t)Read16(0xB8);
	mb = (int16_t)Read16(0xBA);
	mc = (int16_t)Read16(0xBC);
	md = (int16_t)Read16(0xBE);

#if 0
	// use datasheet numbers!
	ac6 = 23153;
	ac5 = 32757;
	mc = -8711;
	md = 2868;
	b1 = 6190;
	b2 = 4;
	ac3 = -14383;
	ac2 = -72;
	ac1 = 408;
	ac4 = 32741;
#endif
}

uint16_t readRawTemperature(void)
{
//  Write8(BMP085_CONTROL, BMP085_READTEMPCMD);
uint8_t val[2];
val[0] = BMP085_CONTROL;
val[1] = BMP085_READTEMPCMD;
  HAL_I2C_Master_Transmit(&I2cAltHandle, 0xee, val, 2, I2cxTimeout);
  HAL_Delay(5);

  return Read16(BMP085_TEMPDATA);
}

uint32_t readRawPressure(void)
{
  uint32_t raw;
  uint8_t val[2];

  val[0] = BMP085_CONTROL;
  val[1] = BMP085_READPRESSURECMD+ (oversampling << 6);

  HAL_I2C_Master_Transmit(&I2cAltHandle, 0xee, val, 2, I2cxTimeout);

  if (oversampling == BMP085_ULTRALOWPOWER)
	  HAL_Delay(5);
  else if (oversampling == BMP085_STANDARD)
	  HAL_Delay(8);
  else if (oversampling == BMP085_HIGHRES)
	  HAL_Delay(14);
  else
	  HAL_Delay(26);

  raw = Read16(BMP085_PRESSUREDATA);

  raw <<= 8;
  raw |= Read16(BMP085_PRESSUREDATA+2);
  raw >>= (8 - oversampling);

 /* this pull broke stuff, look at it later?
  if (oversampling==0) {
    raw <<= 8;
    raw |= read8(BMP085_PRESSUREDATA+2);
    raw >>= (8 - oversampling);
  }
 */

  return raw;
}

int32_t computeB5(int32_t UT)
{
  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1+(int32_t)md);
  return X1 + X2;
}

static void Write8(uint8_t reg, uint8_t val)
{
	WriteData(reg);
	WriteData(val);
}

static uint16_t Read8(uint8_t reg)
{
	uint8_t val=0;

	WriteData(reg);

	HAL_I2C_Master_Receive(&I2cAltHandle, 0xef, (uint8_t*)&val, 1, I2cxTimeout);

	return val;
}

static uint16_t Read16(uint8_t reg)
{
	uint8_t val=0;
	uint16_t value = 0;

	WriteData(reg);

	HAL_I2C_Master_Receive(&I2cAltHandle, 0xef, (uint8_t*)&val, 1, I2cxTimeout);
	value = val << 8;
	HAL_I2C_Master_Receive(&I2cAltHandle, 0xef, (uint8_t*)&val, 1, I2cxTimeout);
	value += val;

	return value;
}

static void WriteData(uint8_t Reg)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Master_Transmit(&I2cAltHandle, 0xee, &Reg, 1, I2cxTimeout);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2Cx_Error();
  }
}

static void I2Cx_Error (void)
{
  /* De-initialize the I2C comunication BUS */
  HAL_I2C_DeInit(&I2cAltHandle);

  /* Re- Initiaize the I2C comunication BUS */
  AltInit();
}

void altdebug(ALTDBG *dbg)
{
	dbg->ac2 = ac2;
	dbg->ac3 = ac3;
	dbg->ac4 = ac4;
	dbg->ac5 = ac5;
	dbg->ac6 = ac6;
	dbg->b1  = b1;
	dbg->b2  = b2;
	dbg->mb  = mb;
	dbg->mc  = mc;
	dbg->md  = md;
}
