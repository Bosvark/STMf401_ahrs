#ifndef _ALT_H_
#define _ALT_H_

#define ALTITUDE_I2Cx                        I2C3
#define ALTITUDE_I2Cx_CLOCK_ENABLE()         __I2C3_CLK_ENABLE()
#define ALTITUDE_I2Cx_GPIO_PORT_SCL          GPIOA
#define ALTITUDE_I2Cx_GPIO_PORT_SDA          GPIOC
#define ALTITUDE_I2Cx_SCL_PIN                GPIO_PIN_8
#define ALTITUDE_I2Cx_SDA_PIN                GPIO_PIN_9
#define ALTITUDE_I2Cx_GPIO_CLK_ENABLE()      {__GPIOA_CLK_ENABLE();__GPIOC_CLK_ENABLE();}
#define ALTITUDE_I2Cx_GPIO_CLK_DISABLE()     {__GPIOA_CLK_DISABLE();__GPIOC_CLK_DISABLE();}
#define ALTITUDE_I2Cx_AF                     GPIO_AF4_I2C3

#define ALTITUDE_I2Cx_FORCE_RESET()          __I2C3_FORCE_RESET()
#define ALTITUDE_I2Cx_RELEASE_RESET()        __I2C3_RELEASE_RESET()

/* I2C interrupt requests */
#define ALTITUDE_I2Cx_EV_IRQn                I2C3_EV_IRQn
#define ALTITUDE_I2Cx_ER_IRQn                I2C3_ER_IRQn

/* I2C speed and timeout max */
#define ALTITUDE_I2Cx_TIMEOUT_MAX			0xA000
#define ALTITUDE_I2Cx_MAX_COMMUNICATION_FREQ	((uint32_t) 100000)

int AltInit(void);


typedef struct
{
	int16_t ac1;
	int16_t ac2;
	int16_t ac3;
	uint16_t ac4;
	uint16_t ac5;
	uint16_t ac6;
	int16_t b1;
	int16_t b2;
	int16_t mb;
	int16_t mc;
	int16_t md;
}ALTDBG;

void altdebug(ALTDBG *dbg);
float AltReadTemperature(void);
int32_t AltReadPressure(void);
float AltReadAltitude(void);

#endif // _ALT_H_
