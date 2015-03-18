#ifndef _EXP_BOARD_H_
#define _EXP_BOARD_H_

#include "stm32f4xx_hal.h"

#define LED_PORT			GPIOD
#define LED_CLK_ENABLE()	__GPIOD_CLK_ENABLE()
#define GREEN_LED			GPIO_PIN_9
#define ORANGE_LED			GPIO_PIN_10
#define RED_LED				GPIO_PIN_11

#define BUZZER_PORT			GPIOC
#define BUZZER_CLK_ENABLE()	__GPIOC_CLK_ENABLE()
#define BUZZER				GPIO_PIN_2

void ExpLedInit(void);
void ExpLedOn(int led);
void ExpLedOff(int led);
void ExpLedToggle(int led);

typedef enum
{
	BUZZER_2_SHORTS,
	BUZZER_2_LONGS,
	BUZZER_CALIBRATION_START,
	BUZZER_2_SHORTS_LONG
}BUZZER_TUNE;

void ExpBuzzerInit(void);
void ExpBuzzerOn(void);
void ExpBuzzerOff(void);
void ExpBuzzerToggle(void);
void ExpBuzzerTune(BUZZER_TUNE tune);
void ExpBuzzerHandler(void);

//
///////////////////////////////////////////////////////////////////
//
void EXTILine0_Config(void);

#define GPIO_PORT                      GPIOB

#define TIM4x                           TIM4
#define TIM4x_CLK_ENABLE()              __TIM4_CLK_ENABLE()

/* Definition for TIM4x Pins */
#define TIM4x_CHANNEL_GPIO_PORT()       __GPIOB_CLK_ENABLE()
#define GPIO_PIN_TIM4x_CHANNEL_2_4      GPIO_PIN_7// | GPIO_PIN_9
#define GPIO_AF_TIM4x                   GPIO_AF2_TIM4
#define TIM4x_IRQn                      TIM4_IRQn
#define TIM4x_IRQHandler                TIM4_IRQHandler

#define TIM3x                           TIM3
#define TIM3x_CLK_ENABLE()              __TIM3_CLK_ENABLE()

/* Definition for TIM3x Pins */
#define TIM3x_CHANNEL_GPIO_PORT()       __GPIOB_CLK_ENABLE()
#define GPIO_PIN_TIM3x_CHANNEL_2_4      GPIO_PIN_1 | GPIO_PIN_5
#define GPIO_AF_TIM3x                   GPIO_AF2_TIM3
#define TIM3x_IRQn                      TIM3_IRQn
#define TIM3x_IRQHandler                TIM3_IRQHandler

typedef struct
{
	uint32_t dutyCycle1;
	uint32_t dutyCycle2;
	uint32_t dutyCycle3;
	uint32_t dutyCycle4;

	uint32_t pwmval1;
	uint32_t pwmval2;
	uint32_t pwmval3;
	uint32_t pwmval4;
}PwmInfo;

// Default ranges for upper and lower rx limits
#define RX_CHAN_DEFAULT_UPPER_LIMIT_HIGH	2000
#define RX_CHAN_DEFAULT_UPPER_LIMIT_LOW		1850
#define RX_CHAN_DEFAULT_LOWER_LIMIT_HIGH	1150
#define RX_CHAN_DEFAULT_LOWER_LIMIT_LOW		1000
#define RX_CHAN_DEFAULT_MIDPOINT			1500

void TIM_Config(void);
int GetPwmInfo(PwmInfo *pwm);

#endif // _EXP_BOARD_H_
