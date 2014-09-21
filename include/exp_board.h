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

void ExpBuzzerInit(void);
void ExpBuzzerOn(void);
void ExpBuzzerOff(void);


//
///////////////////////////////////////////////////////////////////
//
void EXTILine0_Config(void);

#define TIMx                           TIM4
#define TIMx_CLK_ENABLE()              __TIM4_CLK_ENABLE()

/* Definition for TIMx Pins */
#define TIMx_CHANNEL_GPIO_PORT()       __GPIOB_CLK_ENABLE()
#define GPIO_PORT                      GPIOB
#define GPIO_PIN_CHANNEL2              GPIO_PIN_7
#define GPIO_AF_TIMx                   GPIO_AF2_TIM4

/* Definition for TIMx's NVIC */
#define TIMx_IRQn                      TIM4_IRQn
#define TIMx_IRQHandler                TIM4_IRQHandler

typedef struct
{
	uint32_t icVal;
	uint32_t dutyCycle;
	uint32_t Freq;
}PwmInfo;

void TIM_Config(void);
int GetPwmInfo(PwmInfo *pwm);

#endif // _EXP_BOARD_H_
