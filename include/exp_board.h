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




void EXTILine0_Config(void);

#endif // _EXP_BOARD_H_
