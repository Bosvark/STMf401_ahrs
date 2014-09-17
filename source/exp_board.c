#include "exp_board.h"

void ExpLedInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	/* Enable the GPIO_LED Clock */
	LED_CLK_ENABLE();

	/* Configure the GPIO_LED pin */
	GPIO_InitStruct.Pin = GREEN_LED | ORANGE_LED | RED_LED;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

	HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

	HAL_GPIO_WritePin(LED_PORT, GREEN_LED, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_PORT, ORANGE_LED, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_PORT, RED_LED, GPIO_PIN_SET);

}

void ExpLedOn(int led)
{
	HAL_GPIO_WritePin(LED_PORT, led, GPIO_PIN_RESET);
}

void ExpLedOff(int led)
{
	HAL_GPIO_WritePin(LED_PORT, led, GPIO_PIN_SET);
}

void ExpLedToggle(int led)
{
	HAL_GPIO_TogglePin(LED_PORT, led);
}

void ExpBuzzerInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	/* Enable the GPIO_LED Clock */
	BUZZER_CLK_ENABLE();

	/* Configure the GPIO_LED pin */
	GPIO_InitStruct.Pin = BUZZER;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

	HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);
}

void ExpBuzzerOn(void)
{
	HAL_GPIO_WritePin(BUZZER_PORT, BUZZER, GPIO_PIN_RESET);
}

void ExpBuzzerOff(void)
{
	HAL_GPIO_WritePin(BUZZER_PORT, BUZZER, GPIO_PIN_SET);
}

void EXTILine0_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOA clock */
  __GPIOA_CLK_ENABLE();

  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */

void EXTI0_IRQHandler()
{
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
		ExpLedToggle(RED_LED);
		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0));
		HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
	}
}
