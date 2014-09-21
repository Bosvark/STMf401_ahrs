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

//
////////////////////////////////////////////////////
//
TIM_HandleTypeDef        TimHandle;

/* Captured Value */
__IO uint32_t            uwIC2Value = 0;
/* Duty Cycle Value */
__IO uint32_t            uwDutyCycle = 0;
/* Frequency Value */
__IO uint32_t            uwFrequency = 0;

void TIM_Config(void)
{
//	TIM_HandleTypeDef        TimHandle;

	/* Timer Input Capture Configuration Structure declaration */
	TIM_IC_InitTypeDef       sConfig;

	/* Slave configuration structure */
	TIM_SlaveConfigTypeDef   sSlaveConfig;

	/*##-1- Configure the TIM peripheral #######################################*/
	  /* Set TIMx instance */
	  TimHandle.Instance = TIMx;

	  /* Initialize TIMx peripheral as follow:
	       + Period = 0xFFFF
	       + Prescaler = 0
	       + ClockDivision = 0
	       + Counter direction = Up
	  */
	  TimHandle.Init.Period = 0xFFFF;
	  TimHandle.Init.Prescaler = 0;
	  TimHandle.Init.ClockDivision = 0;
	  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	  HAL_TIM_IC_Init(&TimHandle);

	  /*##-2- Configure the Input Capture channels ###############################*/
	  /* Common configuration */
	  sConfig.ICPrescaler = TIM_ICPSC_DIV1;
	  sConfig.ICFilter = 0;

	  /* Configure the Input Capture of channel 1 */
	  sConfig.ICPolarity = TIM_ICPOLARITY_FALLING;
	  sConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	  HAL_TIM_IC_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1);

	  /* Configure the Input Capture of channel 2 */
	  sConfig.ICPolarity = TIM_ICPOLARITY_RISING;
	  sConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
	  HAL_TIM_IC_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2);

	  /*##-3- Configure the slave mode ###########################################*/
	  /* Select the slave Mode: Reset Mode */
	  sSlaveConfig.SlaveMode     = TIM_SLAVEMODE_RESET;
	  sSlaveConfig.InputTrigger  = TIM_TS_TI2FP2;
	  HAL_TIM_SlaveConfigSynchronization(&TimHandle, &sSlaveConfig);

	  /*##-4- Start the Input Capture in interrupt mode ##########################*/
	  HAL_TIM_IC_Start_IT(&TimHandle, TIM_CHANNEL_2);

	  /*##-5- Start the Input Capture in interrupt mode ##########################*/
	  HAL_TIM_IC_Start_IT(&TimHandle, TIM_CHANNEL_1);
}

void TIMx_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
	  ExpLedToggle(ORANGE_LED);
    /* Get the Input Capture value */
    uwIC2Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    if (uwIC2Value != 0)
    {
      /* Duty cycle computation */
      uwDutyCycle = ((HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1)) * 100) / uwIC2Value;

      /* uwFrequency computation
      TIM4 counter clock = (RCC_Clocks.HCLK_Frequency) */
      uwFrequency = (HAL_RCC_GetHCLKFreq()) / uwIC2Value;
    }
    else
    {
      uwDutyCycle = 0;
      uwFrequency = 0;
    }
  }
}

int GetPwmInfo(PwmInfo *pwm)
{
	  pwm->Freq = uwIC2Value;
	  pwm->dutyCycle = uwDutyCycle;
	  pwm->icVal = uwIC2Value;

	  return 0;
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
  TIMx_CLK_ENABLE();

  /* Enable GPIO channels Clock */
  TIMx_CHANNEL_GPIO_PORT();

  /* Configure  (TIMx_Channel) in Alternate function, push-pull and 100MHz speed */
  GPIO_InitStruct.Pin = GPIO_PIN_CHANNEL2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF_TIMx;
  HAL_GPIO_Init(GPIO_PORT, &GPIO_InitStruct);

  /*##-2- Configure the NVIC for TIMx #########################################*/
  /* Sets the priority grouping field */
  HAL_NVIC_SetPriority(TIMx_IRQn, 0, 1);

  /* Enable the TIM4 global Interrupt */
  HAL_NVIC_EnableIRQ(TIMx_IRQn);
}
