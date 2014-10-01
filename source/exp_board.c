#include "stm32f4xx_hal_dma.h"	// why? this is messy
#include "stm32f4xx_hal_tim.h"
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

void ExpBuzzerToggle(void)
{
	HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER);
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
TIM_HandleTypeDef        TimHandle3;
TIM_HandleTypeDef        TimHandle4;

__IO uint32_t uwDutyCycle1 = 0;
__IO uint32_t uwDutyCycle2 = 0;
__IO uint32_t uwDutyCycle3 = 0;
__IO uint32_t uwDutyCycle4 = 0;
__IO uint32_t pwmval1 = 0;
__IO uint32_t pwmval2 = 0;
__IO uint32_t pwmval3 = 0;
__IO uint32_t pwmval4 = 0;
__IO char intflag=0;

void TIM_Init(TIM_HandleTypeDef *timh)
{
	TIM_IC_InitTypeDef       sConfig;
	TIM_SlaveConfigTypeDef   sSlaveConfig;

	timh->Init.Period = 0xFFFF;
	timh->Init.Prescaler = 0;
//timh->Init.Period = 20000-1;
timh->Init.Prescaler = ((SystemCoreClock) / 1000000) - 1;	// 1Mhz
	timh->Init.ClockDivision = 0;
	timh->Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_IC_Init(timh);

	// Common configuration
	sConfig.ICPrescaler = TIM_ICPSC_DIV1;
	sConfig.ICFilter = 0;

	// Configure the Input Capture of channel 1
	sConfig.ICPolarity = TIM_ICPOLARITY_RISING;
	sConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	HAL_TIM_IC_ConfigChannel(timh, &sConfig, TIM_CHANNEL_1);

	// Configure the Input Capture of channel 2
	sConfig.ICPolarity = TIM_ICPOLARITY_FALLING;
	sConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
	HAL_TIM_IC_ConfigChannel(timh, &sConfig, TIM_CHANNEL_2);

	// Configure the Input Capture of channel 3
	sConfig.ICPolarity = TIM_ICPOLARITY_RISING;
	sConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	HAL_TIM_IC_ConfigChannel(timh, &sConfig, TIM_CHANNEL_3);

	// Configure the Input Capture of channel 4
	sConfig.ICPolarity = TIM_ICPOLARITY_FALLING;
	sConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
	HAL_TIM_IC_ConfigChannel(timh, &sConfig, TIM_CHANNEL_4);

	// Select the slave Mode: Reset Mode
	sSlaveConfig.SlaveMode     = TIM_SLAVEMODE_RESET;
	sSlaveConfig.InputTrigger  = TIM_TS_TI2FP2;
	HAL_TIM_SlaveConfigSynchronization(timh, &sSlaveConfig);

	HAL_TIM_IC_Start_IT(timh, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(timh, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(timh, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(timh, TIM_CHANNEL_4);
}

void TIM_Config(void)
{
	TimHandle3.Instance = TIM3x;
	TIM_Init(&TimHandle3);

	TimHandle4.Instance = TIM4x;
	TIM_Init(&TimHandle4);
}

void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&TimHandle3);
}

void TIM4_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&TimHandle4);
}
/*
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint32_t uwIC2Value, ccr;

	intflag = 0;

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){


		uwIC2Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);	// Input capture value

		if (uwIC2Value != 0)
		{
			if(htim->Instance == TIM3x){
				uwDutyCycle2 = ((HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1)) * 100) / uwIC2Value;
				intflag = 1;
			}else if(htim->Instance == TIM4x){
				ExpLedToggle(ORANGE_LED);
				uwDutyCycle3 = ((HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1)) * 100) / uwIC2Value;
				intflag = 1;
			}
		}
		else
		{
			if(htim->Instance == TIM3x)
				uwDutyCycle2 = 0;
			else if(htim->Instance == TIM4x)
				uwDutyCycle3 = 0;
		}
	}else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
		ExpLedToggle(GREEN_LED);

		uwIC2Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);	// Input capture value

		if (uwIC2Value != 0)
		{
			if(htim->Instance == TIM3x){
				ccr = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
				pwmval1 = (uwIC2Value - ccr)/10;

				pwmval2 = ccr;
				pwmval3 = uwIC2Value;
				uwDutyCycle1 = (ccr * 100) / uwIC2Value;
				intflag = 1;
			}else if(htim->Instance == TIM4x){
				uwDutyCycle4 = ((pwmval4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3)) * 100) / uwIC2Value;
				intflag = 1;
			}
		}
		else
		{
			if(htim->Instance == TIM3x)
				uwDutyCycle1 = 0;
			else if(htim->Instance == TIM4x)
				uwDutyCycle4 = 0;
		}
	}
}
*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint32_t CCRx_tn=0, CCRx_tn_1=0, period=0, duty_cycle=0;

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
		ExpLedToggle(ORANGE_LED);
		CCRx_tn_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);	// Input capture value
		CCRx_tn = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		intflag = 1;

	}else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
		ExpLedToggle(GREEN_LED);
		CCRx_tn_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);	// Input capture value
		CCRx_tn = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		intflag = 1;

	}

	if(intflag){
		if(CCRx_tn < CCRx_tn_1)
			period = CCRx_tn_1 - CCRx_tn;
		else
			period = (0xffff - CCRx_tn + CCRx_tn_1);

		duty_cycle = (period * 100) / CCRx_tn_1;

		if((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) && (htim->Instance == TIM3x)){
			uwDutyCycle2 = duty_cycle;
			pwmval2 = period;
		}else if((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) && (htim->Instance == TIM4x)){
			uwDutyCycle3 = duty_cycle;
			pwmval3 = period;
		}else if((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) && (htim->Instance == TIM3x)){
			uwDutyCycle1 = duty_cycle;
			pwmval1 = period;
		}else if((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) && (htim->Instance == TIM4x)){
			uwDutyCycle4 = duty_cycle;
			pwmval4 = period;
		}
	}
}

/* Hierdie een werk mooi
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint32_t CCRx_tn, CCRx_tn_1, period;

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
		ExpLedToggle(GREEN_LED);

		CCRx_tn_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);	// Input capture value

		if(htim->Instance == TIM3x){
			CCRx_tn = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

			if(CCRx_tn < CCRx_tn_1){
				period = CCRx_tn_1 - CCRx_tn;
				uwDutyCycle1 = (period * 100) / CCRx_tn_1;
			}
			else{
				period = (0xffff - CCRx_tn + CCRx_tn_1);
				uwDutyCycle1 = (period * 100) / CCRx_tn_1;
			}

			pwmval1 = period;
			intflag = 1;
		}
	}
}
*/
int GetPwmInfo(PwmInfo *pwm)
{
	if(intflag){
	  pwm->dutyCycle1 = uwDutyCycle1;
	  pwm->dutyCycle2 = uwDutyCycle2;
	  pwm->dutyCycle3 = uwDutyCycle3;
	  pwm->dutyCycle4 = uwDutyCycle4;

	  pwm->pwmval1 = pwmval1;
	  pwm->pwmval2 = pwmval2;
	  pwm->pwmval3 = pwmval3;
	  pwm->pwmval4 = pwmval4;

	  intflag = 0;

	  return 1;
	}

	return 0;
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;

  if(htim->Instance == TIM4x)
  {
	  TIM4x_CLK_ENABLE();				// TIM4 peripheral clock
	  TIM4x_CHANNEL_GPIO_PORT();		// GPIO clock

	  // Configure  (TIM4x_Channel) in Alternate function, push-pull and 100MHz speed
	  GPIO_InitStruct.Pin = GPIO_PIN_TIM4x_CHANNEL_2_4;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF_TIM4x;
	  HAL_GPIO_Init(GPIO_PORT, &GPIO_InitStruct);

	  HAL_NVIC_SetPriority(TIM4x_IRQn, 0, 1);	// TIM interrupt priority
	  HAL_NVIC_EnableIRQ(TIM4x_IRQn);			// TIM interrupt enable
  }

  if(htim->Instance == TIM3x){
  	  TIM3x_CLK_ENABLE();				// TIM4 peripheral clock
  	  TIM3x_CHANNEL_GPIO_PORT();		// GPIO clock

  	  // Configure  (TIM4x_Channel) in Alternate function, push-pull and 100MHz speed
  	  GPIO_InitStruct.Pin = GPIO_PIN_TIM3x_CHANNEL_2_4;
  	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  	  GPIO_InitStruct.Pull = GPIO_PULLUP;
  	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  	  GPIO_InitStruct.Alternate = GPIO_AF_TIM3x;
  	  HAL_GPIO_Init(GPIO_PORT, &GPIO_InitStruct);

  	  HAL_NVIC_SetPriority(TIM3x_IRQn, 0, 1);	// TIM interrupt priority
  	  HAL_NVIC_EnableIRQ(TIM3x_IRQn);			// TIM interrupt enable
    }
}
