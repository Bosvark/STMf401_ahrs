#include "stm32f4xx_hal_dma.h"	// why? this is messy
#include "stm32f4xx_hal_tim.h"
#include "exp_board.h"

volatile PwmInfo rx_channel;

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

#define BEAT_ONE		300
#define BEAT_TWO		600

static const uint32_t buzzer_1_1[]={BEAT_ONE, BEAT_ONE, BEAT_ONE, 0};
static const uint32_t buzzer_2_2[]={BEAT_TWO, BEAT_TWO, BEAT_TWO, 0};
static const uint32_t buzzer_1_1_2[]={BEAT_ONE, BEAT_ONE, BEAT_ONE, BEAT_ONE, BEAT_TWO, 0};
uint32_t *buzz=NULL;
static uint8_t buzz_pos=0;
uint32_t beat=0;

void ExpBuzzerTune(BUZZER_TUNE tune)
{
	switch(tune)
	{
		case BUZZER_2_SHORTS:
			buzz = (uint32_t*)buzzer_1_1;
			break;
		case BUZZER_2_LONGS:
			buzz = (uint32_t*)buzzer_2_2;
			break;
		case BUZZER_CALIBRATION_START:
			buzz = (uint32_t*)buzzer_1_1_2;
			break;
		default:
			return;
	}

	buzz_pos = 0;
	beat = HAL_GetTick() + buzz[buzz_pos++];
	ExpBuzzerOn();
}

void ExpBuzzerHandler(void)
{
	if(buzz == NULL){
		return;			// Nothing to do
	}

	uint32_t time = HAL_GetTick();

	if((beat > 0) && (time > beat)){
		ExpBuzzerToggle();

		if(buzz[buzz_pos] == 0){
			ExpBuzzerOff();

			buzz = NULL;
			buzz_pos = 0;
			beat = 0;
			return;
		}else
			beat = time + buzz[buzz_pos++];
	}
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

void TIM_Init(TIM_HandleTypeDef *timh)
{
	TIM_IC_InitTypeDef       sConfig;

	timh->Init.Period = 0xFFFF;
	timh->Init.Prescaler = 0;
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

__IO uint32_t RC1_tn = 0;
__IO uint32_t RC1_tn_1 = 0;
__IO uint32_t RC2_tn = 0;
__IO uint32_t RC2_tn_1 = 0;
__IO uint32_t RC3_tn = 0;
__IO uint32_t RC3_tn_1 = 0;
__IO uint32_t RC4_tn = 0;
__IO uint32_t RC4_tn_1 = 0;
__IO uint32_t uwDutyCycle1 = 0;
__IO uint32_t uwDutyCycle2 = 0;
__IO uint32_t uwDutyCycle3 = 0;
__IO uint32_t uwDutyCycle4 = 0;
__IO uint32_t pwmval1 = 0;
__IO uint32_t pwmval2 = 0;
__IO uint32_t pwmval3 = 0;
__IO uint32_t pwmval4 = 0;
__IO char intflag1=0;
__IO char intflag2=0;
__IO char intflag3=0;
__IO char intflag4=0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint32_t CCRx_tn=0, CCRx_tn_1=0;

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){

		CCRx_tn = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

		if(htim->Instance == TIM3x){
			RC2_tn = CCRx_tn;
			intflag2 = 0;
		}else if(htim->Instance == TIM4x){
			RC3_tn = CCRx_tn;
			intflag3 = 0;
		}
	}else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
		CCRx_tn_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);	// Input capture value

		if(htim->Instance == TIM3x){
			RC2_tn_1 = CCRx_tn_1;
			intflag2 = 1;
		}else if(htim->Instance == TIM4x){
			RC3_tn_1 = CCRx_tn_1;
			intflag3 = 1;
		}
	}else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
		CCRx_tn = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

		if(htim->Instance == TIM3x){
			RC1_tn = CCRx_tn;
			intflag1 = 0;
		}else if(htim->Instance == TIM4x){
			RC4_tn = CCRx_tn;
			intflag4 = 0;
		}
	}else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
		CCRx_tn_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);	// Input capture value

		if(htim->Instance == TIM3x){
			RC1_tn_1 = CCRx_tn_1;
			intflag1 = 1;
		}else if(htim->Instance == TIM4x){
			RC4_tn_1 = CCRx_tn_1;
			intflag4 = 1;
		}

//		ExpLedToggle(GREEN_LED);
	}
}

int GetPwmInfo(PwmInfo *pwm)
{
	uint32_t pwmval;

	if(intflag1 > 0){
		if(RC1_tn < RC1_tn_1)
			pwmval = RC1_tn_1 - RC1_tn;
		else
			pwmval = (0xffffffff - RC1_tn + RC1_tn_1);

		if((pwmval > RX_CHAN_DEFAULT_LOWER_LIMIT_LOW) && (pwmval <= RX_CHAN_DEFAULT_UPPER_LIMIT_HIGH) && (RC1_tn_1 != 0)){
			pwm->pwmval1 = pwmval;
			pwm->dutyCycle1 = (pwm->pwmval1 * 100) / RC1_tn_1;
		}

		RC1_tn = 0;
		RC1_tn_1 = 0;
		intflag1 = 0;
	}

	if(intflag2 > 0){
		if(RC2_tn < RC2_tn_1)
			pwmval = RC2_tn_1 - RC2_tn;
		else
			pwmval = (0xffffffff - RC2_tn + RC2_tn_1);

		if((pwmval > RX_CHAN_DEFAULT_LOWER_LIMIT_LOW) && (pwmval <= RX_CHAN_DEFAULT_UPPER_LIMIT_HIGH) && (RC3_tn_1 != 0)){
			pwm->pwmval3 = pwmval;
			pwm->dutyCycle3 = (pwm->pwmval3 * 100) / RC3_tn_1;
		}

		RC2_tn = 0;
		RC2_tn_1 = 0;
		intflag2 = 0;
	}

	if(intflag3 > 0){
		if(RC3_tn < RC3_tn_1)
			pwmval = RC3_tn_1 - RC3_tn;
		else
			pwmval = (0xffffffff - RC3_tn + RC3_tn_1);

		if((pwmval > RX_CHAN_DEFAULT_LOWER_LIMIT_LOW) && (pwmval <= RX_CHAN_DEFAULT_UPPER_LIMIT_HIGH) && (RC3_tn_1 != 0)){
			pwm->pwmval3 = pwmval;
			pwm->dutyCycle3 = (pwm->pwmval3 * 100) / RC3_tn_1;
		}

		RC3_tn = 0;
		RC3_tn_1 = 0;
		intflag3 = 0;
	}

	if(intflag4 > 0){
		if(RC4_tn < RC4_tn_1)
			pwmval = RC4_tn_1 - RC4_tn;
		else
			pwmval = (0xffffffff - RC4_tn + RC4_tn_1);

		if((pwmval > RX_CHAN_DEFAULT_LOWER_LIMIT_LOW) && (pwmval <= RX_CHAN_DEFAULT_UPPER_LIMIT_HIGH) && (RC4_tn_1 != 0)){
			pwm->pwmval4 = pwmval;
			pwm->dutyCycle4 = (pwm->pwmval4 * 100) / RC4_tn_1;
		}

		RC4_tn = 0;
		RC4_tn_1 = 0;
		intflag4 = 0;
	}

	return 1;
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

	  GPIO_InitStruct.Pin = GPIO_PIN_15;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
