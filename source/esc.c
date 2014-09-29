#include "stm32f4xx_hal.h"
#include "esc.h"

TIM_HandleTypeDef        TimHandle;
TIM_OC_InitTypeDef sConfig;

void ESC_Init(void)
{
	TimHandle.Instance = TIM2;

	TimHandle.Init.Period = 20000-1;
	TimHandle.Init.Prescaler = ((SystemCoreClock) / 1000000) - 1;	// 1Mhz
	TimHandle.Init.ClockDivision = 0;
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_PWM_Init(&TimHandle);

	sConfig.OCMode     = TIM_OCMODE_PWM1;
	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig.OCFastMode = TIM_OCFAST_DISABLE;

	sConfig.Pulse = 0;
	HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_4);
}

void ESC_Start(int esc_channel)
{
	uint32_t tim_channel;

	switch(esc_channel)
	{
		case 1:	tim_channel = TIM_CHANNEL_1; break;
		case 2:	tim_channel = TIM_CHANNEL_2; break;
		case 3:	tim_channel = TIM_CHANNEL_3; break;
		case 4:	tim_channel = TIM_CHANNEL_4; break;
		default: return;
	}

	HAL_TIM_PWM_Start(&TimHandle, tim_channel);
}

void ESC_Speed(uint32_t speed, int esc_channel)
{

	TimHandle.Instance->CCR1 = speed;
	return;

	uint32_t tim_channel;


	switch(esc_channel)
	{
		case 1:	tim_channel = TIM_CHANNEL_1; break;
		case 2:	tim_channel = TIM_CHANNEL_2; break;
		case 3:	tim_channel = TIM_CHANNEL_3; break;
		case 4:	tim_channel = TIM_CHANNEL_4; break;
		default: return;
	}

	sConfig.Pulse = speed;
	HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, tim_channel);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef   GPIO_InitStruct;

	__TIM2_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();

	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;

	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
