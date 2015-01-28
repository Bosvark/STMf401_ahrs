#include <stdio.h>
#include <string.h>
#include <stm32f401_discovery.h>
#include <stm32f4xx_hal.h>
#include "usbd_cdc_if_template.h"
#include "multiwii.h"
#include "types.h"

#define USARTx                           USART6

#define USARTx_TX_PIN                    GPIO_PIN_11
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF8_USART6
#define USARTx_RX_PIN                    GPIO_PIN_12
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF8_USART6

UART_HandleTypeDef UartHandle;

#define BUFFER_SIZE	64

static int8_t serial_command;
static int8_t serial_buffer[BUFFER_SIZE];
static uint8_t serial_size;

typedef enum
{
	STATE_IDLE,
	STATE_HEADER,
	STATE_DIRECTION,
	STATE_SIZE,
	STATE_COMMAND,
	STATE_DATA,
	STATE_LRC,
	STATE_FINISHED
}MW_STATE;

MW_STATE state = STATE_IDLE;

static int8_t parse_response(int8_t *data, uint16_t length);
//static void Error_Handler(void);

void MultiWiiInit(void)
{
/*
	UartHandle.Instance        = USARTx;
	UartHandle.Init.BaudRate   = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits   = UART_STOPBITS_1;
	UartHandle.Init.Parity     = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;

	if(HAL_UART_Init(&UartHandle) != HAL_OK)
	{
	Error_Handler();
	}
*/
	memset(serial_buffer, 0, sizeof(serial_buffer));
	serial_size = 0;
	serial_command = 0;
}

uint32_t MultiWii(MultiWii_Func *funcs)
{
	int8_t command[BUFFER_SIZE];
	uint32_t ret;

	if((ret = VCP_read(command, 1)) <= 0)
		return 0;

	if(parse_response(command, (uint16_t) ret) < 0)
		return -1;

	return 0;
}

static int8_t parse_response(int8_t *data, uint16_t length)
{
	uint8_t serial_buffer_pos = 0, size=0, calc_lrc=0;
	state = STATE_IDLE;

	while((state != STATE_FINISHED) && (serial_buffer_pos < length)){
		switch(state)
		{
			case STATE_IDLE:
				if(data[serial_buffer_pos] == '$'){
					serial_buffer[serial_buffer_pos++];
					state = STATE_HEADER;
				}else
					return -1;

				break;
			case STATE_HEADER:
				if(data[serial_buffer_pos] == 'M'){
					serial_buffer[serial_buffer_pos++];
					state = STATE_DIRECTION;
				}else
					return -2;

				break;
			case STATE_DIRECTION:
				if(data[serial_buffer_pos] == '<'){
					serial_buffer[serial_buffer_pos++];
					state = STATE_SIZE;
				}else
					return-3;

				break;
			case STATE_SIZE:
				calc_lrc ^= data[serial_buffer_pos];
				serial_size = data[serial_buffer_pos++];
				state = STATE_COMMAND;
				break;
			case STATE_COMMAND:
				calc_lrc ^= data[serial_buffer_pos];
				serial_command = data[serial_buffer_pos++];
				state = STATE_DATA;
				break;
			case STATE_DATA:
				if(size == serial_size)
					state = STATE_LRC;
				else{
					calc_lrc ^= data[serial_buffer_pos];
					serial_buffer[size++] = data[serial_buffer_pos++];
				}
				break;
			case STATE_LRC:
				state = STATE_IDLE;
				if(calc_lrc != data[serial_buffer_pos++]){
					memset(serial_buffer, 0, sizeof(serial_buffer));
					serial_size = 0;
					serial_command = 0;
					return -4;
				}
				state = STATE_FINISHED;
				break;
			default:
				state = STATE_IDLE;
				break;
		}
	}

	if(state != STATE_FINISHED){
		memset(serial_buffer, 0, sizeof(serial_buffer));
		serial_size = 0;
		serial_command = 0;
		return -5;
	}

	return 0;
}

/*


if(HAL_UART_Transmit(&UartHandle, (uint8_t*)aTxBuffer, TXBUFFERSIZE, 5000)!= HAL_OK)
{
Error_Handler();
}
 */
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(HAL_UART_Receive(&UartHandle, (uint8_t *)serial_buffer, BUFFER_SIZE, 5000) != HAL_OK)
	{
		Error_Handler();
	}

	parse_response();
}

static void Error_Handler(void)
{
    BSP_LED_On(LED5);
    while(1)
    {
    }
}
*/
