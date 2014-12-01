#ifndef _FLASHMEM_H_
#define _FLASHMEM_H_

#include "stm32f4xx_hal.h"

#define FLASHMEM_PORT					GPIOE
#define FLASHMEM_SPI					SPI4
#define FLASHMEM_SPI_CLK_ENABLE()		__SPI4_CLK_ENABLE()
#define FLASHMEM_SPI_AF					GPIO_AF5_SPI4
#define FLASHMEM_SPI_TIMEOUT			((uint32_t)0x1000)
#define FLASHMEM_GPIO_CLK_ENABLE()		__GPIOE_CLK_ENABLE()
#define FLASHMEM_SCK					GPIO_PIN_12
#define FLASHMEM_MISO					GPIO_PIN_13
#define FLASHMEM_MOSI					GPIO_PIN_14
#define FLASHMEM_CS						GPIO_PIN_11
#define FLASHMEM_CS_LOW()       		HAL_GPIO_WritePin(FLASHMEM_PORT, FLASHMEM_CS, GPIO_PIN_RESET)
#define FLASHMEM_CS_HIGH()      		HAL_GPIO_WritePin(FLASHMEM_PORT, FLASHMEM_CS, GPIO_PIN_SET)

void FlashMemInit(void);
void FlashMemChipID(uint8_t *id, uint8_t *mem_type, uint8_t *capacity);

#endif // _FLASHMEM_H_
