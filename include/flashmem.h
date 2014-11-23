#ifndef _FLASHMEM_H_
#define _FLASHMEM_H_

#include "stm32f4xx_hal.h"

// PB3 - SCK
// PB4 - MISO
// PB5 - MOSI
// PB7 - CS

#define FLASHMEM_PORT					GPIOB
#define FLASHMEM_SPI					SPI3
#define FLASHMEM_SPI_CLK_ENABLE()		__SPI3_CLK_ENABLE()
#define FLASHMEM_SPI_AF					GPIO_AF6_SPI3
#define FLASHMEM_SPI_TIMEOUT			((uint32_t)0x1000)
#define FLASHMEM_GPIO_CLK_ENABLE()		__GPIOB_CLK_ENABLE()
#define FLASHMEM_SCK					GPIO_PIN_3
#define FLASHMEM_MISO					GPIO_PIN_4
#define FLASHMEM_MOSI					GPIO_PIN_5
#define FLASHMEM_CS						GPIO_PIN_7
#define FLASHMEM_CS_LOW()       HAL_GPIO_WritePin(FLASHMEM_PORT, FLASHMEM_CS, GPIO_PIN_RESET)
#define FLASHMEM_CS_HIGH()      HAL_GPIO_WritePin(FLASHMEM_PORT, FLASHMEM_CS, GPIO_PIN_SET)

void FlashMemInit(void);
void FlashMemChipID(uint8_t *id, uint8_t *mem_type, uint8_t *capacity);

#endif // _FLASHMEM_H_
