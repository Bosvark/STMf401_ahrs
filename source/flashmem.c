#include "flashmem.h"

#define FM_CMD_STATUS1			0x05
#define FM_CMD_DEV_ID			0x9f

static SPI_HandleTypeDef FlashMemSpiHandle;

static void flash_mem_not_busy(void);
static uint8_t flash_mem_write(uint8_t reg);
static uint8_t flash_mem_read_write(uint8_t reg);

void FlashMemInit(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;

	FLASHMEM_SPI_CLK_ENABLE();
	FLASHMEM_GPIO_CLK_ENABLE();

	GPIO_InitStructure.Pin = (FLASHMEM_SCK | FLASHMEM_MISO | FLASHMEM_MOSI);
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull  = GPIO_PULLDOWN;
	GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
	GPIO_InitStructure.Alternate = FLASHMEM_SPI_AF;
	HAL_GPIO_Init(FLASHMEM_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = FLASHMEM_CS;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull  = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
	GPIO_InitStructure.Alternate = 0;
	HAL_GPIO_Init(FLASHMEM_PORT, &GPIO_InitStructure);

	FLASHMEM_CS_HIGH();

	FlashMemSpiHandle.Instance = FLASHMEM_SPI;
	FlashMemSpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	FlashMemSpiHandle.Init.Direction = SPI_DIRECTION_2LINES;
	FlashMemSpiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
	FlashMemSpiHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
	FlashMemSpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
	FlashMemSpiHandle.Init.CRCPolynomial = 7;
	FlashMemSpiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
	FlashMemSpiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
	FlashMemSpiHandle.Init.NSS = SPI_NSS_SOFT;
	FlashMemSpiHandle.Init.TIMode = SPI_TIMODE_DISABLED;
	FlashMemSpiHandle.Init.Mode = SPI_MODE_MASTER;
	HAL_SPI_Init(&FlashMemSpiHandle);
}

void FlashMemChipID(uint8_t *id, uint8_t *mem_type, uint8_t *capacity)
{
	FLASHMEM_CS_LOW();

	flash_mem_write(FM_CMD_DEV_ID);
	*id = flash_mem_read_write(0);
	*mem_type = flash_mem_read_write(0);
	*capacity = flash_mem_read_write(0);

	flash_mem_not_busy();

	FLASHMEM_CS_HIGH();
}

static void flash_mem_not_busy(void)
{
	flash_mem_write(FM_CMD_STATUS1);
	while(flash_mem_read_write(0) & 1);
}

static uint8_t flash_mem_write(uint8_t reg)
{
	uint8_t rx=0;

	HAL_SPI_Transmit(&FlashMemSpiHandle, (uint8_t*) &reg, 1, FLASHMEM_SPI_TIMEOUT);

	return rx;
}

static uint8_t flash_mem_read_write(uint8_t reg)
{
	uint8_t rx=0;

	HAL_SPI_TransmitReceive(&FlashMemSpiHandle, (uint8_t*) &reg, (uint8_t*) &rx, 1, FLASHMEM_SPI_TIMEOUT);

	return rx;
}
