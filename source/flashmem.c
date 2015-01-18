#include "flashmem.h"

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

#define FM_DUMMY				0x00
#define FM_CMD_STATUS1			0x05
#define FM_CMD_DEV_ID			0x9f
#define FM_CMD_WREN				0x06
#define FM_CMD_WRDI				0x04
#define FM_CMD_RDSR1			0x05
#define FM_CMD_RDSR2			0x35
#define FM_CMD_FAST_READ		0x0b
#define FM_CMD_PAGE_PROGRAM		0x02
#define FM_CMD_SECTOR_ERASE		0x20
#define FM_CMD_CHIP_ERASE		0x60

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
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull  = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
	GPIO_InitStructure.Alternate = 0;
	HAL_GPIO_Init(FLASHMEM_PORT, &GPIO_InitStructure);

	FLASHMEM_CS_HIGH();

	FlashMemSpiHandle.Instance = FLASHMEM_SPI;
	FlashMemSpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
	*id = flash_mem_read_write(FM_DUMMY);
	*mem_type = flash_mem_read_write(FM_DUMMY);
	*capacity = flash_mem_read_write(FM_DUMMY);

//	flash_mem_not_busy();

	FLASHMEM_CS_HIGH();
}

void FlashWREN(void)
{
	FLASHMEM_CS_LOW();
	flash_mem_write(FM_CMD_WREN);
	FLASHMEM_CS_HIGH();
}

void FlashWRDI(void)
{
	FLASHMEM_CS_LOW();
	flash_mem_write(FM_CMD_WRDI);
	FLASHMEM_CS_HIGH();
}

int32_t FlashFastRead(uint32_t *pointer, uint8_t *data, uint32_t length)
{
	uint32_t i=0;

	FLASHMEM_CS_LOW();

	flash_mem_write(FM_CMD_FAST_READ);

	flash_mem_write((uint8_t)(*pointer >> 16));
	flash_mem_write((uint8_t)(*pointer >> 8));
	flash_mem_write((uint8_t)*pointer);
	flash_mem_write(FM_DUMMY);

	for(i=0; i<length; i++){
		data[i] = flash_mem_read_write(FM_DUMMY);
	}

	FLASHMEM_CS_HIGH();

	return 0;
}

int32_t FlashPageProgram(uint32_t *pointer, uint8_t *data, uint32_t length)
{
	uint32_t i=0;
	uint8_t status1;
	FlashStatusRegister status;

	FlashWREN();

	while(1){
		FlashStatus(&status, NULL, NULL);

		if(status.wel)
			break;
	}

	FLASHMEM_CS_LOW();

	flash_mem_write(FM_CMD_PAGE_PROGRAM);

	flash_mem_write((uint8_t)(*pointer >> 16));
	flash_mem_write((uint8_t)(*pointer >> 8));
	flash_mem_write((uint8_t)*pointer);

	for(i=0; i<length; i++){
		 flash_mem_write(data[i]);
	}

	FLASHMEM_CS_HIGH();

	while(1){
		FlashStatus(&status, NULL, NULL);

		if(status.wip == 0)	// Check that no write is in progress
			break;
	}

	return 0;
}

int32_t FlashSectorErase(uint32_t *pointer)
{
	uint8_t status1;
	FlashStatusRegister status;

	FlashWREN();

	while(1){
		FlashStatus(&status, NULL, NULL);

		if(status.wel)
			break;
	}

	FLASHMEM_CS_LOW();

	flash_mem_write(FM_CMD_SECTOR_ERASE);

	flash_mem_write((uint8_t)(*pointer >> 16));
	flash_mem_write((uint8_t)(*pointer >> 8));
	flash_mem_write((uint8_t)*pointer);

	FLASHMEM_CS_HIGH();

	while(1){
		FlashStatus(&status, NULL, NULL);

		if(status.wip == 0)	// Check that no write is in progress
			break;
	}

	return 0;
}

int32_t FlashChipErase(void)
{
	uint8_t status1;
	FlashStatusRegister status;

	FlashWREN();

	while(1){
		FlashStatus(&status, NULL, NULL);

		if(status.wel)
			break;
	}

	FLASHMEM_CS_LOW();

	flash_mem_write(FM_CMD_CHIP_ERASE);

	FLASHMEM_CS_HIGH();

	while(1){
		FlashStatus(&status, NULL, NULL);

		if(status.wip == 0)	// Check that no write is in progress
			break;
	}

	return 0;
}

void FlashStatus(FlashStatusRegister *stat, uint8_t *rdsr1, uint8_t *rdsr2)
{
	uint8_t status1, status2;

	FLASHMEM_CS_LOW();

	flash_mem_write(FM_CMD_RDSR1);
	status1 = flash_mem_read_write(FM_DUMMY);

	if(rdsr1 != NULL)
		*rdsr1 = status1;

	flash_mem_write(FM_CMD_RDSR2);
	status2 = flash_mem_read_write(FM_DUMMY);

	if(rdsr2 != NULL)
		*rdsr2 = status2;

	FLASHMEM_CS_HIGH();

	stat->wip = status1 >> 0;
	stat->wel = status1 >> 1;
	stat->bp0 = status1 >> 2;
	stat->bp1 = status1 >> 3;
	stat->bp2 = status1 >> 4;
	stat->tb = status1 >> 5;
	stat->sec = status1 >> 6;
	stat->srp0 = status1 >> 7;

	stat->srp1 = status2 >> 0;
	stat->apt = status2 >> 2;
	stat->cmp = status2 >> 6;
}

static void flash_mem_not_busy(void)
{
	FLASHMEM_CS_LOW();

	flash_mem_write(FM_CMD_STATUS1);
	while(flash_mem_read_write(0) & 1);

	FLASHMEM_CS_HIGH();
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
