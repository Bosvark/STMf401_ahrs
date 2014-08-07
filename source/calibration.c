#include <stdint.h>
#include "stm32f401_discovery.h"
#include "calibration.h"

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define FLASH_END_ADDR			((uint32_t)0x08040000)

#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_5   // Start @ of user Flash area
#define FLASH_USER_END_ADDR		FLASH_END_ADDR

int writeCalibration(unsigned char *data, unsigned int len)
{
	FLASH_EraseInitTypeDef EraseInitStruct;

	HAL_FLASH_Unlock();

	uint32_t SectorError = 0;

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = FLASH_SECTOR_5;
	EraseInitStruct.NbSectors = 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
	{
		HAL_FLASH_Lock();
		return -1;
	}

	uint32_t Address = FLASH_USER_START_ADDR, pos=0;
	uint32_t Address_end = FLASH_USER_START_ADDR+len;

	while (Address < Address_end)
	{
		if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, (uint32_t)Address, data[pos++]) == HAL_OK)
		{
			Address++;
		}
		else
		{
			HAL_FLASH_Lock();
			return -1;
		}
	}

	HAL_FLASH_Lock();

	return 0;
}

int readCalibration(unsigned char *data, unsigned int len)
{
	uint32_t address = FLASH_USER_START_ADDR;
	unsigned int pos = 0;

	while(pos < len)
		data[pos++] = *(__IO uint8_t*)address++;

	return pos;
}

//
// Local functions
//
