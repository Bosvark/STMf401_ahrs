#ifndef _EEPROM_H_
#define _EEPROM_H_

#include "stm32f4xx_hal.h"

typedef struct
{
	uint16_t offsets[6];
	float scales[6];
}__attribute__((packed))CalibVals;

typedef enum
{
	VAR_CALIBRATION=1,
	VAR_TEST2,
	VAR_THE_END			// This value indicates the end of the list. Do not allocate values after it.
}VARIABLE;

uint32_t EEPROMInit(void);
int32_t EEPROMGet(VARIABLE varid, uint8_t *data);
int32_t EEPROMSet(VARIABLE varid, uint8_t *data);
uint32_t EEPROMRead(uint32_t virtual_address, uint8_t *data);
uint32_t EEPROMWrite(uint32_t virtual_address, uint8_t *data, uint32_t length);

#endif // _EEPROM_H_
