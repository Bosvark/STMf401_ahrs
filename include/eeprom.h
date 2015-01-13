#ifndef _EEPROM_H_
#define _EEPROM_H_

#include "stm32f4xx_hal.h"

uint32_t EEPROMInit(void);
uint32_t EEPROMRead(uint32_t virtual_address, uint8_t *data);
uint32_t EEPROMWrite(uint32_t virtual_address, uint8_t *data, uint32_t length);

#endif // _EEPROM_H_
