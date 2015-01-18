#ifndef _FLASHMEM_H_
#define _FLASHMEM_H_

#include "stm32f4xx_hal.h"

typedef struct
{
	uint32_t wip:1;		// Write in progress
	uint32_t wel:1;		// Write enable latch
	uint32_t bp0:1;		// Block protect 0
	uint32_t bp1:1;		// Block protect 1
	uint32_t bp2:1;		// Block protect 2
	uint32_t tb:1;		// Top/Bottom
	uint32_t sec:1;		// Sector protect
	uint32_t srp0:1;	// Status Register Protect 0
	uint32_t srp1:1;	// Status Register Protect 1
	uint32_t apt:1;		// All Protect
	uint32_t cmp:1;		// Complement Protect
}FlashStatusRegister;

void FlashMemInit(void);
void FlashMemChipID(uint8_t *id, uint8_t *mem_type, uint8_t *capacity);
void FlashWREN(void);
void FlashWRDI(void);
int32_t FlashFastRead(uint32_t *pointer, uint8_t *data, uint32_t length);
int32_t FlashPageProgram(uint32_t *pointer, uint8_t *data, uint32_t length);
void FlashStatus(FlashStatusRegister *stat, uint8_t *rdsr1, uint8_t *rdsr2);
int32_t FlashSectorErase(uint32_t *pointer);
int32_t FlashChipErase(void);

#endif // _FLASHMEM_H_
