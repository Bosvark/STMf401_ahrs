/*
 * EEPROM emulation works by allocating 2 sectors of 4Kb each. One will be marked as the 'valid' sector,
 * where variables are being written. The second sector will marked as 'erased' and will be used for
 * garbage collection. When a variable changes, it's current virtual address will be over written
 * with 0's to mark it as garbage. The new variable will be appended to the bottom of the first sector.
 * Once the sector is full, garbage collection will take place.
 *
 * Garbage collection starts by marking the current sector as 'receiving'. Garbage collection works
 * by traversing the 'valid' sector, from top to bottom, until a valid virtual address is found. The
 * virtual address, along with its data, is then copied to the second sector. The first sector is erased,
 * once all valid variables have been copied to the receiving sector and the receiving sector is then
 * marked as 'valid'. This sector will then be used for storing variables as they change, until it is
 * full and garbage collection gets triggered again. The reason for marking the second sector as 'receiving'
 * while garbage collection is taking place, is to guard against power failures during garbage collection.
 * In this case, garbage collection will be repeated during the next startup and initialisation sequence.
 *
 * Variable data can be of varying length. To cater for this flexible structure, variable will be stored
 * in 3 parts:
 * - A virtual address (4 bytes)
 * - Length (4 bytes)
 * - Data of 'Length' bytes
 *
 * On startup, all variables will be read in from flash memory. When a value changes, the current virtual
 * address will be over written with 0's, if there is space to store the new value at the end of the
 * current sector.
 */
#include "eeprom.h"
#include "flashmem.h"

#define EEPROM_ADDR_SECTOR1		0x00000000
#define EEPROM_ADDR_SECTOR2		0x00001000
#define EEPROM_SECTOR_SIZE		0x00000fff

#define SECTOR_STATE_ERASED		0xffffffff
#define SECTOR_STATE_RECEIVING	0xeeeeeeee
#define SECTOR_STATE_VALID		0x00000000

static uint32_t valid_sector=0xffffffff;

static uint32_t eeprom_find_valid_sector(void);
static int32_t eeprom_garbage_collection(void);

uint32_t EEPROMInit(void)
{
	FlashMemInit();

	eeprom_find_valid_sector();

	return 0;
}

uint32_t EEPROMRead(uint32_t virtual_address, uint8_t *data)
{
	return 0;
}

uint32_t EEPROMWrite(uint32_t virtual_address, uint8_t *data, uint32_t length)
{
	return 0;
}

//
// Local functions
//
static uint32_t eeprom_find_valid_sector(void)
{
	uint32_t sector_addr, value;
	uint32_t state_sector1, state_sector2;

	sector_addr=EEPROM_ADDR_SECTOR1;
	FlashFastRead(&sector_addr, (uint8_t*)&state_sector1, sizeof(state_sector1));

	sector_addr=EEPROM_ADDR_SECTOR2;
	FlashFastRead(&sector_addr, (uint8_t*)&state_sector2, sizeof(state_sector2));

	switch(state_sector1)
	{
		case SECTOR_STATE_ERASED:
		{
			if(state_sector2 == SECTOR_STATE_ERASED){
				// Mark sector 1 as valid
				sector_addr=EEPROM_ADDR_SECTOR1;
				value = SECTOR_STATE_VALID;
				FlashPageProgram(&sector_addr, (uint8_t*)&value, sizeof(value));
				FlashFastRead(&sector_addr, (uint8_t*)&state_sector1, sizeof(state_sector1));

				valid_sector = EEPROM_ADDR_SECTOR1;
				return 0;

			}else if(state_sector2 == SECTOR_STATE_RECEIVING){
				sector_addr=EEPROM_ADDR_SECTOR2;
				FlashSectorErase(&sector_addr);
				FlashFastRead(&sector_addr, (uint8_t*)&state_sector2, sizeof(state_sector2));

				eeprom_garbage_collection();
				return 0;

			}else if(state_sector2 == SECTOR_STATE_VALID){
				valid_sector = EEPROM_ADDR_SECTOR2;
				return 0;

			}else
				return -1;

			break;
		}
		case SECTOR_STATE_RECEIVING:
			if(state_sector2 == SECTOR_STATE_VALID){
				eeprom_garbage_collection();
				return 0;

			}else
				return -1;

			break;
		case SECTOR_STATE_VALID:
			if(state_sector2 == SECTOR_STATE_ERASED){
				valid_sector = EEPROM_ADDR_SECTOR1;
				return 0;

			}else{
				sector_addr=EEPROM_ADDR_SECTOR2;
				FlashSectorErase(&sector_addr);
				FlashFastRead(&sector_addr, (uint8_t*)&state_sector2, sizeof(state_sector2));
				return 0;
			}
	}

	return -1;
}

static int32_t eeprom_garbage_collection(void)
{
	uint32_t sector_addr, sector_addr_rx, sector_addr_tx, value;
	uint32_t state_sector1, state_sector2;

	sector_addr=EEPROM_ADDR_SECTOR1;
	FlashFastRead(&sector_addr, (uint8_t*)&state_sector1, sizeof(state_sector1));

	sector_addr=EEPROM_ADDR_SECTOR2;
	FlashFastRead(&sector_addr, (uint8_t*)&state_sector2, sizeof(state_sector2));

	if((state_sector1 == SECTOR_STATE_VALID) && (state_sector2 == SECTOR_STATE_ERASED)){
		sector_addr_tx = EEPROM_ADDR_SECTOR1;
		sector_addr_rx = EEPROM_ADDR_SECTOR2;
	}else if((state_sector2 == SECTOR_STATE_VALID) && (state_sector1 == SECTOR_STATE_ERASED)){
		sector_addr_tx = EEPROM_ADDR_SECTOR2;
		sector_addr_rx = EEPROM_ADDR_SECTOR1;
	}else{
		// Houston, we have a problem!!!
		return -1;
	}

	//
	// Start garbage collection
	//

	// Mark receiving sector
	value = SECTOR_STATE_RECEIVING;
	FlashPageProgram(&sector_addr_rx, (uint8_t*)&value, sizeof(value));

	// ...
	uint32_t addrtx=sector_addr_tx+sizeof(uint32_t);	// Start after the sector header bytes
	uint32_t addrrx=sector_addr_rx+sizeof(uint32_t);	// Start after the sector header bytes

	while(addrtx < sector_addr_tx + EEPROM_SECTOR_SIZE){
		uint32_t virt_addr=0, length=0xffffffff;

		FlashFastRead(&addrtx, (uint8_t*)&virt_addr, sizeof(virt_addr));

		if(virt_addr == 0xffffffff){
			// We have reached the end
			break;
		}else
			addrtx += sizeof(uint32_t);

		FlashFastRead(&addrtx, (uint8_t*)&length, sizeof(virt_addr));

// TODO: Read the data and copy it to the receiving page

		addrtx += length;
	}

	// Mark receiving sector as 'valid'
	value = SECTOR_STATE_VALID;
	FlashPageProgram(&sector_addr_rx, (uint8_t*)&value, sizeof(value));
	valid_sector = sector_addr_rx;

	// Erase sending sector
	FlashSectorErase(&sector_addr_rx);

	return 0;
}
