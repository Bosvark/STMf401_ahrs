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

#define ADDRESS_CLEAR			0xffffffff

volatile uint32_t valid_sector=0xffffffff;

static uint32_t eeprom_find_valid_sector(void);
static int32_t eeprom_garbage_collection(void);

typedef struct
{
	VARIABLE var;
	uint32_t virtual_address;
	uint32_t length;
}VarDef;

VarDef vardefs[]={{0,0,0},
		{VAR_TEST1, 0, sizeof(uint32_t)},
		{VAR_TEST2, 0, 13}
};


uint32_t EEPROMInit(void)
{
	uint8_t id=0, mem_type=0, capacity=0;

	FlashMemInit();

	FlashMemChipID(&id, &mem_type, &capacity);

	if(id != 0x37 || mem_type != 0x30 || capacity != 0x16)
		return -1;

	FlashChipErase();

	return eeprom_find_valid_sector();
}

int32_t EEPROMGet(VARIABLE varid, uint8_t *data)
{
	uint32_t sector_addr = valid_sector+sizeof(uint32_t), var=0, length=0,i;

	if(varid >= VAR_THE_END)
		return -1;

	VarDef *def = &vardefs[varid];

	// Start at the previous address
	if(def->virtual_address != 0)
		sector_addr = def->virtual_address;

	while(sector_addr + def->length + (2 * sizeof(uint32_t)) < valid_sector + EEPROM_SECTOR_SIZE){
		FlashFastRead(&sector_addr, (uint8_t*)&var, sizeof(var));

		if(var == (uint32_t)varid)
			def->virtual_address = sector_addr;

		sector_addr += sizeof(uint32_t);

		FlashFastRead(&sector_addr, (uint8_t*)&length, sizeof(length));
		sector_addr += sizeof(uint32_t);

		if(var == (uint32_t)varid){
			for(i=0; i<length; i++){
				FlashFastRead(&sector_addr, (uint8_t*)&data[i], 1);
				sector_addr++;
			}

			return 0;
		}else
			sector_addr += length;
	}

	return -1;
}

int32_t EEPROMSet(VARIABLE varid, uint8_t *data)
{
	uint32_t sector_addr = valid_sector+sizeof(uint32_t), var=0, length=0;

VCP_write("EEPROMSet\r\n", 11);

	if(varid >= VAR_THE_END)
		return -1;

	VarDef *def = &vardefs[varid];

	// Start at the previous address, so that it can be marked
	if(def->virtual_address != 0){
		sector_addr = def->virtual_address;

		// Mark the current variable ID as all 0's to mark it as invalid
		var = 0;
		FlashPageProgram(&sector_addr, (uint8_t*)&var, sizeof(var));		// Variable ID
		def->virtual_address = 0;											// Previous address is not valid anymore
	}

	while(sector_addr + def->length + (2 * sizeof(uint32_t)) < valid_sector + EEPROM_SECTOR_SIZE){
		FlashFastRead(&sector_addr, (uint8_t*)&var, sizeof(var));

		if(var == ADDRESS_CLEAR){
			def->virtual_address = sector_addr;
			FlashPageProgram(&sector_addr, (uint8_t*)&def->var, sizeof(uint32_t));		// Variable ID
			sector_addr += sizeof(uint32_t);
			FlashPageProgram(&sector_addr, (uint8_t*)&def->length, sizeof(uint32_t));	// Length
			sector_addr += sizeof(uint32_t);
			FlashPageProgram(&sector_addr, (uint8_t*)data, def->length);
			return 0;
		}

		// Read the length
		sector_addr += sizeof(uint32_t);
		FlashFastRead(&sector_addr, (uint8_t*)&length, sizeof(length));

		// Skip past the data
		sector_addr += sizeof(uint32_t) +length;
	}

	// If we got here, then we have reached the end of the valid sector and garbage collection needs to be done
	eeprom_garbage_collection();

VCP_write("Going into recursion\r\n", 22);
	// Call ourselves again to retry
	EEPROMSet(varid, data);
VCP_write("Out of recursion\r\n", 18);

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
		case SECTOR_STATE_RECEIVING:
VCP_write("DBG1\r\n", 6);
			if(state_sector2 == SECTOR_STATE_VALID){
				eeprom_garbage_collection();
				return 0;

			}else
				return -1;

			break;
		case SECTOR_STATE_VALID:
VCP_write("DBG2\r\n", 6);
			if(state_sector2 == SECTOR_STATE_ERASED){
				valid_sector = EEPROM_ADDR_SECTOR1;
				return 0;

			}else{
				sector_addr=EEPROM_ADDR_SECTOR2;
				FlashSectorErase(&sector_addr);
				FlashFastRead(&sector_addr, (uint8_t*)&state_sector2, sizeof(state_sector2));
				return 0;
			}
		case SECTOR_STATE_ERASED:
VCP_write("DBG3\r\n", 6);
		default:
		{
VCP_write("DBG4\r\n", 6);
			if(state_sector2 == SECTOR_STATE_ERASED){
VCP_write("DBG5\r\n", 6);
				// Mark sector 1 as valid
				sector_addr=EEPROM_ADDR_SECTOR1;
				value = SECTOR_STATE_VALID;
				FlashSectorErase(&sector_addr);
				FlashPageProgram(&sector_addr, (uint8_t*)&value, sizeof(value));
				FlashFastRead(&sector_addr, (uint8_t*)&state_sector1, sizeof(state_sector1));

				valid_sector = EEPROM_ADDR_SECTOR1;
				return 0;

			}else if(state_sector2 == SECTOR_STATE_RECEIVING){
VCP_write("DBG6\r\n", 6);
				sector_addr=EEPROM_ADDR_SECTOR2;
				FlashSectorErase(&sector_addr);
				FlashFastRead(&sector_addr, (uint8_t*)&state_sector2, sizeof(state_sector2));

				eeprom_garbage_collection();
				return 0;

			}else if(state_sector2 == SECTOR_STATE_VALID){
VCP_write("DBG7\r\n", 6);
				valid_sector = EEPROM_ADDR_SECTOR2;
				return 0;

			}else{
VCP_write("DBG8\r\n", 6);
				return -1;
			}

			break;
		}
	}
VCP_write("DBG9\r\n", 6);

	return -1;
}
#include <stdio.h>
#include <string.h>
static int32_t eeprom_garbage_collection(void)
{
	uint32_t sector_addr, sector_addr_rx, sector_addr_tx, value;
	uint32_t state_sector1, state_sector2;
char outbuff[60];
	VCP_write("Taking out the trash...\r\n", 25);

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
		VCP_write("Houston, we have a problem!!!\r\n", 31);
		return -1;
	}

	// Sanity check
	uint32_t checkaddr = sector_addr_tx + sizeof(uint32_t);
	uint32_t checkval=0;

	FlashFastRead(&checkaddr, (uint8_t*)&checkval, sizeof(checkval));

	if(checkval == ADDRESS_CLEAR){
		// We should not be here
		sprintf(outbuff, "We should not be here!\r\n");
		VCP_write(outbuff, strlen(outbuff));
		return -1;
	}

	//
	// Start garbage collection
	//

	// Mark receiving sector
	value = SECTOR_STATE_RECEIVING;
	FlashPageProgram(&sector_addr_rx, (uint8_t*)&value, sizeof(value));

	uint32_t addrtx=sector_addr_tx+sizeof(uint32_t);	// Start after the sector header bytes
	uint32_t addrrx=sector_addr_rx+sizeof(uint32_t);	// Start after the sector header bytes
VCP_write("DBG1\r\n", 6);
	while(addrtx < sector_addr_tx + EEPROM_SECTOR_SIZE){
		uint32_t virt_addr=0, length=0xffffffff, i=0;
sprintf(outbuff, "DBG2 0x%08x\r\n", (unsigned int)addrtx);
VCP_write(outbuff, strlen(outbuff));
		FlashFastRead(&addrtx, (uint8_t*)&virt_addr, sizeof(virt_addr));

		if(virt_addr == ADDRESS_CLEAR){
			// We have reached the end
VCP_write("DBG3\r\n", 6);
			break;
		}else
			addrtx += sizeof(uint32_t);

VCP_write("DBG4\r\n", 6);
		FlashFastRead(&addrtx, (uint8_t*)&length, sizeof(virt_addr));
		addrtx += sizeof(uint32_t);

		if(virt_addr == 0){	// Dont copy invalid variables
VCP_write("DBG5\r\n", 6);
			addrtx += length;
			continue;
		}
VCP_write("DBG6\r\n", 6);
		// Update the variable lookup table
		VarDef *def = &vardefs[virt_addr];
		def->virtual_address = addrrx;

		FlashPageProgram(&addrrx, (uint8_t*)&virt_addr, sizeof(value));
		addrrx += sizeof(uint32_t);
		FlashPageProgram(&addrrx, (uint8_t*)&length, sizeof(value));
		addrrx += sizeof(uint32_t);

		for(i=0; i<length; i++){
VCP_write("DBG7\r\n", 6);
			uint8_t indata;
			FlashFastRead(&addrtx, (uint8_t*)&indata, 1);
			addrtx++;
			FlashPageProgram(&addrrx, (uint8_t*)&indata, 1);
			addrrx++;
		}
VCP_write("DBG8\r\n", 6);
	}

	// Mark receiving sector as 'valid'
	value = SECTOR_STATE_VALID;
	FlashPageProgram(&sector_addr_rx, (uint8_t*)&value, sizeof(value));
	valid_sector = sector_addr_rx;

	// Erase sending sector
	FlashSectorErase(&sector_addr_tx);

	VCP_write("Trash is out\r\n", 14);

	return 0;
}
