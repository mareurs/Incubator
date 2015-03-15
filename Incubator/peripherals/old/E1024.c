/*
 * _E1024.c
 *
 * Created: 2/23/2015 1:51:38 AM
 *  Author: marius
 */ 
#include "twi.h"
#include "E1024.h"

void E1024_init()
{
	TWIInit();
}

uint8_t E1024_write(uint16_t dataAddress, uint8_t data)
{
	uint8_t status = 0;
	TWIStart();
	if (TWIGetStatus() != 0x08)
	status = TWIGetStatus();
	if (status != 0x00)
		return status;
	//Write address
	TWIWrite((uint8_t)((0x500000 | dataAddress) >> 16));
	status = TWIGetStatus();
	if (status != 0x0)
	return status;
	TWIWrite((uint8_t)((dataAddress & WORD_MASK) >> 8)); //MSB
	status = TWIGetStatus();
	if (status != 0x00)
	return status;
	TWIWrite((uint8_t)(dataAddress & 0xFF)); // LSB
	status = TWIGetStatus();
	if (status != 0x00)
	return status;
	TWIWrite(data);
	status = TWIGetStatus();
	if (status != 0x00)
	return status;
	TWIStop();
	return SUCCESS;
}

uint8_t E1024_read(uint16_t dataAddress, uint8_t* u8data )
{
	uint8_t status;
	TWIStart();
	status = TWIGetStatus();
	if (status != 0x00)
		return status;	
	TWIWrite((uint8_t)((0x500000 | dataAddress) >> 16)); // B1010xxx
	status = TWIGetStatus();
	if (status != 0x00)
		return status;
	TWIWrite((uint8_t)((dataAddress & WORD_MASK) >> 8)); // MSB
	status = TWIGetStatus();
	if (status != 0x00)
		return status;
	TWIWrite((uint8_t)(dataAddress & 0xFF)); // LSB
	status = TWIGetStatus();
	if (status != 0x00)
		return status;
	TWIStart();
	status = TWIGetStatus();
	if (status != 0x00)
		return status;
	TWIWrite(0X50 | 1);
	status = TWIGetStatus();
	if (status != 0x00)
		return status;
    *u8data = TWIReadNACK();
	status = TWIGetStatus();
	if (status != 0x00)
		return status;
    TWIStop();
    return SUCCESS;		
}


uint8_t E1024_writeBuffer(uint16_t address, uint8_t* data, uint8_t size)
{
	int i = 0;
	uint8_t result = 0;
	for(; i < size; i++)
// 		if(E1024_write(address++,*(data++)) == ERROR)
// 			return ERROR;

	 result = E1024_write(address++,*(data++));
	if( result != SUCCESS);
		return result;
	return SUCCESS;
}

uint8_t E1024_readBuffer(uint16_t address, uint8_t*data, uint8_t size)
{
	int i = 0;
	for(; i < size; i++)
		if(E1024_read(address++, data++) == ERROR)
			return ERROR;
			
	return SUCCESS;
}
