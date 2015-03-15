/*
 * UtilsCollection.c
 *
 * Created: 2/27/2015 8:19:07 PM
 *  Author: marius
 */ 
#include <inttypes.h>
#include "UtilsCollection.h"


uint8_t lsb(uint16_t value)
{
	uint8_t result =  (uint8_t)(value & 0x00FF);
	return result;
}

uint8_t msb(uint16_t value)
{
	uint8_t result = (uint8_t) (value >> 8);
	return result;
}

void appendNewLine(char* buff)
{
	strcat(buff,"\n\r");
}
