/*! \file i2ceeprom.c \brief Interface for standard I2C EEPROM memories. */
//*****************************************************************************
//
// File Name	: 'i2ceeprom.c'
// Title		: Interface for standard I2C EEPROM memories
// Author		: Pascal Stang - Copyright (C) 2003
// Created		: 2003.04.23
// Revised		: 2003.04.23
// Version		: 0.1
// Target MCU	: Atmel AVR series
// Editor Tabs	: 4
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/delay.h>

#include "i2c.h"
#include "i2ceeprom.h"
#include "rprintf.h"

// Standard I2C bit rates are:
// 100KHz for slow speed
// 400KHz for high speed

// functions
void i2ceepromInit(void)
{
	// although there is no code here
	// don't forget to initialize the I2C interface itself
}

u08 i2ceepromReadByte(u08 i2cAddr, u32 memAddr)
{
	u08 packet[2];
	// prepare address
	packet[0] = (memAddr>>8);
	packet[1] = (memAddr&0x00FF);
	// send memory address we wish to access to the memory chip
	i2cMasterSend(i2cAddr, 2, packet);
	// retrieve the data at this memory address
	i2cMasterReceive(i2cAddr, 1, packet);
	// return data
	return packet[0];
}

void i2ceepromWriteByte(u08 i2cAddr, u32 memAddr, u08 data)
{
	u08 packet[3];
	// prepare address + data
	packet[0] = (memAddr>>8);
	packet[1] = (memAddr&0x00FF);
	packet[2] = data;
	// send memory address we wish to access to the memory chip
	// along with the data we wish to write
	i2cMasterSend(i2cAddr, 3, packet);
}

void i2ceepromWriteBuffer(u32 memAddr, u08* data, u08 length)
{
	uint8_t i2cAddr = I2C_ADDRESS;
//	bool willOverflow = false;
// 	if(memAddr > 0x7FFF)
// 		i2cAddress |= 1 << 1;	
// 	else if(memAddr + length > 0x7FFF)
// 		willOverflow = true;


	u08 packet[length + 2];
	// prepare address + data
	packet[0] = (memAddr>>8);
	packet[1] = (memAddr&0x00FF);
	for(int i = 0; i < length; i++)
		packet[2+i] = *(data+i);
	// send memory address we wish to access to the memory chip
	// along with the data we wish to write
	i2cMasterSend(i2cAddr, length + 2, packet);		
}

void i2ceepromReadBuffer(u32 memAddr, u08* result, u08 length)
{
 	uint8_t i2cAddr = I2C_ADDRESS;
// 	bool willOverflow = false;
// 	if(memAddr - length > 0x7FFF)
// 		i2cAddress |= 1 << 1;
// 	else if (memAddr > 0x7FFF)
// 	{
// 		i2cAddress |= 1 << 1;
// 		willOverflow = true;	
// 	}
// 	
	u08 packet[2];
	// prepare address
	packet[0] = (memAddr>>8);
	packet[1] = (memAddr&0x00FF);
	// send memory address we wish to access to the memory chip
	u08 status;
	int i = 0;
	do
	{
		i++;
		lcd_gotoXY(4,0);
		i2cMasterSend(i2cAddr, 2, packet);
		status = inb(TWSR) & TWSR_STATUS_MASK;
		rprintf("%d %x ",i, status);
		_delay_ms(5);
	}while(status != TW_MT_DATA_ACK);
	
	// retrieve the data at this memory address
	i2cMasterReceive(i2cAddr, length, result);
}
