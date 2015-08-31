/*
 * MemoryManager.cpp
 *
 * Created: 2/24/2015 10:32:23 PM
 *  Author: marius
 */ 

#include "MemoryManager.h"
#include "peripherals/i2ceeprom.h"
#include <string.h>
#include "peripherals/uart.h"
#include "avrlibtypes.h"
#include <util/delay.h>
#include "rprintf.h"

#define MEMORY_START 20
#define BALANCE_TEMP_POSITION 2
#define BALANCE_HUMID_POSITION 4

//Protos
void writeIndex();

u32 memoryAddress = MEMORY_START;
uint16_t idx = 0;

void incrementAddress()
{
	memoryAddress+= DATAROW_SIZE;
	idx++;
}


void saveBalanceTemp(uint16_t value)
{
	i2ceepromWriteBuffer(BALANCE_TEMP_POSITION, (uint8_t*) &value, 2);
}

void saveBalanceHumid(uint8_t value)
{
	i2ceepromWriteBuffer(BALANCE_HUMID_POSITION, &value, 1);	
}

uint16_t getBalanceTemp()
{
	uint16_t result;
	i2ceepromReadBuffer(BALANCE_TEMP_POSITION, (uint8_t*) &result, 2);
	return result;
}

uint8_t getBalanceHumid()
{
	uint8_t result;
	i2ceepromReadBuffer(BALANCE_HUMID_POSITION, &result, 1);
	return result;
}

void sendData(DataRow* data)
{
	uint8_t buffer[DATAROW_SIZE];
	serialize(buffer, data);
	i2ceepromWriteBuffer(memoryAddress, buffer, DATAROW_SIZE);
	_delay_ms(50);
	writeIndex();
	incrementAddress();
}

DataRow getData()
{
	uint8_t buffer[DATAROW_SIZE];
	i2ceepromReadBuffer(memoryAddress, buffer, DATAROW_SIZE);
	incrementAddress();
	return deserialize(buffer);
	_delay_ms(50);
}

uint16_t getCurrentIndex()
{
	uint16_t currIdx;
	i2ceepromReadBuffer( 0, (uint8_t*) &currIdx, 2 );
	return currIdx;
}

void sendDataToUart()
{
	uartInit();
	uartSendTxBuffer();
	uint16_t currentIdx = getCurrentIndex();
	idx = 0;
	memoryAddress = MEMORY_START;
	_delay_ms(50);
	rprintf("%d\n",currentIdx);
	
	for(int i = 0; i < currentIdx + 1; i++)
	{
		DataRow data = getData();
		rprintfFloat(3, data.T1/10.0);
		rprintf(";");
		rprintfFloat(3, data.T2/10.0);
		rprintf(";");
		rprintfFloat(3, data.T3/10.0);
		rprintf(";");
		rprintf("%d\n", data.U);
	}		
}

void writeIndex()
{
	i2ceepromWriteBuffer(0, (uint8_t*) &idx, 2);
}

 void eraseMemory()
{
	uint8_t zero[256];
	memset(zero,0,256);
	
	for(u32 i = 0; i < 1024*4; i++)
	{
		//i2ceepromWriteBuffer((u32) i*256, zero, 256);
	}
}


