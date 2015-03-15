/*
 * MemoryManager.cpp
 *
 * Created: 2/24/2015 10:32:23 PM
 *  Author: marius
 */ 

#include "MemoryManager.h"

void incrementAddress()
{
	memoryAddress+= DATAROW_SIZE;
	idx++;
}

void sendData(DataRow* data)
{
	uint8_t buffer[DATAROW_SIZE];
	serialize(buffer, data);
	
	incrementAddress();
}

DataRow getData()
{
	uint8_t buffer[DATAROW_SIZE];
	//E1024_readBuffer(memoryAddress, buffer, DATAROW_SIZE);
	incrementAddress();
	return deserialize(buffer);
}

 void eraseMemory()
{
	
}


