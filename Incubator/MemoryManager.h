/*
 * MemoryManager.h
 *
 * Created: 2/23/2015 2:36:15 AM
 *  Author: marius
 */ 

#include "DataRow.h"

#ifndef MEMORYMANAGER_H_
#define MEMORYMANAGER_H_

uint16_t idx;
uint16_t memoryAddress;

void sendData(DataRow* data);
DataRow getData();
void eraseMemory();
			
#endif /* MEMORYMANAGER_H_ */