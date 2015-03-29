/*
 * MemoryManager.h
 *
 * Created: 2/23/2015 2:36:15 AM
 *  Author: marius
 */ 

#include "DataRow.h"
#include "avrlibtypes.h"

#ifndef MEMORYMANAGER_H_
#define MEMORYMANAGER_H_

uint16_t idx;
u32 memoryAddress;

void sendData(DataRow* data);
DataRow getData();
void eraseMemory();
void sendDataToUart();
void saveBalanceTemp(uint16_t value);
void saveBalanceHumid(uint8_t value);
uint16_t getBalanceTemp();
uint8_t getBalanceHumid();

#endif /* MEMORYMANAGER_H_ */