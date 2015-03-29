/*
 * DataRow.h
 *
 * Created: 2/23/2015 2:27:22 AM
 *  Author: marius
 */ 


#ifndef DATAROW_H_
#define DATAROW_H_

#include <inttypes.h>

static const uint8_t DATAROW_SIZE = 8;	//be carefull of packing
//static uint16_t currentIndex;

typedef struct
{
	uint16_t T1;
	uint16_t T2;
	uint16_t T3;
	uint16_t U;
} DataRow;

void serialize(uint8_t* result, const DataRow* data);
DataRow deserialize(const uint8_t* data);

#endif /* DATAROW_H_ */