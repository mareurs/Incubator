/*
 * DataRow.cpp
 *
 * Created: 2/23/2015 2:26:55 AM
 *  Author: marius
 */ 

#include "DataRow.h"
#include "utils/utilsCollection.h"
#include <string.h>

static const uint8_t DATAROW_SIZE;

void serialize(uint8_t* result, const DataRow* data)
{
	memcpy(result, data, sizeof(DataRow));
}

DataRow deserialize(const uint8_t* data)
{
	DataRow result;
	memcpy(&result, data, sizeof(DataRow));
	return result;
}
