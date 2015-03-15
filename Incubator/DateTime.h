/*
 * DateTime.h
 *
 * Created: 2/23/2015 2:39:56 AM
 *  Author: marius
 */ 


#include "global.h"
#include "MachineState.h"

#ifndef DATETIME_H_
#define DATETIME_H_

typedef struct
{
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;	
} DateTime;

void initDateTime();
volatile extern DateTime dateTime;

#endif /* DATETIME_H_ */