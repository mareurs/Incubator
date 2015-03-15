/*
 * i2c_sht.c
 *
 * Created: 3/8/2015 3:49:03 PM
 *  Author: marius
 */ 
#include "i2c_sht.h"
#include "rprintf.h"

void I2C_Write1(uint8_t data)
{
	//uint8_t error =	
	i2cMasterSend(SHT_ADDRESS, 1, &data);
	//rprintf("Error: %d", error);
}

void I2C_Read(uint8_t *data,uint8_t length)
{
	//uint8_t error = 
	i2cMasterReceive(SHT_ADDRESS, length, data);	
	//rprintf("Error: %d", error);
}
