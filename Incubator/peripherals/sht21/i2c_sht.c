/*
 * i2c_sht.c
 *
 * Created: 3/8/2015 3:49:03 PM
 *  Author: marius
 */ 
#include "i2c_sht.h"
#include "rprintf.h"

void I2C_Write(uint8_t data)
{
	i2cMasterSend(SHT_ADDRESS, 1, &data);
}

void I2C_Read(uint8_t *data,uint8_t length)
{
	i2cMasterReceive(SHT_ADDRESS, length, data);	
}
