/*
 * i2c_sht.h
 *
 * Created: 3/8/2015 3:48:48 PM
 *  Author: marius
 */ 

#include <inttypes.h>
#include "../i2c.h"

#ifndef I2C_SHT_H_
#define I2C_SHT_H_

#define SHT_ADDRESS	0x80

void I2C_Write1(uint8_t data);
void I2C_Read(uint8_t *data,uint8_t length);


#endif /* I2C_SHT_H_ */