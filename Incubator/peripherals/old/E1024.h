/*
 * _E1024.h
 *
 * Created: 2/23/2015 2:15:04 AM
 *  Author: marius
 */ 

#ifdef __cplusplus	// If included in C++ code
extern "C" {
#endif

#ifndef E1024_H_
#define E1024_H_

#include <inttypes.h>
#include "twi.h"

#define ERROR 0
#define SUCCESS 1
#define FULL_MASK 0x7FFFF
#define DEVICE_MASK 0x7F0000
#define WORD_MASK 0xFFFF


void E1024_init();
uint8_t E1024_write(uint16_t dataAddress, uint8_t data);
uint8_t E1024_read(uint16_t dataAddress, uint8_t* u8data );
uint8_t	E1024_writeBuffer(uint16_t address, uint8_t* data, uint8_t size);
uint8_t E1024_readBuffer(uint16_t address, uint8_t* data, uint8_t size);

#endif /* E1024_H_ */

#ifdef __cplusplus
}
#endif