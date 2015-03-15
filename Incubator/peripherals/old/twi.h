/*
 * twi.h
 *
 * Created: 2/23/2015 1:45:50 AM
 *  Author: marius
 */ 
#ifdef __cplusplus	// If included in C++ code
extern "C" {
	#endif

#include <inttypes.h>

#ifndef TWI_H_
#define TWI_H_

void TWIInit(void);
void TWIStart(void);
void TWIStop(void);
void TWIWrite(uint8_t u8data);
uint8_t TWIReadACK(void);
uint8_t TWIReadNACK(void);
uint8_t TWIGetStatus(void);

#endif /* TWI_H_ */

#ifdef __cplusplus
}
#endif