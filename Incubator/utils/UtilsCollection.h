/*
 * utilsCollection.h
 *
 * Created: 2/23/2015 2:54:11 AM
 *  Author: marius
 */ 

#include <inttypes.h>
#include <string.h>

#ifndef UTILSCOLLECTION_H_
#define UTILSCOLLECTION_H_

uint8_t lsb(uint16_t value);
uint8_t msb(uint16_t value);
void appendNewLine(char* buff);

#endif /* UTILSCOLLECTION_H_ */