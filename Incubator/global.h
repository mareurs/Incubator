/*
 * defaults.h
 *
 * Created: 2/22/2015 9:20:06 PM
 *  Author: marius
 */ 


#ifndef DEFAULTS_H_
#define DEFAULTS_H_

#include <inttypes.h>

#define RPRINTF_FLOAT
#define CYCLES_PER_US ((F_CPU+500000)/1000000) 	// cpu cycles per microsecond

typedef void (*USART_FPTR) (char* data);
extern USART_FPTR USART_Callback;

// global AVRLIB defines
#include "utils/avrlibdefs.h"
// global AVRLIB types definitions
#include "utils/avrlibtypes.h"

#endif /* DEFAULTS_H_ */