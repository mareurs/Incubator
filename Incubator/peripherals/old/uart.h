/*
 * uart.h
 *
 * Created: 2/22/2015 9:37:52 PM
 *  Author: marius
 */ 

#ifdef __cplusplus	// If included in C++ code
extern "C" {
#endif


#include "../defaults.h"
#include <avr/io.h>

#ifndef UART_H_
#define UART_H_

//Declaration of our functions
void USART_init(int baudRate);
unsigned char USART_receive(void);
void USART_send( unsigned char data);
void USART_putstring(char* StringPtr);

#endif /* UART_H_ */


#ifdef __cplusplus
}
#endif