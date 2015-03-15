/*
 * uart.c
 *
 * Created: 2/22/2015 9:37:20 PM
 *  Author: marius
 */ 

#include "uart.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include "../defaults.h"

#define MAX_BUFFER_SIZE	48

void USART_init(int baudRate)
{
	int baudPrescaller = (F_CPU / (baudRate * 16UL)) - 1; 		
	UBRR0H = (uint8_t)(baudPrescaller>>8);
	UBRR0L = (uint8_t)(baudPrescaller);
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
    UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);
    //UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
	
}

unsigned char USART_receive(void)
{	
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
}

void USART_send( unsigned char data)
{	
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

void enableTX()
{
	UCSR0B &= ~(1<<RXEN0);
	UCSR0B |= (1<<TXEN0);
}

void enableRX()
{
	UCSR0B &= ~(1<<TXEN0);
	UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);
}

void USART_putstring(char* StringPtr)
{	
	enableTX();
	while(*StringPtr != 0x00)
	{
		USART_send(*StringPtr);
		StringPtr++;
	}
	enableRX();	
}

ISR ( USART_RX_vect )
{
	static char buffer[MAX_BUFFER_SIZE];
	static uint8_t idx = 0;
	char byte = UDR0;
	buffer[idx++] = byte;
	
	if(byte == '.' || byte == '\n' || byte == '\r')
	{
		(*USART_Callback)(buffer);
		int i = 0;
		for(; i <= idx; i++)
			buffer[i] = 0;
		idx = 0;
	}
}