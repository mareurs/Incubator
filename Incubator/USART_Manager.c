/*
 * USART_Manager.c
 *
 * Created: 2/26/2015 9:58:44 PM
 *  Author: marius
 */ 

#include "global.h"
#include <string.h>
//#include "peripherals/uart.h"

void init()
{
// 	uartInit();
// 	uartSetBaudRate(9600);
// 	uartSendTxBuffer();
}

void processUSARTLine(char* data)
{
	//Process line
//	uartSendBuffer("Line received:\r\n",17);
//	uartSendBuffer(data,20);	
}

USART_FPTR USART_Callback = &processUSARTLine;

