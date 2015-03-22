/*
 * Incubator.cpp
 *
 * Created: 2/22/2015 9:11:15 PM
 *  Author: marius
 */ 
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>
#include "global.h"
#include "peripherals/uart.h"
#include "peripherals/lcd/hd44780.h"
#include "utils/rprintf.h"
#include "utils/UtilsCollection.h"
#include "peripherals/ds18x20/ds18x20.h"
#include "peripherals/sht21/sht21.h"
#include "MachineState.h"

int main(void)
{
	lcd_init();
	uartInit();
	uartSetBaudRate(9600);
	rprintfInit(uartSendByte);
	rprintfStr("Initialize:\n");
	lcd_puts("Initialize:");
	MachineInit();
	sei();
	while(1)
	{
		checkMachineStatus();
	}


}