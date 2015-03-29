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
#include "MenuManager.h"
#include "peripherals/i2ceeprom.h"

int main(void)
{
	lcd_init();
	lcd_puts("Initialize:");
	MachineInit();
	initMenuManager();
	i2cInit();
	sei();
/*	
	while(1)
	{
		lcd_clrscr();
		rprintfInit(lcd_putc);
		rprintf("Writing ...");
		for( uint16_t i = 256; i < 280; i+= 2 )
		{
			lcd_gotoXY(2,0);
			rprintf("%d",i);
			i2ceepromWriteBuffer(i, (uint8_t*) &i, 2);
			lcd_gotoXY(3,0);
			uint16_t result;
			_delay_ms(100);
			i2ceepromReadBuffer(i, (uint8_t*) &result, 2);
// 			uint16_t tmp;
// 			i2ceepromReadBuffer(0, (uint8_t*) &tmp, 2);
			rprintf("%d",result);
			//_delay_ms(50);
		}
		lcd_clrscr();
// 		rprintf("Reading ...");
// 		for(uint16_t i = 256; i < 280; i+=2)
// 		{
// 			lcd_gotoXY(2,0);
// 			uint16_t result;
// 			i2ceepromReadBuffer(i, (uint8_t*) &result, 2);
// 			lcd_gotoXY(3,0);
// 			rprintf("%d", result );
// 			_delay_ms(50);
// 			if(i != result)
// 			{
// 				lcd_gotoXY(4,0);
// 				rprintf("E %d - %d", i, result);
// 				_delay_ms(50);
// 			}
// 		}
	}
	*/
	while(1)
	{
		checkMachineStatus();
		if(menuActivated)
			menuLoop();
	}


}