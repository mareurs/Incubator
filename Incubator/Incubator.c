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

void verifyMemory();
void testI2cMemory(void);

#define LOCAL_ADDR	0xA0
#define TARGET_ADDR	0xA0


int main(void)
{
 	lcd_init();
 	lcd_puts("Initialize:");
 	//MachineInit();
 	//initMenuManager();
	i2cInit();
	uartInit();
	uartSetBaudRate(9600);
	sei();
	verifyMemory();
	
	while(1)
	{
		testI2cMemory();
		_delay_ms(1000);
		
	}
	while(1)
	{
		checkMachineStatus();
		if(menuActivated)
			menuLoop();
	}

}


void testI2cMemory(void)
{
	unsigned char localBuffer[] = "Pascal is cool!!Pascal is Cool!!";
	unsigned char localBufferLength = 0x20;
	
	u08 i;
	u08 txdata[66];
	u08 rxdata[66];

	rprintfInit(uartSendByte);
	rprintf("\r\nRunning I2C memory test (24xxyy devices)\r\n");

	// compose address
	txdata[0] = 0;
	txdata[1] = 0;
	// compose data
	for(i=0; i<16; i++)
		txdata[2+i] = localBuffer[i];
	rprintf("Stored data: ");
	// null-terminate data
	txdata[18] = 0;
	rprintfStr(&txdata[2]);
	rprintfCRLF();

	// send address and data
	i2cMasterSend(TARGET_ADDR, 18, txdata);
	
	
	//_delay_ms(100);

	// send address
	i2cMasterSend(TARGET_ADDR, 2, txdata);
	// get data
	i2cMasterReceive(TARGET_ADDR, 16, rxdata);
	// null-terminate received string
	rxdata[16] = 0;

	rprintf("Received data: ");
	rprintfStr(rxdata);
	rprintfCRLF();

/*
	u08 c;
	u16 addr=0;

	while(1)
	{
		while(!uartReceiveByte(&c));

		switch(c)
		{
		case '+':
			addr+=64;
			break;
		case '-':
			addr-=64;
			break;
		}
		c = 0;
		txdata[0] = (addr>>8);
		txdata[1] = (addr&0x00FF);
		i2cMasterSendNI(TARGET_ADDR, 2, txdata);
		i2cMasterReceiveNI(TARGET_ADDR, 64, rxdata);
		rprintf("Received data at ");
		rprintfu16(addr);
		rprintf(":\r\n");
		debugPrintHexTable(64, rxdata);
	}
*/
}


void verifyMemory()
{
	i2cSetBitrate(400);
	while(1)
	{
		menuActivated = true;
		lcd_clrscr();
		rprintfInit(lcd_putc);
		int j = 0;
		for( uint32_t i = 256; i < 300; i+= 2 )
		{
			lcd_clrscr();
			i2ceepromWriteBuffer(i, (uint8_t*) &i, 4);
			uint32_t result;
 			lcd_gotoXY(1,0);
 			rprintf("i=%d s1=%d %d\n\r",i, i2cGetState(), i2cGetStatus());
			_delay_ms(50);
			i2ceepromReadBuffer(i, (uint8_t*) &result, 4);
			_delay_ms(25000);
 			rprintf("r=%d s2=%d %d\n\r",result, i2cGetState(), i2cGetStatus());			
			{
				lcd_gotoXY(3,0);
				rprintf("Error %d %d\n\r", i, result );
				_delay_ms(40000);
				lcd_clrscr();
			}
		}
	}
	
}