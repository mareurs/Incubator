/*
 * MenuManager.c
 *
 * Created: 3/22/2015 5:52:49 PM
 *  Author: marius
 */ 

#include <avr/interrupt.h>
#include <avr/io.h>
#include "utils/rprintf.h"
#include "MenuManager.h"
#include "peripherals/lcd/hd44780.h"
#include "avrlibdefs.h"
#include "avrlibtypes.h"
#include <stdbool.h>
#include <util/delay.h>
#include "MachineState.h"
#include "DateTime.h"
#include "MemoryManager.h"
#include "rprintf.h"

volatile bool menuActivated = false;
volatile uint8_t previousValue = 0xFF;
uint8_t position = 1;
volatile bool menuIsClicked = false;
volatile bool okIsClicked = false;
volatile bool downIsClicked = false;
volatile bool upIsClicked = false;

//Protos

void goDown();
void goUp();
void showSubMenu(uint8_t position);

void initMenuManager()
{
	cbi(MENU_BTN_DDR, MENU_BTN);
	sbi(MENU_BTN_PORT, MENU_BTN);	//activate pull_up
	cbi(UP_BTN_DDR, UP_BTN);
	sbi(UP_BTN_PORT, UP_BTN);	
	cbi(DOWN_BTN_DDR, DOWN_BTN);
	sbi(DOWN_BTN_PORT, DOWN_BTN);
	cbi(OK_BTN_DDR, OK_BTN);
	sbi(OK_BTN_PORT, OK_BTN);

	EIMSK |= 1 << INT0;
	PCICR |= 1 << PCIE2 | 1 << PCIE1;
	PCMSK2 = 1 << PCINT16 | 1<<PCINT17;
	PCMSK1 = 1 << PCINT11;
}

void showMainMenu()
{
	initMenuManager();
	lcd_clrscr();
	rprintfInit(lcd_putc);
	lcd_gotoXY(1,2);
	rprintf("SET TEMP");
	lcd_gotoXY(2,2);
	rprintf("SET UMID");
	lcd_gotoXY(3,2);
	rprintf("SET TIMP");
	lcd_gotoXY(4,2);
	rprintf("DESCARCARE");
	lcd_gotoXY(position,0);
	lcd_putc(MARKER);
}

void menuLoop()
{
	menuIsClicked = false;
	for(;;)
	{
		showMainMenu();
		if(upIsClicked)
		{
			upIsClicked = false;
			goUp();
		}
		else if(downIsClicked)
		{
			downIsClicked = false;
			goDown();
		}
		else if(okIsClicked)
		{
			okIsClicked = false;
			showSubMenu(position);
		}
		else if(menuIsClicked)
			break;
		_delay_ms(100);
	}
	menuIsClicked = okIsClicked = menuActivated = false;
}


void goDown()
{
	lcd_gotoXY(position,0);
	lcd_putc(' ');
	if(position < 4)
		position++;
	lcd_gotoXY(position, 0);
	lcd_putc(MARKER);	
}

void goUp()
{
	lcd_gotoXY(position,0);
	lcd_putc(' ');
	if(position > 1)
		position--;
	lcd_gotoXY(position, 0);
	lcd_putc(MARKER);	
}


void decreaseTemp()
{
	while( (DOWN_BTN_PIN & (1 << DOWN_BTN)) == 0 )
	{
		balanceTemperature -= 0.1;
		lcd_gotoXY(2,0);
		lcd_puts("         ");
		lcd_gotoXY(2,0);
		rprintfFloat(3, balanceTemperature);
		_delay_ms(200);
	}
}

void increaseTemp()
{
	while( (UP_BTN_PIN & (1 << UP_BTN)) == 0 )
	{
		balanceTemperature += 0.1;
		lcd_gotoXY(2,0);
		lcd_puts("         ");
		lcd_gotoXY(2,0);
		rprintfFloat(3, balanceTemperature);
		_delay_ms(200);
	}
}

void decreaseHumid()
{
	while( (DOWN_BTN_PIN & (1 << DOWN_BTN)) == 0 )
	{
		balanceHumidity --;
		lcd_gotoXY(2,0);
		lcd_puts("         ");
		lcd_gotoXY(2,0);
		rprintf("%d",balanceHumidity);
		_delay_ms(200);
	}
}

void increaseHumid()
{
	while( (UP_BTN_PIN & (1 << UP_BTN)) == 0 )
	{
		balanceHumidity ++;
		lcd_gotoXY(2,0);
		lcd_puts("         ");
		lcd_gotoXY(2,0);
		rprintf("%d",balanceHumidity);		
		_delay_ms(200);
	}
}

void showTempMenu()
{
	lcd_clrscr();
	rprintf("   -SET TEMP-");
	lcd_gotoXY(2,0);
	rprintfFloat(4,balanceTemperature);
	for(;;)
	{
		if(upIsClicked)
		{
			increaseTemp();
			upIsClicked = false;
		}
		else if (downIsClicked)
		{
			decreaseTemp();
			downIsClicked = false;
		}
		else if(okIsClicked || menuIsClicked)
			break;
		_delay_ms(100);
	}
	okIsClicked = menuIsClicked = false;	
	uint16_t t = balanceTemperature * 10;
	saveBalanceTemp(t);
}

void showHumidMenu()
{
	lcd_clrscr();
	rprintf(" -SET UMIDITATE-");
	lcd_gotoXY(2,0);
	rprintf("%d",balanceHumidity);
	for(;;)
	{				
		if(upIsClicked)
		{			
			increaseHumid();
			upIsClicked = false;
		}
		else if (downIsClicked)
		{
			decreaseHumid();
			downIsClicked = false;				
		}
		else if(okIsClicked || menuIsClicked)
			break;
		_delay_ms(100);
	}
	okIsClicked = menuIsClicked = false;
	saveBalanceHumid(balanceHumidity);
}

void printDateTime()
{
	lcd_gotoXY(2,0);
	rprintf("                ");
	lcd_gotoXY(2,0);
	rprintf("Zi:%d %d:%d:%d", dateTime.day, dateTime.hour, dateTime.minute, dateTime.second);
}


void showTimeMenu()
{
	lcd_clrscr();
	rprintf("   -SET TIMP-");
	lcd_gotoXY(2,0);
	rprintf("Zi:%d %d:%d:%d", dateTime.day, dateTime.hour, dateTime.minute, dateTime.second);
	for(;;)
	{
		if(upIsClicked)
		{
			if(dateTime.day == 22)
				continue;
			dateTime.day++;
			dateTime.hour = dateTime.minute = dateTime.second = 0;
			lcd_gotoXY(2,0);
			printDateTime();
			_delay_ms(200);
			upIsClicked = false;
		}
		
		if(downIsClicked)
		{
			if(dateTime.day == 1)
				continue;
			dateTime.day--;
			dateTime.hour = dateTime.minute = dateTime.second = 0;			
			printDateTime();
			_delay_ms(200);
			downIsClicked = false;
		}
		
		if(okIsClicked || menuIsClicked)
		{
			okIsClicked = menuIsClicked = false;
			break;						
		}
		_delay_ms(100);	
	}
}

void showDownloadMenu()
{
	lcd_clrscr();
	rprintf("Pornesc in 10s");
	for(int i = 0; i < 10;i ++)
	{
		lcd_clrscr();
		lcd_gotoXY(2,8);
		rprintf("%d", 10 - i);		
		_delay_ms(1000);
	}
	lcd_clrscr();
	rprintf("Descarc ...");
	sendDataToUart();
	lcd_clrscr();
	rprintf("Terminat" );
	_delay_ms(1000);
	okIsClicked = false;
}

void showSubMenu(uint8_t position)
{
	switch(position)
	{
		case 1:
			showTempMenu();
			break;
		case 2:
			showHumidMenu();
			break;
		case 3:
			showTimeMenu();
			break;
		case 4:
			showDownloadMenu();
			break;
	}
}


ISR(INT0_vect)
{
	menuIsClicked = true;
	menuActivated = true;
}

ISR(PCINT2_vect)
{
	if( (DOWN_BTN_PIN & (1 << DOWN_BTN)) == 0)
		downIsClicked = true;
	else if((UP_BTN_PIN & (1 << UP_BTN)) == 0)
		upIsClicked = true;
}

ISR(PCINT1_vect)
{
	if( (OK_BTN_PIN & (1 << OK_BTN)) == 0)
		okIsClicked = true;	
}