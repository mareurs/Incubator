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

volatile bool menuActivated = false;
volatile uint8_t previousValue = 0xFF;
uint8_t position = 2;
uint8_t menu = NO_MENU;

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
	PCMSK1 = 1 << PCINT8;
}

void showMainMenu()
{
	lcd_clrscr();
	lcd_gotoXY(1,0);
	rprintfInit(lcd_putc);
	rprintf(" -- MAIN MENU --");
	lcd_gotoXY(position,0);
	lcd_putc(MARKER);
	lcd_gotoXY(2,2);
	rprintf("Set Temp");
	lcd_gotoXY(3,2);
	rprintf("Set Umid");
	lcd_gotoXY(4,2);
	rprintf("Set Time");
}

void menuLoop()
{
	showMainMenu();
	while(menuActivated)
	{
		_delay_ms(1000);
	}
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
	if(position > 2)
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
		_delay_ms(50);
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
		_delay_ms(50);
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
		_delay_ms(50);
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
		_delay_ms(50);
	}
}

void downClicked()
{
	switch(menu)
	{
		case MAIN:
			goDown();
			break;
		case TEMP:
			decreaseTemp();
			break;
		case HUMID:
			decreaseHumid();
			break;			
	}	
}

void upClicked()
{
	switch(menu)
	{
		case MAIN:
			goUp();
			break;
		case TEMP:
			increaseTemp();
			break;
		case HUMID:
			increaseHumid();
			break;
	}

}

void showTempMenu()
{
	menu = TEMP;
	lcd_clrscr();
	rprintf("Temp Menu");
	lcd_gotoXY(2,0);
	rprintfFloat(4,balanceTemperature);
}

void showHumidMenu()
{
	menu = HUMID;
	lcd_clrscr();
	rprintf("Umid Menu");
	lcd_gotoXY(2,0);
	rprintf("%d",balanceHumidity);
}

void showTimeMenu()
{
	menu = TIME;
	lcd_clrscr();
	rprintf("Time Menu");
	lcd_gotoXY(2,0);
	rprintf("Zi:%d %d:%d:%d", dateTime.day, dateTime.hour, dateTime.minute, dateTime.second);
}


void showSubMenu(uint8_t position)
{
	switch(position)
	{
		case 2:
			showTempMenu();
			break;
		case 3:
			showHumidMenu();
			break;
		case 4:
			showTimeMenu();
			break;
	}
}

void okClicked()
{
	switch(menu)
	{
		case MAIN:
			showSubMenu(position);
			break;
	}
}



ISR(INT0_vect)
{
	if(menu == MAIN)
		menuActivated = false;
	else
	{
		menuActivated = true;
		menu = MAIN;		
		showMainMenu();
	}
}

ISR(PCINT2_vect)
{
	if( (DOWN_BTN_PIN & (1 << DOWN_BTN)) == 0)
		downClicked();
	else if((UP_BTN_PIN & (1 << UP_BTN)) == 0)
		upClicked();
}

ISR(PCINT1_vect)
{
	if( (OK_BTN_PIN & (1 << OK_BTN)) == 0)
		okClicked();	
}