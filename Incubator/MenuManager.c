/*
 * MenuManager.c
 *
 * Created: 3/22/2015 5:52:49 PM
 *  Author: marius
 */ 

#include <avr/interrupt.h>
#include <avr/io.h>
#include "rprintf.h"
#include "MenuManager.h"
#include "peripherals/lcd/hd44780.h"
#include "avrlibdefs.h"
#include "avrlibtypes.h"
#include <stdbool.h>
#include <util/delay.h>

volatile bool menuActivated = false;
volatile uint8_t previousValue = 0xFF;
uint8_t position = 2;

void initMenuManager()
{
	cbi(MENU_BTN_DDR, MENU_BTN);
	sbi(MENU_BTN_PORT, MENU_BTN);	//activate pull_up
	cbi(UP_BTN_DDR, UP_BTN);
	sbi(UP_BTN_PORT, UP_BTN);	
	cbi(DOWN_BTN_DDR, DOWN_BTN);
	sbi(DOWN_BTN_PORT, DOWN_BTN);

	EIMSK |= 1 << INT0;
	PCICR |= 1 << PCIE2;
	PCMSK2 = 1 << PCINT16 | 1<<PCINT17;
}

void showMenu()
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
	showMenu();
	while(1)
	{
		_delay_ms(1000);
	}
}

void moveUp()
{
	lcd_gotoXY(position,0);
	lcd_putc(' ');
	if(position > 2)
		position--;
	lcd_gotoXY(position, 0);
	lcd_putc(MARKER);
}

void moveDown()
{
	lcd_gotoXY(position,0);
	lcd_putc(' ');
	if(position < 4)
	position++;
	lcd_gotoXY(position, 0);
	lcd_putc(MARKER);
}


ISR(INT0_vect)
{
	//show menu
	menuActivated = true;
}

ISR(PCINT2_vect)
{
	if( (DOWN_BTN_PIN & (1 << DOWN_BTN)) == 0)
		moveDown();
	else if((UP_BTN_PIN & (1 << UP_BTN)) == 0)
		moveUp();
}