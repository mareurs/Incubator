/*
 * DateTime.c
 *
 * Created: 3/14/2015 2:16:31 AM
 *  Author: marius
 */ 


#include "DateTime.h"
#include <avr/interrupt.h>
#include <stdbool.h>
#include "MenuManager.h"
#include "MachineState.h"

volatile DateTime dateTime;

void initDateTime()
{
	TCCR1A = 0;
	OCR1A = 15625; //7812;		//1s for 8Mhz / 1024 prescaler
	TIMSK1 = 1 << OCIE1A;
	TCCR1B = 1 << WGM12 | 1 << CS12 | 1 << CS10;
	dateTime.second = 1;
	dateTime.day = 1;
}


ISR(TIMER1_COMPA_vect)
{
	static uint8_t t0 = 0;
	static bool startRolling = false;
	
	if(startRolling)
	{
		t0++;
		if(t0 == 6)
		{
			startRolling = false;
			cbi(ROLLING_PORT, ROLLING);
		}
	}
	
	if(++dateTime.second >= 60)
	{			
		dateTime.second = 0;
		if(dateTime.minute % 15 == 0)
		readWaterSensor();
					
		if( dateTime.minute%5 == 0)
		fiveMinutesPassed = true;

		if(++dateTime.minute >= 60)
		{
			dateTime.minute = 0;
			
			if(doRolling)
			{
				sbi(ROLLING_PORT, ROLLING);
				t0 = 0;
				startRolling = true;			
			}
			if(++dateTime.hour >= 24)
				++dateTime.day;
		}
	}

  	if(!menuActivated)
	{
		if(startR1)
			toggleR1();
		if(startR2)
			toggleR2();
	
		if(buzzIsOn)
			startBuzz();
		else
			stopBuzz();
	
		printToLCD();		 
	}

	

}

