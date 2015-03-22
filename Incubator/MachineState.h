/*
 * MachineState.h
 *
 * Created: 2/23/2015 2:43:03 AM
 *  Author: marius
 */ 
#include <stdbool.h>

#ifndef MACHINESTATE_H_
#define MACHINESTATE_H_

#define ROLLING_DDR		DDRD
#define ROLLING_PORT	PORTD
#define ROLLING_PIN		PIND
#define ROLLING			PORTD7

#define OVRHEAT_DDR		DDRC
#define OVRHEAT_PORT	PORTC
#define OVRHEAT_PIN		PINC
#define OVRHEAT			PORTC3

#define BUZZ_DDR		DDRC
#define BUZZ_PORT		PORTC
#define BUZZ_PIN		PINC
#define BUZZ			PORTC0

#define FAN_DDR			DDRD
#define FAN_PORT		PORTD
#define FAN_PIN			PIND
#define FAN				PORTD6

#define WATER_DDR		DDRC
#define WATER_PORT		PORTC
#define WATER_PIN		PINC
#define WATER			PORTC2

#define HUMRELAY_DDR	DDRC
#define HUMRELAY_PORT	PORTC
#define HUMRELAY_PIN	PINC
#define HUMRELAY		PORTC1

#define R1_DDR			DDRB
#define R1_PORT			PORTB
#define R1_PIN			PINB
#define R1				PORTB1

#define R2_DDR			DDRB
#define R2_PORT			PORTB
#define R2_PIN			PINB
#define R2				PORTB2

#define FIRST_STAGE_TEMP 38.0
#define SECOND_STAGE_TEMP 37.0
#define FIRST_STAGE_HUMIDITY 55
#define SECOND_STAGE_HUMIDITY 75
#define MAX_TEMP_DELTA		0.5

typedef enum
{
	OUT_OF_WATER, LOW_TEMPERATURE, HIGH_TEMPERATURE, MISSING_SENSOR, NONE
} MachineErrors;

volatile bool startR1;
volatile bool startR2;
bool buzzIsOn;
double balanceTemperature;
uint8_t balanceHumidity;

void checkMachineStatus();
void MachineInit();
void printToLCD();
void toggleR1();
void toggleR2();
void startBuzz();
void stopBuzz();

#endif /* MACHINESTATE_H_ */