/*
 * MachineState.c
 *
 * Created: 3/8/2015 11:58:55 PM
 *  Author: marius
 */ 

#include "peripherals/ds18x20/ds18x20.h"
#include "peripherals/lcd/hd44780.h"
#include "rprintf.h"
#include "MachineState.h"
#include "DateTime.h"
#include "peripherals/sht21/sht21.h"
#include <string.h>
#include <stdio.h>
#include "peripherals/uart.h"
#include <stdlib.h>
#include <avr/interrupt.h>
#include "MenuManager.h"

volatile bool startR1 = false;
volatile bool startR2 = false;
bool buzzIsOn = false;

char* LCD_ERROR_MESSAGES[] = {"NIVEL APA SCAZUT","TEMP SUB NIVEL","TEMP PESTE NIVEL","LIPSA SENZOR"};

MachineErrors machineError;
double balanceTemperature = FIRST_STAGE_TEMP;
uint8_t balanceHumidity = FIRST_STAGE_HUMIDITY;

int16_t T1 = 0;
int16_t T2 = 0;
int16_t T3 = 0;
uint8_t U = 0;
uint8_t humidityValue = 0;
uint8_t fanValue = 50;
int16_t minTemp = 1000;
int16_t maxTemp = 0;
volatile uint8_t waterLevel;

// Protos
void initPins();
void initDDRDs();
void setFanSpeed(uint8_t percent);
void initADC();
void initPWMTimer();
void readSHTSensor();
void readWaterSensor();
void readSensors();
void readBSSensors();
void checkMachineStatus();
void printToLCD();
void raiseError(MachineErrors error);
void incrementFanSpeed();
void decrementFanSpeed();
void startHRelay();
void stopHRelay();
void printToUart();
void turnR1Off();
void turnR1On();
void turnR2On();
void turnR2Off();
void toggleR1();
void toggleR2();
void checkWaterSensor();
void checkHumidity();
void checkTemperatures();
//

void MachineInit()
{
	initPins();
	SHT21_Init();
	initDateTime();
	rprintfInit(lcd_putc);
}


void initPins()
{
	initDDRDs();	
	initADC();
	initPWMTimer();
	setFanSpeed(fanValue);
}

void initDDRDs()
{
	sbi(R1_DDR,R1);
	sbi(R2_DDR,R2);
	sbi(BUZZ_DDR, BUZZ);
	sbi(HUMRELAY_DDR,HUMRELAY);
	sbi(OVRHEAT_DDR,OVRHEAT);
	sbi(ROLLING_DDR,ROLLING);
}

void setFanSpeed(uint8_t percent)
{
	OCR0A = (percent / 100.0) * 255;
}

void initADC()
{
	ADMUX = (1<< REFS0);				//Use AVCC as refernce
	ADMUX |= (1 << ADLAR);				// 8bit conversion
	ADMUX &= 0xF0;						// Make last 4bits 0
	ADMUX |= 0b00000010;				// Choose channel ADC2
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
}

void initPWMTimer()
{
	sbi(FAN_DDR, FAN);
	TCCR0A = 1 << COM0A1 | 1 << WGM01 | 1 << WGM00;
	OCR0A = fanValue;
	TIMSK0 = 1 << TOIE0;
	TCCR0B = 1 << CS00 | 1 << CS02;		
}


void readSHTSensor()
{
	 SHT21_Read(&T2,&U);
}

void readWaterSensor()
{
	ADCSRA |= (1 << ADSC);
}

void readSensors()
{
	readBSSensors();
	readSHTSensor();
	readWaterSensor();
}

void readBSSensors()
{
	readTempSensor(&T1,&T3);
}


void checkMachineStatus()
{
	initPins();
	readSensors();	
	checkHumidity();
	checkTemperatures();
	checkWaterSensor();
	if(machineError == NONE)
		buzzIsOn = false;
	machineError = NONE;
}

void checkHumidity()
{
	if (humidityValue < balanceHumidity)
	{
		incrementFanSpeed();
		cbi(HUMRELAY_PORT,HUMRELAY);
	}
	else
	{
		decrementFanSpeed();
		sbi(HUMRELAY_PORT,HUMRELAY);
	}
}

void setMinMax()
{
	if(T1 > 1000 || T2 > 1000 || T3 > 1000 || T1 < 0 || T2 < 0 || T3 <0)
	{
		raiseError(MISSING_SENSOR);
		return;
	}
	if(T1 < minTemp)
	minTemp = T1;
	if(T2 < minTemp)
	minTemp = T2;
	if(T3 < minTemp)
	minTemp = T3;
	if(T1 > maxTemp)
	maxTemp = T1;
	if(T2 > maxTemp)
	maxTemp = T2;
	if(T3 > maxTemp)
	maxTemp = T3;
}

void checkTemperatures()
{
	setMinMax();
	
	bool startResistor = false;
	double t1 = T1/10.0;
	double t2 = T2/10.0;
	double t3 = T3/10.0;
	
	if(t1 < 0 || t1 > 100)
	{
		if(t2 > 0 && t2 < 100)
			t1 = t2;
		else if(t3 > 0 && t3 < 100)
			t1 = t3;
	}

	if(t2 < 0 || t2 > 100)
	{
		if(t1 > 0 && t1 < 100)
			t2 = t1;
		else if(t3 > 0 && t3 < 100)
			t2 = t3;
	}

	if(t3 < 0 || t3 > 100)
	{
		if(t2 > 0 && t2 < 100)
			t3 = t2;
		else if (t1 > 0 && t1 < 100)
			t3 = t1;
	}
	
	if(t1 < balanceTemperature)
	{
		startResistor = true;
		if(t1 < balanceTemperature - 1)
		{
			turnR1On();
			startResistor = false;
		}
		else
			turnR1Off();
		if(startResistor)
			startR1 = true;
	}
	else
	{
		turnR1Off();
		startR1 = false;
	}
	
	if(t3 < balanceTemperature)
	{
		startResistor = true;
		if(t3 < balanceTemperature - 1)
		{
			turnR2On();
			startResistor = false;
		}
		else
			turnR2Off();
		if(startResistor)
			startR2 = true;
	}
	else
	{
		startR2 = false;
		turnR2Off();
	}
	if(t1 < balanceTemperature - MAX_TEMP_DELTA || t2 < balanceTemperature - MAX_TEMP_DELTA || t3 < balanceTemperature - MAX_TEMP_DELTA )
		raiseError(LOW_TEMPERATURE);
	if(t1 > balanceTemperature + MAX_TEMP_DELTA  || t2 > balanceTemperature + MAX_TEMP_DELTA  || t3 > balanceTemperature + MAX_TEMP_DELTA)
		raiseError(HIGH_TEMPERATURE);
}

void raiseError(MachineErrors error)
{
	lcd_gotoXY(4,0);
	lcd_puts("                ");
	if(menuActivated)
		return;
	switch(error)
	{
		case OUT_OF_WATER:
			lcd_gotoXY(4,0);
			lcd_puts(LCD_ERROR_MESSAGES[OUT_OF_WATER]);
			buzzIsOn = true;
			break;
		case LOW_TEMPERATURE:
			lcd_gotoXY(4,0);
			lcd_puts(LCD_ERROR_MESSAGES[LOW_TEMPERATURE]);
			buzzIsOn = true;
			break;
		case HIGH_TEMPERATURE:
			lcd_gotoXY(4,0);
			lcd_puts(LCD_ERROR_MESSAGES[HIGH_TEMPERATURE]);
			buzzIsOn = true;
			break;
		case MISSING_SENSOR:
			lcd_gotoXY(4,0);
			lcd_puts(LCD_ERROR_MESSAGES[MISSING_SENSOR]);
			buzzIsOn = true;
			break;
		case NONE:
			break;		
	}
	_delay_ms(2000);
}

void printToLCD()
{
	//partially clear display
	lcd_gotoXY(1,0);
	lcd_puts("                ");
	lcd_gotoXY(2,0);
	lcd_puts("                ");
	lcd_gotoXY(3,0);
	lcd_puts("                ");

	rprintfInit(lcd_putc);
	lcd_gotoXY(1,0);
	rprintfFloat(3,T1/10.0);
	lcd_putc(' ');
	rprintfFloat(3,T2/10.0);
	lcd_putc(' ');
	rprintfFloat(3,T3/10.0);
	lcd_putc(' ');
	lcd_gotoXY(2,0);
	rprintf("U%d ", U);
	rprintf("m");
	rprintfFloat(3,minTemp/10.0);
	lcd_putc(' ');
	rprintf("M");
	rprintfFloat(3,maxTemp/10.0);
	lcd_putc(' ');
	lcd_gotoXY(3,0);
	rprintf("Zi:%d %d:%d:%d", dateTime.day, dateTime.hour, dateTime.minute, dateTime.second);
}

void printToUart()
{
	rprintfInit(uartSendByte);
	rprintfFloat(3,T1/10.0);
	rprintfStr(" ");
	rprintfFloat(3,T2/10.0);
	rprintfStr(" ");
	rprintfFloat(3,T3/10.0);
	rprintf("\n");
	rprintf("U:%d ", U);
	rprintf("m:");
	rprintfFloat(3,minTemp/10.0);
	rprintfStr(" ");
	rprintf("M:");
	rprintfFloat(3,maxTemp/10.0);
	rprintf("\n");
	rprintf("Zi:%d %d:%d:%d", dateTime.day, dateTime.hour, dateTime.minute, dateTime.second);
	rprintf("\r\n");
}


void incrementFanSpeed()
{
	if( fanValue + 5 <= 100 )
		fanValue+= 5;
	setFanSpeed(fanValue);
}

void decrementFanSpeed()
{
	if( fanValue - 5 >= 0)
		fanValue-= 5;
	setFanSpeed(fanValue);
}

void startHRelay()
{
	sbi(HUMRELAY_PORT,HUMRELAY);
}

void stopHRelay()
{
	cbi(HUMRELAY_PORT,HUMRELAY);
}

void turnR1On()
{
	sbi(R1_PORT,R1);
}

void turnR1Off()
{
	cbi(R1_PORT,R1);	
}

void toggleR1()
{
	R1_PORT ^= 1 << R1;
}

void turnR2On()
{
	sbi(R2_PORT,R2);
}

void turnR2Off()
{
	cbi(R2_PORT,R2);
}

void toggleR2()
{
	R2_PORT ^= 1 << R2;
}

void checkWaterSensor()
{
	if(waterLevel < 150)
		raiseError(OUT_OF_WATER);
}

void startBuzz()
{
	sbi(BUZZ_PORT,BUZZ);
}

void stopBuzz()
{
	cbi(BUZZ_PORT,BUZZ);
}

ISR(ADC_vect)
{
	waterLevel = ADCH;
	ADCSRA &= ~(1 << ADSC);
}


ISR(TIMER0_OVF_vect)
{
	
}

/*

  //Check for overheat
  if((allTemps[0] > 39 || allTemps[1] > 39 || allTemps[2] > 39) && (allTemps[0] < 80 || allTemps[1] < 80 || allTemps[2] < 80)){
	  digitalWrite(OverHeatPin,HIGH);
	  for(int i = 0; i < 3; i++)
	  {
		  digitalWrite(buzzPin,HIGH);
		  delay(100);
		  digitalWrite(buzzPin,LOW);
	  }
  }
  else
  digitalWrite(OverHeatPin,LOW);
  
  //Save data
  if(minutes %5 != 0)
  interval5Passed =false;
  if(minutes%5==0 && !interval5Passed){
	  interval5Passed = true;
	  saveData();
  }
  
  //Check if need to send data
  char message[20];

  if(Serial.available() > 0){
	  int i = 0;
	  while(Serial.available() > 0){
		  message[i] = Serial.read();
		  i++;
	  }
	  message[i]=0;

	  if(strstr(message,"setT")!=NULL){
		  balanceTemp = atof(&(message[4]));
		  Serial.print("The new balanceTemp is ");Serial.println(balanceTemp);
		  for(int j = 0; j<20;j++)
		  message[j] = 0;
	  }
	  
	  if(strstr(message,"setU")!=NULL){
		  balanceHumidity = atof(&(message[4]));
		  Serial.print("The new balanceHumidity is ");Serial.println(balanceHumidity);
		  for(int j = 0; j<20;j++)
		  message[j] = 0;
	  }
	  if(strstr(message,"reset")!=NULL){
		  byte buf[]={0,0};
		  EEPROM1024.writeBuffer(0,buf,2);
		  delay(10);
		  wdt_enable(WDTO_2S);
		  while(1);
	  }
	  if(strstr(message,"setD")!=NULL){
		  int tmpDay;
		  tmpDay = atoi(&(message[4]));
		  Serial.print("The new day is ");Serial.println(tmpDay);
		  days=tmpDay;
		  setIDX(days*288);
		  for(int j = 0; j<20;j++)
		  message[j] = 0;
	  }
	  
	  if(strstr(message,"down")!=NULL){
		  for(int j = 0; j<20;j++)
		  message[j] = 0;
		  sendData();
	  }
  }

  lcd.setCursor(4,3);
  lcd_printf("%02d",hours);
  lcd.print(":");
  lcd_printf("%02d",minutes);
  lcd.print(":");
  lcd_printf("%02d",seconds);
  //Go to second stage
  if(days >= 19){
	  balanceTemp = SECOND_STAGE_TEMP;
	  balanceHumidity = SECOND_STAGE_HUMIDITY;
  }
*/