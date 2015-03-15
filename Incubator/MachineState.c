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

volatile bool oneSecondPassed = false;
volatile bool startR1 = false;
volatile bool startR2 = false;
bool buzzIsOn = false;

char* LCD_ERROR_MESSAGES[] = {"NIVEL APA SCAZUT","TEMP SUB NIVEL","TEMP PESTE NIVEL","LIPSA SENZOR"};

MachineErrors machineError;
uint8_t balanceTemperature = FIRST_STAGE_TEMP;
uint8_t balanceHumidity = FIRST_STAGE_HUMIDITY;

int16_t T1 = 0;
int16_t T2 = 0;
int16_t T3 = 0;
uint8_t U = 0;
uint8_t humidityValue = 0;
uint8_t fanValue = 80;
int16_t minTemp = 100;
int16_t maxTemp = 0;
int8_t waterLevel;

// Protos
void initPins();
void initDDRDs();
void setFanSpeed(double percent);
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
	setFanSpeed(80);
}

void initDDRDs()
{
	sbi(R1_DDR,R1);
	sbi(R2_DDR,R2);
	sbi(BUZZ_DDR, BUZZ);
	sbi(HUMRELAY_DDR,HUMRELAY);
	cbi(WATER_DDR, WATER);
	sbi(OVRHEAT_DDR,OVRHEAT);
	sbi(ROLLING_DDR,ROLLING);
}

void setFanSpeed(double percent)
{
	OCR0A = (percent / 100) * 255;
}

void initADC()
{
	ADMUX = (1<< REFS0);				//Use ADC as refernce
	ADMUX = (1 << ADLAR) | (1 << ADC2D ); // 8bit conversion + ADC2
	ADMUX |= (1 << ADC);
}

void initPWMTimer()
{
	DDRD |= 1 << PORTD6;
	TCCR0A = 1 << COM0A1 | 1 << WGM01 | 1 << WGM00;
	OCR0A = 0;
	TIMSK0 = 1 << TOIE0;
	TCCR0B = 1 << CS00 | 1 << CS02;	
}


void readSHTSensor()
{
	 SHT21_Read(&T2,&U);
}

void readWaterSensor()
{
	ADCSRA = (1 << ADEN);
	ADCSRA |= (1 << ADSC);
	while( (ADCSRA & ( 1 << ADSC )) != 0 );
	waterLevel = ADCH;
	ADCSRA = 0;
}

void readSensors()
{
	readBSSensors();
	readSHTSensor();
	readWaterSensor();
}

void readBSSensors()
{
	T1 = readTempSensor(1);
	T3 = readTempSensor(2);
}


void checkMachineStatus()
{
	readSensors();	
	readWaterSensor();
	checkHumidity();
	checkTemperatures();
	checkWaterSensor();
	if(machineError == NONE)
		buzzIsOn = false;
	machineError = NONE;	
//  	if( oneSecondPassed)
//  	{
//  		//printToUart();
//  		//printToLCD();
//  		oneSecondPassed = false;
//  	}
// 	_delay_ms(200);
// 	machineError = NONE;	
}

void checkHumidity()
{
	if (humidityValue > balanceHumidity)
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
	bool startResistor = false;
	setMinMax();
	double t1 = T1/10;
	double t2 = T2/10;
	double t3 = T3/10;
	
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
		if(t1 < balanceTemperature*10)
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
	startR1 = false;
	
	if(t3 < balanceTemperature)
	{
		startResistor = true;
		if(t2 < balanceTemperature - 1)
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
		startR2 = false;
		
	if(t1 < balanceTemperature - MAX_TEMP_DELTA || t2 < balanceTemperature - MAX_TEMP_DELTA || t3 < balanceTemperature - MAX_TEMP_DELTA )
	raiseError(LOW_TEMPERATURE);
	if(t1 > balanceTemperature + MAX_TEMP_DELTA  || t2 > balanceTemperature + MAX_TEMP_DELTA  || t3 > balanceTemperature + MAX_TEMP_DELTA)
	raiseError(HIGH_TEMPERATURE);
}

void raiseError(MachineErrors error)
{
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
	}
	_delay_ms(500);
}

void printToLCD()
{
	rprintfInit(lcd_putc);
	lcd_gotoXY(1,0);
	rprintfFloat(3,T1/10.0);
	rprintfStr(" ");
	rprintfFloat(3,T2/10.0);
	rprintfStr(" ");
	rprintfFloat(3,T3/10.0);
	lcd_gotoXY(2,0);
	rprintf("U:%d ", U);
	rprintf("m:");
	rprintfFloat(3,minTemp/10.0);
	rprintfStr("");
	rprintf("M:");
	rprintfFloat(3,maxTemp/10.0);
	
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
	setFanSpeed(++fanValue);
}

void decrementFanSpeed()
{
	setFanSpeed(--fanValue);
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
	if(waterLevel < 200)
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