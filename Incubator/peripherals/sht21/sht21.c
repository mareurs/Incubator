//--------------------------------------------------------------------------------------------------
//                                  _            _     
//                                 | |          | |    
//      ___ _ __ ___  ___ _   _ ___| |_ ___  ___| |__  
//     / _ \ '_ ` _ \/ __| | | / __| __/ _ \/ __| '_ \. 
//    |  __/ | | | | \__ \ |_| \__ \ ||  __/ (__| | | |
//     \___|_| |_| |_|___/\__, |___/\__\___|\___|_| |_|
//                         __/ |                       
//                        |___/    Engineering (www.emsystech.de)
//
// Filename:    sht21.c
// Description: sht21.h
//
// Open Source Licensing 
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// Dieses Programm ist Freie Software: Sie können es unter den Bedingungen
// der GNU General Public License, wie von der Free Software Foundation,
// Version 3 der Lizenz oder (nach Ihrer Option) jeder späteren
// veröffentlichten Version, weiterverbreiten und/oder modifizieren.
//
// Dieses Programm wird in der Hoffnung, dass es nützlich sein wird, aber
// OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite
// Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
// Siehe die GNU General Public License für weitere Details.
//
// Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
// Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.
// Author:      Martin Steppuhn
// History:     01.01.2011 Initial version
//--------------------------------------------------------------------------------------------------

//=== Includes =====================================================================================

#include "sht21.h"
#include <util/delay.h>
#include "rprintf.h"

//=== Preprocessing directives (#define) ===========================================================

//=== Type definitions (typedef) ===================================================================

//=== Global constants =============================================================================

//=== Global variables =============================================================================

uint8_t	Sht21Error;

//=== Local constants  =============================================================================

//=== Local variables ==============================================================================

//=== Local function prototypes ====================================================================

void SHT21_Init()
{
	PORTC |= 1 << PORTC5;
	_delay_ms(15);
	i2cInit();
}

uint8_t CalcSht21Crc(uint8_t *data,uint8_t nbrOfBytes);

//--------------------------------------------------------------------------------------------------
// Name:      SHT21_Read
// Function:  
//            
// Parameter: 
// Return:    
//--------------------------------------------------------------------------------------------------
uint8_t SHT21_Read(int16_t *temperature,uint8_t *humidity)
{
	uint32_t	val;
	uint8_t	buf[4];
		
	Sht21Error = 0;
	
	//=== Softreset ==================================================
	
	I2C_Write(0xFE);			// softreset < 15ms
	_delay_ms(50);
	
	//=== Temperature =================================================

	I2C_Write(0xF3);			// start temperature measurement
	_delay_ms(260);				// Temperature@14Bit typ=66ms max=85ms
	I2C_Read(buf,3);			// read temperature data
	
	
	if(buf[2] == CalcSht21Crc(buf,2))  // check CRC
	{
		val = buf[0];
		val <<= 8;
		val += buf[1];
		val &= 0xFFFC;
  		  		
		//	T = -46,85 + 175,72 * St/65535      da 1/10K -->  * 10
		//	T = -468,5 + 1757,2 * St/65535		verinfachen
		//	T = -468,5 + St / 37,2956..			damit Konstante ganzzahlig wird mit 2 erweitern
		//  T = -937 + 2*St / 37,2956..			Bruch für Division mit 256 erweitern  
		//	T = (-937 +  (St * 512) / (37,2956.. * 256)  )  / 2
		//	T = (((St * 512) / 9548) - 937) / 2
  	  		
		//	val = (((val * 512) / 9548) - 937) / 2;
		*temperature = ((val * 512) / 9548);
		*temperature = ((*temperature) - 937) / 2;       
	}
	else
	{
		Sht21Error |= ERROR_SHT21_CRC_TEMP;
		*temperature = 2500;
	}
	
	//=== Humidity ===================================================

	I2C_Write(0xF5);			// start humidity measurement
	_delay_ms(260);				// RH@12Bit typ=22ms max=20ms 
	I2C_Read(buf,3);			// read humidity data
		
  	if(buf[2] == CalcSht21Crc(buf,2))
	{	
  		val = buf[0];
  		val <<= 8;
  		val += buf[1];
  		val &= 0xFFFC;
  		  			
  		//   T = -6 + 125* Srh/65535      
  		//	 T = -6 + Srh / 524,28
  		//   T = -6 + (Srh * 256) / 134215      |  *256	 wegen Numerik erweitern
  	  		  		
  		val = ((val * 256) / 134215) - 6;
  		*humidity = val;
	}	
	else
	{
		Sht21Error |= ERROR_SHT21_CRC_TEMP;
		//*humidity = 250;
	}
	
	//if(I2cError) Sht21Error |= ERROR_SHT21_I2C;
	
	
	
	return Sht21Error;
}

//------------------------------------------------------------------------------
// Name:      
// Function:  
//            
// Parameter: 
// Return:    
//------------------------------------------------------------------------------
uint8_t CalcSht21Crc(uint8_t *data,uint8_t nbrOfBytes)
{
	// CRC
	//const u16t POLYNOMIAL = 0x131; //P(x)=x^8+x^5+x^4+1 = 100110001
	
	uint8_t byteCtr,bit,crc;

	crc = 0;

	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
	{ 
		crc ^= (data[byteCtr]);
		for (bit = 8; bit > 0; --bit)
		{
			if (crc & 0x80) crc = (crc << 1) ^ 0x131;
				else 		crc = (crc << 1);
		}
	}
	return(crc);
}

//--------------------------------------------------------------------------------------------------
// Name:      PrintSht21Error
// Function:  Print error flags as readable text.
//            
// Parameter: -
// Return:    -
//--------------------------------------------------------------------------------------------------
void PrintSht21Error(void)
{	
	if(Sht21Error & ERROR_SHT21_I2C)			rprintf("ERROR I2C-Port\n");
	if(Sht21Error & ERROR_SHT21_CRC_TEMP)		rprintf("ERROR Temperature CRC\n");
	if(Sht21Error & ERROR_SHT21_CRC_HUMIDITY)	rprintf("ERROR Humidity CRC\n");
}

