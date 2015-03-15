/*
 * ds18x20.c
 *
 * Created: 3/7/2015 10:33:20 PM
 *  Author: marius
 */ 

#define DS_DEBUG 0

#include "ds18x20.h"
#include "onewire.h"
#include <util/delay.h>
#include "rprintf.h"
#include <stddef.h>
#include "crc8.h"

uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];

uint8_t search_sensors(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;

	#if DS_DEBUG > 0	
		rprintf("Scanning Bus for DS18X20\r\n");
	#endif
		
	ow_reset();

	nSensors = 0;
	
	diff = OW_SEARCH_FIRST;
	while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ) {
		DS18X20_find_sensor( &diff, &id[0] );
		
		if( diff == OW_PRESENCE_ERR ) {
			#if DS_DEBUG > 0
				rprintf( "No Sensor found");
			#endif
			break;
		}
		
		if( diff == OW_DATA_ERR ) {
			#if DS_DEBUG > 0
				rprintf( "Bus Error\r\n");
			#endif
			break;
		}
		
		for ( i=0; i < OW_ROMCODE_SIZE; i++ )
		gSensorIDs[nSensors][i] = id[i];
		
		nSensors++;
	}
	
	return nSensors;
}

uint8_t DS18X20_find_sensor( uint8_t *diff, uint8_t id[] )
{
	uint8_t go;
	uint8_t ret;

	ret = DS18X20_OK;
	go = 1;
	do {
		*diff = ow_rom_search( *diff, &id[0] );
		if ( *diff == OW_PRESENCE_ERR || *diff == OW_DATA_ERR ||
		*diff == OW_LAST_DEVICE ) {
			go  = 0;
			ret = DS18X20_ERROR;
			} else {
			if ( id[0] == DS18B20_FAMILY_CODE || id[0] == DS18S20_FAMILY_CODE ||
			id[0] == DS1822_FAMILY_CODE ) {
				go = 0;
			}
		}
	} while (go);

	return ret;
}


uint16_t readTempSensor(int senzor)
{
	switch(senzor)
	{
		case 1:
			ow_set_bus(&SENZOR1_IN,&SENZOR1_PORT,&SENZOR1_DDR,SENZOR1_P);
			break;
		case 2:
			ow_set_bus(&SENZOR2_IN,&SENZOR2_PORT,&SENZOR2_DDR,SENZOR2_P);				
	}
	
	int noSensors = search_sensors();
	int16_t decicelsius;
	#if DS_DEBUG > 0 	
		rprintf( " DS18X20 Sensor(s) available: %d\r\n",noSensors );
	#endif
	uint8_t i;

	#if DS_DEBUG > 0		
	for ( i = 0; i < noSensors; i++ ) {
			rprintf( "Sensor# %d is a ",i+1 );
		if ( gSensorIDs[i][0] == DS18S20_FAMILY_CODE ) {
				rprintf( "DS18S20/DS1820" );
			} else if ( gSensorIDs[i][0] == DS1822_FAMILY_CODE ) {
				rprintf( "DS1822" );
		}
		else {
			rprintf( "DS18B20" );
		}
		rprintf( " which is " );
		if ( DS18X20_get_power_status( &gSensorIDs[i][0] ) == DS18X20_POWER_PARASITE ) {
			rprintf( "parasite" );
			} else {
			rprintf( "externally" );
		}
		rprintf( " powered\r\n" );
	}
	#endif	
	
	if ( noSensors == 1 ) {
		#if DS_DEBUG > 0
			rprintf("\r\nThere is only one sensor "
			"-> Demo of \"DS18X20_read_decicelsius_single\":\r\n" );
		#endif
		i = gSensorIDs[0][0]; // family-code for conversion-routine
		DS18X20_start_meas( DS18X20_POWER_PARASITE, NULL );
		_delay_ms( DS18B20_TCONV_12BIT );
		DS18X20_read_decicelsius_single( i, &decicelsius );
		#if DS_DEBUG > 0
			rprintf("%d\r\n", decicelsius );
		#endif
	}
	return decicelsius;
}

uint8_t DS18X20_start_meas( uint8_t with_power_extern, uint8_t id[])
{
	uint8_t ret;

	ow_reset();
	if( ow_input_pin_state() ) { // only send if bus is "idle" = high
		if ( with_power_extern != DS18X20_POWER_EXTERN ) {
			ow_command_with_parasite_enable( DS18X20_CONVERT_T, id );
			/* not longer needed: ow_parasite_enable(); */
			} else {
			ow_command( DS18X20_CONVERT_T, id );
		}
		ret = DS18X20_OK;
	}
	else {
		#if DS_DEBUG > 0
			rprintf( "DS18X20_start_meas: Short Circuit!\r\n" );
		#endif
		ret = DS18X20_START_FAIL;
	}

	return ret;
}

static uint8_t read_scratchpad( uint8_t id[], uint8_t sp[], uint8_t n )
{
	uint8_t i;
	uint8_t ret;

	ow_command( DS18X20_READ, id );
	for ( i = 0; i < n; i++ ) {
		sp[i] = ow_byte_rd();
	}
	if ( crc8( &sp[0], DS18X20_SP_SIZE ) ) {
		ret = DS18X20_ERROR_CRC;
		} else {
		ret = DS18X20_OK;
	}

	return ret;
}


/* reads temperature (scratchpad) of sensor without id (single sensor)
   output: decicelsius 
   returns DS18X20_OK on success */
uint8_t DS18X20_read_decicelsius_single( uint8_t familycode, int16_t *decicelsius )
{
	uint8_t sp[DS18X20_SP_SIZE];
	uint8_t ret;
	
	ret = read_scratchpad( NULL, sp, DS18X20_SP_SIZE );
	if ( ret == DS18X20_OK ) {
		*decicelsius = DS18X20_raw_to_decicelsius( familycode, sp );
	}
	return ret;
}

uint8_t DS18X20_read_scratchpad( uint8_t id[], uint8_t sp[], uint8_t n )
{
	uint8_t ret;

	ow_reset();
	if( ow_input_pin_state() ) { // only send if bus is "idle" = high
		ret = read_scratchpad( id, sp, n );
	}
	else {
		#if DS_DEBUG > 0
			rprintf( "DS18X20_read_scratchpad: Short Circuit!\r\n" );
		#endif
		ret = DS18X20_ERROR;
	}

	return ret;
}

/* convert scratchpad data to physical value in unit decicelsius */
int16_t DS18X20_raw_to_decicelsius( uint8_t familycode, uint8_t sp[] )
{
	uint16_t measure;
	uint8_t  negative;
	int16_t  decicelsius;
	uint16_t fract;

	measure = sp[0] | (sp[1] << 8);
	//measure = 0xFF5E; // test -10.125
	//measure = 0xFE6F; // test -25.0625

	if( familycode == DS18S20_FAMILY_CODE ) {   // 9 -> 12 bit if 18S20
		/* Extended measurements for DS18S20 contributed by Carsten Foss */
		measure &= (uint16_t)0xfffe;   // Discard LSB, needed for later extended precicion calc
		measure <<= 3;                 // Convert to 12-bit, now degrees are in 1/16 degrees units
		measure += (16 - sp[6]) - 4;   // Add the compensation and remember to subtract 0.25 degree (4/16)
	}

	// check for negative
	if ( measure & 0x8000 )  {
		negative = 1;       // mark negative
		measure ^= 0xffff;  // convert to positive => (twos complement)++
		measure++;
	}
	else {
		negative = 0;
	}

	// clear undefined bits for DS18B20 != 12bit resolution
	if ( familycode == DS18B20_FAMILY_CODE || familycode == DS1822_FAMILY_CODE ) {
		switch( sp[DS18B20_CONF_REG] & DS18B20_RES_MASK ) {
			case DS18B20_9_BIT:
			measure &= ~(DS18B20_9_BIT_UNDF);
			break;
			case DS18B20_10_BIT:
			measure &= ~(DS18B20_10_BIT_UNDF);
			break;
			case DS18B20_11_BIT:
			measure &= ~(DS18B20_11_BIT_UNDF);
			break;
			default:
			// 12 bit - all bits valid
			break;
		}
	}

	decicelsius = (measure >> 4);
	decicelsius *= 10;

	// decicelsius += ((measure & 0x000F) * 640 + 512) / 1024;
	// 625/1000 = 640/1024
	fract = ( measure & 0x000F ) * 640;
	if ( !negative ) {
		fract += 512;
	}
	fract /= 1024;
	decicelsius += fract;

	if ( negative ) {
		decicelsius = -decicelsius;
	}

	if ( /* decicelsius == 850 || */ decicelsius < -550 || decicelsius > 1250 ) {
		return DS18X20_INVALID_DECICELSIUS;
		} else {
		return decicelsius;
	}
}

uint8_t DS18X20_get_power_status( uint8_t id[] )
{
	uint8_t pstat;

	ow_reset();
	ow_command( DS18X20_READ_POWER_SUPPLY, id );
	pstat = ow_bit_io( 1 );
	ow_reset();
	return ( pstat ) ? DS18X20_POWER_EXTERN : DS18X20_POWER_PARASITE;
}
