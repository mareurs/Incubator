/*
 * ds18x20.h
 *
 * Created: 3/7/2015 10:34:41 PM
 *  Author: marius
 */ 

#include <avr/io.h>

#ifndef DS18X20_H_
#define DS18X20_H_

/*
 * ds18x20.c
 *
 * Created: 3/7/2015 10:33:20 PM
 *  Author: marius
 */ 

/*
 * ds18x20.c
 *
 * Created: 3/7/2015 10:33:20 PM
 *  Author: marius
 */ 

#include "onewire.h"
#include <util/delay.h>

#define DSB_PORT	PORTB
#define DSB_DDR		DDRB
#define DSB_IN		PINB
#define DSB			PINB0

#define MAXSENSORS	2

/* return values */
#define DS18X20_OK                0x00
#define DS18X20_ERROR             0x01
#define DS18X20_START_FAIL        0x02
#define DS18X20_ERROR_CRC         0x03

#define DS18X20_INVALID_DECICELSIUS  2000

#define DS18X20_POWER_PARASITE    0x00
#define DS18X20_POWER_EXTERN      0x01

#define DS18X20_CONVERSION_DONE   0x00
#define DS18X20_CONVERTING        0x01

/* DS18X20 specific values (see datasheet) */
#define DS18S20_FAMILY_CODE       0x10
#define DS18B20_FAMILY_CODE       0x28
#define DS1822_FAMILY_CODE        0x22

#define DS18X20_CONVERT_T         0x44
#define DS18X20_READ              0xBE
#define DS18X20_WRITE             0x4E
#define DS18X20_EE_WRITE          0x48
#define DS18X20_EE_RECALL         0xB8
#define DS18X20_READ_POWER_SUPPLY 0xB4

#define DS18B20_CONF_REG          4
#define DS18B20_9_BIT             0
#define DS18B20_10_BIT            (1<<5)
#define DS18B20_11_BIT            (1<<6)
#define DS18B20_12_BIT            ((1<<6)|(1<<5))
#define DS18B20_RES_MASK          ((1<<6)|(1<<5))

// undefined bits in LSB if 18B20 != 12bit
#define DS18B20_9_BIT_UNDF        ((1<<0)|(1<<1)|(1<<2))
#define DS18B20_10_BIT_UNDF       ((1<<0)|(1<<1))
#define DS18B20_11_BIT_UNDF       ((1<<0))
#define DS18B20_12_BIT_UNDF       0

// conversion times in milliseconds
#define DS18B20_TCONV_12BIT       750
#define DS18B20_TCONV_11BIT       DS18B20_TCONV_12_BIT/2
#define DS18B20_TCONV_10BIT       DS18B20_TCONV_12_BIT/4
#define DS18B20_TCONV_9BIT        DS18B20_TCONV_12_BIT/8
#define DS18S20_TCONV             DS18B20_TCONV_12_BIT

// constant to convert the fraction bits to cel*(10^-4)
#define DS18X20_FRACCONV          625

// scratchpad size in bytes
#define DS18X20_SP_SIZE           9

// DS18X20 EEPROM-Support
#define DS18X20_WRITE_SCRATCHPAD  0x4E
#define DS18X20_COPY_SCRATCHPAD   0x48
#define DS18X20_RECALL_E2         0xB8
#define DS18X20_COPYSP_DELAY      10 /* ms */
#define DS18X20_TH_REG            2
#define DS18X20_TL_REG            3

#define DS18X20_DECIMAL_CHAR      '.'


uint8_t search_sensors(void);
uint8_t DS18X20_find_sensor( uint8_t *diff, uint8_t id[] );
void readTempSensor(int16_t* s1, int16_t* s2);
int16_t DS18X20_raw_to_decicelsius( uint8_t familycode, uint8_t sp[] );
uint8_t DS18X20_read_decicelsius_single( uint8_t familycode, int16_t *decicelsius );
uint8_t DS18X20_start_meas( uint8_t with_power_extern, uint8_t id[]);
uint8_t DS18X20_get_power_status( uint8_t id[] );
uint8_t DS18X20_read_maxres( uint8_t id[], int32_t *temperaturevalue );
uint8_t DS18X20_format_from_maxres( int32_t temperaturevalue, char str[], uint8_t n);
uint8_t DS18X20_read_decicelsius( uint8_t id[], int16_t *decicelsius );
#endif /* DS18X20_H_ */