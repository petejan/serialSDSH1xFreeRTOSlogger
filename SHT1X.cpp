/**
 * SHT1x Library
 *
 * Copyright 2009 Jonathan Oxer <jon@oxer.com.au> / <www.practicalarduino.com>
 * Based on previous work by:
 *    Maurice Ribble: <www.glacialwanderer.com/hobbyrobotics/?p=5>
 *    Wayne ?: <ragingreality.blogspot.com/2008/01/ardunio-and-sht15.html>
 *
 * Updated for Arduino 1.6.5 Library Manger by Joel Bartlett 
 * SparkFun Electronics 
 * September 16, 2015
 *
 * Manages communication with SHT1x series (SHT10, SHT11, SHT15)
 * temperature / humidity sensors from Sensirion (www.sensirion.com).
 */

#include "FreeRTOS.h"
#include "task.h"
#include <util/delay.h>

#include "SHT1x.h"
#include <avr/io.h>

#define CLK_HI() {PORTD |= _BV(PORTD5);}
#define CLK_LOW() {PORTD &= ~_BV(PORTD5);}
#define CLK_OUT() {DDRD |= _BV(DDD5);}

#define DATA_IN() {DDRD &= ~_BV(DDD4);}
#define DATA_OUT() {DDRD |= _BV(DDD4);}


SHT1x::SHT1x()
{
	DATA_OUT();
	CLK_OUT();

	PORTD |= _BV(PORTD4);

	CLK_LOW();
}


/* ================  Public methods ================ */
//Reads the current temperature in degrees Celsius

float SHT1x::readTemperatureC()
{
	int _val;                // Raw value returned from sensor
	float _temperature;      // Temperature derived from raw value

	// Conversion coefficients from SHT15 datasheet
	const float D1 = -40.0;  // for 14 Bit @ 5V
	const float D2 =   0.01; // for 14 Bit DEGC

	// Fetch raw value
	_val = readTemperatureRaw();

	// Convert raw value to degrees Celsius
	_temperature = (_val * D2) + D1;

	return (_temperature);
}

////////////////////////////////////////////////////////////////////////
//Reads the current temperature in degrees Fahrenheit
float SHT1x::readTemperatureF()
{
	int _val;                 // Raw value returned from sensor
	float _temperature;       // Temperature derived from raw value

	// Conversion coefficients from SHT15 datasheet
	const float D1 = -40.0;   // for 14 Bit @ 5V
	const float D2 =   0.018; // for 14 Bit DEGF

	// Fetch raw value
	_val = readTemperatureRaw();

	// Convert raw value to degrees Fahrenheit
	_temperature = (_val * D2) + D1;

	return (_temperature);
}
////////////////////////////////////////////////////////////////////////
//Reads current temperature-corrected relative humidity
float SHT1x::readHumidity()
{
	int _val;                    // Raw humidity value returned from sensor
	float _linearHumidity;       // Humidity with linear correction applied
	float _correctedHumidity;    // Temperature-corrected humidity
	float _temperature;          // Raw temperature value

	// Conversion coefficients from SHT15 datasheet
	const float C1 = -4.0;       // for 12 Bit
	const float C2 =  0.0405;    // for 12 Bit
	const float C3 = -0.0000028; // for 12 Bit
	const float T1 =  0.01;      // for 14 Bit @ 5V
	const float T2 =  0.00008;   // for 14 Bit @ 5V

	// Command to send to the SHT1x to request humidity
	int _gHumidCmd = 0b00000101;

	// Fetch the value from the sensor
	sendCommandSHT(_gHumidCmd);
	waitForResultSHT();
	_val = getData16SHT();
	skipCrcSHT();

	// Apply linear conversion to raw value
	_linearHumidity = C1 + C2 * _val + C3 * _val * _val;

	// Get current temperature for humidity correction
	_temperature = readTemperatureC();

	// Correct humidity value for current temperature
	_correctedHumidity = (_temperature - 25.0 ) * (T1 + T2 * _val) + _linearHumidity;

	return (_correctedHumidity);
}

void SHT1x::setHeater(int on)
{
	sendCommandSHT(0b00000110); // write status
	if (on != 0)
	{
		sendData(0x04); // htr on
	}
	else
	{
		sendData(0); // htr off
	}
	skipCrcSHT();
}

/* ================  Private methods ================ */
float SHT1x::readTemperatureRaw()
{
	int _val;

	// Command to send to the SHT1x to request Temperature
	int _gTempCmd  = 0b00000011;

	sendCommandSHT(_gTempCmd);
	waitForResultSHT();
	_val = getData16SHT();
	skipCrcSHT();

	return (_val);
}
////////////////////////////////////////////////////////////////////////
int SHT1x::shiftIn()// commands for reading/sending data to a SHTx sensor
{
	int ret = 0;
	int i;
	int8_t ack;

	for (i=0; i<8; ++i)
	{
		CLK_HI();
//		delay(10);  // I don't know why I need this, but without it I don't get my 8 lsb of temp
		ack = PIND & _BV(PORTD4);

		if (ack == 0)
			ack = 0;
		else
			ack = 1;
		ret = ret*2 + ack;

		CLK_LOW();
	}

	return(ret);
}

////////////////////////////////////////////////////////////////////////
void SHT1x::sendCommandSHT(int _command)// send a command to the SHTx sensor
{
	// Transmission Start
	DATA_OUT();
	CLK_OUT();

	PORTD |= _BV(PORTD4);

	// start command
	CLK_HI();
	PORTD &= ~_BV(PORTD4);
	CLK_LOW();
	CLK_HI();
	PORTD |= _BV(PORTD4);
	CLK_LOW();

	sendData(_command);
}
void SHT1x::sendData(int _data)
{
	// send the data
	uint8_t i;

	DATA_OUT();
	for (i = 0; i < 8; i++)
	{
		if ((_data & (1 << (7 - i))) != 0)
			PORTD |= _BV(PORTD4);
		else
			PORTD &= ~_BV(PORTD4);

		//digitalWrite(dataPin, !!(val & (1 << (7 - i))));

		CLK_HI();
		if (i == 7)
			DATA_IN();
		CLK_LOW();
	}

	// Verify we get the correct ack
	CLK_HI();
	if ((PIND && _BV(PORTD4)) != 0)
	{
		//Serial.println("Ack Error 0");
	}
	//DATA_IN();
	//ack = digitalRead(_dataPin);
	CLK_LOW();
	if ((PIND && _BV(PORTD4)) == 0)
	{
		//Serial.println("Ack Error 1");
	}
}
////////////////////////////////////////////////////////////////////////
void SHT1x::waitForResultSHT()// wait for the SHTx answer
{
	int i;
	int ack;

	DATA_IN();

	// Could be interrupt driven

	// sensor takes 250 ms to read
	for(i= 0; i < 100; ++i)
	{
		_delay_ms(25);
		ack = PIND & _BV(PORTD4);

		if (ack == 0)
		{
			break;
		}
	}

	if (ack != 0)
	{
		//Serial.println("Ack Error 2"); // Can't do serial stuff here, need another way of reporting errors
	}
}
////////////////////////////////////////////////////////////////////////
int SHT1x::getData16SHT()// get data from the SHTx sensor
{
	int val;

	// get the MSB (most significant bits)
	DATA_IN();
	CLK_OUT();

	val = shiftIn();
	val *= 256; // this is equivalent to val << 8;

	// send the required ACK
	DATA_OUT();
	PORTD |= _BV(PORTD4);
	PORTD &= ~_BV(PORTD4);
	CLK_HI();
	CLK_LOW();

	// get the LSB (less significant bits)
	DATA_IN();
	val |= shiftIn();

	return val;
}
////////////////////////////////////////////////////////////////////////
void SHT1x::skipCrcSHT()
{
	// Skip acknowledge to end trans (no CRC)
	DATA_OUT();
	CLK_OUT();

	PORTD |= _BV(PORTD4);
	CLK_HI();
	CLK_LOW();
}
