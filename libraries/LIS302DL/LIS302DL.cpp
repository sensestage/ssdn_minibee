/**************************************************************************
 *                                                                         *
 * LIS302DL Driver for Arduino                                              *
 *                                                                         *
 ***************************************************************************
 *                                                                         * 
 * This program is free software; you can redistribute it and/or modify    *
 * it under the terms of the GNU License.                                  *
 * This program is distributed in the hope that it will be useful,         *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 * GNU License V2 for more details.                                        *
 *                                                                         *
 ***************************************************************************/

#include "LIS302DL.h"

#include <Wire.h>

LIS302DL::LIS302DL(){
}

void LIS302DL::setup(){
      //------- LIS302DL setup --------------
      Wire.beginTransmission(LIS302DL_addres1);
#if defined(ARDUINO) && ARDUINO >= 100
      Wire.write( (byte) 0x21); // CTRL_REG2 (21h)
      Wire.write( (byte) B01000000);
#else
      Wire.send( 0x21); // CTRL_REG2 (21h)
      Wire.send( B01000000);
#endif
      Wire.endTransmission();
   
	//SPI 4/3 wire
	//1=ReBoot - reset chip defaults
	//n/a
	//filter off/on
	//filter for freefall 2
	//filter for freefall 1
	//filter freq MSB
	//filter freq LSB - Hipass filter (at 400hz) 00=8hz, 01=4hz, 10=2hz, 11=1hz (lower by 4x if sample rate is 100hz)   

      Wire.beginTransmission(LIS302DL_addres1);
#if defined(ARDUINO) && ARDUINO >= 100
      Wire.write( (byte) 0x20); // CTRL_REG1 (20h)
      Wire.write( (byte) B01000111);
#else
      Wire.send(0x20); // CTRL_REG1 (20h)
      Wire.send(B01000111);
#endif
      Wire.endTransmission();
      
	//sample rate 100/400hz
	//power off/on
	//2g/8g
	//self test
	//self test
	//z enable
	//y enable
	//x enable 

      //-------end LIS302DL setup --------------
}

void LIS302DL::read( int* accx, int* accy, int* accz ){
    /// reading LIS302DL
    *accx = readTWI( LIS302DL_addres1, LIS302DL_accelX, 1 ) + 128 % 256;
    *accy = readTWI( LIS302DL_addres1, LIS302DL_accelY, 1 ) + 128 % 256;
    *accz = readTWI( LIS302DL_addres1, LIS302DL_accelZ, 1 ) + 128 % 256;
}

int LIS302DL::readTWI(int address, int bytes) {
	uint8_t i = 0;
	int twi_reading[bytes];
	Wire.requestFrom(address, bytes);
	while(Wire.available()) {
#if defined(ARDUINO) && ARDUINO >= 100
		twi_reading[i] = Wire.read();
#else
		twi_reading[i] = Wire.receive();
#endif
		i++;
	}
	return *twi_reading;
}

///read a specific register on a particular device.
int LIS302DL::readTWI(int address, int reg, int bytes) {
	uint8_t i = 0;
	int twi_reading[bytes];
	Wire.beginTransmission(address);
#if defined(ARDUINO) && ARDUINO >= 100
	Wire.write( (byte) reg);                   //set x register
#else
	Wire.send(reg);                   //set x register
#endif
	Wire.endTransmission();
	Wire.requestFrom(address, bytes);            //retrieve x value
	while(Wire.available()) {   
#if defined(ARDUINO) && ARDUINO >= 100
		twi_reading[i] = Wire.read();
#else
		twi_reading[i] = Wire.receive();
#endif
		i++;
	}
	return *twi_reading;
}
