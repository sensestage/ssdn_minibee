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

#ifndef TMP102_h
#define TMP102_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif


/* ------- Register names ------- */

#define TMP102_addres1 0x48 // assume ADR0 is tied to GND
#define TMP102_addres2 0x49 // assume ADR0 is tied to VCC
#define TMP102_addres3 0x4A // assume ADR0 is tied to SDA
#define TMP102_addres4 0x4B // assume ADR0 is tied to SCL

// #define TMP102_addres1 0x92 // assume ADR0 is tied to VCC
// #define TMP102_addres2 0x92 // assume ADR0 is tied to VCC
// #define LIS302DL_addres2 0x1D // ??? check!

// #define TMP102_RD	0x93
// #define TMP102_WR	0x92 //Assume ADR0 is tied to VCC

#define TMP102_REG 	0x00
#define TMP102_CONF	0x01
#define TMP102_LOW	0x02
#define TMP102_HIGH	0x03

class TMP102
{
public:
  TMP102();
  void init();
  void setup();
  byte readTemp();
  int readLow();
  int readHigh();
  
  int currentTemp;

private:
//   int readTWI(int address, int bytes);
//   int readTWI(int address, int reg, int bytes);
//   byte _buff[6] ;    //6 bytes buffer for saving data read from the device
};

#endif // TMP102_h
