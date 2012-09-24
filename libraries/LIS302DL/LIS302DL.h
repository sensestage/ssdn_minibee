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

#ifndef LIS302DL_h
#define LIS302DL_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Wire.h>


/* ------- Register names ------- */

#define LIS302DL_addres1 0x1C
// #define LIS302DL_addres2 0x1D // ??? check!

#define LIS302DL_accelX 0x29
#define LIS302DL_accelY 0x2B
#define LIS302DL_accelZ 0x2D


class LIS302DL
{
public:
  LIS302DL();
  void setup();
  void read( int* accx, int* accy, int* accz );

private:
  int readTWI(int address, int bytes);
  int readTWI(int address, int reg, int bytes);
//   byte _buff[6] ;    //6 bytes buffer for saving data read from the device
};

#endif // LIS302DL_h

