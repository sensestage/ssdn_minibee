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
#include "WProgram.h"

#ifndef LIS302DL_h
#define LIS302DL_h

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
  void read( uint8_t* accx, uint8_t* accy, uint8_t* accz );
  
//   void readAccel(int* xyx);
//   void readAccel(int* x, int* y, int* z);
//   void readAccelRaw(char *data, int dboff);
//   void get_Gxyz(double *xyz);


private:
  int readTWI(int address, int bytes);
  int readTWI(int address, int reg, int bytes);
//   byte _buff[6] ;    //6 bytes buffer for saving data read from the device
};
// void print_byte(byte val);
#endif // LIS302DL_h

