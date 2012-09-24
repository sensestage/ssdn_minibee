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
#include "TMP102.h"
#include <Wire.h>


TMP102::TMP102(){ 
init();
}

void TMP102::init(){
  currentTemp = 0;
}
  
void TMP102::setup(){
   Wire.beginTransmission(TMP102_addres1);  // select temperature register
#if defined(ARDUINO) && ARDUINO >= 100
   Wire.write( (byte) TMP102_REG);		   // (if only temperature is needed this can
#else
   Wire.send(TMP102_REG);		   // (if only temperature is needed this can
#endif
   Wire.endTransmission();	     // be done once in setup() ) 
}

byte TMP102::readTemp(){  

//   int val = -1;
  
  byte res = Wire.requestFrom(TMP102_addres1, 2);     // request temperature
  if (res == 2) {
#if defined(ARDUINO) && ARDUINO >= 100
    byte msb = Wire.read();
    byte lsb = Wire.read();    
#else
    byte msb = Wire.receive();
    byte lsb = Wire.receive();    
#endif
//  CALCULATING TEMPERATURES

    currentTemp = ((msb) << 4);   /* MSB */
    currentTemp |= (lsb >> 4);    /* LSB */
  }
  //    return val*0.0625;
  return res;
}

int TMP102::readLow(){  
  Wire.beginTransmission(TMP102_addres1);  // select temperature register
#if defined(ARDUINO) && ARDUINO >= 100
  Wire.write( (byte) TMP102_LOW);		   // (if only temperature is needed this can
  Wire.write( (byte) 24);  // (if only temperature is needed this can
#else
  Wire.send(TMP102_LOW);		   // (if only temperature is needed this can
  Wire.send(24);  // (if only temperature is needed this can
#endif
  Wire.endTransmission();	     // be done once in setup() )

  Wire.requestFrom(TMP102_addres1, 2);     // request temperature
#if defined(ARDUINO) && ARDUINO >= 100
  byte lowbyte1 = Wire.read();
  byte lowbyte2 = Wire.read();    
#else
  byte lowbyte1 = Wire.receive();
  byte lowbyte2 = Wire.receive();    
#endif

 //  Low_temp  
  int lowtempint = lowbyte1 << 8;	   // shift first byte to high byte in an int
  lowtempint = lowtempint | lowbyte2;	  // or the second byte into the int
  lowtempint = lowtempint >> 4;	     // right shift the int 4 bits per chip doc
//   float lowtempflt = float( lowtempint ) * .0625; // calculate actual temperature per chip doc

//    return val*0.0625;
   return lowtempint;
}

int TMP102::readHigh(){  
  Wire.beginTransmission(TMP102_addres1);  // select temperature register
#if defined(ARDUINO) && ARDUINO >= 100
  Wire.write( (byte) TMP102_HIGH);		   // (if only temperature is needed this can
  Wire.write( (byte) 28);  // (if only temperature is needed this can
#else
  Wire.send(TMP102_HIGH);		   // (if only temperature is needed this can
  Wire.send(28);  // (if only temperature is needed this can
#endif
  Wire.endTransmission();	     // be done once in setup() )

  Wire.requestFrom(TMP102_addres1, 2);     // request temperature
#if defined(ARDUINO) && ARDUINO >= 100
  byte hibyte1 = Wire.read();
  byte hibyte2 = Wire.read();    
#else
  byte hibyte1 = Wire.receive();
  byte hibyte2 = Wire.receive();    
#endif
  //  Hi_temp
  int hitempint = hibyte1 << 8;	   // shift first byte to high byte in an int
  hitempint = hitempint | hibyte2;	  // or the second byte into the int
  hitempint = hitempint >> 4;	     // right shift the int 4 bits per chip doc
//   float hitempflt = float( hitempint ) * .0625; // calculate actual temperature per chip doc
  return hitempint;
//   return val*0.0625;
}
