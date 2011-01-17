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
#include "TMP102.h"
#include <Wire.h>

int TMP102::readTemp(){  
  Wire.beginTransmission(TMP102_addres1);  // select temperature register
  Wire.send(TMP102_REG);		   // (if only temperature is needed this can
  Wire.endTransmission();	     // be done once in setup() )

  Wire.requestFrom(TMP102_addres1, 2);     // request temperature
  byte byte1 = Wire.receive();
  byte byte2 = Wire.receive();    

//  CALCULATING TEMPERATURES
  int tempint = byte1 << 8;	   // shift first byte to high byte in an int
  tempint = tempint | byte2;	  // or the second byte into the int
  tempint = tempint >> 4;	     // right shift the int 4 bits per chip doc
//   float tempflt = float( tempint ) * .0625; // calculate actual temperature per chip doc
    
//    return val*0.0625;
   return tempint;
}

int TMP102::readLow(){  
  Wire.beginTransmission(TMP102_addres1);  // select temperature register
  Wire.send(TMP102_LOW);		   // (if only temperature is needed this can
  Wire.send(24);  // (if only temperature is needed this can
  Wire.endTransmission();	     // be done once in setup() )

  Wire.requestFrom(TMP102_addres1, 2);     // request temperature
  byte lowbyte1 = Wire.receive();
  byte lowbyte2 = Wire.receive();    

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
  Wire.send(TMP102_HIGH);		   // (if only temperature is needed this can
  Wire.send(28);  // (if only temperature is needed this can
  Wire.endTransmission();	     // be done once in setup() )

  Wire.requestFrom(TMP102_addres1, 2);     // request temperature
  byte hibyte1 = Wire.receive();
  byte hibyte2 = Wire.receive();    

  //  Hi_temp
  int hitempint = hibyte1 << 8;	   // shift first byte to high byte in an int
  hitempint = hitempint | hibyte2;	  // or the second byte into the int
  hitempint = hitempint >> 4;	     // right shift the int 4 bits per chip doc
//   float hitempflt = float( hitempint ) * .0625; // calculate actual temperature per chip doc
  return hitempint;
//   return val*0.0625;
}
